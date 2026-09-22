// Safety layer around cubemars_hardware::CubeMarsSystemHardware.
//
// The upstream plugin (src/external/cubemars_hardware, pinned upstream commit) leaves
// the motors running at their last command whenever the controller manager stops or
// crashes, only logs motor faults and missing status frames, sends position commands
// straight to the motor's full-speed position loop and has no notion of a motor that is
// mounted the other way round. This subclass keeps the upstream CAN protocol code and
// adds, through a second SocketCAN socket that only receives status frames:
//
//   * zero speed followed by zero current (release) to every motor of the component on
//     deactivate, cleanup, shutdown, error and destruction;
//   * a watchdog in read() that returns ERROR when a motor reports a fault code, stops
//     sending status frames or (optional, per joint) pushes harder than stall_effort
//     for stall_time_ms, so the controller manager stops the controllers that use the
//     component and calls on_error();
//   * no command frames at all unless the component is active;
//   * zero speed followed by release to a joint whose controller lets go of it while the
//     component stays active (a controller deactivated or switched off by the controller
//     manager). The upstream plugin then stops sending frames, and the motor would keep
//     its last speed or current command until its own CAN timeout;
//   * a per-joint direction: commands and states are negated between ros2_control and
//     the upstream plugin, so a mirror-mounted motor needs no negative gear_ratio;
//   * for joints with a position command interface: every command is clamped to the
//     URDF position limits and slew-limited at the URDF velocity limit, starting from the
//     measured position, so the motor's position loop (servo mode 4, which runs at full
//     speed) never receives a jump;
//   * zero_on_activate: the motor gets a temporary origin (servo mode 5) on activation,
//     so the pose the joint is in at that moment becomes position 0. For single-encoder
//     motors such as the AK45-10, which forget their position at power-off.
//
// Hardware parameters, in addition to the upstream ones:
//   status_timeout_ms  silence after which a motor counts as lost (default 100)
//   brake_time_ms      how long zero speed is held before the release (default 300)
// Joint parameters, in addition to the upstream ones:
//   direction          1 (default) or -1: the sign between the motor and the joint
//   zero_on_activate   true or false (default): set a temporary origin on activation
//   stall_effort       |effort| in Nm above which the joint counts as stalled; 0 or
//                      absent (default) disables the stall guard
//   stall_time_ms      how long the effort may stay above stall_effort (default 500)

#ifndef CUBEMARS_HARDWARE_SAFE__SAFE_SYSTEM_HPP_
#define CUBEMARS_HARDWARE_SAFE__SAFE_SYSTEM_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "cubemars_hardware/system.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace cubemars_hardware_safe
{
class SafeCubeMarsSystemHardware : public cubemars_hardware::CubeMarsSystemHardware
{
public:
  ~SafeCubeMarsSystemHardware() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using Clock = std::chrono::steady_clock;
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  // Index into base_states_ / base_commands_, or -1 when the upstream plugin does not
  // export that interface
  struct BaseIndex
  {
    int position = -1;
    int velocity = -1;
    int effort = -1;
  };

  struct Joint
  {
    std::string name;
    std::uint32_t can_id = 0;
    double direction = 1.0;
    // Upstream unit conversion parameters, for the status frames read on our own socket
    double enc_off = 0.0;
    double kt = 0.0;
    double gear_ratio = 1.0;
    bool zero_on_activate = false;

    // Position safety, for joints with a position command interface
    bool position_commanded = false;
    bool has_position_limits = false;
    double min_position = -std::numeric_limits<double>::infinity();
    double max_position = std::numeric_limits<double>::infinity();
    double max_velocity = 0.0;
    // A controller holds the position command interface
    bool position_claimed = false;
    // Last position target sent to the motor, in the joint frame
    double target = kNaN;

    // No controller commands the joint any more: write() sends zero speed until
    // release_at, then zero current
    bool releasing = false;
    Clock::time_point release_at{};

    // Stall guard; stall_effort 0 disables it
    double stall_effort = 0.0;
    std::chrono::milliseconds stall_time{500};
    bool over_effort = false;
    Clock::time_point over_effort_since{};

    // Latest status frame from our own socket
    bool has_status = false;
    Clock::time_point last_status{};
    std::int16_t raw_position = 0;
    std::int16_t raw_current = 0;
    std::int8_t temperature = 0;
    std::uint8_t fault = 0;

    // Interfaces exported to ros2_control, in the joint frame
    double state_position = kNaN;
    double state_velocity = kNaN;
    double state_effort = kNaN;
    double state_temperature = kNaN;
    double command_position = kNaN;
    double command_velocity = kNaN;
    double command_effort = kNaN;

    BaseIndex base_state;
    BaseIndex base_command;
  };

  bool parse_joint(const hardware_interface::ComponentInfo & info, Joint & joint);

  bool open_socket();
  void close_socket();
  bool send(std::uint32_t id, const std::uint8_t * data, std::uint8_t length);
  bool send_int32(std::uint32_t id, std::int32_t value);
  // Zero speed for brake_time_, then zero current. Safe to call repeatedly.
  void stop_motors(const char * reason);
  // One cycle of the non-blocking stop of a joint that no controller commands any more.
  void continue_release(Joint & joint, Clock::time_point now);
  // Read every buffered status frame into joints_. False after a socket error.
  bool drain_status();
  // Drain status frames; false (after logging why) on a fault code or a silent motor.
  bool motors_healthy();
  // False (after logging why) when a joint has pushed harder than stall_effort for
  // longer than stall_time.
  bool efforts_ok();
  // Wait up to timeout for a status frame from every motor; false after logging which
  // motors are silent or report a fault.
  bool wait_for_status(std::chrono::milliseconds timeout);
  // Set a temporary origin on every zero_on_activate joint and check that it took.
  bool zero_joints();
  // Measured position in the joint frame from the latest status frame, or NaN.
  double measured_position(const Joint & joint) const;

  // Handles into the upstream plugin's private state and command vectors. They hold
  // raw pointers into vectors that the upstream on_init sizes once and never resizes.
  std::vector<hardware_interface::StateInterface> base_states_;
  std::vector<hardware_interface::CommandInterface> base_commands_;
  double base_state(int index) const;
  void set_base_command(int index, double value);

  std::string interface_;
  // Sized once in on_init; the exported interfaces point into it
  std::vector<Joint> joints_;
  std::chrono::milliseconds status_timeout_{100};
  std::chrono::milliseconds brake_time_{300};

  int socket_ = -1;
  // Command frames are only passed through while active. Atomic because the destructor
  // may run on another thread than the control loop
  std::atomic<bool> active_{false};
  // Set after a stop, cleared on activation, so repeated stops are skipped
  bool stopped_ = true;
};

}  // namespace cubemars_hardware_safe

#endif  // CUBEMARS_HARDWARE_SAFE__SAFE_SYSTEM_HPP_
