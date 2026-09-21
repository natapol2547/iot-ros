// Safety layer around cubemars_hardware::CubeMarsSystemHardware.
//
// The upstream plugin (src/external/cubemars_hardware, pinned upstream commit) leaves
// the motors running at their last speed command whenever the controller manager stops
// or crashes, and it only logs motor faults and missing status frames. This subclass
// keeps the upstream CAN protocol code and adds, through a second SocketCAN socket:
//
//   * zero speed followed by zero current (release) to every motor on deactivate,
//     cleanup, shutdown, error and destruction;
//   * a watchdog in read() that returns ERROR when a motor reports a fault code or
//     stops sending status frames, so the controller manager stops the controllers and
//     calls on_error(), which stops both wheels;
//   * no command frames at all unless the component is active.
//
// Hardware parameters, in addition to the upstream ones:
//   status_timeout_ms  silence after which a motor counts as lost (default 100)
//   brake_time_ms      how long zero speed is held before the release (default 300)

#ifndef CUBEMARS_HARDWARE_SAFE__SAFE_SYSTEM_HPP_
#define CUBEMARS_HARDWARE_SAFE__SAFE_SYSTEM_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "cubemars_hardware/system.hpp"
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

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using Clock = std::chrono::steady_clock;

  bool open_socket();
  void close_socket();
  bool send(std::uint32_t id, std::int32_t value);
  // Zero speed for brake_time_, then zero current. Safe to call repeatedly.
  void stop_motors(const char * reason);
  // Drain status frames; false (after logging why) on a fault code or a silent motor.
  bool motors_healthy();

  std::string interface_;
  std::vector<std::uint32_t> motor_ids_;
  std::vector<std::string> joint_names_;
  std::vector<Clock::time_point> last_status_;
  std::chrono::milliseconds status_timeout_{100};
  std::chrono::milliseconds brake_time_{300};

  int socket_ = -1;
  // Command frames are only passed through while active. Atomic because lifecycle
  // transitions and the control loop may run on different threads
  std::atomic<bool> active_{false};
  // Set after a stop, cleared on activation, so repeated stops are skipped
  bool stopped_ = true;
};

}  // namespace cubemars_hardware_safe

#endif  // CUBEMARS_HARDWARE_SAFE__SAFE_SYSTEM_HPP_
