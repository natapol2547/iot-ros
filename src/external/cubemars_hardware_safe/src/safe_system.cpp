#include "cubemars_hardware_safe/safe_system.hpp"

#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_limits/joint_limits.hpp"
#include "joint_limits/joint_limits_urdf.hpp"
#include "rclcpp/rclcpp.hpp"
// urdf/model.h includes a header of its own dependency that emits a deprecation #warning
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"
#include "urdf/model.h"
#pragma GCC diagnostic pop

namespace cubemars_hardware_safe
{
namespace
{
// Servo-mode CAN protocol (CubeMars AK Series Driver Manual, section 5): command ID
// (mode << 8) | motor ID, status ID (0x29 << 8) | ID
constexpr std::uint32_t kModeCurrent = 1;
constexpr std::uint32_t kModeSpeed = 3;
constexpr std::uint32_t kModeSetOrigin = 5;
constexpr std::uint32_t kStatus = 0x29;
// Set-origin payload byte: 0 temporary origin (lost at power-off), 1 permanent origin,
// 2 restore the default origin (both written to the motor's flash)
constexpr std::uint8_t kTemporaryOrigin = 0;
// Status frame position unit
constexpr double kDegPerCount = 0.1;
constexpr std::chrono::milliseconds kBrakePeriod{20};
// How long activation waits for status frames, and for a new origin to show in them
constexpr std::chrono::milliseconds kActivationWait{500};
constexpr std::chrono::milliseconds kPollPeriod{5};
// A zeroed joint must read within this of 0 (the status resolution is 0.1 deg)
constexpr double kZeroToleranceDeg = 1.0;
// Longest period one write() slews over, so a late cycle cannot become a jump
constexpr double kMaxSlewPeriod = 0.05;

const char * fault_text(std::uint8_t code)
{
  switch (code) {
    case 1: return "motor over-temperature";
    case 2: return "over-current";
    case 3: return "over-voltage";
    case 4: return "under-voltage";
    case 5: return "encoder fault";
    case 6: return "MOSFET over-temperature";
    case 7: return "motor stall";
    default: return "unknown fault";
  }
}

double degrees(double radians)
{
  return radians * 180.0 / M_PI;
}

bool parse_ms(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & name,
  std::chrono::milliseconds & value, const std::string & owner, const rclcpp::Logger & logger)
{
  const auto it = parameters.find(name);
  if (it == parameters.end()) {
    return true;
  }
  try {
    const long parsed = std::stol(it->second);
    if (parsed <= 0) {
      throw std::out_of_range(name);
    }
    value = std::chrono::milliseconds(parsed);
    return true;
  } catch (const std::exception &) {
    RCLCPP_FATAL(logger, "%s: parameter %s must be a positive integer, got '%s'",
      owner.c_str(), name.c_str(), it->second.c_str());
    return false;
  }
}

bool parse_double(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & name,
  double & value, const std::string & owner, const rclcpp::Logger & logger)
{
  const auto it = parameters.find(name);
  if (it == parameters.end()) {
    return true;
  }
  try {
    std::size_t used = 0;
    const double parsed = std::stod(it->second, &used);
    if (used != it->second.size() || !std::isfinite(parsed)) {
      throw std::invalid_argument(name);
    }
    value = parsed;
    return true;
  } catch (const std::exception &) {
    RCLCPP_FATAL(logger, "%s: parameter %s must be a number, got '%s'",
      owner.c_str(), name.c_str(), it->second.c_str());
    return false;
  }
}

bool parse_bool(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & name,
  bool & value, const std::string & owner, const rclcpp::Logger & logger)
{
  const auto it = parameters.find(name);
  if (it == parameters.end()) {
    return true;
  }
  std::string text = it->second;
  std::transform(text.begin(), text.end(), text.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  if (text == "true" || text == "1") {
    value = true;
    return true;
  }
  if (text == "false" || text == "0") {
    value = false;
    return true;
  }
  RCLCPP_FATAL(logger, "%s: parameter %s must be true or false, got '%s'",
    owner.c_str(), name.c_str(), it->second.c_str());
  return false;
}

// Position and velocity limits for a position-commanded joint: the <limit> of the joint
// in the URDF, narrowed by any limits set in its <ros2_control> tag (HardwareInfo::limits).
// The URDF is read directly because the bringup URDF disables the controller manager's
// joint limiter for these joints (see iot_robot_bringup/urdf/iot_robot.urdf.xacro),
// which also clears every limit in HardwareInfo::limits.
bool position_command_limits(
  const urdf::Model & model, const hardware_interface::HardwareInfo & info,
  const std::string & joint_name, joint_limits::JointLimits & limits,
  const rclcpp::Logger & logger)
{
  if (!joint_limits::getJointLimits(model.getJoint(joint_name), limits)) {
    RCLCPP_FATAL(
      logger, "Joint %s has a position command interface but no <limit> in the URDF.",
      joint_name.c_str());
    return false;
  }
  const auto it = info.limits.find(joint_name);
  if (it != info.limits.end()) {
    const auto & narrower = it->second;
    if (narrower.has_position_limits) {
      if (limits.has_position_limits) {
        limits.min_position = std::max(limits.min_position, narrower.min_position);
        limits.max_position = std::min(limits.max_position, narrower.max_position);
      } else {
        limits.min_position = narrower.min_position;
        limits.max_position = narrower.max_position;
        limits.has_position_limits = true;
      }
    }
    if (narrower.has_velocity_limits) {
      limits.max_velocity = std::min(limits.max_velocity, narrower.max_velocity);
    }
  }
  if (!limits.has_velocity_limits || !(limits.max_velocity > 0.0) ||
    !std::isfinite(limits.max_velocity))
  {
    RCLCPP_FATAL(
      logger, "Joint %s has a position command interface but no positive velocity limit; "
      "set velocity=\"...\" in its URDF <limit>.", joint_name.c_str());
    return false;
  }
  if (limits.has_position_limits && limits.min_position > limits.max_position) {
    RCLCPP_FATAL(
      logger, "Joint %s: lower position limit %f is above the upper limit %f.",
      joint_name.c_str(), limits.min_position, limits.max_position);
    return false;
  }
  return true;
}
}  // namespace

SafeCubeMarsSystemHardware::~SafeCubeMarsSystemHardware()
{
  // Last resort when the controller manager is torn down without deactivating
  if (!stopped_) {
    stop_motors("hardware component destroyed");
  }
  close_socket();
}

hardware_interface::CallbackReturn SafeCubeMarsSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // The upstream plugin checks can_interface and the per-joint can_id, kt, pole_pairs
  // and gear_ratio
  const auto result = CubeMarsSystemHardware::on_init(info);
  if (result != hardware_interface::CallbackReturn::SUCCESS) {
    return result;
  }

  interface_ = info_.hardware_parameters.at("can_interface");
  // Sized once: the exported interfaces point into these elements
  joints_.clear();
  joints_.resize(info_.joints.size());
  bool position_commanded = false;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    if (!parse_joint(info_.joints[i], joints_[i])) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    position_commanded = position_commanded || joints_[i].position_commanded;
    for (std::size_t j = 0; j < i; ++j) {
      if (joints_[j].can_id == joints_[i].can_id) {
        RCLCPP_FATAL(
          get_logger(), "Joints %s and %s both have CAN ID %u.", joints_[j].name.c_str(),
          joints_[i].name.c_str(), joints_[i].can_id);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  if (position_commanded) {
    urdf::Model model;
    if (!model.initString(info_.original_xml)) {
      RCLCPP_FATAL(
        get_logger(), "Could not parse the robot description to read the joint limits.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (auto & joint : joints_) {
      if (!joint.position_commanded) {
        continue;
      }
      joint_limits::JointLimits limits;
      if (!position_command_limits(model, info_, joint.name, limits, get_logger())) {
        return hardware_interface::CallbackReturn::ERROR;
      }
      joint.has_position_limits = limits.has_position_limits;
      if (joint.has_position_limits) {
        joint.min_position = limits.min_position;
        joint.max_position = limits.max_position;
      }
      joint.max_velocity = limits.max_velocity;
      RCLCPP_INFO(
        get_logger(), "%s: position commands clamped to [%.3f, %.3f] rad, at most %.3f rad/s",
        joint.name.c_str(), joint.min_position, joint.max_position, joint.max_velocity);
    }
  }

  if (!parse_ms(info_.hardware_parameters, "status_timeout_ms", status_timeout_, info_.name,
    get_logger()) ||
    !parse_ms(info_.hardware_parameters, "brake_time_ms", brake_time_, info_.name,
    get_logger()))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool SafeCubeMarsSystemHardware::parse_joint(
  const hardware_interface::ComponentInfo & info, Joint & joint)
{
  const auto & parameters = info.parameters;
  const auto & logger = get_logger();
  joint.name = info.name;
  double can_id = 0.0;
  double stall_time_ms = static_cast<double>(joint.stall_time.count());
  if (!parse_double(parameters, "can_id", can_id, joint.name, logger) ||
    !parse_double(parameters, "kt", joint.kt, joint.name, logger) ||
    !parse_double(parameters, "gear_ratio", joint.gear_ratio, joint.name, logger) ||
    !parse_double(parameters, "enc_off", joint.enc_off, joint.name, logger) ||
    !parse_double(parameters, "direction", joint.direction, joint.name, logger) ||
    !parse_bool(parameters, "zero_on_activate", joint.zero_on_activate, joint.name, logger) ||
    !parse_double(parameters, "stall_effort", joint.stall_effort, joint.name, logger) ||
    !parse_double(parameters, "stall_time_ms", stall_time_ms, joint.name, logger))
  {
    return false;
  }
  joint.can_id = static_cast<std::uint32_t>(can_id);
  if (joint.direction != 1.0 && joint.direction != -1.0) {
    RCLCPP_FATAL(logger, "%s: direction must be 1 or -1, got '%s'", joint.name.c_str(),
      parameters.at("direction").c_str());
    return false;
  }
  if (!(joint.gear_ratio > 0.0)) {
    RCLCPP_FATAL(
      logger, "%s: gear_ratio must be positive. To reverse a motor set direction to -1.",
      joint.name.c_str());
    return false;
  }
  if (joint.stall_effort < 0.0 || !(stall_time_ms > 0.0)) {
    RCLCPP_FATAL(
      logger, "%s: stall_effort must be 0 (off) or positive and stall_time_ms positive.",
      joint.name.c_str());
    return false;
  }
  joint.stall_time = std::chrono::milliseconds(static_cast<long>(stall_time_ms));

  for (const auto & interface : info.command_interfaces) {
    if (interface.name == hardware_interface::HW_IF_POSITION) {
      joint.position_commanded = true;
    } else if (interface.name != hardware_interface::HW_IF_VELOCITY &&
      interface.name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        logger, "%s: unsupported command interface '%s' (position, velocity or effort)",
        joint.name.c_str(), interface.name.c_str());
      return false;
    }
  }
  for (const auto & interface : info.state_interfaces) {
    if (interface.name != hardware_interface::HW_IF_POSITION &&
      interface.name != hardware_interface::HW_IF_VELOCITY &&
      interface.name != hardware_interface::HW_IF_EFFORT && interface.name != "temperature")
    {
      RCLCPP_FATAL(
        logger,
        "%s: unsupported state interface '%s' (position, velocity, effort or temperature)",
        joint.name.c_str(), interface.name.c_str());
      return false;
    }
  }
  return true;
}

std::vector<hardware_interface::StateInterface>
SafeCubeMarsSystemHardware::export_state_interfaces()
{
  // Keep the upstream handles: they are the only access to its private state vectors
  base_states_ = CubeMarsSystemHardware::export_state_interfaces();
  for (std::size_t k = 0; k < base_states_.size(); ++k) {
    const auto & handle = base_states_[k];
    for (auto & joint : joints_) {
      if (handle.get_prefix_name() != joint.name) {
        continue;
      }
      const auto & name = handle.get_interface_name();
      const int index = static_cast<int>(k);
      if (name == hardware_interface::HW_IF_POSITION) {
        joint.base_state.position = index;
      } else if (name == hardware_interface::HW_IF_VELOCITY) {
        joint.base_state.velocity = index;
      } else if (name == hardware_interface::HW_IF_EFFORT) {
        joint.base_state.effort = index;
      }
    }
  }

  // Export exactly the interfaces declared in the URDF, backed by joints_, which is
  // never resized after on_init
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    auto & joint = joints_[i];
    for (const auto & declared : info_.joints[i].state_interfaces) {
      double * value = &joint.state_temperature;
      if (declared.name == hardware_interface::HW_IF_POSITION) {
        value = &joint.state_position;
      } else if (declared.name == hardware_interface::HW_IF_VELOCITY) {
        value = &joint.state_velocity;
      } else if (declared.name == hardware_interface::HW_IF_EFFORT) {
        value = &joint.state_effort;
      }
      // The upstream plugin exports its interfaces the same (pointer-based) way
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      interfaces.emplace_back(joint.name, declared.name, value);
#pragma GCC diagnostic pop
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
SafeCubeMarsSystemHardware::export_command_interfaces()
{
  base_commands_ = CubeMarsSystemHardware::export_command_interfaces();
  for (std::size_t k = 0; k < base_commands_.size(); ++k) {
    const auto & handle = base_commands_[k];
    for (auto & joint : joints_) {
      if (handle.get_prefix_name() != joint.name) {
        continue;
      }
      const auto & name = handle.get_interface_name();
      const int index = static_cast<int>(k);
      if (name == hardware_interface::HW_IF_POSITION) {
        joint.base_command.position = index;
      } else if (name == hardware_interface::HW_IF_VELOCITY) {
        joint.base_command.velocity = index;
      } else if (name == hardware_interface::HW_IF_EFFORT) {
        joint.base_command.effort = index;
      }
    }
  }

  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    auto & joint = joints_[i];
    for (const auto & declared : info_.joints[i].command_interfaces) {
      double * value = &joint.command_effort;
      if (declared.name == hardware_interface::HW_IF_POSITION) {
        value = &joint.command_position;
      } else if (declared.name == hardware_interface::HW_IF_VELOCITY) {
        value = &joint.command_velocity;
      }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      interfaces.emplace_back(joint.name, declared.name, value);
#pragma GCC diagnostic pop
    }
  }
  return interfaces;
}

double SafeCubeMarsSystemHardware::base_state(int index) const
{
  if (index < 0) {
    return kNaN;
  }
  // Private handles, so the non-blocking read never finds the lock taken
  return base_states_[static_cast<std::size_t>(index)].get_optional().value_or(kNaN);
}

void SafeCubeMarsSystemHardware::set_base_command(int index, double value)
{
  if (index >= 0) {
    std::ignore = base_commands_[static_cast<std::size_t>(index)].set_value(value, true);
  }
}

hardware_interface::CallbackReturn SafeCubeMarsSystemHardware::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // Checked first because the upstream plugin reports "Communication active" even when
  // it could not bind the interface
  if (!open_socket()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto result = CubeMarsSystemHardware::on_configure(previous_state);
  if (result != hardware_interface::CallbackReturn::SUCCESS) {
    close_socket();
  }
  return result;
}

hardware_interface::CallbackReturn SafeCubeMarsSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  active_ = false;
  if (!stopped_) {
    stop_motors("cleanup");
  }
  close_socket();
  return CubeMarsSystemHardware::on_cleanup(previous_state);
}

hardware_interface::CallbackReturn SafeCubeMarsSystemHardware::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  const auto result = CubeMarsSystemHardware::on_activate(previous_state);
  if (result != hardware_interface::CallbackReturn::SUCCESS) {
    return result;
  }
  // The launch file's pre-flight check has usually confirmed this already; checking
  // here as well gives a clear message instead of a watchdog stop right after activation
  if (!wait_for_status(kActivationWait)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Commands left over from before a deactivation or an error must not be sent again
  for (auto & joint : joints_) {
    joint.command_position = kNaN;
    joint.command_velocity = kNaN;
    joint.command_effort = kNaN;
    joint.over_effort = false;
    joint.releasing = false;
  }
  stopped_ = false;
  if (!zero_joints()) {
    stop_motors("activation failed");
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Position targets start where the joints are
  for (auto & joint : joints_) {
    joint.target = joint.position_commanded ? measured_position(joint) : kNaN;
  }
  active_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SafeCubeMarsSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  active_ = false;
  stop_motors("deactivate");
  return CubeMarsSystemHardware::on_deactivate(previous_state);
}

hardware_interface::CallbackReturn SafeCubeMarsSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  active_ = false;
  if (!stopped_) {
    stop_motors("shutdown");
  }
  close_socket();
  return CubeMarsSystemHardware::on_shutdown(previous_state);
}

hardware_interface::CallbackReturn SafeCubeMarsSystemHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_ = false;
  if (!stopped_) {
    stop_motors("error");
  }
  RCLCPP_ERROR(
    get_logger(),
    "The motors of %s were stopped after an error. They stay stopped until the robot is "
    "restarted.", info_.name.c_str());
  // SUCCESS moves the component to unconfigured, so nothing is commanded any more
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SafeCubeMarsSystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  const auto result =
    CubeMarsSystemHardware::perform_command_mode_switch(start_interfaces, stop_interfaces);
  if (result != hardware_interface::return_type::OK) {
    return result;
  }
  const auto has_prefix = [](const std::vector<std::string> & names, const std::string & prefix)
    {
      return std::any_of(names.begin(), names.end(), [&prefix](const std::string & name) {
               return name.compare(0, prefix.size(), prefix) == 0;
             });
    };
  for (auto & joint : joints_) {
    const std::string prefix = joint.name + "/";
    const bool stopped = has_prefix(stop_interfaces, prefix);
    const bool started = has_prefix(start_interfaces, prefix);
    // The upstream plugin clears its commands of every stopped joint; so do we
    if (stopped) {
      joint.command_position = kNaN;
      joint.command_velocity = kNaN;
      joint.command_effort = kNaN;
      joint.position_claimed = false;
    }
    if (stopped && !started && active_) {
      // Its controller let go of the joint while the component stays active. The upstream
      // plugin sends nothing for it from now on, so the motor would keep its last speed or
      // current until its own CAN timeout. write() stops it without blocking the loop
      joint.releasing = true;
      joint.release_at = Clock::now() + brake_time_;
      RCLCPP_WARN(
        get_logger(), "No controller commands %s (CAN ID %u) any more: zero speed for %ld ms, "
        "then release", joint.name.c_str(), joint.can_id,
        static_cast<long>(brake_time_.count()));
    }
    if (started) {
      joint.releasing = false;
      const std::string position = prefix + hardware_interface::HW_IF_POSITION;
      joint.position_claimed = std::find(
        start_interfaces.begin(), start_interfaces.end(), position) != start_interfaces.end();
      if (joint.position_claimed) {
        // The new controller starts from where the joint is; NaN (no status yet) makes
        // write() retry
        joint.target = measured_position(joint);
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SafeCubeMarsSystemHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto result = CubeMarsSystemHardware::read(time, period);
  const bool healthy = motors_healthy();
  for (auto & joint : joints_) {
    joint.state_position = joint.direction * base_state(joint.base_state.position);
    joint.state_velocity = joint.direction * base_state(joint.base_state.velocity);
    joint.state_effort = joint.direction * base_state(joint.base_state.effort);
    joint.state_temperature = joint.has_status ? joint.temperature : kNaN;
  }
  // Stop here rather than rely on on_error: no motor of this component may keep driving
  if (!healthy) {
    active_ = false;
    stop_motors("motor fault or lost motor");
    return hardware_interface::return_type::ERROR;
  }
  if (active_ && !efforts_ok()) {
    active_ = false;
    stop_motors("stalled joint");
    return hardware_interface::return_type::ERROR;
  }
  return result;
}

hardware_interface::return_type SafeCubeMarsSystemHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  const double dt = std::clamp(period.seconds(), 0.0, kMaxSlewPeriod);
  const auto now = Clock::now();
  for (auto & joint : joints_) {
    if (joint.releasing) {
      continue_release(joint, now);
    }
    set_base_command(joint.base_command.velocity, joint.direction * joint.command_velocity);
    set_base_command(joint.base_command.effort, joint.direction * joint.command_effort);
    if (!joint.position_commanded) {
      continue;
    }
    // The upstream plugin only sends position frames while a controller holds the
    // position interface, and skips NaN
    double position = kNaN;
    if (joint.position_claimed) {
      if (std::isnan(joint.target)) {
        joint.target = measured_position(joint);
      }
      if (!std::isnan(joint.target)) {
        // A NaN command (no command from the controller yet) holds the current target
        if (!std::isnan(joint.command_position)) {
          double wanted = joint.command_position;
          if (joint.has_position_limits) {
            wanted = std::clamp(wanted, joint.min_position, joint.max_position);
          }
          const double step = joint.max_velocity * dt;
          joint.target += std::clamp(wanted - joint.target, -step, step);
        }
        position = joint.direction * joint.target;
      }
    }
    set_base_command(joint.base_command.position, position);
  }
  return CubeMarsSystemHardware::write(time, period);
}

double SafeCubeMarsSystemHardware::measured_position(const Joint & joint) const
{
  if (!joint.has_status) {
    return kNaN;
  }
  // Same conversion as the upstream plugin: output degrees, no gear ratio
  return joint.direction * (joint.raw_position * kDegPerCount * M_PI / 180.0 - joint.enc_off);
}

bool SafeCubeMarsSystemHardware::open_socket()
{
  close_socket();

  const unsigned int index = if_nametoindex(interface_.c_str());
  if (index == 0) {
    RCLCPP_FATAL(
      get_logger(),
      "CAN interface '%s' does not exist. Plug in the USB-CAN adapter and bring the "
      "link up (docs/hardware.md, 'CAN adapter'), or use mock:=true.",
      interface_.c_str());
    return false;
  }

  socket_ = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
  if (socket_ < 0) {
    RCLCPP_FATAL(get_logger(), "Could not create a CAN socket: %s", std::strerror(errno));
    return false;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_, SIOCGIFFLAGS, &ifr) < 0 || !(ifr.ifr_flags & IFF_UP)) {
    RCLCPP_FATAL(
      get_logger(),
      "CAN interface '%s' is down. Bring it up at 1 Mbit/s (pixi run -e robot can-up) "
      "or install deploy/ so it comes up at boot.",
      interface_.c_str());
    close_socket();
    return false;
  }

  // Only the status frames of our motors; the command frames the upstream plugin sends
  // are looped back to every local socket and are of no interest here
  std::vector<struct can_filter> filters;
  for (const auto & joint : joints_) {
    struct can_filter filter;
    filter.can_id = CAN_EFF_FLAG | (kStatus << 8) | joint.can_id;
    filter.can_mask = CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK;
    filters.push_back(filter);
  }
  if (setsockopt(
      socket_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
      static_cast<socklen_t>(filters.size() * sizeof(struct can_filter))) < 0)
  {
    RCLCPP_FATAL(get_logger(), "Could not set CAN filters: %s", std::strerror(errno));
    close_socket();
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = static_cast<int>(index);
  if (bind(socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_FATAL(
      get_logger(), "Could not bind CAN interface '%s': %s", interface_.c_str(),
      std::strerror(errno));
    close_socket();
    return false;
  }
  return true;
}

void SafeCubeMarsSystemHardware::close_socket()
{
  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
}

bool SafeCubeMarsSystemHardware::send(
  std::uint32_t id, const std::uint8_t * data, std::uint8_t length)
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = id | CAN_EFF_FLAG;
  frame.can_dlc = length;
  std::memcpy(frame.data, data, length);
  return ::write(socket_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

bool SafeCubeMarsSystemHardware::send_int32(std::uint32_t id, std::int32_t value)
{
  const auto raw = static_cast<std::uint32_t>(value);
  const std::uint8_t data[4] = {
    static_cast<std::uint8_t>(raw >> 24), static_cast<std::uint8_t>(raw >> 16),
    static_cast<std::uint8_t>(raw >> 8), static_cast<std::uint8_t>(raw)};
  return send(id, data, 4);
}

void SafeCubeMarsSystemHardware::continue_release(Joint & joint, Clock::time_point now)
{
  // The upstream plugin sends nothing for a joint without a controller, so these frames
  // are the only ones the motor gets
  if (now < joint.release_at) {
    send_int32((kModeSpeed << 8) | joint.can_id, 0);
    return;
  }
  joint.releasing = false;
  if (!send_int32((kModeCurrent << 8) | joint.can_id, 0)) {
    RCLCPP_ERROR(
      get_logger(), "Could not release %s (CAN ID %u) on '%s': %s", joint.name.c_str(),
      joint.can_id, interface_.c_str(), std::strerror(errno));
  }
}

void SafeCubeMarsSystemHardware::stop_motors(const char * reason)
{
  stopped_ = true;
  for (auto & joint : joints_) {
    joint.releasing = false;
  }
  if (socket_ < 0) {
    return;
  }
  RCLCPP_WARN(
    get_logger(), "Stopping the motors of %s (%s): zero speed for %ld ms, then release",
    info_.name.c_str(), reason, static_cast<long>(brake_time_.count()));

  int error = 0;
  const auto end = Clock::now() + brake_time_;
  while (true) {
    for (const auto & joint : joints_) {
      if (!send_int32((kModeSpeed << 8) | joint.can_id, 0) && error == 0) {
        error = errno;
      }
    }
    if (Clock::now() + kBrakePeriod > end) {
      break;
    }
    std::this_thread::sleep_for(kBrakePeriod);
  }
  // Zero current: the motors stop driving and turn freely
  for (const auto & joint : joints_) {
    if (!send_int32((kModeCurrent << 8) | joint.can_id, 0) && error == 0) {
      error = errno;
    }
  }
  if (error != 0) {
    RCLCPP_ERROR(
      get_logger(),
      "Could not send every stop frame on '%s' (%s). The motors now rely on their own "
      "CAN timeout (timeout_msec, docs/hardware.md).",
      interface_.c_str(), std::strerror(error));
  }
}

bool SafeCubeMarsSystemHardware::drain_status()
{
  if (socket_ < 0) {
    return true;
  }
  const auto now = Clock::now();
  // A fault code seen anywhere in this batch counts, even if a later frame clears it
  std::vector<bool> heard(joints_.size(), false);
  struct can_frame frame;
  while (true) {
    const ssize_t received = recv(socket_, &frame, sizeof(frame), MSG_DONTWAIT);
    if (received < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR || !active_) {
        return true;
      }
      RCLCPP_FATAL(
        get_logger(), "Reading CAN interface '%s' failed: %s. Stopping the motors of %s.",
        interface_.c_str(), std::strerror(errno), info_.name.c_str());
      return false;
    }
    if (received != static_cast<ssize_t>(sizeof(frame)) || frame.can_dlc < 8) {
      continue;
    }
    const std::uint32_t id = frame.can_id & 0xFFU;
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      auto & joint = joints_[i];
      if (joint.can_id != id) {
        continue;
      }
      joint.has_status = true;
      joint.last_status = now;
      joint.raw_position = static_cast<std::int16_t>((frame.data[0] << 8) | frame.data[1]);
      joint.raw_current = static_cast<std::int16_t>((frame.data[4] << 8) | frame.data[5]);
      joint.temperature = static_cast<std::int8_t>(frame.data[6]);
      if (frame.data[7] != 0 || !heard[i]) {
        joint.fault = frame.data[7];
      }
      heard[i] = true;
    }
  }
}

bool SafeCubeMarsSystemHardware::motors_healthy()
{
  if (socket_ < 0) {
    return !active_;
  }
  if (!drain_status()) {
    return false;
  }
  if (!active_) {
    return true;
  }
  const auto now = Clock::now();
  for (const auto & joint : joints_) {
    if (joint.fault != 0) {
      RCLCPP_FATAL(
        get_logger(), "Motor on %s (CAN ID %u) reports fault %u: %s. Stopping the motors of "
        "%s.", joint.name.c_str(), joint.can_id, joint.fault, fault_text(joint.fault),
        info_.name.c_str());
      return false;
    }
    const auto silent =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - joint.last_status);
    if (silent > status_timeout_) {
      RCLCPP_FATAL(
        get_logger(),
        "No status frames from the motor on %s (CAN ID %u) for %ld ms (limit %ld ms). "
        "Check its CAN cable and power, and that it sends status at 100-200 Hz. "
        "Stopping the motors of %s.",
        joint.name.c_str(), joint.can_id, static_cast<long>(silent.count()),
        static_cast<long>(status_timeout_.count()), info_.name.c_str());
      return false;
    }
  }
  return true;
}

bool SafeCubeMarsSystemHardware::efforts_ok()
{
  const auto now = Clock::now();
  for (auto & joint : joints_) {
    if (joint.stall_effort <= 0.0 || !joint.has_status) {
      continue;
    }
    // Same conversion as the upstream plugin: current [0.01 A] * kt * gear_ratio
    const double effort = std::abs(joint.raw_current * 0.01 * joint.kt * joint.gear_ratio);
    if (effort <= joint.stall_effort) {
      joint.over_effort = false;
      continue;
    }
    if (!joint.over_effort) {
      joint.over_effort = true;
      joint.over_effort_since = now;
      continue;
    }
    const auto held =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - joint.over_effort_since);
    if (held >= joint.stall_time) {
      RCLCPP_FATAL(
        get_logger(),
        "%s (CAN ID %u) has pushed with %.2f Nm (stall_effort %.2f Nm) for %ld ms: it is "
        "blocked or against a hard stop, for example because its zero was set in the wrong "
        "pose. Stopping the motors of %s.",
        joint.name.c_str(), joint.can_id, effort, joint.stall_effort,
        static_cast<long>(held.count()), info_.name.c_str());
      return false;
    }
  }
  return true;
}

bool SafeCubeMarsSystemHardware::wait_for_status(std::chrono::milliseconds timeout)
{
  const auto deadline = Clock::now() + timeout;
  const auto recent = [this](const Joint & joint) {
      return joint.has_status && Clock::now() - joint.last_status <= status_timeout_;
    };
  while (true) {
    if (!drain_status()) {
      return false;
    }
    if (std::all_of(joints_.begin(), joints_.end(), recent) || Clock::now() >= deadline) {
      break;
    }
    std::this_thread::sleep_for(kPollPeriod);
  }

  bool ok = true;
  for (const auto & joint : joints_) {
    if (!recent(joint)) {
      RCLCPP_FATAL(
        get_logger(),
        "Cannot activate %s: no status frames from the motor on %s (CAN ID %u) on '%s'. "
        "Check its power, CAN cable and ID (motors.yaml, cubemars_tool check).",
        info_.name.c_str(), joint.name.c_str(), joint.can_id, interface_.c_str());
      ok = false;
    } else if (joint.fault != 0) {
      RCLCPP_FATAL(
        get_logger(), "Cannot activate %s: the motor on %s (CAN ID %u) reports fault %u: %s.",
        info_.name.c_str(), joint.name.c_str(), joint.can_id, joint.fault,
        fault_text(joint.fault));
      ok = false;
    }
  }
  return ok;
}

bool SafeCubeMarsSystemHardware::zero_joints()
{
  std::vector<Clock::time_point> sent(joints_.size());
  bool any = false;
  if (!drain_status()) {
    return false;
  }
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & joint = joints_[i];
    if (!joint.zero_on_activate) {
      continue;
    }
    any = true;
    // Release first, so that the motor holds no position target from before, which the
    // new origin would turn into a jump
    const bool released = send_int32((kModeCurrent << 8) | joint.can_id, 0);
    const bool zeroed = send((kModeSetOrigin << 8) | joint.can_id, &kTemporaryOrigin, 1);
    if (!released || !zeroed) {
      RCLCPP_FATAL(
        get_logger(), "Could not send the set-origin command to %s (CAN ID %u) on '%s': %s",
        joint.name.c_str(), joint.can_id, interface_.c_str(), std::strerror(errno));
      return false;
    }
    sent[i] = Clock::now();
  }
  if (!any) {
    return true;
  }

  const auto deadline = Clock::now() + kActivationWait;
  const auto zeroed = [&](std::size_t i) {
      const auto & joint = joints_[i];
      return !joint.zero_on_activate || (joint.last_status > sent[i] &&
             std::abs(joint.raw_position * kDegPerCount) <= kZeroToleranceDeg);
    };
  while (true) {
    if (!drain_status()) {
      return false;
    }
    bool done = true;
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      done = done && zeroed(i);
    }
    if (done) {
      break;
    }
    if (Clock::now() >= deadline) {
      for (std::size_t i = 0; i < joints_.size(); ++i) {
        const auto & joint = joints_[i];
        if (zeroed(i)) {
          continue;
        }
        if (joint.last_status <= sent[i]) {
          RCLCPP_FATAL(
            get_logger(), "Could not zero %s (CAN ID %u): no status frames after the "
            "set-origin command.", joint.name.c_str(), joint.can_id);
        } else {
          RCLCPP_FATAL(
            get_logger(),
            "Could not zero %s (CAN ID %u): it reads %.1f deg %ld ms after the set-origin "
            "command (servo mode 5), expected 0 +- %.1f deg. Check that the motor runs "
            "servo mode firmware and that nothing else commands it.",
            joint.name.c_str(), joint.can_id, joint.raw_position * kDegPerCount,
            static_cast<long>(kActivationWait.count()), kZeroToleranceDeg);
        }
      }
      return false;
    }
    std::this_thread::sleep_for(kPollPeriod);
  }
  for (const auto & joint : joints_) {
    if (joint.zero_on_activate) {
      RCLCPP_INFO(
        get_logger(), "%s (CAN ID %u): temporary origin set, its current pose is now "
        "position 0 (reads %.1f deg)", joint.name.c_str(), joint.can_id,
        degrees(measured_position(joint)));
    }
  }
  return true;
}

}  // namespace cubemars_hardware_safe

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cubemars_hardware_safe::SafeCubeMarsSystemHardware, hardware_interface::SystemInterface)
