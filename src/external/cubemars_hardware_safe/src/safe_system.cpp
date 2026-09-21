#include "cubemars_hardware_safe/safe_system.hpp"

#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace cubemars_hardware_safe
{
namespace
{
// Servo-mode CAN protocol: command ID (mode << 8) | motor ID, status ID (0x29 << 8) | ID
constexpr std::uint32_t kModeCurrent = 1;
constexpr std::uint32_t kModeSpeed = 3;
constexpr std::uint32_t kStatus = 0x29;
constexpr std::chrono::milliseconds kBrakePeriod{20};

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

bool parse_ms(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  std::chrono::milliseconds & value, const rclcpp::Logger & logger)
{
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) {
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
    RCLCPP_FATAL(logger, "Hardware parameter %s must be a positive integer, got '%s'",
      name.c_str(), it->second.c_str());
    return false;
  }
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
  // The upstream plugin checks can_interface and the per-joint can_id
  const auto result = CubeMarsSystemHardware::on_init(info);
  if (result != hardware_interface::CallbackReturn::SUCCESS) {
    return result;
  }

  interface_ = info_.hardware_parameters.at("can_interface");
  motor_ids_.clear();
  joint_names_.clear();
  for (const auto & joint : info_.joints) {
    motor_ids_.push_back(static_cast<std::uint32_t>(std::stoul(joint.parameters.at("can_id"))));
    joint_names_.push_back(joint.name);
  }
  last_status_.assign(motor_ids_.size(), Clock::now());

  if (!parse_ms(info_, "status_timeout_ms", status_timeout_, get_logger()) ||
    !parse_ms(info_, "brake_time_ms", brake_time_, get_logger()))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
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
  if (result == hardware_interface::CallbackReturn::SUCCESS) {
    // The silence timer starts now; the launch file's pre-flight check has already
    // confirmed that both motors send status frames
    last_status_.assign(motor_ids_.size(), Clock::now());
    stopped_ = false;
    active_ = true;
  }
  return result;
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
    "Wheel motors stopped after an error. They stay stopped until the robot is restarted.");
  // SUCCESS moves the component to unconfigured, so nothing is commanded any more
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SafeCubeMarsSystemHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto result = CubeMarsSystemHardware::read(time, period);
  if (!motors_healthy()) {
    // Stop here rather than rely on on_error: the other wheel must not keep driving
    active_ = false;
    stop_motors("motor fault or lost motor");
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
  return CubeMarsSystemHardware::write(time, period);
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
  for (const auto id : motor_ids_) {
    struct can_filter filter;
    filter.can_id = CAN_EFF_FLAG | (kStatus << 8) | id;
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

bool SafeCubeMarsSystemHardware::send(std::uint32_t id, std::int32_t value)
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = id | CAN_EFF_FLAG;
  frame.can_dlc = 4;
  const auto raw = static_cast<std::uint32_t>(value);
  frame.data[0] = static_cast<std::uint8_t>(raw >> 24);
  frame.data[1] = static_cast<std::uint8_t>(raw >> 16);
  frame.data[2] = static_cast<std::uint8_t>(raw >> 8);
  frame.data[3] = static_cast<std::uint8_t>(raw);
  return ::write(socket_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

void SafeCubeMarsSystemHardware::stop_motors(const char * reason)
{
  stopped_ = true;
  if (socket_ < 0) {
    return;
  }
  RCLCPP_WARN(
    get_logger(), "Stopping the wheel motors (%s): zero speed for %ld ms, then release",
    reason, static_cast<long>(brake_time_.count()));

  int error = 0;
  const auto end = Clock::now() + brake_time_;
  while (true) {
    for (const auto id : motor_ids_) {
      if (!send((kModeSpeed << 8) | id, 0) && error == 0) {
        error = errno;
      }
    }
    if (Clock::now() + kBrakePeriod > end) {
      break;
    }
    std::this_thread::sleep_for(kBrakePeriod);
  }
  // Zero current: the motors stop driving and the wheels turn freely
  for (const auto id : motor_ids_) {
    if (!send((kModeCurrent << 8) | id, 0) && error == 0) {
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

bool SafeCubeMarsSystemHardware::motors_healthy()
{
  if (socket_ < 0) {
    return !active_;
  }
  const auto now = Clock::now();
  struct can_frame frame;
  while (true) {
    const ssize_t received = recv(socket_, &frame, sizeof(frame), MSG_DONTWAIT);
    if (received < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR || !active_) {
        break;
      }
      RCLCPP_FATAL(
        get_logger(), "Reading CAN interface '%s' failed: %s. Stopping both wheels.",
        interface_.c_str(), std::strerror(errno));
      return false;
    }
    if (received != static_cast<ssize_t>(sizeof(frame)) || frame.can_dlc < 8) {
      continue;
    }
    const std::uint32_t id = frame.can_id & 0xFFU;
    const auto it = std::find(motor_ids_.begin(), motor_ids_.end(), id);
    if (it == motor_ids_.end()) {
      continue;
    }
    const auto i = static_cast<std::size_t>(std::distance(motor_ids_.begin(), it));
    last_status_[i] = now;
    const std::uint8_t fault = frame.data[7];
    if (fault != 0 && active_) {
      RCLCPP_FATAL(
        get_logger(), "Motor on %s (CAN ID %u) reports fault %u: %s. Stopping both wheels.",
        joint_names_[i].c_str(), id, fault, fault_text(fault));
      return false;
    }
  }

  if (!active_) {
    return true;
  }
  for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
    const auto silent =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status_[i]);
    if (silent > status_timeout_) {
      RCLCPP_FATAL(
        get_logger(),
        "No status frames from the motor on %s (CAN ID %u) for %ld ms (limit %ld ms). "
        "Check its CAN cable and power, and that it sends status at 100-200 Hz. "
        "Stopping both wheels.",
        joint_names_[i].c_str(), motor_ids_[i], static_cast<long>(silent.count()),
        static_cast<long>(status_timeout_.count()));
      return false;
    }
  }
  return true;
}

}  // namespace cubemars_hardware_safe

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cubemars_hardware_safe::SafeCubeMarsSystemHardware, hardware_interface::SystemInterface)
