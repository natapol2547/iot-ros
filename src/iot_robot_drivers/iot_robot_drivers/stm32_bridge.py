"""Bridge the Nucleo-F401RE sensor board to ROS: ultrasonics, battery voltage, gizmo joints.

Replaces the first ultrasonic_node.py. Distances are published as sensor_msgs/Range in
metres, the serial port is read on its own thread instead of polled from a timer, and the
node reconnects when the board is unplugged or reset. The line format is described in
stm32_protocol. The bridge only reads: the board drives no motors, and the Raspberry Pi
controls all four motors over CAN.

gizmo_mode says who owns the gizmo joints:
    can    the gizmo motors are in ros2_control, whose joint_state_broadcaster publishes
           their joint states; the bridge leaves the gizmo alone
    fixed  no gizmo drive; the bridge publishes constant joint states (fixed_yaw,
           fixed_pitch) so that robot_state_publisher can place the camera
"""

import math
import threading
import time

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, JointState, Range
from std_msgs.msg import Float64MultiArray

from iot_robot_drivers import stm32_protocol as protocol

GIZMO_JOINTS = ["gizmo_yaw_joint", "gizmo_pitch_joint"]
GIZMO_MODES = ("can", "fixed")
# A line is at most ~40 bytes; anything much longer is noise, e.g. a wrong baud rate
MAX_LINE_BYTES = 256


class ConfigError(Exception):
    pass


class Stm32Bridge(Node):
    def __init__(self, **kwargs):
        # kwargs lets tests pass a context and parameter overrides
        super().__init__("stm32_bridge", **kwargs)

        self.port = self.declare_parameter("port", "/dev/stm32").value
        self.baudrate = self.declare_parameter("baudrate", 115200).value
        self.reconnect_period = self.declare_parameter(
            "reconnect_period", 1.0).value
        # The board sends ~15 lines/s, so a second of silence means it is stuck or
        # running other firmware
        self.stale_timeout = self.declare_parameter("stale_timeout", 1.0).value

        # HC-SR04 figures, the same ones the simulated sensors report
        self.field_of_view = self.declare_parameter(
            "field_of_view", 0.26).value
        self.min_range = self.declare_parameter("min_range", 0.02).value
        self.max_range = self.declare_parameter("max_range", 4.0).value
        self.frames = {
            "left": self.declare_parameter(
                "left_frame", "ultrasonic_left_link").value,
            "right": self.declare_parameter(
                "right_frame", "ultrasonic_right_link").value,
        }

        self.gizmo_mode = self.declare_parameter("gizmo_mode", "fixed").value
        if self.gizmo_mode not in GIZMO_MODES:
            raise ConfigError(
                "gizmo_mode must be can (the gizmo motors are driven over CAN by "
                "ros2_control) or fixed (the gizmo is not driven), "
                f"got '{self.gizmo_mode}'")
        # Joint limits from iot_robot_description; negative pitch looks up
        self.yaw_limits = self.declare_parameter(
            "yaw_limits", [-0.785398, 0.785398]).value
        self.pitch_limits = self.declare_parameter(
            "pitch_limits", [-0.785398, 0.0]).value
        # How the gizmo sits in fixed mode
        fixed_yaw = self.declare_parameter("fixed_yaw", 0.0).value
        fixed_pitch = self.declare_parameter("fixed_pitch", 0.0).value
        joint_state_rate = self.declare_parameter(
            "joint_state_rate", 20.0).value

        self.gizmo = self.clamp_gizmo(fixed_yaw, fixed_pitch)
        self.ignored_command_logged = False

        # The reader thread owns opening and closing the port; destroy_node closes it
        # under the lock if the reader thread has not finished
        self.serial_lock = threading.Lock()
        self.serial = None
        self.outage_logged = False
        self.stopping = threading.Event()
        # Sides the firmware reported as faulty; only the reader thread uses this
        self.faulty = dict.fromkeys(self.frames, False)

        self.range_pubs = {
            side: self.create_publisher(Range, f"/ultrasonic/{side}", 10)
            for side in self.frames
        }
        self.battery_pub = self.create_publisher(
            BatteryState, "/battery_state", 10)
        # In can mode joint_state_broadcaster publishes the gizmo joints; a second
        # publisher of the same joints would make them jump between two values
        if self.gizmo_mode == "fixed":
            self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
            # Only to say once why pan/tilt commands (e.g. from target_follower) do
            # not move the gizmo
            self.create_subscription(
                Float64MultiArray, "/gizmo_controller/commands", self.on_gizmo_command, 10)
            self.create_timer(1.0 / joint_state_rate, self.publish_joint_states)

        self.get_logger().info(
            f"STM32 bridge on {self.port} at {self.baudrate} baud, "
            f"gizmo_mode {self.gizmo_mode}")
        self.reader = threading.Thread(target=self.run, daemon=True)
        self.reader.start()

    # Serial thread ------------------------------------------------------------------

    def running(self):
        # SIGINT shuts the context down before main() reaches destroy_node(), and
        # rclpy runs no on_shutdown callbacks in that case, so check both
        return not self.stopping.is_set() and self.context.ok()

    def run(self):
        while self.running():
            connection = self.connect()
            if connection is None:
                self.stopping.wait(self.reconnect_period)
                continue
            self.read_lines(connection)
            with self.serial_lock:
                self.serial = None
            connection.close()

    def connect(self):
        try:
            # exclusive stops a second bridge from reading half the lines
            connection = serial.Serial(
                self.port, self.baudrate, timeout=0.2, exclusive=True)
        except (serial.SerialException, OSError, ValueError) as err:
            # Log once per outage, not once per retry
            if not self.outage_logged:
                self.get_logger().warn(
                    f"Cannot open {self.port}: {err}. Retrying every "
                    f"{self.reconnect_period:g} s")
                self.outage_logged = True
            return None

        # Drop whatever queued up while nobody was reading; the first line is
        # probably cut anyway
        connection.reset_input_buffer()
        with self.serial_lock:
            self.serial = connection
        # The board may have reset while unplugged, which clears its fault state, and
        # its banner was probably dropped above. It repeats a lasting fault within ~10 s
        self.faulty = dict.fromkeys(self.frames, False)
        self.outage_logged = False
        self.get_logger().info(f"Connected to STM32 on {self.port}")
        return connection

    def read_lines(self, connection):
        pending = b""
        last_line = time.monotonic()
        stale_logged = False
        while self.running():
            try:
                # Returns early with a partial line when the timeout expires
                chunk = connection.readline()
            except (serial.SerialException, OSError) as err:
                # During shutdown the context is gone and logging would only error
                if self.context.ok():
                    self.get_logger().warn(f"Lost STM32 on {self.port}: {err}")
                self.outage_logged = True
                return

            pending += chunk
            now = time.monotonic()
            if not pending.endswith(b"\n"):
                if len(pending) > MAX_LINE_BYTES:
                    pending = b""
                if not stale_logged and now - last_line > self.stale_timeout:
                    self.get_logger().warn(
                        f"No data from STM32 on {self.port} for "
                        f"{self.stale_timeout:g} s. Check the firmware and the "
                        f"baud rate ({self.baudrate})")
                    stale_logged = True
                continue

            line = pending.decode("ascii", errors="replace")
            pending = b""
            last_line = now
            if stale_logged:
                self.get_logger().info("STM32 data resumed")
                stale_logged = False
            if not self.running():
                return
            try:
                self.handle_line(line)
            except Exception:
                # The context can still shut down between the check above and a
                # publish, which then raises. Anything else is a real error
                if self.context.ok():
                    raise
                return

    def handle_line(self, line):
        reading = protocol.parse_line(line)
        if reading is None:
            text = line.strip()
            if protocol.is_comment(text):
                self.handle_comment(text)
            elif text:
                self.get_logger().warn(
                    f"Ignoring malformed line from STM32: {text!r}",
                    throttle_duration_sec=5.0)
            return

        stamp = self.get_clock().now().to_msg()
        for side, distance in (("left", reading.left_cm), ("right", reading.right_cm)):
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frames[side]
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = self.field_of_view
            msg.min_range = self.min_range
            msg.max_range = self.max_range
            msg.range = protocol.distance_to_range(
                distance, self.min_range, self.max_range, self.faulty[side])
            self.range_pubs[side].publish(msg)

        if reading.volts is not None:
            self.battery_pub.publish(self.battery_state(stamp, reading.volts))

    def handle_comment(self, text):
        message = text.lstrip("#").strip()
        status = protocol.parse_sensor_status(text)
        if status is not None:
            self.faulty[status.side] = status.faulty
            if status.faulty:
                # The firmware repeats this about every 10 s while the fault lasts
                self.get_logger().warn(
                    f"STM32: {message}. Publishing NaN on /ultrasonic/{status.side} "
                    "until the sensor recovers")
            else:
                self.get_logger().info(f"STM32: {message}")
            return
        if protocol.is_banner(text) and any(self.faulty.values()):
            # A reset clears the firmware's fault state; it warns again if the fault lasts
            self.faulty = dict.fromkeys(self.frames, False)
            self.get_logger().info("STM32 restarted; sensor faults cleared")
        self.get_logger().info(f"STM32: {message}")

    @staticmethod
    def battery_state(stamp, volts):
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.voltage = volts
        # Only the pack voltage is measured. BatteryState marks unmeasured fields NaN
        msg.temperature = math.nan
        msg.current = math.nan
        msg.charge = math.nan
        msg.capacity = math.nan
        msg.design_capacity = math.nan
        msg.percentage = math.nan
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = True
        return msg

    # ROS thread ---------------------------------------------------------------------

    def clamp_gizmo(self, yaw, pitch):
        return (protocol.clamp(yaw, *self.yaw_limits),
                protocol.clamp(pitch, *self.pitch_limits))

    def on_gizmo_command(self, msg):
        if not self.ignored_command_logged:
            self.get_logger().info(
                "Ignoring /gizmo_controller/commands: gizmo_mode is fixed")
            self.ignored_command_logged = True

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = GIZMO_JOINTS
        msg.position = list(self.gizmo)
        self.joint_pub.publish(msg)

    def destroy_node(self):
        self.stopping.set()
        self.reader.join(timeout=2.0)
        with self.serial_lock:
            if self.serial is not None:
                self.serial.close()
                self.serial = None
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = Stm32Bridge()
    except ConfigError as err:
        get_logger("stm32_bridge").fatal(str(err))
        rclpy.try_shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # Ctrl-C in a terminal reaches the node twice, from the terminal and from
            # ros2 launch; the process is exiting either way and the OS closes the port
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
