"""Publish the LSM9DS1 IMU on the Raspberry Pi's I2C bus.

    ros2 run iot_robot_drivers lsm9ds1_node --ros-args --params-file imu.yaml

Publishes imu/data_raw (sensor_msgs/Imu: angular rate and acceleration, no orientation)
and imu/mag (sensor_msgs/MagneticField, tesla), both in the frame imu_link.
imu_filter_madgwick turns imu/data_raw into imu/data with an orientation.

A missing or failing device does not stop the node: it logs why and retries. The sensor
is also reinitialised when it stops producing new samples, e.g. after a supply dropout
reset its configuration.
"""

import math
import time

import rclpy
from rclpy.exceptions import ParameterException
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

from iot_robot_drivers.lsm9ds1 import (
    DEFAULT_AG_ADDRESS, DEFAULT_MAG_ADDRESS, GyroBiasEstimator, Lsm9ds1, Lsm9ds1Error)

# A repeated failure is logged again after this long; a different one immediately
ERROR_LOG_PERIOD = 30.0


def open_smbus(bus):
    # Imported here so that the module, and its tests, load without smbus2
    from smbus2 import SMBus
    return SMBus(bus)


def diagonal(stddev):
    # Zero means unknown, which sensor_msgs expresses as an all-zero covariance
    variance = stddev * stddev
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


class Lsm9ds1Node(Node):
    def __init__(self, bus_factory=open_smbus, **kwargs):
        # kwargs lets tests pass a context and parameter overrides
        super().__init__("lsm9ds1", **kwargs)

        self.bus_number = self.declare_parameter("bus", 1).value
        ag_address = self.declare_parameter("ag_address", DEFAULT_AG_ADDRESS).value
        mag_address = self.declare_parameter("mag_address", DEFAULT_MAG_ADDRESS).value
        rate = self.declare_parameter("rate", 100.0).value
        # Output data rate of the accelerometer and gyroscope, Hz
        odr = self.declare_parameter("odr", 119.0).value
        accel_range = self.declare_parameter("accel_range", 4).value
        gyro_range = self.declare_parameter("gyro_range", 500).value
        mag_range = self.declare_parameter("mag_range", 4).value
        self.frame_id = self.declare_parameter("frame_id", "imu_link").value
        # Noise estimates for the message covariances, not datasheet values; measure them
        # as the standard deviation of each axis with the robot standing still
        accel_stddev = self.declare_parameter("linear_acceleration_stddev", 0.03).value
        gyro_stddev = self.declare_parameter("angular_velocity_stddev", 0.005).value
        mag_stddev = self.declare_parameter("magnetic_field_stddev", 0.0).value
        # Gyroscope bias from the first samples while the robot stands still; 0 disables
        bias_samples = self.declare_parameter("gyro_bias_samples", 100).value
        bias_tolerance = self.declare_parameter("gyro_bias_tolerance", 0.05).value
        self.reconnect_period = self.declare_parameter("reconnect_period", 1.0).value
        self.stale_timeout = self.declare_parameter("stale_timeout", 0.5).value

        self.bus_factory = bus_factory
        # Validates the choices before anything opens the bus; raises ValueError
        self.settings = {
            "ag_address": ag_address, "mag_address": mag_address,
            "accel_range": accel_range, "gyro_range": gyro_range, "mag_range": mag_range,
            "odr": odr}
        Lsm9ds1(None, **self.settings)
        if rate <= 0.0:
            raise ValueError(f"rate must be positive, got {rate:g}")

        self.accel_covariance = diagonal(accel_stddev)
        self.gyro_covariance = diagonal(gyro_stddev)
        self.mag_covariance = diagonal(mag_stddev)
        if bias_samples > 0:
            self.bias = None
            self.bias_estimator = GyroBiasEstimator(bias_samples, bias_tolerance)
            self.bias_duration = bias_samples / min(rate, odr)
        else:
            self.bias = (0.0, 0.0, 0.0)

        self.bus = None
        self.sensor = None
        self.next_attempt = 0.0
        self.last_error = None
        self.last_error_time = 0.0
        self.last_ag = self.last_mag = 0.0

        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self.mag_pub = self.create_publisher(MagneticField, "imu/mag", 10)
        self.create_timer(1.0 / rate, self.poll)

    # Connection ---------------------------------------------------------------------

    def connect(self):
        try:
            self.bus = self.bus_factory(self.bus_number)
            sensor = Lsm9ds1(self.bus, **self.settings)
            sensor.begin()
        except (OSError, Lsm9ds1Error) as err:
            self.fail(self.explain(err))
            return
        self.sensor = sensor
        self.last_ag = self.last_mag = time.monotonic()
        self.last_error = None
        s = self.settings
        self.get_logger().info(
            f"LSM9DS1 on /dev/i2c-{self.bus_number} at 0x{s['ag_address']:02X} and "
            f"0x{s['mag_address']:02X}: +-{s['accel_range']} g, +-{s['gyro_range']} dps, "
            f"+-{s['mag_range']} gauss, {s['odr']:g} Hz")
        if self.bias is None and self.bias_estimator.rejected == 0:
            self.get_logger().info(
                f"Estimating the gyroscope bias: keep the robot still for "
                f"{self.bias_duration:.1f} s")

    def explain(self, err):
        device = f"/dev/i2c-{self.bus_number}"
        if isinstance(err, FileNotFoundError):
            return (f"{device} does not exist. Enable I2C (sudo raspi-config nonint "
                    "do_i2c 0, which deploy/install.sh runs) and reboot")
        if isinstance(err, PermissionError):
            return (f"No permission to open {device}. Add the user to the i2c group "
                    "(deploy/install.sh does) and log in again")
        if isinstance(err, Lsm9ds1Error):
            return (f"{err}. Check the wiring and power, and list the devices on the bus "
                    f"with: i2cdetect -y {self.bus_number}")
        return f"I2C error on {device}: {err}"

    def close_bus(self):
        self.sensor = None
        if self.bus is not None:
            try:
                self.bus.close()
            except OSError:
                pass
            self.bus = None

    def fail(self, message):
        """Close the bus, log the reason and try again after reconnect_period."""
        self.close_bus()
        now = time.monotonic()
        self.next_attempt = now + self.reconnect_period
        # Log once per outage, plus a reminder while it lasts
        if message != self.last_error or now - self.last_error_time >= ERROR_LOG_PERIOD:
            self.get_logger().warn(
                f"{message}. Retrying every {self.reconnect_period:g} s")
            self.last_error = message
            self.last_error_time = now

    # Timer --------------------------------------------------------------------------

    def poll(self):
        if self.sensor is None:
            if time.monotonic() >= self.next_attempt:
                self.connect()
            return
        try:
            sample = self.sensor.read_accel_gyro()
            field = self.sensor.read_mag()
        except OSError as err:
            self.fail(f"Lost the LSM9DS1: {self.explain(err)}")
            return

        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        if sample is not None:
            self.last_ag = now
            self.handle_sample(stamp, *sample)
        if field is not None:
            self.last_mag = now
            self.mag_pub.publish(self.mag_message(stamp, field))
        stale = now - min(self.last_ag, self.last_mag)
        if stale > self.stale_timeout:
            self.fail(f"No new LSM9DS1 samples for {stale:.1f} s, reinitialising it")

    def handle_sample(self, stamp, accel, gyro):
        if self.bias is None:
            bias = self.bias_estimator.add(gyro)
            if bias is None:
                if self.bias_estimator.rejected and not self.bias_estimator.window:
                    self.get_logger().warn(
                        "The robot moved while the gyroscope bias was measured. Trying "
                        "again; imu/data_raw starts once it stands still",
                        throttle_duration_sec=10.0)
                return
            self.bias = bias
            self.get_logger().info(
                "Gyroscope bias [" + ", ".join(f"{math.degrees(b):.2f}" for b in bias)
                + "] deg/s")
        gyro = tuple(value - offset for value, offset in zip(gyro, self.bias))
        self.imu_pub.publish(self.imu_message(stamp, accel, gyro))

    def imu_message(self, stamp, accel, gyro):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        # sensor_msgs/Imu: -1 in the first element means there is no orientation estimate
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.angular_velocity_covariance = self.gyro_covariance
        (msg.linear_acceleration.x, msg.linear_acceleration.y,
         msg.linear_acceleration.z) = accel
        msg.linear_acceleration_covariance = self.accel_covariance
        return msg

    def mag_message(self, stamp, field):
        msg = MagneticField()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z = field
        msg.magnetic_field_covariance = self.mag_covariance
        return msg

    def destroy_node(self):
        self.close_bus()
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = Lsm9ds1Node()
    except (ValueError, ParameterException) as err:
        # A parameter of the wrong type, e.g. "rate: 100" for a double, also lands here
        get_logger("lsm9ds1").fatal(str(err))
        rclpy.try_shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
