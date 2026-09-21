#!/usr/bin/env python3
"""Turn the simulated ultrasonic ray fans into the sensor_msgs/Range the real robot publishes."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range


class ScanToRange(Node):
    def __init__(self):
        super().__init__("scan_to_range")

        sensors = self.declare_parameter("sensors", ["left", "right"]).value
        # HC-SR04 figures, the same ones stm32_bridge reports on the real robot
        self.field_of_view = self.declare_parameter(
            "field_of_view", 0.26).value
        self.min_range = self.declare_parameter("min_range", 0.02).value
        self.max_range = self.declare_parameter("max_range", 4.0).value

        for name in sensors:
            publisher = self.create_publisher(
                Range, f"/ultrasonic/{name}", 10)
            self.create_subscription(
                LaserScan, f"/sim/ultrasonic/{name}/scan",
                lambda msg, publisher=publisher: self.on_scan(msg, publisher),
                qos_profile_sensor_data)

    def on_scan(self, scan, publisher):
        # MuJoCo reports -1 for a ray that hits nothing. Past max_range the real sensor
        # hears no echo either
        hits = [r for r in scan.ranges
                if math.isfinite(r) and 0.0 <= r <= self.max_range]

        msg = Range()
        msg.header = scan.header
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.field_of_view
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        # The first echo comes back from the nearest object anywhere in the cone.
        # No echo is +inf (REP 117)
        msg.range = max(min(hits), self.min_range) if hits else math.inf
        publisher.publish(msg)


def main():
    rclpy.init()
    node = ScanToRange()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
