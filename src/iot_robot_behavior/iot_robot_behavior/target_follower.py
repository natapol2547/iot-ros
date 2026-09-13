"""Point the camera gizmo at a target and drive the base to follow it."""

import math

import rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value, low, high):
    return max(low, min(high, value))


class TargetFollower(Node):
    def __init__(self):
        super().__init__("target_follower")

        self.follow_distance = self.declare_parameter(
            "follow_distance", 0.6).value
        self.linear_gain = self.declare_parameter("linear_gain", 1.0).value
        self.angular_gain = self.declare_parameter("angular_gain", 2.5).value
        self.max_linear = self.declare_parameter("max_linear", 0.5).value
        self.max_angular = self.declare_parameter("max_angular", 2.0).value
        self.lost_timeout = self.declare_parameter("lost_timeout", 0.5).value
        self.search_angular = self.declare_parameter(
            "search_angular", 0.8).value
        self.yaw_limit = self.declare_parameter("yaw_limit", 0.785398).value
        self.pitch_min = self.declare_parameter("pitch_min", -0.785398).value
        self.pitch_max = self.declare_parameter("pitch_max", 0.0).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target = None  # latest target position in base_link
        self.last_seen = None  # node time of the latest detection
        self.last_bearing = 0.0

        self.create_subscription(PointStamped, "target", self.on_target, 10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.gizmo_pub = self.create_publisher(
            Float64MultiArray, "gizmo_commands", 10)
        self.create_timer(0.05, self.control_loop)

    def on_target(self, msg):
        try:
            # Time() means "latest available". The image is usually newer than the
            # latest TF (robot_state_publisher publishes at 20 Hz), so asking for the
            # exact image stamp would fail with an extrapolation error
            camera_to_base = self.tf_buffer.lookup_transform(
                "base_link", msg.header.frame_id, Time())
            pitch_joint = self.tf_buffer.lookup_transform(
                "base_link", "gizmo_pitch_link", Time())
        except TransformException as error:
            self.get_logger().warn(
                f"TF lookup failed: {error}", throttle_duration_sec=2.0)
            return

        self.target = do_transform_point(msg, camera_to_base).point
        self.last_seen = self.get_clock().now()
        self.last_bearing = math.atan2(self.target.y, self.target.x)

        # Aim the gizmo. The yaw axis passes through the base_link origin and the camera
        # sits (almost) on it, so the yaw angle is simply the bearing to the target
        yaw = clamp(self.last_bearing, -self.yaw_limit, self.yaw_limit)
        # The camera lies on the pitch link's x axis, so tilt that axis from the pitch
        # joint towards the target. Positive pitch tilts down, so looking up is negative
        origin = pitch_joint.transform.translation
        height = self.target.z - origin.z
        horizontal = math.hypot(self.target.x - origin.x,
                                self.target.y - origin.y)
        pitch = clamp(-math.atan2(height, horizontal),
                      self.pitch_min, self.pitch_max)
        self.gizmo_pub.publish(Float64MultiArray(data=[yaw, pitch]))

    def control_loop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        visible = (self.last_seen is not None
                   and self.get_clock().now() - self.last_seen < Duration(seconds=self.lost_timeout))
        if visible:
            distance = math.hypot(self.target.x, self.target.y)
            bearing = math.atan2(self.target.y, self.target.x)
            cmd.twist.angular.z = clamp(
                self.angular_gain * bearing, -self.max_angular, self.max_angular)
            # Only drive forward once roughly facing the target
            heading_scale = max(0.0, math.cos(bearing))
            cmd.twist.linear.x = heading_scale * clamp(
                self.linear_gain * (distance - self.follow_distance), -self.max_linear, self.max_linear)
        else:
            # Target lost: recentre the gizmo and turn towards where it was last seen
            self.gizmo_pub.publish(Float64MultiArray(data=[0.0, 0.0]))
            cmd.twist.angular.z = math.copysign(
                self.search_angular, self.last_bearing)

        self.cmd_vel_pub.publish(cmd)


def main():
    rclpy.init()
    node = TargetFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
