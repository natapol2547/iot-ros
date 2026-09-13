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
from tf2_ros import Buffer, ExtrapolationException, TransformException, TransformListener


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
        # Gizmo pitch while searching. A ball lies on the floor (0.0), but a person's
        # torso is above a level camera's view unless the camera tilts up
        self.search_pitch = self.declare_parameter("search_pitch", 0.0).value
        # A frame that stays put while the robot moves
        self.world_frame = self.declare_parameter("world_frame", "odom").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target = None  # latest target position in world_frame
        self.last_seen = None  # node time of the latest detection
        self.last_bearing = 0.0

        self.create_subscription(PointStamped, "target", self.on_target, 10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.gizmo_pub = self.create_publisher(
            Float64MultiArray, "gizmo_commands", 10)
        self.create_timer(0.05, self.control_loop)

    def on_target(self, msg):
        # Store the target in the world frame. Between detections the robot keeps
        # turning and driving, and the stored point stays valid while it does
        try:
            try:
                # Where the camera was when the image was taken. Slow detectors (pose
                # models) finish well after that, so this transform is usually available
                camera_to_world = self.tf_buffer.lookup_transform(
                    self.world_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
            except ExtrapolationException:
                # Fast detectors can be newer than the latest TF; use the latest instead
                camera_to_world = self.tf_buffer.lookup_transform(
                    self.world_frame, msg.header.frame_id, Time())
        except TransformException as error:
            self.get_logger().warn(
                f"TF lookup failed: {error}", throttle_duration_sec=2.0)
            return

        self.target = do_transform_point(msg, camera_to_world).point
        self.last_seen = self.get_clock().now()

    def control_loop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        visible = (self.last_seen is not None
                   and self.get_clock().now() - self.last_seen < Duration(seconds=self.lost_timeout))
        if not visible:
            # Target lost: recentre the gizmo and turn towards where it was last seen
            self.gizmo_pub.publish(Float64MultiArray(
                data=[0.0, self.search_pitch]))
            cmd.twist.angular.z = math.copysign(
                self.search_angular, self.last_bearing)
            self.cmd_vel_pub.publish(cmd)
            return

        try:
            world_to_base = self.tf_buffer.lookup_transform(
                "base_link", self.world_frame, Time())
            pitch_joint = self.tf_buffer.lookup_transform(
                "base_link", "gizmo_pitch_link", Time())
        except TransformException as error:
            self.get_logger().warn(
                f"TF lookup failed: {error}", throttle_duration_sec=2.0)
            return

        target = do_transform_point(
            PointStamped(point=self.target), world_to_base).point
        bearing = math.atan2(target.y, target.x)
        distance = math.hypot(target.x, target.y)
        self.last_bearing = bearing

        # Aim the gizmo. The yaw axis passes through the base_link origin and the camera
        # sits (almost) on it, so the yaw angle is simply the bearing to the target
        yaw = clamp(bearing, -self.yaw_limit, self.yaw_limit)
        # The camera lies on the pitch link's x axis, so tilt that axis from the pitch
        # joint towards the target. Positive pitch tilts down, so looking up is negative
        origin = pitch_joint.transform.translation
        height = target.z - origin.z
        horizontal = math.hypot(target.x - origin.x, target.y - origin.y)
        pitch = clamp(-math.atan2(height, horizontal),
                      self.pitch_min, self.pitch_max)
        self.gizmo_pub.publish(Float64MultiArray(data=[yaw, pitch]))

        cmd.twist.angular.z = clamp(
            self.angular_gain * bearing, -self.max_angular, self.max_angular)
        # Only drive forward once roughly facing the target
        heading_scale = max(0.0, math.cos(bearing))
        cmd.twist.linear.x = heading_scale * clamp(
            self.linear_gain * (distance - self.follow_distance), -self.max_linear, self.max_linear)
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
