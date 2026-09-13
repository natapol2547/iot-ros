"""Detect a yellow ball in the camera image and estimate its 3D position."""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class BallDetector(Node):
    def __init__(self):
        super().__init__("ball_detector")

        self.ball_diameter = self.declare_parameter(
            "ball_diameter", 0.10).value
        self.hsv_lower = np.array(self.declare_parameter(
            "hsv_lower", [20, 120, 80]).value, dtype=np.uint8)
        self.hsv_upper = np.array(self.declare_parameter(
            "hsv_upper", [35, 255, 255]).value, dtype=np.uint8)
        self.min_radius_px = self.declare_parameter("min_radius_px", 3.0).value

        self.bridge = CvBridge()
        self.camera_matrix = None  # (fx, fy, cx, cy), filled from CameraInfo
        # (stamp, u, v, z, radius), checked against sim depth
        self.last_detection = None

        self.create_subscription(
            CameraInfo, "/camera/camera_info", self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            Image, "/camera/depth", self.on_depth, qos_profile_sensor_data)

        self.position_pub = self.create_publisher(
            PointStamped, "/ball/position", 10)
        self.debug_pub = self.create_publisher(Image, "/ball/debug_image", 1)

    def on_camera_info(self, msg):
        # K is the row-major 3x3 intrinsic matrix: [fx 0 cx; 0 fy cy; 0 0 1]
        self.camera_matrix = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_depth(self, msg):
        """Compare the size-based estimate with the simulated depth image."""
        # The depth frame arrives just after its colour frame. Only compare frames with
        # the same stamp, otherwise robot motion between frames shows up as fake error
        if self.last_detection is None or self.last_detection[0] != msg.header.stamp:
            return
        _, u, v, z_estimate, radius = self.last_detection
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        depth_at_ball = float(depth[int(v), int(u)])
        if not math.isfinite(depth_at_ball) or depth_at_ball <= 0.0:
            return
        # Depth hits the ball's front surface; its centre is one radius further away
        z_true = depth_at_ball + self.ball_diameter / 2.0
        error = z_estimate - z_true
        self.get_logger().info(
            f"radius {radius:5.1f} px | estimate {z_estimate:.3f} m | depth {z_true:.3f} m | "
            f"error {100.0 * error:+.1f} cm ({100.0 * error / z_true:+.1f}%)",
            throttle_duration_sec=1.0,
        )

    def on_image(self, msg):
        if self.camera_matrix is None:
            self.get_logger().info("Waiting for camera_info...", throttle_duration_sec=2.0)
            return
        fx, fy, cx, cy = self.camera_matrix

        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        # Opening removes isolated noise pixels before looking for blobs
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                np.ones((3, 3), np.uint8))

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            (u, v), radius = cv2.minEnclosingCircle(
                max(contours, key=cv2.contourArea))
            if radius >= self.min_radius_px:
                # Pinhole model: an object of size D at depth Z spans fx * D / Z pixels.
                # Away from the image centre a sphere projects to an ellipse stretched
                # by 1 / cos(theta), and the enclosing circle follows the long axis
                cos_theta = 1.0 / \
                    math.sqrt(1.0 + ((u - cx) / fx) **
                              2 + ((v - cy) / fy) ** 2)
                z = fx * self.ball_diameter / (2.0 * radius * cos_theta)

                point = PointStamped()
                point.header = msg.header  # camera_optical_frame, same timestamp as the image
                point.point.x = (u - cx) * z / fx
                point.point.y = (v - cy) * z / fy
                point.point.z = z
                self.position_pub.publish(point)

                self.last_detection = (msg.header.stamp, u, v, z, radius)
                cv2.circle(bgr, (int(u), int(v)), int(radius), (0, 0, 255), 2)
                cv2.putText(bgr, f"{z:.2f} m", (int(u - radius), int(v - radius - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        debug = self.bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
        debug.header = msg.header
        self.debug_pub.publish(debug)

    def log_against_depth(self, u, v, z_estimate, radius):
        """Compare the size-based estimate with the simulated depth image."""
        if self.depth is None:
            return
        depth_at_ball = float(self.depth[int(v), int(u)])
        if not math.isfinite(depth_at_ball) or depth_at_ball <= 0.0:
            return
        # Depth hits the ball's front surface; its centre is one radius further away
        z_true = depth_at_ball + self.ball_diameter / 2.0
        error = z_estimate - z_true
        self.get_logger().info(
            f"radius {radius:5.1f} px | estimate {z_estimate:.3f} m | depth {z_true:.3f} m | "
            f"error {100.0 * error:+.1f} cm ({100.0 * error / z_true:+.1f}%)",
            throttle_duration_sec=1.0,
        )


def main():
    # One thread is faster here: for a 640x480 image, OpenMP's thread coordination
    # costs more than the work itself and steals CPU from the simulator
    cv2.setNumThreads(1)

    rclpy.init()
    node = BallDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
