"""Detect people with a YOLO pose model and estimate their 3D position."""

import math

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

# COCO keypoint indices
LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP = 5, 6, 11, 12
SKELETON = [(5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12),
            (11, 12), (11, 13), (13, 15), (12, 14), (14, 16)]

# Optical frame (x right, y down, z forward) to a level frame (x forward, y left, z up)
OPTICAL_TO_LEVEL = np.array(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def letterbox(bgr, size):
    """Resize keeping the aspect ratio and pad to a square, as the model was trained."""
    h, w = bgr.shape[:2]
    scale = min(size / h, size / w)
    new_h, new_w = round(h * scale), round(w * scale)
    pad_x, pad_y = (size - new_w) // 2, (size - new_h) // 2
    canvas = np.full((size, size, 3), 114, np.uint8)
    canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = cv2.resize(
        bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    blob = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)[None]
    return blob.astype(np.float32) / 255.0, scale, pad_x, pad_y


def rotation_matrix(q):
    """Convert a geometry_msgs Quaternion to a 3x3 rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class PersonDetector(Node):
    def __init__(self):
        super().__init__("person_detector")

        model_path = self.declare_parameter("model_path", "").value
        self.input_size = self.declare_parameter("input_size", 320).value
        self.score_threshold = self.declare_parameter(
            "score_threshold", 0.4).value
        self.keypoint_threshold = self.declare_parameter(
            "keypoint_threshold", 0.5).value
        person_height = self.declare_parameter("person_height", 1.70).value
        torso_ratio = self.declare_parameter("torso_ratio", 0.29).value
        num_threads = self.declare_parameter("num_threads", 4).value
        # A frame whose z axis points straight up, e.g. base_link on a flat floor. Leave
        # empty when there is no TF (a webcam on a desk) to assume the camera is level
        self.level_frame = self.declare_parameter("level_frame", "").value

        # Shoulder-to-hip length. It barely changes when the person turns sideways,
        # unlike shoulder width, and stays in view when the legs are cut off up close
        self.torso_length = person_height * torso_ratio

        options = ort.SessionOptions()
        options.intra_op_num_threads = num_threads
        self.session = ort.InferenceSession(
            model_path, options, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            CameraInfo, "/camera/camera_info", self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.position_pub = self.create_publisher(
            PointStamped, "/person/position", 10)
        self.debug_pub = self.create_publisher(
            Image, "/person/debug_image", 10)

    def on_camera_info(self, msg):
        # An uncalibrated camera publishes an all-zero matrix
        if msg.k[0] > 0.0:
            self.camera_matrix = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def detect(self, bgr):
        """Return boxes (N, 4), scores (N,) and keypoints (N, 17, 3) in image pixels."""
        blob, scale, pad_x, pad_y = letterbox(bgr, self.input_size)
        # End-to-end export: 300 rows of x1, y1, x2, y2, score, class, 17 x (x, y, visibility)
        rows = self.session.run(None, {self.input_name: blob})[0][0]
        rows = rows[rows[:, 4] > self.score_threshold]
        boxes = (rows[:, :4] - [pad_x, pad_y, pad_x, pad_y]) / scale
        keypoints = rows[:, 6:].reshape(-1, 17, 3)
        keypoints[:, :, 0] = (keypoints[:, :, 0] - pad_x) / scale
        keypoints[:, :, 1] = (keypoints[:, :, 1] - pad_y) / scale
        return boxes, rows[:, 4], keypoints

    def torso(self, keypoints):
        """Return the shoulder and hip midpoints, or None if they are not visible."""
        joints = keypoints[[LEFT_SHOULDER,
                            RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP]]
        if joints[:, 2].min() < self.keypoint_threshold:
            return None
        return joints[:2, :2].mean(axis=0), joints[2:, :2].mean(axis=0)

    def camera_rotation(self, frame_id):
        """Return the rotation from the camera optical frame to the level frame."""
        if not self.level_frame:
            return OPTICAL_TO_LEVEL
        try:
            transform = self.tf_buffer.lookup_transform(
                self.level_frame, frame_id, Time())
        except TransformException as error:
            self.get_logger().warn(
                f"TF lookup failed: {error}", throttle_duration_sec=2.0)
            return None
        return rotation_matrix(transform.transform.rotation)

    def on_image(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn("Waiting for a calibrated camera_info",
                                   throttle_duration_sec=5.0)
            return
        fx, fy, cx, cy = self.camera_matrix

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        boxes, scores, keypoints = self.detect(frame)

        # Until re-identification exists, follow the largest (usually closest) person
        # whose torso is visible
        target = None
        for i in np.argsort(-(boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])):
            torso = self.torso(keypoints[i])
            if torso is not None:
                target = (i, *torso)
                break

        distance = None
        rotation = self.camera_rotation(
            msg.header.frame_id) if target is not None else None
        if rotation is not None:
            i, shoulder, hip = target
            # Rays from the camera through the shoulder and hip midpoints, rotated so z is up
            shoulder_ray = rotation @ [(shoulder[0] - cx) /
                                       fx, (shoulder[1] - cy) / fy, 1.0]
            hip_ray = rotation @ [(hip[0] - cx) / fx, (hip[1] - cy) / fy, 1.0]
            shoulder_flat = math.hypot(shoulder_ray[0], shoulder_ray[1])
            hip_flat = math.hypot(hip_ray[0], hip_ray[1])
            # The torso is upright: the shoulders are torso_length straight above the hips,
            # at the same horizontal distance d. So d * (tan(shoulder elevation) -
            # tan(hip elevation)) = torso_length. Unlike the projected torso length, this
            # stays correct when a low camera looks up at the person
            elevation_gap = shoulder_ray[2] / \
                shoulder_flat - hip_ray[2] / hip_flat
            if elevation_gap > 1e-3:
                distance = self.torso_length / elevation_gap
                centre = (shoulder_ray / shoulder_flat +
                          hip_ray / hip_flat) / 2.0 * distance
                x, y, z = rotation.T @ centre

                point = PointStamped()
                point.header = msg.header
                point.point.x, point.point.y, point.point.z = float(
                    x), float(y), float(z)
                self.position_pub.publish(point)

        if self.debug_pub.get_subscription_count() > 0:
            self.draw_debug(frame, boxes, scores, keypoints, target, distance)
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

    def draw_debug(self, frame, boxes, scores, keypoints, target, distance):
        for box, score in zip(boxes.astype(int), scores):
            cv2.rectangle(frame, tuple(box[:2]), tuple(
                box[2:]), (160, 160, 160), 1)
            cv2.putText(frame, f"{score:.2f}", (box[0], box[1] - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)
        if target is None:
            return
        i, shoulder, hip = target
        points = keypoints[i]
        for a, b in SKELETON:
            if min(points[a, 2], points[b, 2]) >= self.keypoint_threshold:
                cv2.line(frame, tuple(points[a, :2].astype(int)),
                         tuple(points[b, :2].astype(int)), (0, 255, 0), 2)
        cv2.line(frame, tuple(shoulder.astype(int)),
                 tuple(hip.astype(int)), (0, 0, 255), 3)
        x1, y1 = boxes[i, :2].astype(int)
        if distance is not None:
            cv2.putText(frame, f"{distance:.2f} m", (x1, y1 - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)


def main():
    # Keep OpenCV from spawning a thread pool that fights ONNX Runtime for the cores
    cv2.setNumThreads(1)
    rclpy.init()
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
