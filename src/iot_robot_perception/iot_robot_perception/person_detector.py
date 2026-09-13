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
from sensor_msgs.msg import CameraInfo, Image

# COCO keypoint indices
LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP = 5, 6, 11, 12
SKELETON = [(5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12),
            (11, 12), (11, 13), (13, 15), (12, 14), (14, 16)]


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

        z = None
        if target is not None:
            i, shoulder, hip = target
            # Normalised image coordinates: a point at depth Z projects to (X/Z, Y/Z).
            # A roughly upright torso is parallel to the image plane, so its projected
            # length is torso_length / Z wherever it appears in the image
            shoulder_n = ((shoulder[0] - cx) / fx, (shoulder[1] - cy) / fy)
            hip_n = ((hip[0] - cx) / fx, (hip[1] - cy) / fy)
            z = self.torso_length / \
                math.hypot(shoulder_n[0] - hip_n[0], shoulder_n[1] - hip_n[1])

            point = PointStamped()
            point.header = msg.header
            point.point.x = (shoulder_n[0] + hip_n[0]) / 2.0 * z
            point.point.y = (shoulder_n[1] + hip_n[1]) / 2.0 * z
            point.point.z = z
            self.position_pub.publish(point)

        if self.debug_pub.get_subscription_count() > 0:
            self.draw_debug(frame, boxes, scores, keypoints, target, z)
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

    def draw_debug(self, frame, boxes, scores, keypoints, target, z):
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
        cv2.putText(frame, f"{z:.2f} m", (x1, y1 - 20),
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
