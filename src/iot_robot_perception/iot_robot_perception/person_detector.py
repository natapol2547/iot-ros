"""Detect people with a YOLO pose model, recognise the enrolled one and locate them in 3D."""

import math
from collections import namedtuple

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
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from iot_robot_perception.reid import Gallery, ReidEncoder

# COCO keypoint indices
LEFT_WRIST, RIGHT_WRIST = 9, 10
LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP = 5, 6, 11, 12
SKELETON = [(5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12),
            (11, 12), (11, 13), (13, 15), (12, 14), (14, 16)]

# Optical frame (x right, y down, z forward) to a level frame (x forward, y left, z up)
OPTICAL_TO_LEVEL = np.array(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])

# A detected person whose torso is visible
Person = namedtuple(
    "Person", ["box", "keypoints", "shoulder", "hip", "embedding"])


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
        reid_model_path = self.declare_parameter("reid_model_path", "").value
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
        # Cosine similarity needed to accept a person as the enrolled one
        self.match_threshold = self.declare_parameter(
            "match_threshold", 0.75).value
        # Similarity above which a match is certain enough to learn its current look
        self.update_threshold = self.declare_parameter(
            "update_threshold", 0.85).value
        # How long both hands must stay raised to enroll
        self.enroll_duration = self.declare_parameter(
            "enroll_duration", 2.0).value
        recent_size = self.declare_parameter("recent_size", 30).value

        # Shoulder-to-hip length. It barely changes when the person turns sideways,
        # unlike shoulder width, and stays in view when the legs are cut off up close
        self.torso_length = person_height * torso_ratio

        options = ort.SessionOptions()
        options.intra_op_num_threads = num_threads
        self.session = ort.InferenceSession(
            model_path, options, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.encoder = ReidEncoder(reid_model_path, num_threads)
        self.gallery = Gallery(recent_size)

        self.enroll_start = None  # image time (s) when both hands went up
        self.hands_last_up = None  # image time (s) of the latest hands-up frame
        self.enroll_embeddings = []
        self.people = []  # people in the latest image, for the enroll service
        self.image_width = 0

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
        # "~/" puts the services under the node name: /person_detector/enroll
        self.create_service(Trigger, "~/enroll", self.on_enroll)
        self.create_service(Trigger, "~/forget", self.on_forget)

    def on_camera_info(self, msg):
        # An uncalibrated camera publishes an all-zero matrix
        if msg.k[0] > 0.0:
            self.camera_matrix = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_enroll(self, request, response):
        # Without a gesture, enroll whoever is nearest the image centre
        if not self.people:
            response.message = "No person with a visible torso in view"
            return response
        person = min(self.people, key=lambda p: abs(
            (p.box[0] + p.box[2]) / 2.0 - self.image_width / 2.0))
        self.gallery.enroll([person.embedding])
        response.success = True
        response.message = "Enrolled the person nearest the image centre"
        self.get_logger().info(response.message)
        return response

    def on_forget(self, request, response):
        self.gallery.clear()
        response.success = True
        response.message = "Forgot the enrolled person"
        self.get_logger().info(response.message)
        return response

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

    def hands_up(self, person):
        """True if both wrists are well above the shoulders."""
        wrists = person.keypoints[[LEFT_WRIST, RIGHT_WRIST]]
        if wrists[:, 2].min() < self.keypoint_threshold:
            return False
        # Image y grows downwards. Measuring in torso lengths works at any distance
        torso_pixels = person.hip[1] - person.shoulder[1]
        return bool(np.all(person.shoulder[1] - wrists[:, 1] > 0.5 * torso_pixels))

    def update_enrollment(self, people, stamp):
        """Enroll the person who keeps both hands raised for enroll_duration."""
        raised = [p for p in people if self.hands_up(p)]
        if not raised:
            # Forgive a single missed detection before cancelling
            if self.enroll_start is not None and stamp - self.hands_last_up > 0.5:
                self.enroll_start = None
            return

        # If several people raise their hands, the closest (largest) one wins
        person = max(raised, key=lambda p: (
            p.box[2] - p.box[0]) * (p.box[3] - p.box[1]))
        if self.enroll_start is None:
            self.enroll_start = stamp
            self.enroll_embeddings = []
        self.hands_last_up = stamp
        # Every frame of the gesture adds a slightly different view
        self.enroll_embeddings.append(person.embedding)
        if stamp - self.enroll_start >= self.enroll_duration:
            self.gallery.enroll(self.enroll_embeddings)
            self.enroll_start = None
            self.get_logger().info(
                f"Enrolled a person from {len(self.enroll_embeddings)} images")

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
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        boxes, scores, keypoints = self.detect(frame)

        # Distance needs the torso, so only people with a visible torso are candidates
        people = []
        for box, points in zip(boxes, keypoints):
            torso = self.torso(points)
            if torso is None:
                continue
            embedding = self.encoder.embed(frame, box)
            if embedding is not None:
                people.append(Person(box, points, *torso, embedding))
        self.people = people
        self.image_width = frame.shape[1]

        self.update_enrollment(people, stamp)

        # Follow the person who looks most like the enrolled one, if they look enough alike
        target = None
        similarities = [self.gallery.similarity(p.embedding)
                        for p in people] if self.gallery else []
        if similarities:
            best = int(np.argmax(similarities))
            if similarities[best] >= self.match_threshold:
                target = people[best]
                # Learn the current look only when nobody else could be mistaken for them
                matches = sum(s >= self.match_threshold for s in similarities)
                if similarities[best] >= self.update_threshold and matches == 1:
                    self.gallery.add(target.embedding)

        distance = None
        rotation = self.camera_rotation(
            msg.header.frame_id) if target is not None else None
        if rotation is not None:
            shoulder, hip = target.shoulder, target.hip
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
            self.draw_debug(frame, boxes, scores, people,
                            similarities, target, distance, stamp)
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

    def draw_debug(self, frame, boxes, scores, people, similarities, target, distance, stamp):
        for box, score in zip(boxes.astype(int), scores):
            cv2.rectangle(frame, tuple(box[:2]), tuple(
                box[2:]), (160, 160, 160), 1)
            cv2.putText(frame, f"{score:.2f}", (box[0], box[1] - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)
        # Similarity to the enrolled person, inside the top of each box
        for person, similarity in zip(people, similarities):
            x1, y1 = person.box[:2].astype(int)
            cv2.putText(frame, f"sim {similarity:.2f}", (x1 + 4, y1 + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if self.enroll_start is not None:
            status = f"Enrolling {stamp - self.enroll_start:.1f} / {self.enroll_duration:.1f} s"
        elif not self.gallery:
            status = "Raise both hands to enroll"
        else:
            status = "Following" if target is not None else "Enrolled person not in view"
        cv2.putText(frame, status, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        if target is None:
            return
        points = target.keypoints
        for a, b in SKELETON:
            if min(points[a, 2], points[b, 2]) >= self.keypoint_threshold:
                cv2.line(frame, tuple(points[a, :2].astype(int)),
                         tuple(points[b, :2].astype(int)), (0, 255, 0), 2)
        cv2.line(frame, tuple(target.shoulder.astype(int)),
                 tuple(target.hip.astype(int)), (0, 0, 255), 3)
        x1, y1 = target.box[:2].astype(int)
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
