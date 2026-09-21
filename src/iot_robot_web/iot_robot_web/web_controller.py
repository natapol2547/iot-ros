"""Serve a browser controller for the robot on the local network.

The web server (aiohttp on asyncio) runs in the main thread and the ROS node spins in a
background thread. The web side only calls the thread-safe methods of RobotBridge, which
keep their state under one lock; subscription changes are handed to the ROS thread with a
guard condition. Every safety rule (speed limits, dead-man timeout, one driver at a time,
obstacle guard, e-stop) is enforced here, never in the browser.
"""

import asyncio
import hashlib
import json
import math
import os
import shutil
import signal
import socket
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from urllib.parse import urlparse

import cv2
import rclpy
from aiohttp import WSCloseCode, WSMsgType, web
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import BatteryState, Image, Range
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

from iot_robot_web.follow_process import FollowProcess
from iot_robot_web.logic import (
    DriverLock, EchoFilter, clamp, classify_range, finite_or_zero, guard_command,
    host_allowed, scale_command)

SIDES = ("left", "right")
DRIVE, BALL, PERSON = "drive", "ball", "person"
MODES = (DRIVE, BALL, PERSON)
NOT_YOUR_TURN = "Someone else is driving. Wait until they let go."
# An engaged E-stop is repeated this often (s) so a restarted twist_mux locks again
ESTOP_HEARTBEAT = 1.0


def websocket_response(heartbeat=5.0):
    """The browser WebSocket. A function of its own so the tests use the same settings.

    heartbeat pings detect clients that vanished without closing the socket. compress=False
    declines permessage-deflate, which every browser offers: aiohttp 3.14 rejects a
    compressed frame that follows a pong as the client's first frame, closing the socket
    with 1002. That is exactly a visitor who watches past the first ping, then presses STOP.
    The messages are a few hundred bytes, so compression would gain nothing anyway.
    """
    return web.WebSocketResponse(
        heartbeat=heartbeat, timeout=1.0, max_msg_size=4096, compress=False)


def number(value, digits=3):
    """A JSON-safe number: browsers cannot parse Infinity or NaN."""
    return round(value, digits) if value is not None and math.isfinite(value) else None


class RobotBridge(Node):
    """The ROS side of the controller. Its public methods are safe to call from any thread.

    Web-side timing (dead-man, driver lock, staleness) uses time.monotonic, so it behaves
    the same on sim time and on the robot. Message stamps use the node clock, which
    diff_drive_controller compares against its own.
    """

    def __init__(self):
        super().__init__("web_controller")

        # Web server
        self.host = self.declare_parameter("host", "0.0.0.0").value
        self.port = self.declare_parameter("port", 8080).value
        # Host names the page may be opened under, besides IP addresses, localhost and
        # this machine's hostname (with and without .local). ["*"] accepts any name
        self.allowed_hosts = self.declare_parameter(
            "allowed_hosts", Parameter.Type.STRING_ARRAY).value or []
        self.telemetry_rate = self.declare_parameter("telemetry_rate", 10.0).value
        self.stream_fps = self.declare_parameter("stream_fps", 10.0).value
        self.stream_width = self.declare_parameter("stream_width", 640).value
        self.jpeg_quality = self.declare_parameter("jpeg_quality", 70).value
        # Driving
        self.max_linear = self.declare_parameter("max_linear", 0.4).value
        self.max_angular = self.declare_parameter("max_angular", 1.5).value
        # The browser streams commands while a control is held; if they stop arriving
        # for this long (dropped Wi-Fi, closed laptop) the robot stops
        self.command_timeout = self.declare_parameter("command_timeout", 0.3).value
        # A driver keeps the controls until they have been idle this long
        self.driver_timeout = self.declare_parameter("driver_timeout", 2.0).value
        publish_rate = self.declare_parameter("publish_rate", 20.0).value
        self.start_estopped = self.declare_parameter("start_estopped", False).value
        # Ultrasonic sensors. 0 disables the guard
        self.stop_distance = self.declare_parameter(
            "obstacle_stop_distance", 0.25).value
        self.warn_distance = self.declare_parameter("warn_distance", 1.0).value
        self.danger_distance = self.declare_parameter("danger_distance", 0.4).value
        self.sensor_timeout = self.declare_parameter("sensor_timeout", 1.0).value
        # Block forward motion while a sensor is faulty or silent, not just near obstacles
        self.require_ultrasonic = self.declare_parameter("require_ultrasonic", True).value
        # Gizmo limits, the joint limits of the URDF
        self.yaw_limit = self.declare_parameter("yaw_limit", 0.785398).value
        self.pitch_min = self.declare_parameter("pitch_min", -0.785398).value
        self.pitch_max = self.declare_parameter("pitch_max", 0.0).value
        # 6S LiPo: 3.5 V per cell counts as empty, 4.2 V as full
        self.battery_empty = self.declare_parameter(
            "battery_empty_voltage", 21.0).value
        self.battery_full = self.declare_parameter(
            "battery_full_voltage", 25.2).value
        # Follow modes
        self.stop_timeout = self.declare_parameter("stop_timeout", 10.0).value
        self.launch_package = self.declare_parameter(
            "launch_package", "iot_robot_behavior").value
        self.launch_files = {
            BALL: self.declare_parameter("ball_launch", "ball_follow.launch.py").value,
            PERSON: self.declare_parameter("person_launch", "person_follow.launch.py").value,
        }
        # The video shows what the current mode is looking at
        self.image_topics = {
            DRIVE: self.declare_parameter("camera_topic", "/camera/image_raw").value,
            BALL: self.declare_parameter("ball_image_topic", "/ball/debug_image").value,
            PERSON: self.declare_parameter("person_image_topic", "/person/debug_image").value,
        }

        self.lock = threading.Lock()
        self.driver_lock = DriverLock(self.driver_timeout)
        self.command = None  # (linear, angular, monotonic time) from the driver
        self.published = None  # (linear, angular) last sent while driving, None when idle
        self.guard = None  # (side, distance) while the guard holds back a forward command
        # twist_mux obeys the latest /e_stop message from anyone. The node mirrors that
        # state, so it never contradicts an E-stop engaged from a terminal or another node
        self.estop = self.start_estopped
        self.estop_echo = EchoFilter()
        self.estop_known = False  # set once /e_stop was heard or published
        self.mode = DRIVE
        self.gizmo = (0.0, 0.0)
        never = -math.inf
        self.ranges = {side: (None, never) for side in SIDES}
        self.odom = ((0.0, 0.0), never)
        self.battery = (None, never)
        self.person_status = (None, never)
        self.follower_ready = False
        self.detector_ready = False
        self.streaming = False
        self.image_topic = None  # the topic the web side wants, None for no video
        self.image_sub = None
        self.image = None
        self.image_seq = 0
        self.image_time = never

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel/web", 10)
        self.estop_pub = self.create_publisher(Bool, "/e_stop", 10)
        self.gizmo_pub = self.create_publisher(
            Float64MultiArray, "/gizmo_controller/commands", 10)

        for side in SIDES:
            self.create_subscription(
                Range, f"/ultrasonic/{side}",
                lambda msg, side=side: self.on_range(side, msg), qos_profile_sensor_data)
        self.create_subscription(
            Odometry, "/diff_drive_controller/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            BatteryState, "/battery_state", self.on_battery, qos_profile_sensor_data)
        self.create_subscription(String, "/person/status", self.on_person_status, 10)
        self.create_subscription(Bool, "/e_stop", self.on_estop, 10)
        self.enroll_client = self.create_client(Trigger, "/person_detector/enroll")
        self.forget_client = self.create_client(Trigger, "/person_detector/forget")

        # Steady-clock timers keep the dead-man and the e-stop heartbeat on wall time even
        # when the node runs on sim time
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / publish_rate, self.on_drive_timer, clock=steady)
        self.create_timer(ESTOP_HEARTBEAT, self.on_estop_timer, clock=steady)
        self.create_timer(0.5, self.on_graph_timer, clock=steady)
        self.image_guard = self.create_guard_condition(self.update_image_subscription)

        if self.start_estopped:
            # Stopping early is always safe; the heartbeat repeats it
            with self.lock:
                self.publish_estop()
        else:
            # Listen before speaking: an E-stop someone keeps engaged (another web
            # controller, say) is heard within one heartbeat and mirrored. If nothing is
            # heard, publish "released", which also clears a lock left behind by a previous
            # run of this node, so the page and twist_mux agree
            self.estop_startup = self.create_timer(
                1.5 * ESTOP_HEARTBEAT, self.on_estop_startup, clock=steady)

    # ROS thread ---------------------------------------------------------------------

    def on_range(self, side, msg):
        # NaN, a sensor the STM32 reports as faulty, is kept: it shows as "no data" at once
        # instead of after sensor_timeout. REP 117: -inf means closer than it can measure
        distance = msg.range
        if distance == -math.inf:
            distance = 0.0
        with self.lock:
            self.ranges[side] = (distance, time.monotonic())

    def on_odom(self, msg):
        twist = msg.twist.twist
        with self.lock:
            self.odom = ((twist.linear.x, twist.angular.z), time.monotonic())

    def on_battery(self, msg):
        if math.isfinite(msg.voltage) and msg.voltage > 0.0:
            with self.lock:
                self.battery = (msg.voltage, time.monotonic())

    def on_person_status(self, msg):
        with self.lock:
            self.person_status = (msg.data, time.monotonic())

    def on_estop(self, msg):
        engaged = bool(msg.data)
        with self.lock:
            # is_echo goes first: it also consumes the echoes that match the current state
            if self.estop_echo.is_echo(engaged, time.monotonic()):
                return
            self.estop_known = True
            if engaged == self.estop:
                return
            self.apply_estop(engaged)
        state = "ENGAGED" if engaged else "released"
        self.get_logger().warning(f"E-stop {state} by another /e_stop publisher")

    def on_image(self, msg):
        with self.lock:
            # A frame from the previous mode's topic may still be queued
            if self.image_sub is None or self.image_sub.topic_name != self.image_topic:
                return
            self.image = msg
            self.image_seq += 1
            self.image_time = time.monotonic()

    def update_image_subscription(self):
        with self.lock:
            topic = self.image_topic
        if self.image_sub is not None and self.image_sub.topic_name != topic:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
        # Subscribe only while someone watches: the follow nodes skip drawing their
        # debug images when nobody subscribes, and the Pi saves the bandwidth
        if topic and self.image_sub is None:
            self.image_sub = self.create_subscription(
                Image, topic, self.on_image, qos_profile_sensor_data)

    def on_drive_timer(self):
        now = time.monotonic()
        with self.lock:
            if self.command is not None and now - self.command[2] >= self.command_timeout:
                self.command = None
            if self.command is None:
                self.guard = None
                # One zero when the driver lets go or goes silent, then nothing, so
                # twist_mux falls back to the follower after its timeout
                if self.published is not None:
                    self.publish_twist(0.0, 0.0)
                    self.published = None
                return
            linear, angular, _ = self.command
            guarded, blocking = guard_command(
                linear, self.fresh_ranges(now), self.stop_distance, self.require_ultrasonic)
            self.guard = blocking if guarded != linear else None
            self.publish_twist(guarded, angular)
            self.published = (guarded, angular)

    def on_estop_timer(self):
        # Only an engaged E-stop is repeated. Released is what a restarted twist_mux
        # assumes anyway, and repeating it would cancel an E-stop engaged elsewhere
        with self.lock:
            if self.estop:
                self.publish_estop()

    def on_estop_startup(self):
        self.estop_startup.cancel()
        with self.lock:
            if not self.estop_known:
                self.publish_estop()

    def on_graph_timer(self):
        follower = self.count_publishers("/cmd_vel/follower") > 0
        detector = self.enroll_client.service_is_ready()
        with self.lock:
            self.follower_ready = follower
            self.detector_ready = detector

    # Called with self.lock held -----------------------------------------------------

    def publish_twist(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def publish_estop(self):
        self.estop_known = True
        self.estop_echo.sent(self.estop, time.monotonic())
        self.estop_pub.publish(Bool(data=self.estop))

    def apply_estop(self, engaged):
        if engaged:
            # Drop the driver's command so it does not resume on release
            self.command = None
            self.guard = None
            self.published = None
        self.estop = engaged

    def fresh_ranges(self, now):
        return {side: (distance if now - stamp < self.sensor_timeout else None)
                for side, (distance, stamp) in self.ranges.items()}

    def publish_gizmo(self, yaw, pitch):
        self.gizmo = (yaw, pitch)
        self.gizmo_pub.publish(Float64MultiArray(data=[yaw, pitch]))

    def wanted_image_topic(self):
        return self.image_topics[self.mode] if self.streaming else None

    # Web thread ---------------------------------------------------------------------

    def may_control(self, client):
        """True unless another client holds the driver lock."""
        with self.lock:
            return self.driver_lock.holder(time.monotonic()) in (None, client)

    def drive(self, client, linear_axis, angular_axis, speed):
        """Store a joystick command. Returns "ok", "estop" or "busy"."""
        now = time.monotonic()
        with self.lock:
            if self.estop:
                return "estop"
            previous = self.driver_lock.holder(now)
            if not self.driver_lock.request(client, now):
                return "busy"
            linear, angular = scale_command(
                linear_axis, angular_axis, speed, self.max_linear, self.max_angular)
            self.command = (linear, angular, now)
        if previous != client:
            self.get_logger().info(f"Client {client} is driving")
        return "ok"

    def release(self, client, disconnected=False):
        """The driver let go: stop now. A disconnect also frees the driver lock."""
        with self.lock:
            if self.driver_lock.holder(time.monotonic()) != client:
                return
            if disconnected:
                self.driver_lock.release(client)
            self.command = None
            self.guard = None
            if self.published is not None:
                self.publish_twist(0.0, 0.0)
                self.published = None

    def set_estop(self, engaged):
        with self.lock:
            changed = engaged != self.estop
            if engaged:
                # Send a zero through twist_mux before locking it. Once the lock engages the
                # mux forwards nothing, and the last command it passed on (perhaps from a
                # follower) would stay in effect until diff_drive_controller times out
                self.publish_twist(0.0, 0.0)
            self.apply_estop(engaged)
            self.publish_estop()
        return changed

    def aim_gizmo(self, client, yaw, pitch):
        """Point the camera. Returns None on success, else the reason it was refused."""
        now = time.monotonic()
        with self.lock:
            if self.mode != DRIVE:
                return "The follower aims the camera in this mode"
            if self.estop:
                return "Release the E-stop first"
            if not self.driver_lock.request(client, now):
                return NOT_YOUR_TURN
            self.publish_gizmo(
                clamp(finite_or_zero(yaw), -self.yaw_limit, self.yaw_limit),
                clamp(finite_or_zero(pitch), self.pitch_min, self.pitch_max))
        return None

    def set_mode(self, mode):
        with self.lock:
            self.mode = mode
            self.image_topic = self.wanted_image_topic()
            self.image = None
            if mode == DRIVE:
                # Whatever the follower left it at, drive mode starts looking ahead
                self.publish_gizmo(0.0, 0.0)
        self.image_guard.trigger()

    def set_streaming(self, streaming):
        with self.lock:
            self.streaming = streaming
            self.image_topic = self.wanted_image_topic()
            if not streaming:
                self.image = None
        self.image_guard.trigger()

    def latest_image(self):
        with self.lock:
            return self.image, self.image_seq

    def snapshot(self):
        """The telemetry shared by every client, as JSON-safe values."""
        now = time.monotonic()
        with self.lock:
            distances = self.fresh_ranges(now)
            ultrasonic = {}
            for side, distance in distances.items():
                level, period = classify_range(
                    distance, self.warn_distance, self.danger_distance)
                ultrasonic[side] = {
                    # null when stale, "clear" when nothing echoes back
                    "distance": "clear" if distance == math.inf else number(distance),
                    "level": level,
                    "period": period,
                }
            _, obstacle = guard_command(
                1.0, distances, self.stop_distance, self.require_ultrasonic)
            (linear, angular), odom_time = self.odom
            odom = None
            if now - odom_time < self.sensor_timeout:
                odom = {"linear": number(linear), "angular": number(angular)}
            voltage, battery_time = self.battery
            battery = None
            if now - battery_time < 5.0:
                span = self.battery_full - self.battery_empty
                battery = {
                    "voltage": number(voltage, 2),
                    "percent": round(100.0 * clamp((voltage - self.battery_empty) / span,
                                                   0.0, 1.0)),
                }
            person_status, person_time = self.person_status
            return {
                "estop": self.estop,
                "holder": self.driver_lock.holder(now),
                "command": None if self.published is None else {
                    "linear": number(self.published[0]),
                    "angular": number(self.published[1])},
                "ultrasonic": ultrasonic,
                "guard": {
                    # An obstacle is inside the stop distance, or a sensor has no data,
                    # so forward is disabled
                    "obstacle": obstacle is not None,
                    # ... and the driver is pushing forward into it right now
                    "blocking": self.guard is not None,
                    "side": obstacle[0] if obstacle else None,
                    # null when the side has no data rather than an obstacle
                    "distance": number(obstacle[1]) if obstacle else None,
                },
                "odom": odom,
                "battery": battery,
                "person_status": person_status if now - person_time < 2.0 else None,
                "gizmo": {"yaw": number(self.gizmo[0]), "pitch": number(self.gizmo[1])},
                "video_age": number(now - self.image_time, 2),
                "follower_ready": self.follower_ready,
                "detector_ready": self.detector_ready,
            }


class FrameHub:
    """Encodes the newest camera frame to JPEG once, for every MJPEG viewer."""

    def __init__(self, bridge):
        self.bridge = bridge
        self.period = 1.0 / bridge.stream_fps
        self.width = bridge.stream_width
        self.quality = bridge.jpeg_quality
        self.cv_bridge = CvBridge()
        # One worker: frames are encoded in order and never pile up
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="jpeg")
        self.viewers = 0
        self.jpeg = None
        self.seq = 0
        self.changed = asyncio.Condition()
        self.task = None
        self.closed = False
        self.warned = False

    def add_viewer(self):
        self.viewers += 1
        if self.viewers == 1:
            self.bridge.set_streaming(True)
            self.task = asyncio.create_task(self.run())

    def remove_viewer(self):
        self.viewers -= 1
        if self.viewers == 0:
            self.bridge.set_streaming(False)
            self.task.cancel()
            self.jpeg = None

    def reset(self):
        """Forget the last frame, e.g. when the mode switches to another topic."""
        self.jpeg = None

    async def run(self):
        loop = asyncio.get_running_loop()
        encoded = None
        while True:
            started = loop.time()
            msg, seq = self.bridge.latest_image()
            if msg is not None and seq != encoded:
                encoded = seq
                jpeg = await loop.run_in_executor(self.pool, self.encode, msg)
                if jpeg is not None:
                    async with self.changed:
                        self.jpeg = jpeg
                        self.seq += 1
                        self.changed.notify_all()
            await asyncio.sleep(max(0.0, self.period - (loop.time() - started)))

    def encode(self, msg):
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as error:  # noqa: BLE001 - any bad frame just gets skipped
            if not self.warned:
                self.bridge.get_logger().warning(f"Cannot convert the video frame: {error}")
                self.warned = True
            return None
        height, width = frame.shape[:2]
        if width > self.width:
            frame = cv2.resize(frame, (self.width, round(height * self.width / width)),
                               interpolation=cv2.INTER_AREA)
        ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        return jpeg.tobytes() if ok else None

    async def next_frame(self, seen, timeout):
        """Wait for a frame newer than `seen`. Returns (jpeg or None, seq)."""
        async with self.changed:
            try:
                await asyncio.wait_for(
                    self.changed.wait_for(lambda: self.seq != seen or self.closed), timeout)
            except asyncio.TimeoutError:
                return None, seen
            return self.jpeg, self.seq

    async def close(self):
        self.closed = True
        async with self.changed:
            self.changed.notify_all()
        if self.task is not None:
            self.task.cancel()
        self.pool.shutdown(wait=False, cancel_futures=True)


def same_origin(request):
    """Refuse WebSockets opened by other sites' pages, which browsers allow by default.

    A rebound hostile domain passes this check; WebServer.check_host refuses it.
    """
    origin = request.headers.get("Origin")
    return origin is None or urlparse(origin).netloc == request.host


class WebServer:
    def __init__(self, bridge):
        self.bridge = bridge
        self.logger = bridge.get_logger()
        self.static_dir = os.path.join(
            get_package_share_directory("iot_robot_web"), "static")
        self.clients = {}  # client id -> WebSocketResponse
        self.mode = DRIVE
        self.mode_status = "ready"
        self.mode_detail = ""
        self.mode_since = time.monotonic()
        self.mode_lock = asyncio.Lock()
        self.follow = None
        self.hub = FrameHub(bridge)
        self.tasks = set()
        self.ui_version = self.hash_static()
        self.runner = None
        self.telemetry_task = None
        hostname = socket.gethostname().lower()
        short = hostname.split(".")[0]
        self.host_names = {hostname, short, f"{short}.local"} | {
            name.lower().rstrip(".") for name in bridge.allowed_hosts}
        self.refused_hosts = set()

        app = web.Application(middlewares=[self.check_host])
        app.router.add_get("/", self.index)
        app.router.add_get("/ws", self.websocket)
        app.router.add_get("/stream.mjpg", self.stream)
        # Only the files that exist at start are served. An explicit list rather than
        # add_static, which refuses the symlinks a --symlink-install build creates
        self.static_files = set(os.listdir(self.static_dir))
        app.router.add_get("/static/{name}", self.static)
        app.on_response_prepare.append(self.add_headers)
        self.app = app

    def hash_static(self):
        """Lets open pages notice a new UI after the robot restarts, and reload."""
        digest = hashlib.sha1()
        for name in sorted(os.listdir(self.static_dir)):
            with open(os.path.join(self.static_dir, name), "rb") as file:
                digest.update(file.read())
        return digest.hexdigest()[:12]

    async def start(self):
        self.runner = web.AppRunner(self.app, access_log=None, shutdown_timeout=1.0)
        await self.runner.setup()
        site = web.TCPSite(self.runner, self.bridge.host, self.bridge.port)
        await site.start()
        self.telemetry_task = asyncio.create_task(self.telemetry())
        self.logger.info(
            f"Web controller on http://{self.bridge.host}:{self.bridge.port}, "
            f"on the LAN as http://{socket.gethostname()}.local:{self.bridge.port}")

    async def shutdown(self, child_timeout):
        self.telemetry_task.cancel()
        for task in list(self.tasks):
            task.cancel()
        for client in list(self.clients):
            self.bridge.release(client, disconnected=True)
        await asyncio.gather(
            *(ws.close(code=WSCloseCode.GOING_AWAY, message=b"Server shutting down")
              for ws in list(self.clients.values())),
            return_exceptions=True)
        await self.hub.close()
        await self.stop_follow(child_timeout)
        await self.runner.cleanup()

    def kill_follow(self):
        if self.follow is not None:
            self.follow.kill()

    # HTTP ---------------------------------------------------------------------------

    @web.middleware
    async def check_host(self, request, handler):
        """Refuse requests addressed to a name other than this machine's (DNS rebinding)."""
        if not host_allowed(request.host, self.host_names):
            if request.host not in self.refused_hosts and len(self.refused_hosts) < 20:
                self.refused_hosts.add(request.host)
                self.logger.warning(
                    f"Refused a request for host '{request.host}' from {request.remote}. "
                    "If that is a name of this robot, add it to allowed_hosts")
            raise web.HTTPMisdirectedRequest(
                text="Unknown host name. Open the page by the robot's IP address or "
                     "hostname, or add this name to the allowed_hosts parameter.\n")
        return await handler(request)

    async def add_headers(self, request, response):
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["Referrer-Policy"] = "no-referrer"
        if request.path == "/" or request.path.startswith("/static/"):
            # Revalidate every load so a robot-side update shows up at once
            response.headers["Cache-Control"] = "no-cache"
            host = request.host
            response.headers["Content-Security-Policy"] = (
                "default-src 'none'; script-src 'self'; style-src 'self'; "
                f"img-src 'self' data:; connect-src 'self' ws://{host} wss://{host}; "
                "base-uri 'none'; form-action 'none'; frame-ancestors 'none'")

    async def index(self, request):
        return web.FileResponse(os.path.join(self.static_dir, "index.html"))

    async def static(self, request):
        name = request.match_info["name"]
        if name not in self.static_files:
            raise web.HTTPNotFound()
        return web.FileResponse(os.path.join(self.static_dir, name))

    async def stream(self, request):
        """MJPEG: a never-ending multipart response, one JPEG part per frame."""
        response = web.StreamResponse(headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame",
            "Cache-Control": "no-store",
        })
        await response.prepare(request)
        self.hub.add_viewer()
        try:
            seen = -1
            while not self.hub.closed:
                jpeg, seen = await self.hub.next_frame(seen, timeout=1.0)
                # aiohttp does not cancel handlers when the viewer leaves, so check
                if request.transport is None or request.transport.is_closing():
                    break
                if jpeg is not None:
                    await response.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
        except ConnectionError:
            pass
        finally:
            self.hub.remove_viewer()
        return response

    # WebSocket ----------------------------------------------------------------------

    async def websocket(self, request):
        if not same_origin(request):
            raise web.HTTPForbidden(text="Cross-origin WebSocket refused")
        ws = websocket_response()
        await ws.prepare(request)
        client = uuid.uuid4().hex[:6]
        self.clients[client] = ws
        self.logger.info(
            f"Client {client} connected from {request.remote} ({len(self.clients)} total)")
        try:
            await ws.send_json(self.hello(client))
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    await self.handle(ws, client, request.remote, msg.data)
        except ConnectionError:
            pass
        finally:
            del self.clients[client]
            self.bridge.release(client, disconnected=True)
            self.logger.info(
                f"Client {client} disconnected ({len(self.clients)} left)")
        return ws

    def hello(self, client):
        bridge = self.bridge
        return {
            "type": "hello",
            "id": client,
            "ui_version": self.ui_version,
            "command_rate": 20,
            "limits": {"linear": bridge.max_linear, "angular": bridge.max_angular},
            "ultrasonic": {"warn": bridge.warn_distance, "danger": bridge.danger_distance,
                           "stop": bridge.stop_distance},
            "gizmo": {"yaw": bridge.yaw_limit, "pitch_min": bridge.pitch_min,
                      "pitch_max": bridge.pitch_max},
        }

    async def handle(self, ws, client, remote, text):
        try:
            data = json.loads(text)
            kind = data["type"]
        except (ValueError, TypeError, KeyError):
            return
        if kind == "drive":
            self.bridge.drive(client, data.get("linear"), data.get("angular"),
                              data.get("speed"))
        elif kind == "release":
            self.bridge.release(client)
        elif kind == "estop":
            engaged = data.get("engaged") is True
            if self.bridge.set_estop(engaged):
                state = "ENGAGED" if engaged else "released"
                self.logger.warning(f"E-stop {state} by client {client} ({remote})")
        elif kind == "gizmo":
            refused = self.bridge.aim_gizmo(client, data.get("yaw"), data.get("pitch"))
            if refused:
                await self.notice(ws, refused)
        elif kind == "mode":
            self.spawn(self.change_mode(ws, client, data.get("mode")))
        elif kind == "person":
            self.spawn(self.person_action(ws, client, data.get("action")))

    def spawn(self, coroutine):
        # Long actions run as tasks so the socket keeps reading (e-stop included)
        task = asyncio.create_task(coroutine)
        self.tasks.add(task)
        task.add_done_callback(self.tasks.discard)

    async def notice(self, ws, text):
        try:
            await ws.send_json({"type": "notice", "text": text})
        except ConnectionError:
            pass

    async def telemetry(self):
        period = 1.0 / self.bridge.telemetry_rate
        while True:
            await asyncio.sleep(period)
            if not self.clients:
                continue
            snapshot = self.bridge.snapshot()
            holder = snapshot.pop("holder")
            status, detail = self.follow_status(snapshot)
            # A follower this page did not start (launched from a terminal, or left over
            # from a crash) also drives the robot. The grace period lets one this page just
            # stopped leave the ROS graph first
            external = self.mode == DRIVE and self.follow is None \
                and snapshot["follower_ready"] and time.monotonic() - self.mode_since > 3.0
            shared = {
                "type": "state",
                "mode": self.mode,
                "mode_status": status,
                "mode_detail": detail,
                "external_follower": external,
                "clients": len(self.clients),
                **snapshot,
            }
            await asyncio.gather(
                *(self.send_state(client, ws, shared, holder)
                  for client, ws in list(self.clients.items())),
                return_exceptions=True)

    async def send_state(self, client, ws, shared, holder):
        driver = "none" if holder is None else "you" if holder == client else "other"
        # A slow phone must not hold up everyone else's telemetry
        await asyncio.wait_for(ws.send_str(json.dumps({**shared, "driver": driver})), 1.0)

    # Modes --------------------------------------------------------------------------

    def follow_status(self, snapshot):
        follow = self.follow
        if self.mode == DRIVE or self.mode_status in ("stopping", "failed") \
                or follow is None or follow.process is None:
            return self.mode_status, self.mode_detail
        if not follow.running:
            self.mode_status = "failed"
            self.mode_detail = follow.last_line \
                or f"The follow launch exited with code {follow.process.returncode}"
        elif snapshot["follower_ready"] and (
                self.mode != PERSON or snapshot["detector_ready"]):
            self.mode_status = "running"
        return self.mode_status, self.mode_detail

    async def change_mode(self, ws, client, mode):
        if mode not in MODES:
            return
        if not self.bridge.may_control(client):
            await self.notice(ws, NOT_YOUR_TURN)
            return
        async with self.mode_lock:
            # Choosing a failed mode again retries it
            if mode == self.mode and self.mode_status != "failed":
                return
            self.logger.info(f"Client {client} switched to {mode} mode")
            await self.stop_follow(self.bridge.stop_timeout)
            self.mode = mode
            self.mode_since = time.monotonic()
            self.hub.reset()
            self.bridge.set_mode(mode)
            self.mode_detail = ""
            if mode == DRIVE:
                self.mode_status = "ready"
                return
            use_sim_time = self.bridge.get_parameter("use_sim_time").value
            argv = [shutil.which("ros2") or "ros2", "launch", self.bridge.launch_package,
                    self.bridge.launch_files[mode],
                    f"use_sim_time:={'true' if use_sim_time else 'false'}"]
            self.follow = FollowProcess(argv, self.logger)
            self.mode_status = "starting"
            try:
                await self.follow.start()
            except OSError as error:
                self.mode_status = "failed"
                self.mode_detail = f"Could not start ros2 launch: {error}"

    async def stop_follow(self, timeout):
        if self.follow is None:
            return
        self.mode_status = "stopping"
        await self.follow.stop(timeout)
        self.follow = None

    async def person_action(self, ws, client, action):
        services = {"enroll": self.bridge.enroll_client, "forget": self.bridge.forget_client}
        if action not in services:
            return
        result = {"type": "result", "action": action, "success": False}
        if self.mode != PERSON:
            result["message"] = "Switch to person mode first"
        elif not self.bridge.may_control(client):
            result["message"] = NOT_YOUR_TURN
        elif not services[action].service_is_ready():
            result["message"] = "The person detector is not running yet"
        else:
            ros_future = services[action].call_async(Trigger.Request())
            try:
                response = await asyncio.wait_for(asyncio_future(ros_future), 5.0)
                result["success"] = response.success
                result["message"] = response.message
            except asyncio.TimeoutError:
                services[action].remove_pending_request(ros_future)
                result["message"] = "The person detector did not answer"
            self.bridge.get_logger().info(
                f"Client {client} asked to {action}: {result['message']}")
        try:
            await ws.send_json(result)
        except ConnectionError:
            pass


def asyncio_future(ros_future):
    """Wrap an rclpy future, completed on the ROS thread, for awaiting on asyncio."""
    loop = asyncio.get_running_loop()
    future = loop.create_future()

    def transfer(done):
        if future.done():
            return
        if done.exception() is not None:
            future.set_exception(done.exception())
        else:
            future.set_result(done.result())

    ros_future.add_done_callback(lambda done: loop.call_soon_threadsafe(transfer, done))
    return future


async def serve(bridge, stopped):
    loop = asyncio.get_running_loop()
    server = WebServer(bridge)
    terminated = False

    def on_signal(signum):
        nonlocal terminated
        if signum == signal.SIGTERM:
            # ros2 launch escalates to SIGTERM when shutdown takes too long: stop waiting
            # for the follow launch to exit cleanly
            terminated = True
            server.kill_follow()
        stopped.set()

    # These handlers also make the children start with default signal handling, even if
    # this process was started with SIGINT ignored (a background job of a script)
    for signum in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(signum, on_signal, signum)

    try:
        await server.start()
    except OSError as error:
        bridge.get_logger().fatal(
            f"Cannot listen on {bridge.host}:{bridge.port}: {error}")
        raise SystemExit(1)
    try:
        await stopped.wait()
    finally:
        # ros2 launch sends SIGTERM 5 s after SIGINT, so finish well before that
        await server.shutdown(0.0 if terminated else 3.5)
        server.kill_follow()


def main():
    # rclpy's SIGINT handler would shut the context down under the web server. asyncio
    # takes the signals instead and shuts everything down in order
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    bridge = RobotBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    loop = asyncio.new_event_loop()
    stopped = asyncio.Event()

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass
        except Exception as error:  # noqa: BLE001 - reported, then the server stops
            bridge.get_logger().fatal(f"ROS executor failed: {error!r}")
            raise
        finally:
            # Stop the web server too if ROS went away first
            try:
                loop.call_soon_threadsafe(stopped.set)
            except RuntimeError:
                pass  # the loop already finished

    spin_thread = threading.Thread(target=spin, name="ros", daemon=True)
    spin_thread.start()
    try:
        loop.run_until_complete(serve(bridge, stopped))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()
        executor.shutdown(timeout_sec=2.0)
        # A spin thread still inside rclpy while the interpreter exits aborts the process
        # ("terminate called without an active exception")
        spin_thread.join(timeout=2.0)
        bridge.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
