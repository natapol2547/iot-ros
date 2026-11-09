"""The MJPEG stream: shared encodes, quality changes, and slow viewers without lag.

These run the real WebServer and FrameHub on a local port, with a fake ROS side that
publishes numbered frames. Each frame carries its number as a row of black and white
blocks, so a client can tell from the picture how old the frame it sees is.
"""

import asyncio
import json
import logging
import re
import socket
import time

import aiohttp
import cv2
import numpy as np
from aiohttp import web
from cv_bridge import CvBridge

from iot_robot_web.logic import video_profile
from iot_robot_web.web_controller import WebServer

BITS = 16
BLOCK = 40  # px per bit at 640 wide, still 20 px at the lowest quality


class FakeBridge:
    """The parts of RobotBridge that WebServer and FrameHub use."""

    max_linear, max_angular, turbo_speed = 0.4, 1.5, 3.0
    warn_distance, danger_distance, stop_distance = 1.0, 0.4, 0.25
    yaw_limit, pitch_min, pitch_max = 0.785, -0.785, 0.0
    allowed_hosts = []

    def __init__(self):
        self.video_profiles = [
            video_profile("high", 640, 70, 15.0),
            video_profile("medium", 480, 60, 10.0),
            video_profile("low", 320, 50, 8.0),
        ]
        self.video_default = "auto"
        self.video_backlog = 16384
        self.image_listener = None
        self.streaming = False
        self.msg = None
        self.seq = 0
        self.published = {}  # frame number -> time.monotonic() when published
        self.cv_bridge = CvBridge()
        # Compresses like a camera picture: about 25 KB at 640 x 480, 14 KB at 480 x 360
        rng = np.random.default_rng(0)
        y, x = np.mgrid[0:480, 0:640]
        gradient = np.dstack([(x * 0.35) % 255, (y * 0.5) % 255, ((x + y) * 0.2) % 255])
        noise = cv2.GaussianBlur(
            rng.integers(0, 255, (480, 640, 3), dtype=np.uint8), (0, 0), 1.2)
        self.background = cv2.addWeighted(gradient.astype(np.uint8), 0.6, noise, 0.4, 0)

    def get_logger(self):
        return logging.getLogger("test_video")

    def latest_image(self):
        return self.msg, self.seq

    def set_streaming(self, streaming):
        self.streaming = streaming

    def release(self, client, disconnected=False):
        pass

    def publish(self):
        """Publish the next numbered frame, like the camera would."""
        self.seq += 1
        frame = self.background.copy()
        for bit in range(BITS):
            on = (self.seq >> (BITS - 1 - bit)) & 1
            frame[:48, bit * BLOCK:(bit + 1) * BLOCK] = 255 if on else 0
        self.published[self.seq] = time.monotonic()
        self.msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        if self.image_listener is not None:
            self.image_listener()


def frame_number(jpeg):
    """The frame number in a JPEG part, and its width."""
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
    height, width = image.shape
    image = cv2.resize(image, (640, round(height * 640 / width)))
    number = 0
    for bit in range(BITS):
        number = (number << 1) | int(image[24, bit * BLOCK + BLOCK // 2] > 128)
    return number, width


class Viewer:
    """An MJPEG client that reads at most `rate` bytes a second (None: as fast as it can)."""

    def __init__(self, port, query, rate=None):
        self.port, self.query, self.rate = port, query, rate
        self.frames = []  # (time received, frame number, width, size)
        self.task = None

    async def run(self):
        loop = asyncio.get_running_loop()
        sock = socket.socket()
        # A small receive window, so a slow reader really holds the server back
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16384)
        sock.setblocking(False)
        await loop.sock_connect(sock, ("127.0.0.1", self.port))
        await loop.sock_sendall(
            sock, f"GET /stream.mjpg?{self.query} HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\n".encode())
        data = b""
        try:
            while True:
                chunk = await loop.sock_recv(sock, 2048 if self.rate else 65536)
                if not chunk:
                    return
                if self.rate:
                    await asyncio.sleep(len(chunk) / self.rate)
                data += chunk
                while True:
                    match = re.search(rb"Content-Length: (\d+)\r\n\r\n", data)
                    if not match or len(data) < match.end() + int(match.group(1)):
                        break
                    jpeg = data[match.end():match.end() + int(match.group(1))]
                    data = data[match.end() + len(jpeg):]
                    number, width = frame_number(jpeg)
                    self.frames.append((time.monotonic(), number, width, len(jpeg)))
        finally:
            sock.close()

    def start(self):
        self.task = asyncio.create_task(self.run())

    async def stop(self):
        self.task.cancel()
        await asyncio.gather(self.task, return_exceptions=True)

    def since(self, start):
        return [frame for frame in self.frames if frame[0] >= start]


async def serve(test, **settings):
    """Run `test(bridge, server, port)` against a server whose camera runs at 30 Hz."""
    bridge = FakeBridge()
    for name, value in settings.items():
        setattr(bridge, name, value)
    server = WebServer(bridge)
    runner = web.AppRunner(server.app)
    await runner.setup()
    site = web.TCPSite(runner, "127.0.0.1", 0)
    await site.start()
    port = site._server.sockets[0].getsockname()[1]

    async def camera():
        while True:
            bridge.publish()
            await asyncio.sleep(1 / 30)

    camera_task = asyncio.create_task(camera())
    try:
        return await test(bridge, server, port)
    finally:
        camera_task.cancel()
        await server.hub.close()
        await runner.cleanup()


def lag(bridge, frames):
    """How old each frame was when it arrived (s)."""
    return [received - bridge.published[number] for received, number, _, _ in frames]


def test_each_quality_has_its_size_and_frame_rate():
    async def test(bridge, server, port):
        viewers = {q: Viewer(port, f"quality={q}") for q in ("high", "medium", "low")}
        for viewer in viewers.values():
            viewer.start()
        await asyncio.sleep(1.0)
        start = time.monotonic()
        await asyncio.sleep(2.0)
        for viewer in viewers.values():
            await viewer.stop()
        return {q: viewer.since(start) for q, viewer in viewers.items()}

    frames = asyncio.run(serve(test))
    for quality, width, fps in (("high", 640, 15), ("medium", 480, 10), ("low", 320, 8)):
        assert {frame[2] for frame in frames[quality]} == {width}
        assert 2 * fps * 0.8 <= len(frames[quality]) <= 2 * fps + 2


def test_viewers_of_one_quality_share_the_encoding():
    async def test(bridge, server, port):
        viewers = [Viewer(port, "quality=medium") for _ in range(4)]
        for viewer in viewers:
            viewer.start()
        await asyncio.sleep(2.0)
        for viewer in viewers:
            await viewer.stop()
        return server.hub.encodes, [len(viewer.frames) for viewer in viewers]

    encodes, received = asyncio.run(serve(test))
    # Four viewers at 10 fps, but each frame was encoded once for all of them
    assert min(received) >= 15
    assert encodes <= max(received) + 4


def test_a_slow_viewer_gets_recent_frames_not_a_backlog():
    async def test(bridge, server, port):
        slow = Viewer(port, "quality=medium", rate=40_000)  # about 0.3 Mb/s
        fast = Viewer(port, "quality=medium")
        slow.start()
        fast.start()
        await asyncio.sleep(3.0)
        start = time.monotonic()
        await asyncio.sleep(3.0)
        await slow.stop()
        await fast.stop()
        return lag(bridge, slow.since(start)), lag(bridge, fast.since(start))

    slow, fast = asyncio.run(serve(test))
    # The slow viewer skips frames, so what it sees stays recent: about a second here, most
    # of it this reader's own receive buffer. Without the backlog limit the lag grew by
    # several seconds every second, past 15 s by now
    assert len(slow) >= 3
    assert max(slow) < 2.0
    assert slow[-1] < slow[0] + 0.5
    # ... and it does not slow down anyone else
    assert len(fast) >= 25
    assert max(fast) < 0.3


def test_auto_steps_down_on_a_slow_connection():
    async def test(bridge, server, port):
        viewer = Viewer(port, "quality=auto", rate=40_000)
        viewer.start()
        await asyncio.sleep(5.0)
        start = time.monotonic()
        await asyncio.sleep(2.0)
        await viewer.stop()
        late = viewer.since(start)
        return viewer.frames[0][2], {frame[2] for frame in late}, lag(bridge, late)

    first_width, late_widths, late_lag = asyncio.run(serve(test))
    # Auto starts at medium (480 px) and settles on low (320 px)
    assert first_width == 480
    assert late_widths == {320}
    assert max(late_lag) < 1.0


def test_the_page_switches_quality_without_reconnecting():
    async def test(bridge, server, port):
        async with aiohttp.ClientSession() as session:
            async with session.ws_connect(f"http://127.0.0.1:{port}/ws") as ws:
                hello = await ws.receive_json()
                viewer = Viewer(port, f"quality=high&client={hello['id']}")
                viewer.start()
                await asyncio.sleep(1.0)
                before = server.hub.stats(hello["id"])
                await ws.send_str(json.dumps({"type": "video", "quality": "low"}))
                await asyncio.sleep(0.5)
                start = time.monotonic()
                await asyncio.sleep(1.0)
                after = server.hub.stats(hello["id"])
                # Someone else's stream is not theirs to change
                other = Viewer(port, "quality=high")
                other.start()
                await asyncio.sleep(1.0)
                await viewer.stop()
                await other.stop()
                return hello, before, after, viewer.since(start), other.frames

    hello, before, after, frames, other = asyncio.run(serve(test))
    assert [q["name"] for q in hello["video"]["qualities"]] == ["high", "medium", "low"]
    assert hello["video"]["default"] == "auto"
    assert before["quality"] == "high" and before["width"] == 640
    assert after["quality"] == "low" and after["width"] == 320 and after["height"] == 240
    assert after["fps"] > 0 and after["kbps"] > 0
    assert {frame[2] for frame in frames} == {320}
    assert {frame[2] for frame in other} == {640}


def test_nobody_watching_unsubscribes_the_camera():
    async def test(bridge, server, port):
        viewer = Viewer(port, "quality=low")
        viewer.start()
        await asyncio.sleep(0.5)
        watching = bridge.streaming
        await viewer.stop()
        await asyncio.sleep(1.5)
        return watching, bridge.streaming

    watching, after = asyncio.run(serve(test))
    assert watching and not after
