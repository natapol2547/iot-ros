"""The MJPEG stream: shared encodes, quality changes, and slow viewers without lag.

These run the real WebServer and FrameHub on a local port, with a fake ROS side that
publishes numbered frames. Each frame carries its number as a row of black and white
blocks, so a client can tell from the picture how old the frame it sees is.
"""

import asyncio
import json
import logging
import os
import socket
import time
from unittest import mock

import aiohttp
import cv2
import numpy as np
from aiohttp import web
from cv_bridge import CvBridge

from iot_robot_web.logic import OFF, video_profile
from iot_robot_web import web_controller
from iot_robot_web.web_controller import (
    MAX_STREAMS_PER_HOST, PART, RobotBridge, WebServer, backlog_limit, path_capacity)

# The package source, so the tests also run before `colcon build` has installed the page
PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

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


class Dechunker:
    """Undo HTTP/1.1 chunked transfer encoding, which aiohttp uses for the stream."""

    def __init__(self):
        self.raw = b""
        self.headers_done = False

    def feed(self, data):
        """Add received bytes; returns the body bytes that are now complete."""
        self.raw += data
        if not self.headers_done:
            end = self.raw.find(b"\r\n\r\n")
            if end < 0:
                return b""
            self.raw = self.raw[end + 4:]
            self.headers_done = True
        body = b""
        while True:
            line = self.raw.find(b"\r\n")
            if line < 0:
                return body
            size = int(self.raw[:line], 16)
            if len(self.raw) < line + 2 + size + 2:
                return body
            body += self.raw[line + 2:line + 2 + size]
            self.raw = self.raw[line + 2 + size + 2:]


def split_parts(body):
    """Complete JPEG parts in a multipart body, and the rest to keep for later.

    A part is complete once the next part's header has arrived, as in a browser.
    """
    parts = []
    while True:
        start = body.find(PART)
        if start < 0:
            return parts, body
        end = body.find(b"\r\n" + PART, start + len(PART))
        if end < 0:
            return parts, body[start:]
        parts.append(body[start + len(PART):end])
        body = body[end + 2:]


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
        chunks = Dechunker()
        body = b""
        try:
            while True:
                data = await loop.sock_recv(sock, 2048 if self.rate else 65536)
                if not data:
                    return
                if self.rate:
                    await asyncio.sleep(len(data) / self.rate)
                parts, body = split_parts(body + chunks.feed(data))
                for jpeg in parts:
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
    with mock.patch.object(web_controller, "get_package_share_directory",
                           lambda package: PACKAGE):
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
        assert 2 * fps * 0.6 <= len(frames[quality]) <= 2 * fps + 2


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
    # Auto starts at medium (480 px) and settles on low (320 px). The lag, mostly this
    # reader's own receive buffer, stays bounded instead of growing
    assert first_width == 480
    assert late_widths == {320}
    assert max(late_lag) < 2.0
    assert late_lag[-1] < late_lag[0] + 0.5


def test_the_page_switches_quality_without_reconnecting():
    async def test(bridge, server, port):
        async with aiohttp.ClientSession() as session:
            async with session.ws_connect(f"http://127.0.0.1:{port}/ws") as ws, \
                    session.ws_connect(f"http://127.0.0.1:{port}/ws") as other_ws:
                hello = await ws.receive_json()
                other_hello = await other_ws.receive_json()
                viewer = Viewer(port, f"quality=high&client={hello['id']}")
                # Another page's stream, open before the switch: not this page's to change
                other = Viewer(port, f"quality=high&client={other_hello['id']}")
                viewer.start()
                other.start()
                await asyncio.sleep(1.0)
                before = server.hub.stats(hello["id"])
                await ws.send_str(json.dumps({"type": "video", "quality": "low"}))
                await asyncio.sleep(0.5)
                start = time.monotonic()
                await asyncio.sleep(1.0)
                after = server.hub.stats(hello["id"])
                other_after = server.hub.stats(other_hello["id"])
                await viewer.stop()
                await other.stop()
                return (hello, before, after, other_after, viewer.since(start),
                        other.since(start))

    hello, before, after, other_after, frames, other = asyncio.run(serve(test))
    assert [q["name"] for q in hello["video"]["qualities"]] == ["high", "medium", "low"]
    assert hello["video"]["default"] == "auto"
    assert before["quality"] == "high" and before["width"] == 640
    assert after["quality"] == "low" and after["width"] == 320 and after["height"] == 240
    assert after["fps"] > 0 and after["kbps"] > 0
    assert {frame[2] for frame in frames} == {320}
    assert other_after["quality"] == "high"
    assert other and {frame[2] for frame in other} == {640}


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


def test_each_frame_is_complete_without_waiting_for_the_next():
    async def test(bridge, server, port):
        # One frame a second: a frame that waited for the next one would arrive 1 s late
        viewer = Viewer(port, "quality=low")
        viewer.start()
        await asyncio.sleep(3.5)
        await viewer.stop()
        return lag(bridge, viewer.frames)

    ages = asyncio.run(serve(test, video_profiles=[
        video_profile("high", 640, 70, 15.0), video_profile("medium", 480, 60, 10.0),
        video_profile("low", 320, 50, 1.0)]))
    assert len(ages) >= 3
    assert max(ages) < 0.3


def test_a_new_stream_of_a_page_replaces_its_old_one():
    async def test(bridge, server, port):
        async with aiohttp.ClientSession() as session:
            async with session.ws_connect(f"http://127.0.0.1:{port}/ws") as ws:
                hello = await ws.receive_json()
                first = Viewer(port, f"quality=low&client={hello['id']}")
                first.start()
                await asyncio.sleep(1.0)
                second = Viewer(port, f"quality=low&client={hello['id']}")
                second.start()
                await asyncio.sleep(1.0)
                done = first.task.done()
                viewers = len(server.hub.viewers)
                await first.stop()
                await second.stop()
                return done, viewers, len(second.frames)

    first_closed, viewers, second_frames = asyncio.run(serve(test))
    assert first_closed
    assert viewers == 1
    assert second_frames >= 5


def test_a_malformed_video_message_keeps_the_socket_open():
    async def test(bridge, server, port):
        async with aiohttp.ClientSession() as session:
            async with session.ws_connect(f"http://127.0.0.1:{port}/ws") as ws:
                hello = await ws.receive_json()
                viewer = Viewer(port, f"quality=high&client={hello['id']}")
                viewer.start()
                await asyncio.sleep(0.5)
                # The same socket carries the E-stop: one bad message must not close it
                for quality in (["x"], {"a": 1}, 7, None, "ultra"):
                    await ws.send_str(json.dumps({"type": "video", "quality": quality}))
                await ws.send_str(json.dumps({"type": "video", "quality": "low"}))
                await asyncio.sleep(0.5)
                stats = server.hub.stats(hello["id"])
                await viewer.stop()
                return ws.closed, stats

    closed, stats = asyncio.run(serve(test))
    assert not closed
    assert stats["quality"] == "low"


def test_a_stalled_viewer_is_dropped_and_aborted(monkeypatch):
    monkeypatch.setattr(web_controller, "STALLED_VIEWER", 1.0)

    async def test(bridge, server, port):
        loop = asyncio.get_running_loop()
        sock = socket.socket()
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
        sock.setblocking(False)
        await loop.sock_connect(sock, ("127.0.0.1", port))
        await loop.sock_sendall(
            sock, b"GET /stream.mjpg?quality=high HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\n")
        await asyncio.sleep(0.5)
        watching = len(server.hub.viewers)
        # Never read: the backlog stops moving and the stream gives up after 1 s
        await asyncio.sleep(2.5)
        left = len(server.hub.viewers)
        # The connection was aborted rather than ended politely, so a browser retries
        received = b""
        reset = False
        try:
            while True:
                data = await asyncio.wait_for(loop.sock_recv(sock, 65536), 1.0)
                if not data:
                    break
                received += data
        except ConnectionResetError:
            reset = True
        sock.close()
        return watching, left, reset or not received.endswith(b"0\r\n\r\n")

    watching, left, aborted = asyncio.run(serve(test))
    assert watching == 1
    assert left == 0
    assert aborted


def test_streams_per_computer_are_limited():
    async def test(bridge, server, port):
        viewers = [Viewer(port, "quality=low") for _ in range(MAX_STREAMS_PER_HOST)]
        for viewer in viewers:
            viewer.start()
        await asyncio.sleep(0.5)
        async with aiohttp.ClientSession() as session:
            async with session.get(f"http://127.0.0.1:{port}/stream.mjpg") as response:
                status = response.status
        for viewer in viewers:
            await viewer.stop()
        return status, [len(viewer.frames) for viewer in viewers]

    status, received = asyncio.run(serve(test))
    assert status == 503
    assert min(received) >= 1


def test_the_backlog_limit_follows_the_path():
    async def test(bridge, server, port):
        loop = asyncio.get_running_loop()
        received = []

        async def handler(reader, writer):
            # Send 2 MB, so the kernel has a delivery rate and a round trip to report
            for _ in range(40):
                writer.write(b"x" * 50000)
                await writer.drain()
            received.append((path_capacity(writer.transport),
                             backlog_limit(writer.transport, 16384)))
            writer.close()

        tcp = await asyncio.start_server(handler, "127.0.0.1", 0)
        reader, writer = await asyncio.open_connection(
            "127.0.0.1", tcp.sockets[0].getsockname()[1])
        await reader.read()
        writer.close()
        tcp.close()
        await loop.shutdown_default_executor()
        return received[0]

    capacity, limit = asyncio.run(serve(test))
    # Loopback: a tiny round trip, so the plain limit applies; never more than 4 times it
    assert capacity >= 0
    assert 16384 <= limit <= 4 * 16384


def test_video_default_off_from_yaml_is_accepted():
    import rclpy

    # A bare `off`, in YAML or on the command line, arrives as the boolean false. A string
    # parameter would refuse it and stop the node, and with it the page's E-stop
    rclpy.init(args=["--ros-args", "-p", "video_default:=off"])
    try:
        bridge = RobotBridge()
        try:
            assert bridge.video_default == OFF
        finally:
            bridge.destroy_node()
    finally:
        rclpy.shutdown()
