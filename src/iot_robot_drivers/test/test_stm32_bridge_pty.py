"""Run stm32_bridge against fake_stm32 over a real pseudo-terminal."""

import math
import os
import subprocess
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import BatteryState, JointState, Range  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

from iot_robot_drivers.stm32_bridge import Stm32Bridge  # noqa: E402

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def package_env():
    """Return the environment with this source tree first on PYTHONPATH, for subprocesses."""
    env = dict(os.environ)
    env["PYTHONPATH"] = PACKAGE_ROOT + os.pathsep + env.get("PYTHONPATH", "")
    return env


class FakeStm32:
    """fake_stm32 in a subprocess, collecting the lines it receives from the bridge."""

    def __init__(self, link, *args):
        self.process = subprocess.Popen(
            [sys.executable, "-u", "-m", "iot_robot_drivers.fake_stm32",
             "--link", link, "--battery", *args],
            stdout=subprocess.PIPE, text=True, env=package_env())
        self.lines = []
        threading.Thread(target=self.collect, daemon=True).start()
        wait_for(lambda: os.path.islink(link), 5.0, "fake_stm32 did not create its link")

    def collect(self):
        for line in self.process.stdout:
            self.lines.append(line.strip())

    def received(self):
        """Lines the bridge wrote to the board, which accepts no commands."""
        return [line for line in self.lines if line.startswith("received:")]

    def stop(self):
        self.process.terminate()
        self.process.wait(timeout=5.0)


def wait_for(condition, timeout, message, spin=None):
    deadline = time.monotonic() + timeout
    while not condition():
        if time.monotonic() > deadline:
            pytest.fail(message)
        if spin:
            spin()
        else:
            time.sleep(0.05)


@pytest.fixture
def fake_args():
    """Extra fake_stm32 arguments; a test overrides this with pytest.mark.parametrize."""
    return []


@pytest.fixture
def gizmo_mode():
    """The bridge's gizmo_mode; a test overrides this with pytest.mark.parametrize."""
    return "fixed"


@pytest.fixture
def ros(tmp_path, fake_args, gizmo_mode):
    link = str(tmp_path / "stm32")
    fake = FakeStm32(link, *fake_args)
    context = rclpy.context.Context()
    rclpy.init(context=context)
    bridge = Stm32Bridge(context=context, parameter_overrides=[
        Parameter("port", value=link),
        Parameter("gizmo_mode", value=gizmo_mode),
        Parameter("reconnect_period", value=0.2),
    ])
    probe = rclpy.create_node("probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(bridge)
    executor.add_node(probe)
    yield {"fake": fake, "link": link, "bridge": bridge, "probe": probe,
           "spin": lambda: executor.spin_once(timeout_sec=0.05)}
    executor.shutdown()
    bridge.destroy_node()
    probe.destroy_node()
    rclpy.try_shutdown(context=context)
    fake.stop()


def test_bridge_publishes_ranges_and_battery(ros):
    probe, spin, fake = ros["probe"], ros["spin"], ros["fake"]
    ranges = {"left": [], "right": []}
    batteries = []
    for side in ranges:
        probe.create_subscription(
            Range, f"/ultrasonic/{side}", lambda msg, side=side: ranges[side].append(msg), 10)
    probe.create_subscription(BatteryState, "/battery_state", batteries.append, 10)

    # The fake drops the right echo for 0.5 s every 3 s
    wait_for(lambda: any(math.isinf(m.range) for m in ranges["right"])
             and len(ranges["left"]) >= 5 and batteries, 10.0,
             "no ranges or battery state from the bridge", spin)

    left = ranges["left"][-1]
    assert left.header.frame_id == "ultrasonic_left_link"
    assert left.radiation_type == Range.ULTRASOUND
    assert left.field_of_view == pytest.approx(0.26)
    assert (left.min_range, left.max_range) == (pytest.approx(0.02), pytest.approx(4.0))
    # The fake's left sensor moves between 20 and 100 cm: metres, not centimetres
    assert all(0.19 <= m.range <= 1.01 for m in ranges["left"])
    assert ranges["right"][-1].header.frame_id == "ultrasonic_right_link"
    assert 21.0 <= batteries[-1].voltage <= 25.0

    # The board only reads sensors: the bridge never writes to it
    assert fake.received() == []


def test_bridge_reconnects_after_the_board_disappears(ros):
    probe, spin, link = ros["probe"], ros["spin"], ros["link"]
    left = []
    probe.create_subscription(Range, "/ultrasonic/left", left.append, 10)
    wait_for(lambda: len(left) >= 3, 10.0, "no data before the unplug", spin)

    ros["fake"].stop()
    time.sleep(0.5)
    for _ in range(10):
        spin()
    left.clear()

    replacement = FakeStm32(link)
    try:
        wait_for(lambda: len(left) >= 3, 10.0, "bridge did not reconnect", spin)
        assert replacement.received() == []
    finally:
        replacement.stop()


@pytest.mark.parametrize("fake_args", [["--fault", "left"]])
def test_reported_sensor_fault_publishes_nan_until_recovery(ros):
    probe, spin, bridge = ros["probe"], ros["spin"], ros["bridge"]
    ranges = {"left": [], "right": []}
    for side in ranges:
        probe.create_subscription(
            Range, f"/ultrasonic/{side}", lambda msg, side=side: ranges[side].append(msg), 10)

    # The fake warns after ~1 s and every ~10 s after that, like the firmware; the
    # timeout covers a bridge that connects after the first warning was sent
    wait_for(lambda: ranges["left"] and math.isnan(ranges["left"][-1].range), 15.0,
             "the firmware warning did not turn the faulty side into NaN", spin)
    assert not any(math.isnan(m.range) for m in ranges["right"])

    # The recovery line ends the fault; the fake still sends -1.0, which is +inf again
    bridge.handle_line("# info: left sensor recovered\r\n")
    ranges["left"].clear()
    wait_for(lambda: len(ranges["left"]) >= 3, 5.0, "no readings after recovery", spin)
    assert all(math.isinf(m.range) for m in ranges["left"])

    # So does a reset of the board, announced by its banner
    bridge.handle_line("# warning: left sensor not responding\r\n")
    bridge.handle_line("# iot-stm32 1.2.0\r\n")
    ranges["left"].clear()
    wait_for(lambda: len(ranges["left"]) >= 3, 5.0, "no readings after the banner", spin)
    assert all(math.isinf(m.range) for m in ranges["left"])


def spin_for(spin, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        spin()


@pytest.mark.parametrize("gizmo_mode", ["can"])
def test_can_mode_leaves_the_gizmo_to_ros2_control(ros):
    probe, spin, fake = ros["probe"], ros["spin"], ros["fake"]
    ranges, joints = [], []
    probe.create_subscription(Range, "/ultrasonic/left", ranges.append, 10)
    probe.create_subscription(JointState, "/joint_states", joints.append, 10)
    commands = probe.create_publisher(Float64MultiArray, "/gizmo_controller/commands", 10)
    wait_for(lambda: len(ranges) >= 3, 10.0, "no ranges in can mode", spin)

    # joint_state_broadcaster owns the gizmo joints and gizmo_controller their commands
    assert probe.count_publishers("/joint_states") == 0
    assert probe.count_subscribers("/gizmo_controller/commands") == 0
    commands.publish(Float64MultiArray(data=[0.5, -0.3]))
    spin_for(spin, 0.5)
    assert joints == []
    assert fake.received() == []


@pytest.mark.parametrize("gizmo_mode", ["fixed"])
def test_fixed_mode_publishes_a_constant_pose_and_ignores_commands(ros):
    probe, spin, fake = ros["probe"], ros["spin"], ros["fake"]
    joints = []
    probe.create_subscription(JointState, "/joint_states", joints.append, 10)
    commands = probe.create_publisher(Float64MultiArray, "/gizmo_controller/commands", 10)
    wait_for(lambda: len(joints) >= 3, 10.0, "no gizmo joint states in fixed mode", spin)

    commands.publish(Float64MultiArray(data=[0.5, -0.3]))
    spin_for(spin, 0.5)
    assert list(joints[-1].name) == ["gizmo_yaw_joint", "gizmo_pitch_joint"]
    assert list(joints[-1].position) == [0.0, 0.0]
    assert fake.received() == []


def test_gizmo_mode_servo_is_rejected(tmp_path):
    # The Nucleo drives no motors, so there is no mode in which the bridge commands the
    # gizmo; the node must refuse to start rather than run in some other mode
    result = subprocess.run(
        [sys.executable, "-m", "iot_robot_drivers.stm32_bridge", "--ros-args",
         "-p", "gizmo_mode:=servo", "-p", f"port:={tmp_path / 'stm32'}"],
        capture_output=True, text=True, timeout=30.0, env=package_env())
    output = result.stdout + result.stderr
    assert result.returncode == 1, output
    assert "gizmo_mode must be can" in output
    assert "or fixed (the gizmo is not driven), got 'servo'" in output
