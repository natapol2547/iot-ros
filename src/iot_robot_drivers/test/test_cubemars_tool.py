"""cubemars_tool: check, watch, identify, jog and stop against fake motors on vcan.

The bus tests need a vcan interface; see test_cubemars.py for how to run them without
root. The rest runs anywhere.
"""

import os
import socket
import struct
import threading
import time

import pytest

from iot_robot_drivers import cubemars_tool
from iot_robot_drivers.cubemars import MODE_CURRENT, MODE_SPEED, STATUS, CanBus

MOTORS = """\
# Motors for the tool tests
motors:
  wheel_joint_left:
    can_id: 12   # left wheel
    direction: 1
  wheel_joint_right:
    can_id: 13
    direction: -1
  gizmo_yaw_joint:
    can_id: 10
    direction: 1
    zero_on_start: true
  gizmo_pitch_joint:
    can_id: 11
    direction: 1
    zero_on_start: true
"""


@pytest.fixture
def motors_file(tmp_path):
    path = tmp_path / "motors.yaml"
    path.write_text(MOTORS)
    return path


def answers(*replies):
    """An ask() for identify that returns replies in turn, then acts like Ctrl-D."""
    remaining = list(replies)
    prompts = []

    def ask(prompt):
        prompts.append(prompt)
        if not remaining:
            raise EOFError
        return remaining.pop(0)
    ask.prompts = prompts
    return ask


class TestWithoutBus:
    def test_ask_joint_accepts_numbers_and_names(self, capsys):
        joints = ["wheel_joint_left", "wheel_joint_right"]
        assert cubemars_tool.ask_joint(answers("2"), 12, joints, {}) == "wheel_joint_right"
        assert cubemars_tool.ask_joint(
            answers("wheel_joint_left"), 12, joints, {}) == "wheel_joint_left"
        assert cubemars_tool.ask_joint(answers("r"), 12, joints, {}) == "repeat"
        assert cubemars_tool.ask_joint(answers("s"), 12, joints, {}) == "skip"
        assert cubemars_tool.ask_joint(answers("q"), 12, joints, {}) == "quit"
        assert cubemars_tool.ask_joint(answers(), 12, joints, {}) == "quit"

    def test_ask_joint_asks_again_after_a_bad_answer(self, capsys):
        joints = ["wheel_joint_left", "wheel_joint_right"]
        ask = answers("7", "arm", "", "1")
        assert cubemars_tool.ask_joint(ask, 12, joints, {}) == "wheel_joint_left"
        assert len(ask.prompts) == 4
        assert "Answer with a number from 1 to 2" in capsys.readouterr().out

    def test_ask_joint_refuses_a_joint_that_already_has_a_motor(self, capsys):
        joints = ["wheel_joint_left", "wheel_joint_right"]
        ask = answers("1", "2")
        assigned = {"wheel_joint_left": 12}
        assert cubemars_tool.ask_joint(ask, 13, joints, assigned) == "wheel_joint_right"
        assert "wheel_joint_left is already CAN ID 12" in capsys.readouterr().out

    def test_stop_succeeds_without_link_or_motors_file(self, capsys):
        assert cubemars_tool.main(["--interface", "nocan42", "--motors", "/nonexistent",
                                   "stop"]) == 0

    def test_jog_refuses_a_fast_velocity_before_touching_the_bus(self, motors_file, capsys):
        assert cubemars_tool.main([
            "--interface", "nocan42", "--motors", str(motors_file), "jog",
            "--joint", "wheel_joint_left", "--velocity", "5"]) == 1
        assert "Refusing 5.0 rad/s" in capsys.readouterr().err

    def test_check_reports_a_missing_interface(self, motors_file, capsys):
        assert cubemars_tool.main([
            "--interface", "nocan42", "--motors", str(motors_file), "check"]) == 1
        assert "does not exist" in capsys.readouterr().err

    def test_check_reports_an_invalid_motors_file(self, tmp_path, capsys):
        path = tmp_path / "motors.yaml"
        path.write_text(MOTORS.replace("can_id: 13", "can_id: 12"))
        assert cubemars_tool.main(["--motors", str(path), "check"]) == 1
        assert "CAN ID 12 is given to both" in capsys.readouterr().err


CAN_INTERFACE = os.environ.get("IOT_TEST_CAN_INTERFACE")
needs_vcan = pytest.mark.skipif(
    not CAN_INTERFACE or not hasattr(socket, "AF_CAN"),
    reason="set IOT_TEST_CAN_INTERFACE to a vcan interface to run bus tests")


class FakeServos:
    """CubeMars motors in servo mode: status at 100 Hz, speed follows the last command.

    Records every speed and current command addressed to its IDs as
    (can_id, mode, value) with value in ERPM or mA. speed_scale multiplies the reported
    speed; a large negative value imitates a drive with a mismatched encoder, which runs
    away the wrong way.
    """

    def __init__(self, interface, ids, current=0.0, error=0, speed_scale=1.0):
        self.bus = CanBus(interface)
        self.ids = list(ids)
        self.current = dict.fromkeys(self.ids, current)
        self.error = error
        self.speed_scale = speed_scale
        self.speed = dict.fromkeys(self.ids, 0)
        self.commands = []
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
        time.sleep(0.05)

    def run(self):
        next_status = time.monotonic()
        while not self.stopping.is_set():
            frame = self.bus.recv(max(0.0, next_status - time.monotonic()))
            if frame is not None:
                arbitration_id, data = frame
                mode, can_id = arbitration_id >> 8, arbitration_id & 0xFF
                if can_id in self.ids and mode in (MODE_SPEED, MODE_CURRENT):
                    (value,) = struct.unpack(">i", bytes(data[:4]))
                    self.commands.append((can_id, mode, value))
                    self.speed[can_id] = value if mode == MODE_SPEED else 0
            if time.monotonic() >= next_status:
                for can_id in self.ids:
                    self.bus.send((STATUS << 8) | can_id, struct.pack(
                        ">hhhbB", 0, int(self.speed[can_id] * self.speed_scale / 10),
                        int(self.current[can_id] * 100), 30, self.error))
                next_status += 0.01

    def speeds(self, can_id):
        return [value for motor, mode, value in self.commands
                if motor == can_id and mode == MODE_SPEED]

    def released(self, can_id, timeout=1.0):
        """Wait up to timeout for a release addressed to can_id.

        The tool returns as soon as it has sent the frame, possibly before this thread
        has read it.
        """
        deadline = time.monotonic() + timeout
        while (can_id, MODE_CURRENT, 0) not in self.commands:
            if time.monotonic() > deadline:
                return False
            time.sleep(0.01)
        return True

    def stop(self):
        self.stopping.set()
        self.thread.join()
        self.bus.close()


@pytest.fixture
def servos():
    started = []

    def start(ids, **kwargs):
        fake = FakeServos(CAN_INTERFACE, ids, **kwargs)
        started.append(fake)
        return fake
    yield start
    for fake in started:
        fake.stop()


def tool(motors_file, *args, ask=None):
    argv = ["--interface", CAN_INTERFACE, "--motors", str(motors_file), *args]
    return cubemars_tool.main(argv, ask=ask) if ask else cubemars_tool.main(argv)


@needs_vcan
class TestCheck:
    def test_every_motor_reports(self, motors_file, servos, capsys):
        servos([10, 11, 12, 13])
        assert tool(motors_file, "check", "--timeout", "0.3") == 0
        out = capsys.readouterr().out
        for joint in ("wheel_joint_left", "wheel_joint_right", "gizmo_yaw_joint",
                      "gizmo_pitch_joint"):
            assert joint in out
        # Joint order (wheels, then gizmo), the order check walks the motors in
        assert f"OK: {CAN_INTERFACE} is up and motors 12, 13, 10, 11 respond" in out

    def test_flags_missing_and_unexpected_ids(self, motors_file, servos, capsys):
        servos([12, 13, 14])
        assert tool(motors_file, "check", "--timeout", "0.3") == 1
        captured = capsys.readouterr()

        def row(start):
            return next(line for line in captured.out.splitlines() if line.startswith(start))
        assert "MISSING" in row("gizmo_yaw_joint") and "MISSING" in row("gizmo_pitch_joint")
        assert "UNEXPECTED" in row("(not in motors.yaml)")
        assert "gizmo_yaw_joint (CAN ID 10), gizmo_pitch_joint (CAN ID 11)" in captured.err
        assert "CAN ID 14 (not in motors.yaml)" in captured.err
        assert "can-identify" in captured.err

    def test_gizmo_mode_fixed_checks_the_wheels_only(self, motors_file, servos, capsys):
        servos([12, 13, 10])
        assert tool(motors_file, "check", "--gizmo-mode", "fixed", "--timeout", "0.3") == 0
        out = capsys.readouterr().out
        assert "not used" in next(line for line in out.splitlines()
                                  if line.startswith("gizmo_yaw_joint"))

    def test_motor_fault_fails(self, motors_file, servos, capsys):
        servos([10, 11, 12, 13], error=4)
        assert tool(motors_file, "check", "--timeout", "0.3") == 1
        assert "wheel_joint_left (CAN ID 12): under-voltage" in capsys.readouterr().err


@needs_vcan
def test_watch_labels_motors_with_joint_names(motors_file, servos, capsys):
    servos([12, 14])
    assert tool(motors_file, "watch", "--count", "1", "--rate", "4") == 0
    rows = capsys.readouterr().out.splitlines()
    assert any(row.startswith("wheel_joint_left") and " ok " in row for row in rows)
    assert any(row.startswith("wheel_joint_right") and "silent" in row for row in rows)
    assert any(row.startswith("(not in motors.yaml)") and " 14 " in row for row in rows)


@needs_vcan
class TestStop:
    def collect(self, run):
        """Command frames on the bus while run() executes."""
        with CanBus(CAN_INTERFACE) as listener:
            result = run()
            frames = []
            # The fake keeps sending status, so drain the queue for a fixed time
            end = time.monotonic() + 0.2
            while time.monotonic() < end:
                frame = listener.recv(0.02)
                if frame and frame[0] >> 8 in (MODE_SPEED, MODE_CURRENT):
                    frames.append(frame[0])
        return result, frames

    def test_default_ids_are_motors_yaml_plus_every_motor_heard(self, motors_file, servos,
                                                                capsys):
        servos([20])
        result, frames = self.collect(lambda: tool(motors_file, "stop", "--brake-time",
                                                   "0.05"))
        assert result == 0
        # The motors.yaml IDs in file order, then the extra motor heard on the bus
        ids = [12, 13, 10, 11, 20]
        assert frames[:5] == [(MODE_SPEED << 8) | can_id for can_id in ids]
        assert frames[-5:] == [(MODE_CURRENT << 8) | can_id for can_id in ids]
        assert "Motors 12, 13, 10, 11, 20" in capsys.readouterr().out

    def test_without_motors_yaml_the_heard_motors_are_stopped(self, servos, capsys):
        servos([20])
        result, frames = self.collect(lambda: tool("/nonexistent/motors.yaml", "stop",
                                                   "--brake-time", "0.05"))
        assert result == 0
        assert frames[0] == (MODE_SPEED << 8) | 20
        assert frames[-1] == (MODE_CURRENT << 8) | 20
        assert "Continuing without motors.yaml" in capsys.readouterr().err


IDENTIFY_FAST = ["identify", "--degrees", "3", "--speed", "1", "--repeat", "1"]


@needs_vcan
class TestIdentify:
    def test_maps_every_motor_and_writes_only_the_ids(self, motors_file, servos, capsys):
        fake = servos([10, 11, 12, 13])
        # identify walks the IDs in ascending order: 10 and 11 are the gizmo, 12 and 13
        # the wheels. 13 is first given a joint that 12 already has, which is refused
        ask = answers("3", "4", "2", "2", "1")
        assert tool(motors_file, *IDENTIFY_FAST, "--write", ask=ask) == 0
        out = capsys.readouterr().out
        assert "wheel_joint_right is already CAN ID 12" in out
        assert "Wrote the new CAN IDs" in out
        expected = MOTORS.replace("can_id: 12   # left wheel", "can_id: X   # left wheel") \
            .replace("can_id: 13", "can_id: 12").replace("can_id: X", "can_id: 13")
        assert motors_file.read_text() == expected

        for can_id in (10, 11, 12, 13):
            speeds = fake.speeds(can_id)
            moving = [value for value in speeds if value]
            # Out, back, then zero speed and a release, and never faster than --speed
            assert moving[0] > 0 and moving[-1] < 0
            assert max(abs(value) for value in moving) <= 1.0 * 1336.91
            assert speeds[-1] == 0
            assert fake.released(can_id)

    def test_without_write_the_file_is_unchanged(self, motors_file, servos, capsys):
        servos([12, 13])
        ask = answers("2", "1")
        assert tool(motors_file, *IDENTIFY_FAST, ask=ask) == 0
        assert "Not written" in capsys.readouterr().out
        assert motors_file.read_text() == MOTORS

    def test_a_conflicting_partial_result_is_not_written(self, motors_file, servos, capsys):
        servos([12])
        # CAN ID 12 is identified as the right wheel, while motors.yaml keeps 12 for the
        # left wheel, which was not identified: two joints would share ID 12
        assert tool(motors_file, *IDENTIFY_FAST, "--write", ask=answers("2")) == 1
        assert "not written" in capsys.readouterr().err
        assert motors_file.read_text() == MOTORS

    def test_high_current_stops_the_move(self, motors_file, servos, capsys):
        fake = servos([10], current=5.0)
        assert tool(motors_file, *IDENTIFY_FAST, "--ids", "10", ask=answers("s")) == 0
        out = capsys.readouterr().out
        assert "Stopped CAN ID 10 early: it drew +5.00 A, more than --max-current" in out
        assert fake.released(10)

    def test_runaway_motor_is_released_without_braking(self, motors_file, servos, capsys):
        fake = servos([10], speed_scale=-30.0)
        assert tool(motors_file, *IDENTIFY_FAST, "--ids", "10", ask=answers("s")) == 0
        assert "Stopped CAN ID 10 early: it turned at" in capsys.readouterr().out
        assert fake.released(10)
        # A zero-speed command would drive a runaway motor on; the last speed command
        # is the move itself
        assert fake.speeds(10)[-1] != 0

    def test_refuses_while_another_program_commands_the_motors(self, motors_file, servos,
                                                               capsys):
        servos([10, 11])
        stopping = threading.Event()

        def robot():
            with CanBus(CAN_INTERFACE) as bus:
                while not stopping.wait(0.02):
                    bus.send((MODE_SPEED << 8) | 10, b"\x00\x00\x00\x00")
        thread = threading.Thread(target=robot)
        thread.start()
        try:
            assert tool(motors_file, *IDENTIFY_FAST, ask=answers()) == 1
        finally:
            stopping.set()
            thread.join()
        assert "Another program is sending commands to CAN ID(s) 10" in \
            capsys.readouterr().err


@needs_vcan
def test_jog_limits_gizmo_travel(motors_file, servos, capsys):
    fake = servos([10])
    assert tool(motors_file, "jog", "--joint", "gizmo_yaw_joint", "--velocity", "1.0",
                "--duration", "2", "--max-gizmo-travel", "6") == 0
    out = capsys.readouterr().out
    assert "gizmo_yaw_joint has hard stops" in out
    assert "turn the gizmo to the left" in out
    # 6 deg at 1 rad/s is about 0.1 s: a handful of commands at 50 Hz, not 100
    moving = [value for value in fake.speeds(10) if value]
    assert 1 <= len(moving) <= 8
    assert moving[0] == pytest.approx(1336.9, abs=1)
    assert fake.speeds(10)[-1] == 0


@needs_vcan
def test_jog_stops_a_runaway_motor(motors_file, servos, capsys):
    # The left wheel on the robot, 2026-09-25: +0.5 rad/s commanded, -16.6 rad/s measured
    fake = servos([12], speed_scale=-33.0)
    started = time.monotonic()
    assert tool(motors_file, "jog", "--joint", "wheel_joint_left", "--velocity", "0.5",
                "--duration", "2") == 1
    assert time.monotonic() - started < 1.0
    err = capsys.readouterr().err
    assert "stopped CAN ID 12 early" in err and "Upper Computer" in err
    assert fake.released(12)
    assert fake.speeds(12)[-1] != 0


@needs_vcan
def test_jog_accepts_a_motor_that_follows(motors_file, servos, capsys):
    servos([13], speed_scale=0.9)
    assert tool(motors_file, "jog", "--joint", "wheel_joint_right", "--velocity", "0.5",
                "--duration", "0.3") == 0
