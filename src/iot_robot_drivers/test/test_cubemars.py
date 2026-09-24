"""CubeMars servo protocol codec, plus SocketCAN checks when a virtual CAN bus is available.

The bus tests need a vcan interface, which normally takes root. Without root they can
run inside a user and network namespace:

    unshare -rn sh -c 'ip link set lo up && ip link add dev vcan0 type vcan && \
        ip link set vcan0 up && IOT_TEST_CAN_INTERFACE=vcan0 python -m pytest test'
"""

import os
import socket
import struct
import threading
import time

import pytest

from iot_robot_drivers import cubemars_tool
from iot_robot_drivers.cubemars import (
    CanBus, CanCheckError, Status, check_interface, decode_status, encode_release,
    encode_speed, erpm_per_rad_s, listen, preflight, stop_motors, wait_for_status)


def status_frame(can_id, position=0, speed=0, current=0, temperature=30, error=0):
    return (0x29 << 8) | can_id, struct.pack(">hhhbB", position, speed, current,
                                             temperature, error)


class TestCodec:
    def test_erpm_conversion_matches_cubemars_hardware(self):
        # pole_pairs * gear_ratio * 60 / (2 pi) for the AK45-10
        assert erpm_per_rad_s(14, 10) == pytest.approx(1336.9015)

    def test_speed_command_is_big_endian_int32(self):
        assert encode_speed(1, 1336.9) == (0x301, b"\x00\x00\x05\x38")
        assert encode_speed(2, -1000) == (0x302, b"\xff\xff\xfc\x18")

    def test_speed_command_rejects_what_the_driver_rejects(self):
        with pytest.raises(ValueError):
            encode_speed(1, 100000)

    def test_release_is_zero_current(self):
        assert encode_release(3) == (0x103, b"\x00\x00\x00\x00")

    def test_status_frame(self):
        status = decode_status(*status_frame(
            2, position=900, speed=134, current=-150, temperature=35, error=4))
        assert status == Status(2, pytest.approx(90.0), pytest.approx(1340.0),
                                pytest.approx(-1.5), 35, 4)
        assert status.output_velocity() == pytest.approx(1340.0 / 1336.9015)
        assert status.error_text == "under-voltage"

    def test_other_frames_are_not_status(self):
        assert decode_status(0x301, b"\x00\x00\x05\x38") is None
        assert decode_status(0x2901, b"\x00" * 7) is None


class RecordingBus:
    def __init__(self):
        self.frames = []

    def send(self, arbitration_id, data):
        self.frames.append((arbitration_id, bytes(data)))


def test_stop_holds_zero_speed_then_releases_every_motor():
    bus = RecordingBus()
    stop_motors(bus, [1, 2], brake_time=0.05, period=0.01)
    zero_speed = {encode_speed(1, 0), encode_speed(2, 0)}
    *braking, release_1, release_2 = bus.frames
    # Several rounds of zero speed, so a lost frame does not leave a motor turning
    assert len(braking) >= 4 and set(braking) == zero_speed
    assert [release_1, release_2] == [encode_release(1), encode_release(2)]


def test_missing_interface_explains_what_to_do():
    with pytest.raises(CanCheckError, match="does not exist"):
        check_interface("nocan42")


def test_stop_tool_succeeds_without_a_can_link(capsys):
    # iot-robot.service runs it on every stop; a missing adapter must not fail the unit
    assert cubemars_tool.main(["--interface", "nocan42", "stop"]) == 0
    assert "not up; no stop frames sent" in capsys.readouterr().out


CAN_INTERFACE = os.environ.get("IOT_TEST_CAN_INTERFACE")
needs_vcan = pytest.mark.skipif(
    not CAN_INTERFACE or not hasattr(socket, "AF_CAN"),
    reason="set IOT_TEST_CAN_INTERFACE to a vcan interface to run bus tests")


class FakeMotors:
    """Send status frames for some CAN IDs at 100 Hz until stopped."""

    def __init__(self, interface, ids, error=0):
        self.bus = CanBus(interface)
        self.ids, self.error = ids, error
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stopping.wait(0.01):
            for can_id in self.ids:
                self.bus.send(*status_frame(can_id, speed=134, error=self.error))

    def stop(self):
        self.stopping.set()
        self.thread.join()
        self.bus.close()


@needs_vcan
class TestBus:
    def test_preflight_passes_when_both_motors_report(self):
        motors = FakeMotors(CAN_INTERFACE, [1, 2])
        try:
            heard = preflight(CAN_INTERFACE, [1, 2], timeout=1.0)
        finally:
            motors.stop()
        assert sorted(heard) == [1, 2]
        assert heard[1].speed_erpm == pytest.approx(1340.0)

    def test_preflight_names_the_silent_motor_and_the_unexpected_one(self):
        motors = FakeMotors(CAN_INTERFACE, [1, 3])
        try:
            with pytest.raises(CanCheckError) as err:
                preflight(CAN_INTERFACE, [1, 2], timeout=0.5)
        finally:
            motors.stop()
        assert "No status frames from CAN ID 2 " in str(err.value)
        assert "CAN ID 3 (not in motors.yaml)" in str(err.value)
        assert "can-identify" in str(err.value)

    def test_preflight_labels_motors_with_their_joints(self):
        motors = FakeMotors(CAN_INTERFACE, [12, 10])
        names = {12: "wheel_joint_left", 13: "wheel_joint_right", 10: "gizmo_yaw_joint"}
        try:
            with pytest.raises(CanCheckError) as err:
                preflight(CAN_INTERFACE, [12, 13], timeout=0.5, names=names)
        finally:
            motors.stop()
        # A motors.yaml motor that is not in use is named, not called unexpected
        assert "from wheel_joint_right (CAN ID 13) on" in str(err.value)
        assert "heard from gizmo_yaw_joint (CAN ID 10)." in str(err.value)

    def test_preflight_reports_motor_faults(self):
        motors = FakeMotors(CAN_INTERFACE, [1, 2], error=4)
        try:
            with pytest.raises(CanCheckError, match="under-voltage"):
                preflight(CAN_INTERFACE, [1, 2], timeout=1.0)
        finally:
            motors.stop()

    def test_listen_sees_another_programs_commands(self):
        heard = {}

        def run():
            heard["result"] = listen(CAN_INTERFACE, 0.5, status_only=False)
        listener = threading.Thread(target=run)
        listener.start()
        with CanBus(CAN_INTERFACE) as other:
            for _ in range(10):
                other.send(*encode_speed(10, 0))
                other.send(*status_frame(11))
                time.sleep(0.02)
        listener.join()
        assert heard["result"].commanded == {10}
        assert sorted(heard["result"].status) == [11]

    def test_speed_commands_reach_the_bus(self):
        with CanBus(CAN_INTERFACE) as listener, CanBus(CAN_INTERFACE) as sender:
            sender.send(*encode_speed(1, 1336.9))
            assert listener.recv(1.0) == (0x301, b"\x00\x00\x05\x38")

    def test_wait_for_status_times_out_quietly(self):
        assert wait_for_status(CAN_INTERFACE, [7], timeout=0.2) == {}

    def test_stop_tool_brakes_and_releases_on_the_bus(self):
        with CanBus(CAN_INTERFACE) as listener:
            assert cubemars_tool.main([
                "--interface", CAN_INTERFACE, "stop", "--ids", "1", "2",
                "--brake-time", "0.05"]) == 0
            frames = []
            while (frame := listener.recv(0.2)) is not None:
                frames.append(frame)
        assert frames[:2] == [encode_speed(1, 0), encode_speed(2, 0)]
        assert frames[-2:] == [encode_release(1), encode_release(2)]
