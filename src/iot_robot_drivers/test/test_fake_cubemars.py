"""fake_cubemars: the motor model, plus the emulator on a virtual CAN bus when available.

The bus tests need a vcan interface (see test_cubemars.py for running them without root).
"""

import os
import socket
import time

import pytest

from iot_robot_drivers import fake_cubemars
from iot_robot_drivers.cubemars import CanBus, decode_status, erpm_per_rad_s
from iot_robot_drivers.fake_cubemars import (
    MODE_BRAKE, MODE_CURRENT, MODE_DUTY, MODE_POSITION, MODE_POSITION_SPEED,
    MODE_SET_ORIGIN, MODE_SPEED, ORIGIN_DEFAULT, ORIGIN_PERMANENT, ORIGIN_TEMPORARY,
    FakeBusError, FakeCubeMars, FakeMotor, check_virtual, decode_command, encode_command,
    encode_status)

ERPM_PER_RAD_S = erpm_per_rad_s()


def command(motor, mode, value, **kwargs):
    _, data = encode_command(motor.can_id, mode, value, **kwargs)
    return motor.command(mode, data)


def run(motor, seconds, dt=0.01):
    for _ in range(int(round(seconds / dt))):
        motor.step(dt)


class TestCodec:
    def test_status_decodes_with_the_driver_codec(self):
        status = decode_status(*encode_status(12, -12.34, 2500.0, -1.5, 31, 7))
        assert status.can_id == 12
        assert status.position_deg == pytest.approx(-12.3)
        assert status.speed_erpm == pytest.approx(2500.0)
        assert status.current_a == pytest.approx(-1.5)
        assert (status.temperature_c, status.error) == (31, 7)

    def test_status_position_saturates_like_the_motor(self):
        assert decode_status(*encode_status(1, 5000.0, 0.0, 0.0)).position_deg == 3200.0
        assert decode_status(*encode_status(1, -5000.0, 0.0, 0.0)).position_deg == -3200.0

    @pytest.mark.parametrize("mode, value, payload", [
        (MODE_DUTY, 0.5, b"\x00\x00\xc3\x50"),
        (MODE_CURRENT, -1.0, b"\xff\xff\xfc\x18"),
        (MODE_BRAKE, 2.0, b"\x00\x00\x07\xd0"),
        (MODE_SPEED, 1336.0, b"\x00\x00\x05\x38"),
        (MODE_POSITION, -45.0, b"\xff\xf9\x22\x30"),
        (MODE_SET_ORIGIN, ORIGIN_TEMPORARY, b"\x00"),
    ])
    def test_commands_follow_the_servo_protocol(self, mode, value, payload):
        arbitration_id, data = encode_command(12, mode, value)
        assert arbitration_id == (mode << 8) | 12
        assert data == payload
        assert decode_command(mode, data) == pytest.approx(value)

    def test_position_speed_command_carries_speed_and_acceleration(self):
        _, data = encode_command(3, MODE_POSITION_SPEED, 10.0, speed_erpm=5000.0,
                                 acceleration=20000.0)
        assert data == b"\x00\x01\x86\xa0\x01\xf4\x07\xd0"

    def test_short_frames_are_ignored(self):
        assert decode_command(MODE_SPEED, b"\x00\x01") is None
        assert decode_command(MODE_SET_ORIGIN, b"") is None
        assert not FakeMotor(1).command(MODE_SPEED, b"\x00")


class TestMotorModel:
    def test_speed_mode_integrates_the_output_position(self):
        motor = FakeMotor(10)
        command(motor, MODE_SPEED, ERPM_PER_RAD_S)
        run(motor, 0.5)
        # 1336 ERPM (1 rad/s truncated to an integer) at the output for 0.5 s
        assert motor.position_deg == pytest.approx(1336 * 6.0 / 140.0 * 0.5)
        assert decode_status(*motor.status()).speed_erpm == pytest.approx(1340.0)

    def test_position_mode_moves_at_a_finite_speed(self):
        motor = FakeMotor(12, position_speed_dps=360.0)
        command(motor, MODE_POSITION, 90.0)
        run(motor, 0.1)
        assert motor.position_deg == pytest.approx(36.0)
        assert motor.speed_erpm > 0.0
        run(motor, 0.3)
        assert motor.position_deg == pytest.approx(90.0)
        assert motor.speed_erpm == 0.0

    def test_position_speed_mode_uses_the_commanded_speed(self):
        motor = FakeMotor(12, position_speed_dps=720.0)
        command(motor, MODE_POSITION_SPEED, 90.0, speed_erpm=ERPM_PER_RAD_S * 2.0 * 3.1416)
        run(motor, 0.1)
        # 2 pi rad/s = 360 deg/s, slower than the plain position loop
        assert motor.position_deg == pytest.approx(36.0, abs=0.1)

    def test_release_coasts_and_brake_holds(self):
        motor = FakeMotor(10)
        command(motor, MODE_SPEED, 5000.0)
        run(motor, 0.1)
        command(motor, MODE_CURRENT, 0.0)
        assert motor.mode == "idle"
        run(motor, 1.0)
        assert abs(motor.speed_erpm) < 1.0
        command(motor, MODE_BRAKE, 1.5)
        before = motor.position_deg
        run(motor, 0.2)
        assert motor.position_deg == before
        assert motor.current_a == 1.5

    def test_temporary_origin_is_lost_at_power_off(self):
        motor = FakeMotor(12)
        motor.set_position(30.0)
        command(motor, MODE_SET_ORIGIN, ORIGIN_TEMPORARY)
        assert motor.position_deg == 0.0
        motor.power_cycle(position_deg=5.0)
        assert motor.position_deg == 5.0
        assert motor.origin_deg == 0.0

    def test_permanent_origin_survives_power_off_until_restored(self):
        motor = FakeMotor(12)
        motor.set_position(30.0)
        command(motor, MODE_SET_ORIGIN, ORIGIN_PERMANENT)
        motor.power_cycle(position_deg=5.0)
        assert motor.origin_deg == 30.0 and motor.position_deg == 5.0
        command(motor, MODE_SET_ORIGIN, ORIGIN_DEFAULT)
        assert motor.origin_deg == 0.0 and motor.position_deg == 35.0

    def test_a_position_target_jumps_with_a_new_origin(self):
        motor = FakeMotor(12)
        command(motor, MODE_POSITION, 20.0)
        run(motor, 0.1)
        command(motor, MODE_SET_ORIGIN, ORIGIN_TEMPORARY)
        run(motor, 0.1)
        # Still in position mode: moves to 20 deg past the new origin
        assert motor.position_deg == pytest.approx(20.0)
        assert motor.encoder_deg == pytest.approx(40.0)

    def test_ignored_origin(self):
        motor = FakeMotor(12, ignore_origin=True)
        motor.set_position(30.0)
        assert not command(motor, MODE_SET_ORIGIN, ORIGIN_TEMPORARY)
        assert motor.position_deg == 30.0

    def test_a_fault_stops_the_motor_until_it_clears(self):
        motor = FakeMotor(10)
        command(motor, MODE_SPEED, 5000.0)
        motor.error = 7
        motor.release()
        assert not command(motor, MODE_SPEED, 5000.0)
        run(motor, 0.1)
        assert motor.speed_erpm == 0.0
        assert decode_status(*motor.status()).error == 7

    def test_a_stuck_joint_reports_the_load_current(self):
        motor = FakeMotor(12, stuck=True, load_current_a=2.0)
        command(motor, MODE_POSITION, 45.0)
        run(motor, 0.2)
        assert motor.position_deg == 0.0
        assert decode_status(*motor.status()).current_a == pytest.approx(2.0)


def test_real_interfaces_are_refused():
    with pytest.raises(FakeBusError, match="not a virtual CAN interface"):
        check_virtual("lo")
    check_virtual("lo", allow_real_bus=True)
    with pytest.raises(FakeBusError):
        FakeCubeMars("lo")


def test_command_line_parses_injections():
    args = fake_cubemars.parse_args([
        "--ids", "10", "12", "--position", "12=-30.5", "--fault", "12=7@2.5",
        "--silent", "10@1", "--load-current", "12=1.5", "--stuck", "12"])
    assert args.position == [(12, -30.5, 0.0)]
    assert args.fault == [(12, 7, 2.5)]
    assert args.silent == [(10, 1.0)]
    assert args.load_current == [(12, 1.5, 0.0)]
    assert args.stuck == [12]


def test_command_line_rejects_ids_that_are_not_emulated(capsys):
    with pytest.raises(SystemExit):
        fake_cubemars.parse_args(["--ids", "10", "--fault", "11=7"])
    assert "not emulated" in capsys.readouterr().err


def test_command_line_refuses_a_real_interface(capsys):
    assert fake_cubemars.main(["--interface", "lo"]) == 2
    assert "not a virtual CAN interface" in capsys.readouterr().err


CAN_INTERFACE = os.environ.get("IOT_TEST_CAN_INTERFACE")
needs_vcan = pytest.mark.skipif(
    not CAN_INTERFACE or not hasattr(socket, "AF_CAN"),
    reason="set IOT_TEST_CAN_INTERFACE to a vcan interface to run bus tests")


def statuses(bus, seconds):
    """Status frames heard for seconds, as a list of Status."""
    heard = []
    deadline = time.monotonic() + seconds
    while (remaining := deadline - time.monotonic()) > 0.0:
        frame = bus.recv(remaining)
        if frame is not None and (status := decode_status(*frame)) is not None:
            heard.append(status)
    return heard


@needs_vcan
class TestOnTheBus:
    def test_status_rate_commands_and_log(self):
        with FakeCubeMars(CAN_INTERFACE, ids=[21, 22]) as fake, \
                CanBus(CAN_INTERFACE, status_only=True) as bus:
            heard = statuses(bus, 0.5)
            for can_id in (21, 22):
                count = sum(status.can_id == can_id for status in heard)
                assert 35 <= count <= 60, f"{count} status frames from {can_id} in 0.5 s"

            bus.send(*encode_command(21, MODE_SPEED, ERPM_PER_RAD_S))
            time.sleep(0.3)
            latest = [status for status in statuses(bus, 0.05) if status.can_id == 21][-1]
            assert latest.position_deg > 10.0
            assert latest.speed_erpm == pytest.approx(1340.0)

            bus.send(*encode_command(21, MODE_CURRENT, 0.0))
            bus.send(*encode_command(21, MODE_SET_ORIGIN, ORIGIN_TEMPORARY))
            assert fake.wait_for(lambda: len(fake.commands(21)) == 3, 1.0)
            modes = [command.mode for command in fake.commands(21)]
            assert modes == [MODE_SPEED, MODE_CURRENT, MODE_SET_ORIGIN]
            time.sleep(0.6)
            latest = [status for status in statuses(bus, 0.05) if status.can_id == 21][-1]
            # Coasted to a stop from 0 after the new origin
            assert abs(latest.position_deg) < 6.0
            assert fake.commands(22) == []

    def test_fault_and_silence_injection(self):
        with FakeCubeMars(CAN_INTERFACE, ids=[21, 22]) as fake, \
                CanBus(CAN_INTERFACE, status_only=True) as bus:
            fake.inject_fault(22, 7)
            fake.silence(21, after=0.1)
            time.sleep(0.2)
            # Discard the frames buffered from before the injections
            statuses(bus, 0.05)
            heard = statuses(bus, 0.3)
            assert all(status.can_id == 22 for status in heard)
            assert heard and all(status.error == 7 for status in heard)
            fake.silence(21, silent=False)
            fake.inject_fault(22, 0)
            time.sleep(0.05)
            statuses(bus, 0.05)
            heard = statuses(bus, 0.2)
            assert {status.can_id for status in heard} == {21, 22}
            assert all(status.error == 0 for status in heard[-4:])
