import math

import pytest

from iot_robot_drivers.stm32_protocol import (
    Reading, SensorStatus, distance_to_range, format_gizmo, is_banner, is_comment,
    joint_to_servo_deg, parse_gizmo, parse_line, parse_sensor_status)


class TestParseLine:
    def test_distances_in_cm(self):
        assert parse_line("D1:45.3,D2:120.0\r\n") == Reading(45.3, 120.0, None)

    def test_battery_voltage_is_optional(self):
        assert parse_line("D1:45.3,D2:120.0,V:24.12\r\n") == Reading(45.3, 120.0, 24.12)

    def test_minus_one_is_no_echo(self):
        assert parse_line("D1:-1.0,D2:88.8") == Reading(None, 88.8, None)

    def test_zero_is_no_echo_like_the_first_firmware(self):
        assert parse_line("D1:0.0,D2:0.0") == Reading(None, None, None)

    def test_whitespace_around_fields_is_accepted(self):
        assert parse_line("  D1: 10.0 , D2:20.5 \n") == Reading(10.0, 20.5, None)

    def test_unknown_fields_are_ignored(self):
        assert parse_line("D1:10.0,D2:20.0,T:31") == Reading(10.0, 20.0, None)

    @pytest.mark.parametrize("line", ["", "\r\n", "# iot-stm32 1.0.0", "#"])
    def test_blank_and_comment_lines_are_not_readings(self, line):
        assert parse_line(line) is None

    @pytest.mark.parametrize("line", [
        "D1:45.3",                    # missing D2
        "3,D2:120.0",                 # first field cut off after a reconnect
        "D1:abc,D2:1.0",              # not a number
        "D1:1.0,D2:2.0,D1:3.0",       # duplicate key
        "D1:nan,D2:1.0",              # not finite
        "D1:1.0,D2:inf",
        "D1:1.0,D2:2.0,V:nan",
        "D1 45.3 D2 120.0",           # old or foreign format
        "\x00\xff garbage",
    ])
    def test_malformed_lines_are_rejected(self, line):
        assert parse_line(line) is None

    def test_comment_detection(self):
        assert is_comment("# iot-stm32 1.0.0\r\n")
        assert not is_comment("D1:1.0,D2:2.0")

    def test_banner_detection(self):
        assert is_banner("# iot-stm32 1.0.0\r\n")
        assert not is_banner("# servos=on battery=off\r\n")


class TestSensorStatus:
    @pytest.mark.parametrize("line, status", [
        ("# warning: left sensor not responding\r\n",
         SensorStatus("left", True, "sensor not responding")),
        ("# warning: right echo stuck high", SensorStatus("right", True, "echo stuck high")),
        ("# warning: left echo pulses too short",
         SensorStatus("left", True, "echo pulses too short")),
        ("# info: right sensor recovered\r\n", SensorStatus("right", False, "sensor recovered")),
    ])
    def test_firmware_fault_lines(self, line, status):
        assert parse_sensor_status(line) == status

    @pytest.mark.parametrize("line", [
        "# iot-stm32 1.0.0",
        "# servos=on battery=off",
        "# warning: middle sensor not responding",   # unknown side
        "# warning: left",                           # no fault text
        "# info: left calibrated",                   # an info line that is not a recovery
        "warning: left sensor not responding",       # not a comment
        "D1:1.0,D2:2.0",
    ])
    def test_other_lines_are_not_sensor_status(self, line):
        assert parse_sensor_status(line) is None


class TestDistanceToRange:
    def test_centimetres_become_metres(self):
        assert distance_to_range(150.0, 0.02, 4.0) == pytest.approx(1.5)

    def test_no_echo_is_positive_infinity(self):
        assert distance_to_range(None, 0.02, 4.0) == math.inf

    def test_beyond_max_range_is_no_echo(self):
        assert distance_to_range(450.0, 0.02, 4.0) == math.inf
        assert distance_to_range(400.0, 0.02, 4.0) == pytest.approx(4.0)

    def test_closer_than_min_range_is_clamped(self):
        assert distance_to_range(1.0, 0.02, 4.0) == pytest.approx(0.02)

    def test_faulty_sensor_is_nan_not_clear(self):
        assert math.isnan(distance_to_range(None, 0.02, 4.0, faulty=True))
        assert math.isnan(distance_to_range(150.0, 0.02, 4.0, faulty=True))


class TestGizmo:
    def test_format_rounds_to_a_tenth_of_a_degree(self):
        assert format_gizmo(10.0, -20.04) == "G:10.0,-20.0\n"
        assert format_gizmo(-0.04, 0.0) == "G:-0.0,0.0\n"

    def test_parse_round_trips(self):
        assert parse_gizmo(format_gizmo(12.3, -45.0)) == (12.3, -45.0)

    @pytest.mark.parametrize("line", ["G:1.0", "G:a,b", "X:1,2", "G:1,2,3", "G:nan,1"])
    def test_parse_rejects_malformed(self, line):
        assert parse_gizmo(line) is None

    def test_joint_angle_to_servo_degrees(self):
        assert joint_to_servo_deg(math.pi / 4, 1.0, 0.0) == pytest.approx(45.0)
        assert joint_to_servo_deg(math.pi / 4, -1.0, 0.0) == pytest.approx(-45.0)
        assert joint_to_servo_deg(0.0, 1.0, 5.0) == pytest.approx(5.0)

    def test_servo_angle_is_clamped_to_travel(self):
        assert joint_to_servo_deg(math.pi, 1.0, 0.0) == pytest.approx(90.0)
        assert joint_to_servo_deg(-math.pi, 1.0, 0.0) == pytest.approx(-90.0)
