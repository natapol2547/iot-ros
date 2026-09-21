import math

import pytest

from iot_robot_web.logic import (
    FASTEST_BLINK, GREEN, NO_DATA, RED, SLOWEST_BLINK, YELLOW, DriverLock, EchoFilter,
    classify_range, guard_command, host_allowed, scale_command)


class TestDriverLock:
    def test_first_client_takes_the_lock(self):
        lock = DriverLock(timeout=2.0)
        assert lock.holder(0.0) is None
        assert lock.request("a", 0.0)
        assert lock.holder(0.1) == "a"

    def test_second_client_is_refused_while_driver_is_active(self):
        lock = DriverLock(timeout=2.0)
        lock.request("a", 0.0)
        assert not lock.request("b", 0.5)
        # The driver keeps the lock alive by sending commands
        assert lock.request("a", 1.9)
        assert not lock.request("b", 3.8)
        assert lock.holder(3.8) == "a"

    def test_lock_frees_after_timeout(self):
        lock = DriverLock(timeout=2.0)
        lock.request("a", 0.0)
        assert lock.holder(1.99) == "a"
        assert lock.holder(2.0) is None
        assert lock.request("b", 2.0)
        assert not lock.request("a", 2.1)

    def test_release_frees_immediately(self):
        lock = DriverLock(timeout=2.0)
        lock.request("a", 0.0)
        assert not lock.release("b")
        assert lock.holder(0.1) == "a"
        assert lock.release("a")
        assert lock.request("b", 0.2)


class TestScaleCommand:
    def test_full_deflection_hits_the_limits(self):
        assert scale_command(1.0, -1.0, 1.0, 0.4, 1.5) == pytest.approx((0.4, -1.5))

    def test_speed_slider_scales_both_axes(self):
        assert scale_command(1.0, 1.0, 0.5, 0.4, 1.5) == pytest.approx((0.2, 0.75))

    def test_out_of_range_input_is_clamped(self):
        assert scale_command(5.0, -9.0, 3.0, 0.4, 1.5) == pytest.approx((0.4, -1.5))
        assert scale_command(1.0, 1.0, -1.0, 0.4, 1.5) == pytest.approx((0.0, 0.0))

    @pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf, None, "fast", [1]])
    def test_malformed_input_counts_as_zero(self, bad):
        assert scale_command(bad, bad, 1.0, 0.4, 1.5) == (0.0, 0.0)
        assert scale_command(1.0, 0.0, bad, 0.4, 1.5) == (0.0, 0.0)


class TestObstacleGuard:
    def test_far_obstacles_do_not_limit(self):
        assert guard_command(0.3, {"left": 1.0, "right": math.inf}, 0.25) == (0.3, None)

    def test_close_obstacle_blocks_forward(self):
        linear, blocking = guard_command(0.3, {"left": 0.5, "right": 0.2}, 0.25)
        assert linear == 0.0
        assert blocking == ("right", 0.2)

    def test_reversing_stays_allowed(self):
        linear, blocking = guard_command(-0.2, {"left": 0.1, "right": 0.9}, 0.25)
        assert linear == -0.2
        assert blocking == ("left", 0.1)

    def test_stale_readings_are_ignored(self):
        assert guard_command(0.3, {"left": None, "right": None}, 0.25) == (0.3, None)
        linear, blocking = guard_command(0.3, {"left": None, "right": 0.1}, 0.25)
        assert linear == 0.0 and blocking == ("right", 0.1)

    def test_faulty_readings_are_ignored(self):
        assert guard_command(0.3, {"left": math.nan, "right": 1.0}, 0.25) == (0.3, None)
        linear, blocking = guard_command(0.3, {"left": math.nan, "right": 0.1}, 0.25)
        assert linear == 0.0 and blocking == ("right", 0.1)

    def test_missing_data_blocks_forward_when_required(self):
        for missing in (None, math.nan):
            linear, blocking = guard_command(
                0.3, {"left": missing, "right": 1.0}, 0.25, require_data=True)
            assert linear == 0.0
            assert blocking == ("left", None)
        linear, blocking = guard_command(
            -0.2, {"left": None, "right": None}, 0.25, require_data=True)
        assert linear == -0.2 and blocking == ("left", None)

    def test_close_obstacle_wins_over_missing_data(self):
        linear, blocking = guard_command(
            0.3, {"left": None, "right": 0.1}, 0.25, require_data=True)
        assert linear == 0.0 and blocking == ("right", 0.1)

    def test_valid_data_passes_when_required(self):
        assert guard_command(
            0.3, {"left": math.inf, "right": 0.5}, 0.25, require_data=True) == (0.3, None)

    def test_no_echo_is_clear(self):
        assert guard_command(0.3, {"left": math.inf, "right": math.inf}, 0.25) == (0.3, None)

    def test_zero_stop_distance_disables_the_guard(self):
        assert guard_command(0.3, {"left": 0.05, "right": 0.05}, 0.0) == (0.3, None)
        assert guard_command(
            0.3, {"left": None, "right": None}, 0.0, require_data=True) == (0.3, None)


class TestClassifyRange:
    @pytest.mark.parametrize("distance, level", [
        (math.inf, GREEN),
        (3.0, GREEN),
        (1.0, GREEN),
        (0.99, YELLOW),
        (0.4, YELLOW),
        (0.39, RED),
        (0.02, RED),
        (None, NO_DATA),
        (math.nan, NO_DATA),
    ])
    def test_levels(self, distance, level):
        assert classify_range(distance, 1.0, 0.4)[0] == level

    def test_blink_speeds_up_as_distance_shrinks(self):
        periods = [classify_range(d, 1.0, 0.4)[1] for d in (2.0, 1.0, 0.7, 0.4, 0.1, 0.0)]
        assert periods[0] == periods[1] == SLOWEST_BLINK
        assert periods == sorted(periods, reverse=True)
        assert periods[-1] == FASTEST_BLINK
        assert classify_range(math.inf, 1.0, 0.4)[1] == SLOWEST_BLINK

    def test_no_data_does_not_blink(self):
        assert classify_range(None, 1.0, 0.4) == (NO_DATA, None)


class TestEchoFilter:
    def test_own_messages_are_echoes_in_order(self):
        echo = EchoFilter(timeout=1.0)
        echo.sent(True, 0.0)
        echo.sent(False, 0.1)
        assert echo.is_echo(True, 0.01)
        assert echo.is_echo(False, 0.11)
        assert not echo.is_echo(False, 0.2)

    def test_other_publishers_are_not_echoes(self):
        echo = EchoFilter(timeout=1.0)
        assert not echo.is_echo(True, 0.0)
        echo.sent(False, 1.0)
        # Someone else's True arrives before the node's own False comes back
        assert not echo.is_echo(True, 1.001)
        assert echo.is_echo(False, 1.002)

    def test_stale_heartbeat_after_release_is_an_echo(self):
        # Heartbeat True, then released at once: the True echo must not re-engage
        echo = EchoFilter(timeout=1.0)
        echo.sent(True, 5.0)
        echo.sent(False, 5.0005)
        assert echo.is_echo(True, 5.001)
        assert echo.is_echo(False, 5.002)

    def test_lost_echoes_expire(self):
        echo = EchoFilter(timeout=1.0)
        echo.sent(True, 0.0)
        assert not echo.is_echo(True, 1.5)


class TestHostAllowed:
    NAMES = {"iot-robot", "iot-robot.local"}

    @pytest.mark.parametrize("host", [
        "localhost:8080", "LOCALHOST", "127.0.0.1:8080", "192.168.1.42:8080", "10.0.0.5",
        "[::1]:8080", "[fe80::1]", "iot-robot.local:8080", "IOT-Robot.local.", "iot-robot",
    ])
    def test_own_names_and_addresses_are_allowed(self, host):
        assert host_allowed(host, self.NAMES)

    @pytest.mark.parametrize("host", [
        "attacker.example:8080", "iot-robot.attacker.example", "192.168.1.42.nip.io:8080",
        "", None, "[not-an-ip]", "a b",
    ])
    def test_other_names_are_refused(self, host):
        assert not host_allowed(host, self.NAMES)

    def test_extra_names_and_wildcard(self):
        assert host_allowed("robot.lan:8080", self.NAMES | {"robot.lan"})
        assert host_allowed("anything.example", {"*"})
