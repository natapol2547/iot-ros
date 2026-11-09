import math

import pytest

from iot_robot_web.logic import (
    AUTO, FASTEST_BLINK, GREEN, NO_DATA, RED, SLOWEST_BLINK, YELLOW, AdaptiveQuality,
    DriverLock, EchoFilter, RateMeter, braking_distance, choose_quality, classify_range,
    guard_command, host_allowed, scale_command, video_profile)


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
        assert scale_command(1.0, 0.0, bad, 0.4, 1.5, 3.0) == (0.0, 0.0)

    def test_turbo_raises_the_speed_cap(self):
        assert scale_command(1.0, -1.0, 3.0, 0.4, 1.5, 3.0) == pytest.approx((1.2, -4.5))
        assert scale_command(1.0, 1.0, 2.0, 0.4, 1.5, 3.0) == pytest.approx((0.8, 3.0))

    def test_turbo_speed_is_still_clamped(self):
        assert scale_command(1.0, 1.0, 9.0, 0.4, 1.5, 3.0) == pytest.approx((1.2, 4.5))
        assert scale_command(1.0, 1.0, 9.0, 0.4, 1.5, -1.0) == pytest.approx((0.0, 0.0))


class TestBrakingDistance:
    def test_reaction_plus_braking(self):
        # 1.2 m/s: 0.24 m before braking starts, 0.36 m braking at 2 m/s^2
        assert braking_distance(1.2, 0.2, 2.0) == pytest.approx(0.6)
        assert braking_distance(0.4, 0.2, 2.0) == pytest.approx(0.12)

    def test_reversing_or_standing_needs_no_distance(self):
        assert braking_distance(0.0, 0.2, 2.0) == 0.0
        assert braking_distance(-1.0, 0.2, 2.0) == 0.0

    def test_zero_or_negative_settings_are_ignored(self):
        assert braking_distance(1.0, 0.0, 0.0) == 0.0
        assert braking_distance(1.0, -1.0, 2.0) == pytest.approx(0.25)
        assert braking_distance(1.0, 0.5, -2.0) == pytest.approx(0.5)


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


class TestVideoSettings:
    def test_profile_values_are_kept_in_range(self):
        assert video_profile("low", 2, 300, 0.0) == ("low", 16, 100, 0.5)
        assert video_profile("high", 640.0, 70.0, 15) == ("high", 640, 70, 15.0)
        assert video_profile("high", 640, 70, 15.0).period == pytest.approx(1 / 15)

    @pytest.mark.parametrize("requested, chosen", [
        ("auto", AUTO), ("high", "high"), ("medium", "medium"), ("low", "low"),
        ("off", "medium"), ("HIGH", "medium"), (None, "medium"), ("", "medium"),
    ])
    def test_unknown_qualities_get_the_default(self, requested, chosen):
        assert choose_quality(requested, "medium") == chosen


def run(chooser, congested, seconds, now, fps=10.0):
    """Feed `seconds` of frames at `fps` to the chooser. Returns the time and the levels."""
    levels = []
    for _ in range(round(seconds * fps)):
        now += 1.0 / fps
        levels.append(chooser.sent(congested, now))
    return now, levels


class TestAdaptiveQuality:
    def test_steps_down_once_the_hold_time_has_passed(self):
        auto = AdaptiveQuality(3, 1, now=0.0, hold=2.0)
        now, levels = run(auto, True, 1.9, 0.0)
        assert set(levels) == {1}
        now, levels = run(auto, True, 0.2, now)
        assert levels[-1] == 2

    def test_a_new_level_gets_a_fair_trial(self):
        # The window empties at every change: one congested frame is not enough to go on
        auto = AdaptiveQuality(3, 0, now=0.0, window=8, hold=0.0)
        now, _ = run(auto, True, 0.4, 0.0)
        assert auto.index == 1
        # Half of the window must fill again on the new level
        now, levels = run(auto, True, 0.3, now)
        assert levels == [1, 1, 1]
        _, levels = run(auto, True, 0.1, now)
        assert levels == [2]

    def test_occasional_congestion_does_not_step_down(self):
        auto = AdaptiveQuality(3, 1, now=0.0, window=8)
        now = 0.0
        for frame in range(200):
            now += 0.1
            # one frame in four waited: Wi-Fi hiccups, not a slow link
            assert auto.sent(frame % 4 == 0, now) <= 1
        assert auto.index != 2

    def test_never_leaves_the_range(self):
        auto = AdaptiveQuality(3, 2, now=0.0, hold=0.0)
        _, levels = run(auto, True, 10.0, 0.0)
        assert max(levels) == 2
        auto = AdaptiveQuality(3, 0, now=0.0, hold=0.0, probe=1.0)
        _, levels = run(auto, False, 30.0, 0.0)
        assert min(levels) == 0 and max(levels) == 0

    def test_probes_upwards_on_a_clear_connection(self):
        auto = AdaptiveQuality(3, 2, now=0.0, hold=2.0, probe=8.0)
        now, levels = run(auto, False, 7.9, 0.0)
        assert set(levels) == {2}
        now, levels = run(auto, False, 0.2, now)
        assert levels[-1] == 1
        now, levels = run(auto, False, 8.2, now)
        assert levels[-1] == 0

    def test_failed_probes_back_off(self):
        # The link fits the middle level only: each step up congests at once
        auto = AdaptiveQuality(3, 1, now=0.0, hold=2.0, probe=8.0, trial=10.0)
        now = 0.0
        ups = []
        for _ in range(1500):  # 150 s
            now += 0.1
            before = auto.index
            level = auto.sent(auto.index == 0, now)
            if level < before:
                ups.append(round(now, 1))
        assert len(ups) >= 3
        gaps = [b - a for a, b in zip(ups, ups[1:])]
        # Each failed probe doubles the wait before the next one (8, 16, 32 s plus the trial)
        assert gaps == sorted(gaps)
        assert gaps[-1] > gaps[0] * 1.5
        assert auto.probe <= 64.0

    def test_a_probe_that_holds_shortens_the_next_wait(self):
        auto = AdaptiveQuality(3, 1, now=0.0, hold=2.0, probe=8.0, trial=10.0)
        auto.probe = 32.0
        now, _ = run(auto, False, 32.1, 0.0)
        assert auto.index == 0
        run(auto, False, 10.5, now)
        assert auto.probe == 16.0


class TestRateMeter:
    def test_rates_over_the_span(self):
        meter = RateMeter(span=2.0)
        for i in range(20):
            meter.add(1000, i * 0.1)
        fps, bps = meter.rates(1.95)
        assert fps == pytest.approx(10.0)
        assert bps == pytest.approx(80000.0)

    def test_old_frames_fall_out(self):
        meter = RateMeter(span=2.0)
        meter.add(5000, 0.0)
        assert meter.rates(1.0) == (0.5, 20000.0)
        assert meter.rates(2.5) == (0.0, 0.0)
