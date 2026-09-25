"""Control rules of the web controller, free of ROS and aiohttp so they can be unit tested."""

import collections
import ipaddress
import math
from urllib.parse import urlsplit

# Traffic-light levels of an ultrasonic reading
NO_DATA, GREEN, YELLOW, RED = "none", "green", "yellow", "red"

# Blink period (s) of the traffic light: slow at warn_distance and beyond, fast when close.
# The fastest rate stays at 5 Hz; the lamps are small and never go fully dark, which keeps
# them well inside the WCAG flash thresholds
SLOWEST_BLINK = 1.0
FASTEST_BLINK = 0.2


def clamp(value, low, high):
    return max(low, min(high, value))


def finite_or_zero(value):
    """Coerce a client-supplied number to a finite float; anything else counts as 0."""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return 0.0
    return value if math.isfinite(value) else 0.0


class DriverLock:
    """Lets one client drive at a time.

    A client becomes the driver by sending a command while nobody has driven for
    `timeout` seconds. The driver keeps the lock by continuing to send commands, and gives
    it up by disconnecting or by staying idle for `timeout`. Times are in seconds from any
    monotonic clock.
    """

    def __init__(self, timeout):
        self.timeout = timeout
        self.driver = None
        self.last_command = -math.inf

    def holder(self, now):
        """Return the current driver, or None if nobody has driven within the timeout."""
        if self.driver is not None and now - self.last_command >= self.timeout:
            self.driver = None
        return self.driver

    def request(self, client, now):
        """Record a command from `client`. Returns True if that client is (now) the driver."""
        holder = self.holder(now)
        if holder is not None and holder != client:
            return False
        self.driver = client
        self.last_command = now
        return True

    def release(self, client):
        """Drop the lock immediately if `client` holds it. Returns True if it did."""
        if self.driver != client:
            return False
        self.driver = None
        self.last_command = -math.inf
        return True


def scale_command(linear_axis, angular_axis, speed, max_linear, max_angular,
                  max_speed=1.0):
    """Map joystick axes in [-1, 1] and a speed slider in [0, max_speed] to velocities.

    `max_speed` is the highest slider value accepted: 1.0 is 100 %, and turbo mode lets
    the slider go above it. Out-of-range or malformed inputs are clamped, so no client can
    exceed max_speed times the limits.
    """
    speed = clamp(finite_or_zero(speed), 0.0, max(max_speed, 0.0))
    linear = clamp(finite_or_zero(linear_axis), -1.0, 1.0) * speed * max_linear
    angular = clamp(finite_or_zero(angular_axis), -
                    1.0, 1.0) * speed * max_angular
    return linear, angular


def braking_distance(linear, reaction_time, deceleration):
    """Distance (m) the robot covers before it stops from forward speed `linear` (m/s).

    It keeps going for `reaction_time` (sensor and command latency), then brakes at
    `deceleration` (m/s^2, positive). Reversing or standing still needs no distance.
    """
    if linear <= 0.0:
        return 0.0
    distance = min(linear * max(reaction_time, 0.0), 0.30)
    if deceleration > 0.0:
        distance += linear * linear / (2.0 * deceleration)
    return distance


def is_missing(distance):
    """True for a stale reading (None) or one the sensor flagged as invalid (NaN)."""
    return distance is None or math.isnan(distance)


def nearest_obstacle(distances):
    """Return (side, distance) of the closest reading, ignoring missing ones.

    `distances` maps a side name to metres, math.inf for no echo, NaN for a faulty
    sensor, or None when the reading is stale. Returns (None, None) when no side has data.
    """
    readings = [(d, side)
                for side, d in distances.items() if not is_missing(d)]
    if not readings:
        return None, None
    distance, side = min(readings)
    return side, distance


def guard_command(linear, distances, stop_distance, require_data=False):
    """Apply the obstacle guard to a linear velocity.

    When any fresh ultrasonic reading is below `stop_distance`, forward motion is clamped
    to zero; reversing (and turning, which this does not touch) stays allowed so the robot
    can back away. With `require_data`, a side without a valid reading (stale or faulty)
    blocks forward motion too, since nothing is watching that side. A stop_distance of 0
    or less disables the guard.

    Returns (linear, blocking) where `blocking` is (side, distance) of the obstacle that
    triggered the guard, (side, None) for a side without data, or None.
    """
    if stop_distance <= 0.0:
        return linear, None
    side, distance = nearest_obstacle(distances)
    if distance is not None and distance < stop_distance:
        return min(linear, 0.0), (side, distance)
    if require_data:
        for side in sorted(distances):
            if is_missing(distances[side]):
                return min(linear, 0.0), (side, None)
    return linear, None


def classify_range(distance, warn_distance, danger_distance):
    """Traffic-light level and blink period (s) for one ultrasonic reading.

    GREEN at or beyond warn_distance (including no echo, +inf), YELLOW between
    danger_distance and warn_distance, RED below danger_distance, NO_DATA for a missing
    or stale reading. The blink speeds up linearly as the distance shrinks, like a
    parking sensor.
    """
    if distance is None or math.isnan(distance):
        return NO_DATA, None
    if distance >= warn_distance:
        level = GREEN
    elif distance >= danger_distance:
        level = YELLOW
    else:
        level = RED
    fraction = clamp(distance / warn_distance, 0.0,
                     1.0) if warn_distance > 0.0 else 1.0
    period = FASTEST_BLINK + fraction * (SLOWEST_BLINK - FASTEST_BLINK)
    return level, round(period, 3)


class EchoFilter:
    """Tells a node's own messages apart from other publishers' on a topic it also reads.

    rclpy delivers a node's own publications to its subscriptions and does not say who
    published a message, so the node records each value it publishes with `sent`. Messages
    from one publisher arrive in order, so an incoming value equal to the oldest
    outstanding one is taken as that echo. An echo that never arrives is forgotten after
    `timeout` seconds, so it cannot hide another publisher's message for longer than that.
    """

    def __init__(self, timeout=1.0):
        self.timeout = timeout
        self.outstanding = collections.deque()

    def sent(self, value, now):
        self.outstanding.append((value, now))

    def is_echo(self, value, now):
        """True if `value`, received at `now`, is the echo of a value this node sent."""
        while self.outstanding and now - self.outstanding[0][1] > self.timeout:
            self.outstanding.popleft()
        if self.outstanding and self.outstanding[0][0] == value:
            self.outstanding.popleft()
            return True
        return False


def host_allowed(host, names):
    """True if an HTTP Host header names this machine rather than some other domain.

    Browsers apply the same-origin policy to names, not addresses. A hostile page whose
    domain is re-pointed at the robot's address (DNS rebinding) is therefore same-origin
    with the robot, and its requests pass an Origin check; only their Host header still
    carries the hostile name. IP literals and "localhost" are always accepted, as is any
    name in `names` (lower case, without a port). "*" in `names` accepts every host.
    """
    if "*" in names:
        return True
    if not host:
        return False
    try:
        # Strips the port and the brackets of an IPv6 literal, and lower-cases the name
        hostname = urlsplit("//" + host).hostname
    except ValueError:
        return False
    if not hostname:
        return False
    hostname = hostname.rstrip(".")
    if hostname == "localhost" or hostname in names:
        return True
    try:
        ipaddress.ip_address(hostname)
    except ValueError:
        return False
    return True
