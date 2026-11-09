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
    distance = linear * max(reaction_time, 0.0)
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


# Video ------------------------------------------------------------------------------

# Fixed video qualities, best first. The page also offers "auto", which picks one of these
# per viewer from how well that viewer's connection keeps up
VIDEO_QUALITIES = ("high", "medium", "low")
AUTO = "auto"
OFF = "off"  # only as a default for new pages: they then show no video until asked


_VideoProfile = collections.namedtuple("VideoProfile", ["name", "width", "quality", "fps"])


class VideoProfile(_VideoProfile):
    """One video quality: frames at most `width` pixels wide, JPEG `quality`, `fps` cap."""

    __slots__ = ()

    @property
    def period(self):
        return 1.0 / self.fps


def video_profile(name, width, quality, fps):
    """A VideoProfile with every value forced into a range the encoder accepts."""
    return VideoProfile(name, max(int(width), 16), int(clamp(quality, 1, 100)),
                        clamp(float(fps), 0.5, 60.0))


def choose_quality(requested, default):
    """The quality a viewer asked for, or `default` for anything unknown."""
    return requested if requested == AUTO or requested in VIDEO_QUALITIES else default


class AdaptiveQuality:
    """Picks the video quality for one viewer of the "auto" setting.

    The server cannot see the viewer's screen, only its own send buffer. A frame that must
    wait for the previous one to leave means the connection is slower than the stream, so
    each sent frame is reported with `sent(congested, now)`.

    - Down: when half of the last `window` frames were congested, one level lower. The
      window starts empty after every change, so a new level always gets a fair trial.
    - Up: after `probe` s without a congested frame, one level higher. A step up that
      congests again within `trial` s doubles the wait before the next try (up to
      `max_probe`), so a connection that sits between two levels settles on the lower one
      instead of flipping between them. A step up that holds halves it again.
    - No change within `hold` s of the previous one.

    Levels are indices into a list of qualities ordered best first. Times are in seconds
    from any monotonic clock.
    """

    def __init__(self, levels, start, now, window=8, hold=2.0, probe=8.0, trial=10.0,
                 max_probe=64.0):
        self.levels = levels
        self.index = int(clamp(start, 0, levels - 1))
        self.window = max(int(window), 2)
        self.hold = hold
        self.base_probe = probe
        self.probe = probe
        self.trial = trial
        self.max_probe = max(max_probe, probe)
        self.recent = collections.deque(maxlen=self.window)
        self.changed = now
        self.clear_since = now
        self.probed = None  # when the latest step up happened, while it is on trial

    def sent(self, congested, now):
        """Record one sent frame. Returns the (possibly new) level index."""
        self.recent.append(bool(congested))
        if congested:
            self.clear_since = now
        elif self.probed is not None and now - self.probed >= self.trial:
            # The step up held: the next probe may come sooner again
            self.probed = None
            self.probe = max(self.base_probe, self.probe / 2.0)
        if now - self.changed < self.hold:
            return self.index
        if sum(self.recent) * 2 >= self.window and self.index < self.levels - 1:
            if self.probed is not None and now - self.probed < self.trial:
                self.probe = min(self.probe * 2.0, self.max_probe)
            self.probed = None
            self.change(self.index + 1, now)
        elif now - self.clear_since >= self.probe and self.index > 0:
            self.probed = now
            self.change(self.index - 1, now)
        return self.index

    def change(self, index, now):
        self.index = index
        self.recent.clear()
        self.changed = now
        self.clear_since = now


class RateMeter:
    """Frames per second and bits per second over the last `span` seconds."""

    def __init__(self, span=2.0):
        self.span = span
        self.events = collections.deque()  # (time, bytes)

    def add(self, size, now):
        self.events.append((now, size))
        self.trim(now)

    def trim(self, now):
        while self.events and now - self.events[0][0] > self.span:
            self.events.popleft()

    def rates(self, now):
        """(fps, bits per second). Both 0 when nothing was sent within the span."""
        self.trim(now)
        if not self.events:
            return 0.0, 0.0
        frames = len(self.events)
        size = sum(event[1] for event in self.events)
        return frames / self.span, 8.0 * size / self.span
