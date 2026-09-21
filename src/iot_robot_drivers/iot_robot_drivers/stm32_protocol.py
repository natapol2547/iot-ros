"""Line protocol between the Nucleo-F401RE sensor board and the Raspberry Pi.

STM32 -> Pi, about 15 Hz:  ``D1:<cm>,D2:<cm>[,V:<volts>]\\r\\n``
    D1 is the left ultrasonic sensor, D2 the right one, in centimetres with one
    decimal. -1.0 means no echo. V is the battery voltage with two decimals and
    is only present when the firmware is built with battery sensing.
    Lines starting with ``#`` are comments, e.g. the boot banner. Two of them report
    a lasting sensor fault, which the D1/D2 values alone cannot show because a failed
    ping is sent as -1.0, the same as open space:
    ``# warning: <left|right> <fault>`` after about 1 s of failed pings (repeated
    every ~10 s while it lasts), and ``# info: <left|right> sensor recovered``.
Pi -> STM32:  ``G:<yaw_deg>,<pitch_deg>\\n``
    Gizmo servo angles in degrees. Ignored by firmware built without servos.

Everything here is pure so it can be unit tested without a serial port.
"""

import math
from dataclasses import dataclass
from typing import Optional

BANNER_PREFIX = "# iot-stm32"
# left is D1, right is D2
SIDES = ("left", "right")
RECOVERED = "sensor recovered"


@dataclass(frozen=True)
class Reading:
    """One measurement line. A distance of None means the sensor heard no echo."""

    left_cm: Optional[float]
    right_cm: Optional[float]
    volts: Optional[float] = None


def _parse_distance(text):
    cm = float(text)
    if not math.isfinite(cm):
        raise ValueError(f"non-finite distance {text!r}")
    # -1.0 is the documented no-echo value. The first firmware reported a timeout
    # as 0.0, and a real HC-SR04 echo is never shorter than about 2 cm, so treat
    # zero and anything negative as no echo too.
    return cm if cm > 0.0 else None


def parse_line(line):
    """Parse one line from the STM32.

    Returns a Reading, or None for blank lines, comments and malformed lines.
    Unknown ``KEY:value`` fields are ignored so newer firmware can add fields.
    """
    line = line.strip()
    if not line or line.startswith("#"):
        return None

    fields = {}
    for field in line.split(","):
        key, sep, value = field.partition(":")
        key = key.strip()
        if not sep or not key or key in fields:
            return None
        fields[key] = value.strip()

    if "D1" not in fields or "D2" not in fields:
        return None
    try:
        left = _parse_distance(fields["D1"])
        right = _parse_distance(fields["D2"])
        volts = None
        if "V" in fields:
            volts = float(fields["V"])
            if not math.isfinite(volts):
                return None
    except ValueError:
        return None
    return Reading(left, right, volts)


def is_comment(line):
    return line.strip().startswith("#")


def is_banner(line):
    """True for the boot banner, which also means the firmware's fault state was reset."""
    return line.strip().startswith(BANNER_PREFIX)


@dataclass(frozen=True)
class SensorStatus:
    """A firmware report that one ultrasonic sensor failed or recovered."""

    side: str
    faulty: bool
    detail: str


def parse_sensor_status(line):
    """Parse ``# warning: <side> <fault>`` or ``# info: <side> sensor recovered``.

    Returns a SensorStatus, or None for any other line.
    """
    text = line.strip()
    if not text.startswith("#"):
        return None
    text = text[1:].strip()
    for prefix, faulty in (("warning:", True), ("info:", False)):
        if not text.startswith(prefix):
            continue
        words = text[len(prefix):].split(None, 1)
        if len(words) != 2 or words[0] not in SIDES:
            return None
        side, detail = words[0], words[1].strip()
        # Other info lines may be added later; only this one ends a fault
        if not faulty and detail != RECOVERED:
            return None
        return SensorStatus(side, faulty, detail)
    return None


def distance_to_range(cm, min_range, max_range, faulty=False):
    """Convert a distance in cm to a sensor_msgs/Range value in metres (REP 117).

    No echo, or an echo beyond max_range, is +inf. Echoes closer than min_range
    are clamped to min_range, as the simulated sensors do. A sensor the firmware
    reports as faulty gives NaN, REP 117's invalid reading, so its -1.0 values do
    not pass for a clear path.
    """
    if faulty:
        return math.nan
    if cm is None:
        return math.inf
    metres = cm / 100.0
    if metres > max_range:
        return math.inf
    return max(metres, min_range)


def format_gizmo(yaw_deg, pitch_deg):
    """Encode a gizmo servo command. One decimal is finer than a hobby servo resolves."""
    return f"G:{yaw_deg:.1f},{pitch_deg:.1f}\n"


def parse_gizmo(line):
    """Decode a ``G:<yaw>,<pitch>`` line into (yaw_deg, pitch_deg), or None if malformed.

    Used by the fake STM32 and the tests; mirrors what the firmware accepts.
    """
    line = line.strip()
    if not line.startswith("G:"):
        return None
    parts = line[2:].split(",")
    if len(parts) != 2:
        return None
    try:
        yaw, pitch = float(parts[0]), float(parts[1])
    except ValueError:
        return None
    if not (math.isfinite(yaw) and math.isfinite(pitch)):
        return None
    return yaw, pitch


def clamp(value, low, high):
    return min(max(value, low), high)


def joint_to_servo_deg(angle_rad, sign, offset_deg, servo_limit_deg=90.0):
    """Map a gizmo joint angle (rad, URDF convention) to a servo angle in degrees.

    sign flips the axis when the servo horn turns the opposite way to the joint;
    offset_deg trims the horn position. The result is clamped to the servo's travel.
    """
    servo = sign * math.degrees(angle_rad) + offset_deg
    return clamp(servo, -servo_limit_deg, servo_limit_deg)
