"""CubeMars AK-series servo-mode CAN protocol and SocketCAN helpers.

The ros2_control driver is cubemars_hardware (C++, wrapped by cubemars_hardware_safe).
This module speaks the same servo protocol for the robot.launch.py pre-flight check and
motor stop, and for cubemars_tool bench tests:

    command  extended ID (mode << 8) | can_id, big-endian payload
             speed mode (3): int32 electrical RPM
    status   extended ID (0x29 << 8) | can_id, sent by the motor at its configured rate
             int16 position [0.1 deg], int16 speed [10 ERPM], int16 current [0.01 A],
             int8 temperature [deg C], uint8 error code

Only the Python standard library is used, so it runs in any environment with AF_CAN.
"""

import fcntl
import json
import math
import socket
import struct
import subprocess
import time
from collections import Counter
from dataclasses import dataclass, field
from glob import glob

MODE_CURRENT = 1
MODE_SPEED = 3
STATUS = 0x29
# Command modes 0 (duty cycle) to 6 (position-velocity loop): frames that only a
# controlling program sends, never a motor
COMMAND_MODES = range(0, 7)

ERRORS = {
    0: "no fault",
    1: "motor over-temperature",
    2: "over-current",
    3: "over-voltage",
    4: "under-voltage",
    5: "encoder fault",
    6: "MOSFET over-temperature",
    7: "motor stall",
}

# cubemars_hardware refuses speed commands at or above this magnitude
MAX_ERPM = 100000
# AK45-10 datasheet: 14 pole pairs, 10:1 planetary reduction
POLE_PAIRS = 14
GEAR_RATIO = 10

ARPHRD_CAN = 280
IFF_UP = 0x1
SIOCGIFFLAGS = 0x8913
SIOCGIFHWADDR = 0x8927
CAN_FRAME = struct.Struct("=IB3x8s")


class CanCheckError(RuntimeError):
    """The CAN interface or the motors are not ready. The message says how to fix it."""


def erpm_per_rad_s(pole_pairs=POLE_PAIRS, gear_ratio=GEAR_RATIO):
    """Electrical RPM of the motor per rad/s at the gearbox output, as cubemars_hardware uses."""
    return pole_pairs * gear_ratio * 60.0 / (2.0 * math.pi)


def encode_speed(can_id, erpm):
    """Speed-loop command: (extended arbitration ID, payload)."""
    erpm = int(erpm)
    if abs(erpm) >= MAX_ERPM:
        raise ValueError(f"{erpm} ERPM is beyond the servo-mode limit of {MAX_ERPM}")
    return (MODE_SPEED << 8) | can_id, struct.pack(">i", erpm)


def encode_release(can_id):
    """Zero current: the motor stops driving and freewheels."""
    return (MODE_CURRENT << 8) | can_id, struct.pack(">i", 0)


def stop_motors(bus, can_ids, brake_time=0.3, period=0.02):
    """Hold zero speed for brake_time seconds, then release the motors.

    The same sequence as cubemars_hardware_safe uses when the hardware component stops.
    Raises OSError if a frame cannot be sent (interface down, transmit queue full).
    """
    end = time.monotonic() + brake_time
    while True:
        for can_id in can_ids:
            bus.send(*encode_speed(can_id, 0))
        if time.monotonic() + period > end:
            break
        time.sleep(period)
    for can_id in can_ids:
        bus.send(*encode_release(can_id))


@dataclass(frozen=True)
class Status:
    can_id: int
    # Gearbox output, relative to the drive's origin, limited to +-3200 deg. The
    # single-encoder AK45-10 forgets its position when powered off: after power-up it
    # is only defined within one rotor turn (36 deg at the output)
    position_deg: float
    speed_erpm: float
    current_a: float
    temperature_c: int
    error: int

    @property
    def error_text(self):
        return ERRORS.get(self.error, f"unknown fault {self.error}")

    def output_velocity(self, pole_pairs=POLE_PAIRS, gear_ratio=GEAR_RATIO):
        """Gearbox output speed in rad/s, before any direction flip."""
        return self.speed_erpm / erpm_per_rad_s(pole_pairs, gear_ratio)


def decode_status(arbitration_id, data):
    """Decode a servo-mode status frame, or return None for any other frame."""
    if arbitration_id >> 8 != STATUS or len(data) < 8:
        return None
    position, speed, current, temperature, error = struct.unpack(">hhhbB", bytes(data[:8]))
    return Status(
        can_id=arbitration_id & 0xFF,
        position_deg=position * 0.1,
        speed_erpm=speed * 10.0,
        current_a=current * 0.01,
        temperature_c=temperature,
        error=error)


# SocketCAN ------------------------------------------------------------------------


class CanBus:
    """Raw SocketCAN socket for extended frames. Use as a context manager."""

    def __init__(self, interface, status_only=False):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        if status_only:
            # Let the kernel drop everything except extended status frames
            can_filter = struct.pack(
                "=II", (STATUS << 8) | socket.CAN_EFF_FLAG,
                0xFF00 | socket.CAN_EFF_FLAG)
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, can_filter)
        try:
            self.sock.bind((interface,))
        except OSError:
            self.sock.close()
            raise

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        self.sock.close()

    def send(self, arbitration_id, data):
        frame = CAN_FRAME.pack(arbitration_id | socket.CAN_EFF_FLAG, len(data), bytes(data))
        self.sock.send(frame)

    def recv(self, timeout):
        """Next extended frame as (arbitration_id, data), or None after timeout seconds."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            self.sock.settimeout(remaining)
            try:
                frame = self.sock.recv(CAN_FRAME.size)
            except socket.timeout:
                return None
            can_id, length, data = CAN_FRAME.unpack(frame)
            if can_id & (socket.CAN_ERR_FLAG | socket.CAN_RTR_FLAG):
                continue
            if can_id & socket.CAN_EFF_FLAG:
                return can_id & socket.CAN_EFF_MASK, data[:length]


def interface_ioctl(interface, request):
    """Return the 16-bit field of struct ifreq filled by a SIOCGIF* request.

    ioctl follows the caller's network namespace, unlike /sys/class/net, so the checks
    also work inside containers and the vcan test namespace. Raises OSError (ENODEV)
    when the interface does not exist.
    """
    ifreq = struct.pack("16s24x", interface.encode()[:15])
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        result = fcntl.ioctl(sock.fileno(), request, ifreq)
    return struct.unpack_from("H", result, 16)[0]


def interface_type(interface):
    """ARPHRD_* link type of an interface, or None if it does not exist."""
    try:
        # SIOCGIFHWADDR: sa_family of the hardware address is the link type
        return interface_ioctl(interface, SIOCGIFHWADDR)
    except OSError:
        return None


def list_can_interfaces():
    return sorted(name for _, name in socket.if_nameindex()
                  if interface_type(name) == ARPHRD_CAN)


def link_details(interface):
    """CAN controller state and bitrate from `ip -details`, or {} if unavailable."""
    try:
        result = subprocess.run(
            ["ip", "-json", "-details", "link", "show", "dev", interface],
            capture_output=True, text=True, timeout=2.0, check=True)
        info = json.loads(result.stdout)[0].get("linkinfo", {}).get("info_data", {})
    except (OSError, subprocess.SubprocessError, ValueError, IndexError):
        return {}
    return {
        "state": info.get("state"),
        "bitrate": info.get("bittiming", {}).get("bitrate"),
    }


def check_interface(interface, bitrate=1000000):
    """Raise CanCheckError unless the interface exists, is CAN, is up and not bus-off."""
    guide = "See docs/hardware.md, section 'CAN adapter'."
    link_type = interface_type(interface)
    if link_type is None:
        present = list_can_interfaces()
        serial_adapters = sorted(glob("/dev/ttyUSB*"))
        lines = [f"CAN interface '{interface}' does not exist."]
        lines.append(
            f"CAN interfaces present: {', '.join(present)}" if present
            else "No CAN interfaces are present: the USB-CAN adapter is unplugged or "
                 "is not a SocketCAN (gs_usb/candleLight, usb_8dev, PEAK) device.")
        if serial_adapters:
            lines.append(
                f"Serial adapters found: {', '.join(serial_adapters)}. A CH340-based "
                "USB-CAN-A adapter is a serial device, not SocketCAN; it only works "
                "through slcand if its firmware speaks the slcan protocol.")
        lines.append(guide)
        raise CanCheckError("\n".join(lines))
    if link_type != ARPHRD_CAN:
        raise CanCheckError(f"'{interface}' exists but is not a CAN interface. {guide}")

    if not interface_ioctl(interface, SIOCGIFFLAGS) & IFF_UP:
        raise CanCheckError(
            f"CAN interface '{interface}' is down. Bring it up with\n"
            f"  sudo ip link set {interface} up type can bitrate {bitrate}\n"
            "or `pixi run -e robot can-up`, or install deploy/ so it comes up at boot. "
            + guide)

    details = link_details(interface)
    if details.get("state") == "BUS-OFF":
        raise CanCheckError(
            f"CAN interface '{interface}' is BUS-OFF: the controller saw too many errors. "
            "Check CANH/CANL wiring, 120 ohm termination at both ends and that every "
            f"node runs at {bitrate} bit/s, then restart the link "
            f"(sudo ip link set {interface} down; pixi run -e robot can-up).")
    if details.get("bitrate") and details["bitrate"] != bitrate:
        raise CanCheckError(
            f"CAN interface '{interface}' runs at {details['bitrate']} bit/s; the motors "
            f"expect {bitrate}. Re-create it with\n"
            f"  sudo ip link set {interface} down\n"
            f"  sudo ip link set {interface} up type can bitrate {bitrate}")
    return details


@dataclass
class Heard:
    """What listen() saw on the bus."""

    # Latest status per motor CAN ID
    status: dict = field(default_factory=dict)
    # Status frames per CAN ID, for the motors' status rate
    frames: Counter = field(default_factory=Counter)
    # CAN IDs that another program sent servo commands to
    commanded: set = field(default_factory=set)
    # Seconds actually spent listening
    elapsed: float = 0.0

    def rate(self, can_id):
        return self.frames[can_id] / self.elapsed if self.elapsed > 0.0 else 0.0


def listen(interface, duration, until=(), status_only=True):
    """Listen for up to duration seconds, or until every CAN ID in until has reported.

    With status_only the kernel drops all but status frames. Without it, command frames
    are noted in Heard.commanded, which shows whether another program (robot.launch.py,
    the systemd service) is driving the motors. The own socket never sees its own frames.
    """
    wanted = set(until)
    heard = Heard()
    start = time.monotonic()
    deadline = start + duration
    with CanBus(interface, status_only=status_only) as bus:
        while (not wanted or wanted - set(heard.status)) and (
                remaining := deadline - time.monotonic()) > 0.0:
            frame = bus.recv(remaining)
            if frame is None:
                break
            status = decode_status(*frame)
            if status is not None:
                heard.status[status.can_id] = status
                heard.frames[status.can_id] += 1
            elif frame[0] >> 8 in COMMAND_MODES:
                heard.commanded.add(frame[0] & 0xFF)
    heard.elapsed = time.monotonic() - start
    return heard


def wait_for_status(interface, can_ids, timeout):
    """Return {can_id: Status} for the can_ids heard, stopping early once all are."""
    return listen(interface, timeout, until=can_ids).status


def describe(can_id, names=None):
    """'wheel_joint_left (CAN ID 12)' when names knows the ID, else 'CAN ID 12'."""
    name = (names or {}).get(can_id)
    return f"{name} (CAN ID {can_id})" if name else f"CAN ID {can_id}"


def missing_motor_help(details, others, names=None):
    """Lines explaining why motors send no status, given the other IDs that do."""
    lines = []
    if others:
        lines.append(
            "Status frames were heard from " + ", ".join(
                describe(can_id, names) if can_id in (names or {})
                else f"CAN ID {can_id} (not in motors.yaml)" for can_id in others) + ".")
        lines.append(
            "If a motor has a different ID than motors.yaml says, find out which motor "
            "has which ID with `pixi run -e robot can-identify` and correct "
            "src/iot_robot_bringup/config/motors.yaml (docs/todo.md).")
    if details.get("state") in ("ERROR-PASSIVE", "ERROR-WARNING"):
        lines.append(
            f"The controller is {details['state']}, which usually means nothing "
            "acknowledges its frames: motors unpowered, wrong bitrate or no termination.")
    lines.append(
        "Check that the motors are powered (battery connected, E-stop released), "
        "CANH/CANL are not swapped, the bus has 120 ohm termination at both ends, and "
        "that each motor is in servo mode with CAN status feedback enabled (100-200 Hz) "
        "in the CubeMars Upper Computer. See docs/hardware.md, section "
        "'Motor configuration'.")
    return lines


def fault_help(faults, names=None):
    return ("Motor fault: " + "; ".join(
        f"{describe(status.can_id, names)}: {status.error_text}" for status in faults)
        + ". Resolve it (battery voltage, motor temperature, a blocked wheel or gizmo "
        "joint) before driving; power-cycle the motors if the fault does not clear.")


def preflight(interface, can_ids, timeout=1.0, bitrate=1000000, names=None):
    """Check the CAN link and that every motor reports status without a fault.

    names ({can_id: joint name}, e.g. from motors.yaml) labels the motors in messages.
    Returns {can_id: Status}. Raises CanCheckError with remediation text otherwise.
    """
    details = check_interface(interface, bitrate)
    heard = wait_for_status(interface, can_ids, timeout)
    missing = [can_id for can_id in can_ids if can_id not in heard]
    if missing:
        others = sorted(set(heard) - set(can_ids))
        lines = [
            "No status frames from " + ", ".join(describe(can_id, names) for can_id in missing)
            + f" on '{interface}' within {timeout:g} s."]
        lines += missing_motor_help(details, others, names)
        raise CanCheckError("\n".join(lines))
    faults = [heard[can_id] for can_id in can_ids if heard[can_id].error]
    if faults:
        raise CanCheckError(fault_help(faults, names))
    return {can_id: heard[can_id] for can_id in can_ids}
