"""Emulate CubeMars AK-series motors in servo mode on a virtual SocketCAN bus.

For testing the robot's CAN stack without motors: the cubemars_hardware_safe ros2_control
plugin, cubemars_tool and the robot.launch.py pre-flight check. Every emulated motor
sends status frames (default 100 Hz) and follows the servo-mode commands sent to it:

    mode 0 duty cycle       speed proportional to the duty cycle
    mode 1 current          zero releases the motor; it coasts to a stop either way
    mode 2 brake current    holds still
    mode 3 speed            runs at the commanded electrical RPM
    mode 4 position         moves to the target at --position-speed
    mode 5 set origin       0 temporary, 1 permanent, 2 restore the default origin
    mode 6 position-speed   moves to the target at the commanded speed

The position is kept at the gearbox output in degrees and the speed converted with 14
pole pairs and a 10:1 reduction, the same conventions as cubemars_hardware. Faults,
silence (no status frames), a blocked joint and a load current can be injected, from the
command line or through the FakeCubeMars class:

    sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up
    ros2 run iot_robot_drivers fake_cubemars --interface vcan0 --ids 10 11 12 13 \\
        --position 12=30 --fault 13=7@5
"""

import argparse
import json
import math
import signal
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass

from iot_robot_drivers.cubemars import (
    CanBus, ERRORS, GEAR_RATIO, MODE_CURRENT, MODE_SPEED, POLE_PAIRS, STATUS)

MODE_DUTY = 0
MODE_BRAKE = 2
MODE_POSITION = 4
MODE_SET_ORIGIN = 5
MODE_POSITION_SPEED = 6
MODE_NAMES = {
    MODE_DUTY: "duty", MODE_CURRENT: "current", MODE_BRAKE: "brake", MODE_SPEED: "speed",
    MODE_POSITION: "position", MODE_SET_ORIGIN: "set origin",
    MODE_POSITION_SPEED: "position-speed",
}

# Set-origin payload byte (servo mode 5)
ORIGIN_TEMPORARY = 0
ORIGIN_PERMANENT = 1
ORIGIN_DEFAULT = 2

DEFAULT_IDS = (10, 11, 12, 13)
DEFAULT_RATE = 100.0
# Output speed of the position loop (mode 4), which the real motor runs at full speed
DEFAULT_POSITION_SPEED = 720.0
# Speed at duty cycle 1.0: KV 75 rpm/V at 24 V, times 14 pole pairs (AK45-10 datasheet)
FULL_DUTY_ERPM = 75.0 * 24.0 * POLE_PAIRS
# A released motor coasts to a stop with this time constant, in seconds
COAST_TIME_CONSTANT = 0.1
# The status frame reports the position in servo mode within +-3200 deg
MAX_STATUS_POSITION = 3200.0


class FakeBusError(ValueError):
    """The interface cannot be used for emulated motors. The message says why."""


def output_dps(erpm, pole_pairs=POLE_PAIRS, gear_ratio=GEAR_RATIO):
    """Gearbox output speed in deg/s for a motor speed in electrical RPM."""
    return erpm * 6.0 / (pole_pairs * gear_ratio)


def erpm_from_dps(dps, pole_pairs=POLE_PAIRS, gear_ratio=GEAR_RATIO):
    return dps * pole_pairs * gear_ratio / 6.0


def _saturate(value, low, high):
    return max(low, min(high, int(round(value))))


def encode_status(can_id, position_deg, speed_erpm, current_a, temperature_c=25, error=0):
    """Servo-mode status frame: (extended arbitration ID, payload)."""
    position_deg = max(-MAX_STATUS_POSITION, min(MAX_STATUS_POSITION, position_deg))
    payload = struct.pack(
        ">hhhbB",
        _saturate(position_deg * 10.0, -32768, 32767),
        _saturate(speed_erpm / 10.0, -32768, 32767),
        _saturate(current_a * 100.0, -32768, 32767),
        _saturate(temperature_c, -128, 127),
        error & 0xFF)
    return (STATUS << 8) | can_id, payload


def encode_command(can_id, mode, value, speed_erpm=0.0, acceleration=0.0):
    """Servo-mode command frame for tests and bench use: (arbitration ID, payload).

    value is the duty cycle (0), current in A (1, 2), speed in ERPM (3), position in deg
    (4, 6) or the origin type (5). Mode 6 also takes the speed in ERPM and the
    acceleration in ERPM/s.
    """
    arbitration_id = (mode << 8) | can_id
    if mode == MODE_DUTY:
        return arbitration_id, struct.pack(">i", int(value * 100000.0))
    if mode in (MODE_CURRENT, MODE_BRAKE):
        return arbitration_id, struct.pack(">i", int(value * 1000.0))
    if mode == MODE_SPEED:
        return arbitration_id, struct.pack(">i", int(value))
    if mode == MODE_POSITION:
        return arbitration_id, struct.pack(">i", int(value * 10000.0))
    if mode == MODE_SET_ORIGIN:
        return arbitration_id, bytes([int(value)])
    if mode == MODE_POSITION_SPEED:
        return arbitration_id, struct.pack(
            ">ihh", int(value * 10000.0), int(speed_erpm / 10.0), int(acceleration / 10.0))
    raise ValueError(f"unknown servo mode {mode}")


def decode_command(mode, data):
    """Main value of a servo command in the units of encode_command, or None if too short."""
    data = bytes(data)
    if mode == MODE_SET_ORIGIN:
        return data[0] if data else None
    if len(data) < 4:
        return None
    raw = struct.unpack(">i", data[:4])[0]
    if mode == MODE_DUTY:
        return raw / 100000.0
    if mode in (MODE_CURRENT, MODE_BRAKE):
        return raw / 1000.0
    if mode == MODE_SPEED:
        return float(raw)
    if mode in (MODE_POSITION, MODE_POSITION_SPEED):
        return raw / 10000.0
    return None


def link_kind(interface):
    """Link kind from `ip -details` ('vcan', 'can', ...), or None if unknown."""
    try:
        result = subprocess.run(
            ["ip", "-json", "-details", "link", "show", "dev", interface],
            capture_output=True, text=True, timeout=2.0, check=True)
        return json.loads(result.stdout)[0].get("linkinfo", {}).get("info_kind")
    except (OSError, subprocess.SubprocessError, ValueError, IndexError, AttributeError):
        return None


def check_virtual(interface, allow_real_bus=False):
    """Raise FakeBusError unless the interface is a vcan (or allow_real_bus is set)."""
    if allow_real_bus:
        return
    kind = link_kind(interface)
    if kind != "vcan":
        raise FakeBusError(
            f"'{interface}' is not a virtual CAN interface (link kind: {kind or 'unknown'}). "
            "Emulated status frames on a real bus would hide missing or faulty motors "
            "from the robot, so fake_cubemars only runs on vcan. Create one with\n"
            "  sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up\n"
            "or pass --allow-real-bus (allow_real_bus=True) for a bus without motors.")


@dataclass(frozen=True)
class Command:
    """A servo command frame received by an emulated motor."""

    time: float
    can_id: int
    mode: int
    data: bytes

    @property
    def value(self):
        return decode_command(self.mode, self.data)

    def __str__(self):
        name = MODE_NAMES.get(self.mode, f"mode {self.mode}")
        return f"CAN ID {self.can_id}: {name} {self.value}"


@dataclass
class FakeMotor:
    """State of one emulated servo-mode motor. Modify it with FakeCubeMars.lock held."""

    can_id: int
    # Output position in degrees since power-up, before any origin
    encoder_deg: float = 0.0
    origin_deg: float = 0.0
    permanent_origin_deg: float = 0.0
    speed_erpm: float = 0.0
    temperature_c: int = 25
    error: int = 0
    # idle (released), duty, current, brake, speed, position
    mode: str = "idle"
    command_current_a: float = 0.0
    target_erpm: float = 0.0
    target_deg: float = 0.0
    move_dps: float = DEFAULT_POSITION_SPEED
    position_speed_dps: float = DEFAULT_POSITION_SPEED
    # Injected behaviour
    silent: bool = False
    ignore_origin: bool = False
    stuck: bool = False
    load_current_a: float = 0.0
    last_command: float = None
    pole_pairs: int = POLE_PAIRS
    gear_ratio: int = GEAR_RATIO

    @property
    def position_deg(self):
        """Position relative to the origin, as the status frame reports it."""
        return self.encoder_deg - self.origin_deg

    def set_position(self, position_deg):
        """Move the joint by hand to position_deg relative to the origin."""
        self.encoder_deg = position_deg + self.origin_deg

    def release(self):
        self.mode = "idle"
        self.command_current_a = 0.0

    def power_cycle(self, position_deg=0.0):
        """Power off and on: the motor forgets its temporary origin and position."""
        self.release()
        self.speed_erpm = 0.0
        self.error = 0
        self.origin_deg = self.permanent_origin_deg
        self.encoder_deg = position_deg + self.origin_deg

    def command(self, mode, data, now=None):
        """Apply a servo command frame. Returns False if the frame was ignored."""
        value = decode_command(mode, data)
        if value is None:
            return False
        self.last_command = time.monotonic() if now is None else now
        if mode == MODE_SET_ORIGIN:
            if self.ignore_origin:
                return False
            if value == ORIGIN_DEFAULT:
                self.origin_deg = self.permanent_origin_deg = 0.0
            else:
                self.origin_deg = self.encoder_deg
                if value == ORIGIN_PERMANENT:
                    self.permanent_origin_deg = self.origin_deg
            # A position target is kept as a number, now relative to the new origin, so
            # the joint jumps unless it was released first. The manual does not say what
            # the drive does; this is the worse case
            return True
        if self.error:
            # A faulted drive does not drive until the fault clears
            return False
        if mode == MODE_DUTY:
            self.mode, self.target_erpm = "speed", value * FULL_DUTY_ERPM
        elif mode == MODE_CURRENT:
            self.command_current_a = value
            self.mode = "current" if value != 0.0 else "idle"
        elif mode == MODE_BRAKE:
            self.mode, self.command_current_a = "brake", value
        elif mode == MODE_SPEED:
            self.mode, self.target_erpm = "speed", value
        elif mode == MODE_POSITION:
            self.mode, self.target_deg = "position", value
            self.move_dps = self.position_speed_dps
        elif mode == MODE_POSITION_SPEED:
            speed_erpm = abs(struct.unpack(">h", bytes(data[4:6]))[0] * 10.0) \
                if len(data) >= 6 else 0.0
            self.mode, self.target_deg = "position", value
            self.move_dps = min(self.position_speed_dps, output_dps(
                speed_erpm, self.pole_pairs, self.gear_ratio)) if speed_erpm \
                else self.position_speed_dps
        else:
            return False
        return True

    def step(self, dt):
        """Advance the motion by dt seconds."""
        dps = 0.0
        if self.error or self.stuck or self.mode == "brake":
            dps = 0.0
        elif self.mode == "speed":
            dps = output_dps(self.target_erpm, self.pole_pairs, self.gear_ratio)
        elif self.mode == "position":
            remaining = self.target_deg - self.position_deg
            step = self.move_dps * dt
            if abs(remaining) <= step or dt <= 0.0:
                self.encoder_deg = self.target_deg + self.origin_deg
                dps = 0.0
            else:
                dps = math.copysign(self.move_dps, remaining)
        else:
            # Released or current mode: coast
            dps = output_dps(self.speed_erpm, self.pole_pairs, self.gear_ratio) * math.exp(
                -dt / COAST_TIME_CONSTANT) if dt > 0.0 else 0.0
        self.encoder_deg += dps * dt
        self.speed_erpm = erpm_from_dps(dps, self.pole_pairs, self.gear_ratio)

    @property
    def current_a(self):
        """Reported current: commanded in current and brake mode, the load while driving."""
        if self.error:
            return 0.0
        if self.mode in ("current", "brake"):
            return self.command_current_a
        if self.mode in ("speed", "position"):
            return self.load_current_a
        return 0.0

    def status(self):
        return encode_status(
            self.can_id, self.position_deg, self.speed_erpm, self.current_a,
            self.temperature_c, self.error)


class FakeCubeMars:
    """Emulated motors on a SocketCAN interface, served by a background thread.

    Use as a context manager, or call close(). Every command frame received for an
    emulated motor is kept in a log (commands()). Motor state is in motors[can_id];
    hold lock while reading or changing it.
    """

    def __init__(self, interface="vcan0", ids=DEFAULT_IDS, rate=DEFAULT_RATE,
                 position_speed=DEFAULT_POSITION_SPEED, timeout=0.0, allow_real_bus=False,
                 verbose=False):
        check_virtual(interface, allow_real_bus)
        if rate <= 0.0:
            raise ValueError("rate must be positive")
        self.interface = interface
        self.rate = rate
        # The motors' own CAN timeout in seconds (0: off): released without commands
        self.timeout = timeout
        self.verbose = verbose
        self.lock = threading.RLock()
        self.motors = {
            can_id: FakeMotor(can_id, position_speed_dps=position_speed) for can_id in ids}
        self._log = []
        self._last_printed = {}
        self._events = []
        self._stopping = threading.Event()
        self.bus = CanBus(interface)
        self._thread = threading.Thread(target=self._run, name="fake_cubemars", daemon=True)
        self._thread.start()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        if not self._stopping.is_set():
            self._stopping.set()
            self._thread.join(timeout=2.0)
            self.bus.close()

    # Injection -------------------------------------------------------------------

    def after(self, delay, action):
        """Run action() with the lock held on the emulator thread after delay seconds."""
        with self.lock:
            self._events.append((time.monotonic() + delay, action))

    def inject_fault(self, can_id, code, after=0.0):
        """Report fault code (1-7, 0 clears) from after seconds on; a fault stops the motor."""
        def fault():
            motor = self.motors[can_id]
            motor.error = code
            if code:
                motor.release()
        self.after(after, fault)

    def silence(self, can_id, after=0.0, silent=True):
        """Stop (or resume) sending status frames for can_id after seconds."""
        self.after(after, lambda: setattr(self.motors[can_id], "silent", silent))

    def set_position(self, can_id, position_deg):
        with self.lock:
            self.motors[can_id].set_position(position_deg)

    def power_cycle(self, can_id, position_deg=0.0):
        with self.lock:
            self.motors[can_id].power_cycle(position_deg)

    # Inspection ------------------------------------------------------------------

    def motor(self, can_id):
        return self.motors[can_id]

    def commands(self, can_id=None, mode=None, since=None):
        """Logged command frames, optionally for one CAN ID, mode and after a time."""
        with self.lock:
            return [
                command for command in self._log
                if (can_id is None or command.can_id == can_id)
                and (mode is None or command.mode == mode)
                and (since is None or command.time >= since)]

    def clear_log(self):
        with self.lock:
            self._log.clear()

    def wait_for(self, condition, timeout, poll=0.01):
        """Poll condition() until it is true (returns True) or timeout seconds pass."""
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() >= deadline:
                return False
            time.sleep(poll)
        return True

    # Emulator thread -------------------------------------------------------------

    def _run(self):
        period = 1.0 / self.rate
        last = time.monotonic()
        next_tick = last
        while not self._stopping.is_set():
            now = time.monotonic()
            if now >= next_tick:
                self._tick(now, now - last)
                last = now
                next_tick += period
                if next_tick < now:
                    next_tick = now + period
                continue
            try:
                frame = self.bus.recv(min(next_tick - now, 0.1))
            except OSError:
                if self._stopping.is_set():
                    return
                raise
            if frame is not None:
                self._receive(*frame, time.monotonic())

    def _tick(self, now, dt):
        frames = []
        with self.lock:
            due = [event for event in self._events if event[0] <= now]
            self._events = [event for event in self._events if event[0] > now]
            for _, action in sorted(due, key=lambda event: event[0]):
                action()
            for motor in self.motors.values():
                if (self.timeout > 0.0 and motor.mode != "idle" and motor.last_command
                        is not None and now - motor.last_command > self.timeout):
                    motor.release()
                motor.step(dt)
                if not motor.silent:
                    frames.append(motor.status())
        for frame in frames:
            try:
                self.bus.send(*frame)
            except OSError:
                # Transmit queue full or interface down: the frame is lost, as on a bus
                pass

    def _receive(self, arbitration_id, data, now):
        mode, can_id = arbitration_id >> 8, arbitration_id & 0xFF
        if mode not in MODE_NAMES:
            # Status frames of other (emulated or real) motors, or unrelated traffic
            return
        with self.lock:
            motor = self.motors.get(can_id)
            if motor is None:
                return
            command = Command(now, can_id, mode, bytes(data))
            self._log.append(command)
            motor.command(mode, data, now)
            if self.verbose:
                key = (mode, command.value)
                if self._last_printed.get(can_id) != key:
                    self._last_printed[can_id] = key
                    print(command, flush=True)


# Command line --------------------------------------------------------------------


def _parse_assignment(text, convert, name):
    """'ID=VALUE[@SECONDS]' -> (id, value, seconds)."""
    try:
        key, _, rest = text.partition("=")
        value, _, delay = rest.partition("@")
        if not value:
            raise ValueError(text)
        return int(key, 0), convert(value), float(delay) if delay else 0.0
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"{name} expects ID=VALUE (or ID=VALUE@SECONDS), got '{text}'") from None


def _parse_delayed_id(text):
    """'ID[@SECONDS]' -> (id, seconds)."""
    try:
        key, _, delay = text.partition("@")
        return int(key, 0), float(delay) if delay else 0.0
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"expected ID or ID@SECONDS, got '{text}'") from None


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        epilog="IDs are the motors' CAN IDs; SECONDS delays an injection from start-up.")
    parser.add_argument("--interface", default="vcan0", help="SocketCAN interface (vcan)")
    parser.add_argument("--ids", type=lambda text: int(text, 0), nargs="+",
                        default=list(DEFAULT_IDS), help="CAN IDs to emulate")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE,
                        help="status frames per second per motor")
    parser.add_argument("--position-speed", type=float, default=DEFAULT_POSITION_SPEED,
                        metavar="DEG_PER_S", help="output speed of the position loop")
    parser.add_argument("--timeout-ms", type=float, default=0.0,
                        help="release a motor that gets no command for this long (0: off)")
    parser.add_argument("--position", action="append", default=[], metavar="ID=DEG",
                        type=lambda text: _parse_assignment(text, float, "--position"),
                        help="start position at the output in degrees")
    parser.add_argument("--fault", action="append", default=[], metavar="ID=CODE[@S]",
                        type=lambda text: _parse_assignment(text, int, "--fault"),
                        help="report fault CODE (1-7) from S seconds on: "
                        + ", ".join(f"{code} {text}" for code, text in ERRORS.items() if code))
    parser.add_argument("--silent", action="append", default=[], metavar="ID[@S]",
                        type=_parse_delayed_id, help="stop sending status from S seconds on")
    parser.add_argument("--load-current", action="append", default=[], metavar="ID=A",
                        type=lambda text: _parse_assignment(text, float, "--load-current"),
                        help="current reported while the motor drives")
    parser.add_argument("--stuck", action="append", default=[], metavar="ID",
                        type=lambda text: int(text, 0), help="the joint is blocked")
    parser.add_argument("--ignore-origin", action="append", default=[], metavar="ID",
                        type=lambda text: int(text, 0), help="ignore set-origin commands")
    parser.add_argument("--allow-real-bus", action="store_true",
                        help="run on an interface that is not vcan (a bus without motors)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="print every change of command received")
    args = parser.parse_args(argv)
    used = [entry[0] for entry in args.position + args.fault + args.silent
            + args.load_current] + args.stuck + args.ignore_origin
    unknown = sorted(set(used) - set(args.ids))
    if unknown:
        parser.error(f"CAN IDs {unknown} are not emulated (--ids {' '.join(map(str, args.ids))})")
    return args


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        fake = FakeCubeMars(
            args.interface, args.ids, rate=args.rate, position_speed=args.position_speed,
            timeout=args.timeout_ms / 1000.0, allow_real_bus=args.allow_real_bus,
            verbose=args.verbose)
    except FakeBusError as err:
        print(f"fake_cubemars: {err}", file=sys.stderr)
        return 2
    except OSError as err:
        print(f"fake_cubemars: cannot open '{args.interface}': {err.strerror}",
              file=sys.stderr)
        return 2

    with fake.lock:
        for can_id, degrees, _ in args.position:
            fake.motors[can_id].set_position(degrees)
        for can_id, current, _ in args.load_current:
            fake.motors[can_id].load_current_a = current
        for can_id in args.stuck:
            fake.motors[can_id].stuck = True
        for can_id in args.ignore_origin:
            fake.motors[can_id].ignore_origin = True
    for can_id, code, delay in args.fault:
        fake.inject_fault(can_id, code, after=delay)
    for can_id, delay in args.silent:
        fake.silence(can_id, after=delay)

    stopping = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopping.set())
    print(f"fake_cubemars: emulating CAN IDs {', '.join(map(str, args.ids))} on "
          f"{args.interface} at {args.rate:g} Hz. Ctrl+C stops.", flush=True)
    try:
        while not stopping.wait(0.2):
            if not fake._thread.is_alive():
                print("fake_cubemars: the emulator thread stopped", file=sys.stderr)
                return 1
    finally:
        fake.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
