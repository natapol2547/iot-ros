"""Bench tool for the CubeMars motors: check, watch, identify, jog or stop them.

The motor CAN IDs and the joint names printed next to them come from motors.yaml
(iot_robot_bringup/config/motors.yaml, or --motors PATH). The tool talks to the motors
directly over SocketCAN, so stop the robot first (Ctrl-C, or `sudo systemctl stop
iot-robot`): two programs sending commands to the same motor fight each other. identify
and jog refuse to run while another program commands the motors.

    ros2 run iot_robot_drivers cubemars_tool check                # every motor reporting?
    ros2 run iot_robot_drivers cubemars_tool watch                # live status per motor
    ros2 run iot_robot_drivers cubemars_tool identify --write     # which ID is which joint
    ros2 run iot_robot_drivers cubemars_tool jog --joint wheel_joint_left --velocity 1.0
    ros2 run iot_robot_drivers cubemars_tool stop
"""

import argparse
import math
import os
import sys
import time

from iot_robot_drivers import motor_config
from iot_robot_drivers.cubemars import (
    ARPHRD_CAN, GEAR_RATIO, IFF_UP, POLE_PAIRS, SIOCGIFFLAGS, CanBus, CanCheckError,
    check_interface, decode_status, describe, encode_release, encode_speed, erpm_per_rad_s,
    fault_help, interface_ioctl, interface_type, listen, missing_motor_help, stop_motors)
from iot_robot_drivers.motor_config import GIZMO_JOINTS, MotorConfigError

SEND_RATE = 50.0  # Hz, the same rate the controller manager commands the motors
# How long identify and jog listen for status frames and for another program's commands
BUS_CHECK_TIME = 0.3
# Below the controller manager's update rate the driver warns on every cycle
MIN_STATUS_RATE = 50.0
# What a positive command should do, for the direction advice after a jog
POSITIVE_MOTION = {
    "wheel_joint_left": "turn the left wheel so that it drives the robot forward",
    "wheel_joint_right": "turn the right wheel so that it drives the robot forward",
    "gizmo_yaw_joint": "turn the gizmo to the left (counter-clockwise seen from above)",
    "gizmo_pitch_joint": "tilt the camera down",
}


# A motor whose measured output speed differs from the command by more than this
# (rad/s), or by more than the command itself, is not following its speed loop. A drive
# whose encoder or phase calibration does not match the motor does exactly that: it
# turns the wrong way and accelerates to near full speed whatever it is told.
RUNAWAY_MARGIN = 1.0


class MotorStopped(Exception):
    """A motor reported a fault, drew too much current or ran away during a move.

    release_only: brake no further, only release. A runaway motor's speed loop pushes
    the wrong way, so a zero-speed command would drive it on instead of stopping it.
    """

    def __init__(self, message, release_only=False):
        super().__init__(message)
        self.release_only = release_only


def check_following(status, commanded, args):
    """Raise MotorStopped if the motor's speed does not follow commanded (rad/s)."""
    measured = status.output_velocity(args.pole_pairs, args.gear_ratio)
    if abs(measured - commanded) > max(RUNAWAY_MARGIN, abs(commanded)):
        raise MotorStopped(
            f"it turned at {measured:+.2f} rad/s while {commanded:+.2f} rad/s was "
            "commanded. Its speed loop does not follow the command, which points at the "
            "drive's encoder or phase calibration; recalibrate the motor with the CubeMars "
            "Upper Computer before using it (docs/hardware.md)", release_only=True)


def end_move(bus, can_id, release_only):
    """Brake and release the motor after a move, or only release a runaway one."""
    if release_only:
        bus.send(*encode_release(can_id))
    else:
        stop_motors(bus, [can_id])


def load_motors(args, required=True):
    """The MotorConfig from --motors or the default path.

    Without required a broken or missing file prints a warning and returns None, so
    that stop and watch still work.
    """
    try:
        return motor_config.load(args.motors)
    except MotorConfigError as err:
        if required:
            raise
        print(f"WARNING: {err}\nContinuing without motors.yaml.", file=sys.stderr)
        return None


def format_values(status, args):
    return (f"{status.output_velocity(args.pole_pairs, args.gear_ratio):+7.2f} rad/s  "
            f"{status.position_deg:+8.1f} deg  "
            f"{status.current_a:+6.2f} A  {status.temperature_c:3d} C  {status.error_text}")


def table_header():
    return (f"{'joint':<20} {'CAN ID':>6}  {'state':<10} {'velocity':>13}  "
            f"{'position':>12}  {'current':>8}  temp  fault")


def table_row(name, can_id, state, status, args, rate=None):
    values = format_values(status, args) if status else "no status frames"
    if status and rate is not None:
        values += f"  {rate:4.0f} frames/s"
    return f"{name:<20} {can_id:>6}  {state:<10} {values}"


def refuse_if_busy(heard, interface):
    if heard.commanded:
        raise CanCheckError(
            f"Another program is sending commands to CAN ID(s) "
            f"{', '.join(map(str, sorted(heard.commanded)))} on {interface}, probably the "
            "robot (robot.launch.py or iot-robot.service). Stop it first (Ctrl-C, or "
            "`sudo systemctl stop iot-robot`): two programs commanding the same motor "
            "fight each other.")


def cmd_check(args):
    config = load_motors(args, required=not args.ids)
    names = config.names() if config else {}
    if args.ids:
        expected = args.ids
    else:
        expected = [motor.can_id for motor in config.in_use(args.gizmo_mode)]
    details = check_interface(args.interface, args.bitrate)
    heard = listen(args.interface, args.timeout)

    print(table_header())
    missing, faults, slow = [], [], []
    for can_id in expected:
        status = heard.status.get(can_id)
        if status is None:
            missing.append(can_id)
            state = "MISSING"
        elif status.error:
            faults.append(status)
            state = "FAULT"
        else:
            state = "ok"
        if status and heard.rate(can_id) < MIN_STATUS_RATE:
            slow.append(can_id)
        print(table_row(names.get(can_id, "-"), can_id, state, status, args,
                        heard.rate(can_id)))
    others = sorted(set(heard.status) - set(expected))
    for can_id in others:
        # A motors.yaml motor outside the ones checked, e.g. the gizmo with --gizmo-mode
        # fixed, is fine; an ID that motors.yaml does not know is not
        state = "not used" if can_id in names else "UNEXPECTED"
        print(table_row(names.get(can_id, "(not in motors.yaml)"), can_id, state,
                        heard.status[can_id], args, heard.rate(can_id)))

    unexpected = [can_id for can_id in others if can_id not in names]
    for can_id in slow:
        print(f"WARNING: {describe(can_id, names)} sends status at "
              f"{heard.rate(can_id):.0f} Hz; set 100 to 200 Hz in the CubeMars Upper "
              "Computer (docs/hardware.md, section 'Motor configuration').")
    if missing:
        lines = ["No status frames from " + ", ".join(
            describe(can_id, names) for can_id in missing)
            + f" on {args.interface} within {args.timeout:g} s."]
        lines += missing_motor_help(details, others, names)
        print("FAIL: " + "\n".join(lines), file=sys.stderr)
        return 1
    if faults:
        print("FAIL: " + fault_help(faults, names), file=sys.stderr)
        return 1
    if unexpected:
        print(f"WARNING: CAN ID(s) {', '.join(map(str, unexpected))} are not in "
              f"motors.yaml. Run `cubemars_tool identify` if a motor's ID is wrong there.")
    print(f"OK: {args.interface} is up and motors {', '.join(map(str, expected))} respond")
    return 0


def cmd_watch(args):
    config = load_motors(args, required=False)
    names = config.names() if config else {}
    check_interface(args.interface, args.bitrate)
    clear = sys.stdout.isatty()
    printed = 0
    while not args.count or printed < args.count:
        heard = listen(args.interface, 1.0 / args.rate)
        ids = args.ids or sorted(set(names) | set(heard.status),
                                 key=lambda can_id: (can_id not in names, can_id))
        if clear:
            # Cursor home and clear screen, so the table updates in place
            print("\033[H\033[2J", end="")
        print(f"{args.interface}, status frames of the last {heard.elapsed:.1f} s. "
              "Values as the motors report them, before the motors.yaml direction.")
        print(table_header())
        for can_id in ids:
            status = heard.status.get(can_id)
            state = ("FAULT" if status and status.error else "ok" if status
                     else "silent")
            print(table_row(names.get(can_id, "(not in motors.yaml)"), can_id, state,
                            status, args, heard.rate(can_id)))
        if not ids:
            print(f"no status frames on {args.interface}")
        print(flush=True)
        printed += 1
    return 0


def send_for(bus, command, duration, can_id, on_status=None):
    """Send a command at SEND_RATE for duration seconds, passing status frames to on_status.

    command is a (arbitration ID, payload) pair, or a function of the elapsed time that
    returns one, for a speed profile.
    """
    period = 1.0 / SEND_RATE
    start = time.monotonic()
    end = start + duration
    next_send = start
    while (now := time.monotonic()) < end:
        if now >= next_send:
            bus.send(*(command(now - start) if callable(command) else command))
            next_send += period
        frame = bus.recv(max(0.0, min(next_send, end) - time.monotonic()))
        status = decode_status(*frame) if frame else None
        if on_status and status and status.can_id == can_id:
            on_status(status)


def wiggle(bus, can_id, args):
    """Move a motor a few degrees out and back, then brake and release it.

    Each leg follows a triangular speed profile (up to --speed and back to zero), so
    there is no speed step, and the move ends where it started. It stops early on a
    motor fault, when the current exceeds --max-current, for example because the
    joint is pressed against a hard stop, or when the speed does not follow the
    command (a runaway motor).
    """
    erpm_per = erpm_per_rad_s(args.pole_pairs, args.gear_ratio)
    # A triangle of height --speed covers speed * leg / 2
    leg = 2.0 * math.radians(args.degrees) / args.speed
    commanded = [0.0]  # rad/s, the latest command sent

    def profile(sign):
        def command(t):
            commanded[0] = sign * max(0.0, 1.0 - abs(2.0 * t / leg - 1.0)) * args.speed
            return encode_speed(can_id, commanded[0] * erpm_per)
        return command

    def supervise(status):
        if status.error:
            raise MotorStopped(f"it reports a fault: {status.error_text}")
        if abs(status.current_a) > args.max_current:
            raise MotorStopped(
                f"it drew {status.current_a:+.2f} A, more than --max-current "
                f"{args.max_current:g} A. Is the joint against a hard stop? Move it "
                "away from the stop by hand and repeat")
        check_following(status, commanded[0], args)

    release_only = False
    try:
        for index in range(args.repeat):
            if index:
                # A pause between the moves makes each one easier to see
                commanded[0] = 0.0
                send_for(bus, encode_speed(can_id, 0), 0.3, can_id, supervise)
            for sign in (1.0, -1.0):
                send_for(bus, profile(sign), leg, can_id, supervise)
    except MotorStopped as err:
        release_only = err.release_only
        print(f"Stopped CAN ID {can_id} early: {err}.", flush=True)
    finally:
        end_move(bus, can_id, release_only)


def ask_joint(ask, can_id, joints, assigned):
    """Ask which joint moved. Returns a joint name, 'repeat', 'skip' or 'quit'."""
    choices = "  ".join(f"{number} {joint}" for number, joint in enumerate(joints, 1))
    prompt = f"Which joint moved?  {choices}  r repeat  s skip  q quit: "
    while True:
        try:
            reply = ask(prompt).strip()
        except EOFError:
            return "quit"
        if reply in ("r", "s", "q"):
            return {"r": "repeat", "s": "skip", "q": "quit"}[reply]
        if reply.isdigit() and 1 <= int(reply) <= len(joints):
            joint = joints[int(reply) - 1]
        elif reply in joints:
            joint = reply
        else:
            print(f"Answer with a number from 1 to {len(joints)}, a joint name, r, s or q.")
            continue
        owner = assigned.get(joint)
        if owner is not None and owner != can_id:
            print(f"{joint} is already CAN ID {owner}. Two motors cannot drive one joint: "
                  "pick another joint, r to watch the motor again, or s to skip it.")
            continue
        return joint


def cmd_identify(args):
    config = motor_config.load(args.motors)
    path = config.path
    joints = [motor.joint for motor in config.motors]
    details = check_interface(args.interface, args.bitrate)
    heard = listen(args.interface, BUS_CHECK_TIME, status_only=False)
    refuse_if_busy(heard, args.interface)
    ids = args.ids or sorted(heard.status)
    if not ids:
        raise CanCheckError("\n".join(
            [f"No motor sends status frames on {args.interface}."]
            + missing_motor_help(details, [])))

    times = "once" if args.repeat == 1 else f"{args.repeat} times"
    print(f"Motors on {args.interface}: CAN ID {', '.join(map(str, ids))}. Each one moves "
          f"about {args.degrees:g} deg out and back {times}, then goes limp.\n"
          "Keep the wheels off the ground and watch the wheels and the gizmo.")
    silent = [motor for motor in config.motors
              if motor.can_id not in heard.status and motor.can_id not in ids]
    if silent:
        print("No status from " + ", ".join(
            describe(motor.can_id, config.names()) for motor in silent)
            + ": unpowered, or the motor has one of the IDs above.")
    assigned = {}  # joint -> CAN ID
    with CanBus(args.interface, status_only=True) as bus:
        for can_id in ids:
            answer = "repeat"
            while answer == "repeat":
                print(f"\nMoving CAN ID {can_id} ...", flush=True)
                wiggle(bus, can_id, args)
                answer = ask_joint(args.ask, can_id, joints, assigned)
            if answer == "quit":
                break
            if answer != "skip":
                assigned = {joint: owner for joint, owner in assigned.items()
                            if owner != can_id}
                assigned[answer] = can_id

    print("\nResult:")
    for motor in config.motors:
        can_id = assigned.get(motor.joint)
        if can_id is None:
            note = f"not identified, motors.yaml keeps CAN ID {motor.can_id}"
            print(f"  {motor.joint:<20} {note}")
        else:
            change = ("unchanged" if can_id == motor.can_id
                      else f"motors.yaml has {motor.can_id}")
            print(f"  {motor.joint:<20} CAN ID {can_id:<4} ({change})")
    changes = {joint: can_id for joint, can_id in assigned.items()
               if config.get(joint).can_id != can_id}
    if not changes:
        print(f"{path} already matches.")
        return 0
    if not args.write:
        print(f"Not written. Run again with --write, or set these can_id values in {path}.")
        return 0
    try:
        written = motor_config.write_can_ids(path, changes)
    except MotorConfigError as err:
        print(f"FAIL: not written. {err}\nIdentify the other motors too, or edit the file "
              "by hand.", file=sys.stderr)
        return 1
    print(f"Wrote the new CAN IDs to {written}"
          + (f" (through the link {path})" if written != os.path.abspath(path) else ""))
    if args.motors is None and not os.path.islink(path):
        print("WARNING: this is an installed copy, not the source file. Make the same "
              "change in src/iot_robot_bringup/config/motors.yaml, or the next build "
              "overwrites it.")
    print("Next, check each motor's direction with "
          "`cubemars_tool jog --joint <name>` (docs/todo.md).")
    return 0


def cmd_jog(args):
    config = load_motors(args, required=args.joint is not None)
    if args.joint:
        motor = config.get(args.joint)
        if motor is None:
            raise MotorConfigError(f"{config.path} has no entry for {args.joint}")
        can_id, joint = motor.can_id, motor.joint
    else:
        can_id = args.id
        joint = config.joint_of(can_id) if config else None
    if abs(args.velocity) > args.max_velocity:
        print(f"Refusing {args.velocity} rad/s: above --max-velocity {args.max_velocity}",
              file=sys.stderr)
        return 1

    duration = args.duration
    if joint in GIZMO_JOINTS:
        print(f"{joint} has hard stops: start with the gizmo in the middle of its range. "
              f"Travel is limited to {args.max_gizmo_travel:g} deg (--max-gizmo-travel).")
        if args.velocity and duration * abs(args.velocity) > math.radians(
                args.max_gizmo_travel):
            duration = math.radians(args.max_gizmo_travel) / abs(args.velocity)

    check_interface(args.interface, args.bitrate)
    heard = listen(args.interface, BUS_CHECK_TIME, status_only=False)
    refuse_if_busy(heard, args.interface)
    if can_id not in heard.status:
        print(f"WARNING: no status frames from CAN ID {can_id}; jogging anyway.")

    erpm = args.velocity * erpm_per_rad_s(args.pole_pairs, args.gear_ratio)
    command = encode_speed(can_id, erpm)
    last_print = [0.0]

    def show(status):
        if time.monotonic() - last_print[0] >= 0.25:
            print(f"CAN ID {status.can_id}: {format_values(status, args)}", flush=True)
            last_print[0] = time.monotonic()
        # The gizmo travel limit above is speed x time, so it only holds while the
        # motor turns at the commanded speed
        check_following(status, args.velocity, args)

    print(f"Jogging {describe(can_id, config.names() if config else None)} at "
          f"{args.velocity:+.2f} rad/s ({erpm:+.0f} ERPM) for {duration:.2g} s. "
          "Ctrl-C stops early.")
    release_only = False
    with CanBus(args.interface, status_only=True) as bus:
        try:
            send_for(bus, command, duration, can_id, show)
        except KeyboardInterrupt:
            print("Stopping")
        except MotorStopped as err:
            release_only = err.release_only
            print(f"FAIL: stopped CAN ID {can_id} early: {err}.", file=sys.stderr)
            return 1
        finally:
            # Brake to zero speed, then release so the joint turns freely
            end_move(bus, can_id, release_only)
    motion = POSITIVE_MOTION.get(joint, "move the joint in its positive direction")
    print(f"A positive velocity should {motion}. If it does, set direction: 1 for "
          "this joint in motors.yaml, otherwise -1.")
    return 0


def cmd_stop(args):
    # iot-robot.service runs this after every stop of the robot. Without a CAN link
    # nothing can reach the motors, which then rely on their own CAN timeout
    if (interface_type(args.interface) != ARPHRD_CAN
            or not interface_ioctl(args.interface, SIOCGIFFLAGS) & IFF_UP):
        print(f"CAN interface {args.interface} is not up; no stop frames sent")
        return 0
    if args.ids:
        can_ids = args.ids
    else:
        # The motors.yaml IDs, plus any motor that reports status, in case its ID was
        # changed or a different motors.yaml was used
        config = load_motors(args, required=False)
        known = [motor.can_id for motor in config.motors] if config else []
        heard = listen(args.interface, args.listen)
        can_ids = known + sorted(set(heard.status) - set(known))
        if not can_ids:
            print(f"No motor IDs to stop on {args.interface}: motors.yaml could not be "
                  "read and no motor sends status")
            return 0
    with CanBus(args.interface, status_only=True) as bus:
        stop_motors(bus, can_ids, args.brake_time)
    print(f"Motors {', '.join(map(str, can_ids))} on {args.interface}: "
          f"zero speed for {args.brake_time:g} s, then released")
    return 0


def positive(value):
    number = float(value)
    if not number > 0.0:
        raise argparse.ArgumentTypeError(f"must be greater than 0, got {value}")
    return number


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="\n".join(__doc__.splitlines()[2:]))
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--pole-pairs", type=int, default=POLE_PAIRS)
    parser.add_argument("--gear-ratio", type=int, default=GEAR_RATIO)
    parser.add_argument("--motors", metavar="PATH",
                        help="motor configuration (default: motors.yaml of the installed "
                             "iot_robot_bringup package)")
    commands = parser.add_subparsers(dest="command", required=True)

    check = commands.add_parser(
        "check", help="interface up and every motor in motors.yaml reporting")
    check.add_argument("--ids", type=int, nargs="+",
                       help="check these CAN IDs instead of the motors.yaml ones")
    check.add_argument("--gizmo-mode", choices=("can", "fixed"), default="can",
                       help="as robot.launch.py: fixed checks the wheels only")
    check.add_argument("--timeout", type=positive, default=1.0,
                       help="seconds to listen (default 1)")
    check.set_defaults(func=cmd_check)

    watch = commands.add_parser(
        "watch", aliases=["monitor"],
        help="live position, speed, current, temperature and fault of every motor")
    watch.add_argument("--ids", type=int, nargs="*", default=[],
                       help="only these CAN IDs (default: motors.yaml and any other heard)")
    watch.add_argument("--rate", type=positive, default=2.0,
                       help="printouts per second (default 2)")
    watch.add_argument("--count", type=int, default=0,
                       help="stop after this many printouts (default: until Ctrl-C)")
    watch.set_defaults(func=cmd_watch)

    identify = commands.add_parser(
        "identify", help="move one motor at a time and ask which joint it is")
    identify.add_argument("--ids", type=int, nargs="+",
                          help="only these CAN IDs (default: every motor heard)")
    identify.add_argument("--write", action="store_true",
                          help="save the identified CAN IDs to motors.yaml")
    identify.add_argument("--degrees", type=positive, default=5.0,
                          help="travel out and back, output degrees (default 5)")
    identify.add_argument("--speed", type=positive, default=0.5,
                          help="peak output speed in rad/s (default 0.5)")
    identify.add_argument("--repeat", type=int, default=2,
                          help="out-and-back moves per motor (default 2)")
    identify.add_argument("--max-current", type=positive, default=1.5,
                          help="stop the move above this motor current in A (default 1.5)")
    identify.set_defaults(func=cmd_identify)

    jog = commands.add_parser("jog", help="spin one motor at a fixed output speed")
    target = jog.add_mutually_exclusive_group(required=True)
    target.add_argument("--id", type=int, help="motor CAN ID")
    target.add_argument("--joint", choices=motor_config.JOINTS,
                        help="joint name; its CAN ID comes from motors.yaml")
    jog.add_argument("--velocity", type=float, default=1.0,
                     help="gearbox output speed in rad/s (default 1.0)")
    jog.add_argument("--duration", type=positive, default=2.0, help="seconds (default 2)")
    jog.add_argument("--max-velocity", type=float, default=3.0,
                     help="safety cap in rad/s (default 3.0)")
    jog.add_argument("--max-gizmo-travel", type=positive, default=15.0,
                     help="travel limit for the gizmo joints in degrees (default 15)")
    jog.set_defaults(func=cmd_jog)

    stop = commands.add_parser(
        "stop", help="zero speed, then release (what the robot does when it stops)")
    stop.add_argument("--ids", type=int, nargs="+",
                      help="stop only these CAN IDs (default: the motors.yaml IDs plus "
                           "every motor heard on the bus)")
    stop.add_argument("--brake-time", type=float, default=0.3,
                      help="seconds of zero speed before the release (default 0.3)")
    stop.add_argument("--listen", type=float, default=0.2,
                      help="seconds to listen for motors that motors.yaml does not list "
                           "(default 0.2)")
    stop.set_defaults(func=cmd_stop)
    return parser.parse_args(argv)


def main(argv=None, ask=input):
    """Run the tool; ask replaces input() for identify's questions in tests."""
    args = parse_args(sys.argv[1:] if argv is None else argv)
    args.ask = ask
    try:
        return args.func(args)
    except (CanCheckError, MotorConfigError) as err:
        print(f"FAIL: {err}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 0
    except OSError as err:
        print(f"CAN error on {args.interface}: {err}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
