"""Bench tool for the CubeMars wheel motors: check the bus, watch status, jog or stop them.

It talks to the motors directly over SocketCAN, so stop robot.launch.py first: two
programs sending speed commands to the same motor fight each other.

    ros2 run iot_robot_drivers cubemars_tool check
    ros2 run iot_robot_drivers cubemars_tool monitor
    ros2 run iot_robot_drivers cubemars_tool jog --id 1 --velocity 1.0
    ros2 run iot_robot_drivers cubemars_tool stop
"""

import argparse
import sys
import time
from collections import Counter

from iot_robot_drivers.cubemars import (
    ARPHRD_CAN, GEAR_RATIO, IFF_UP, POLE_PAIRS, SIOCGIFFLAGS, CanBus, CanCheckError,
    check_interface, decode_status, encode_speed, erpm_per_rad_s, interface_ioctl,
    interface_type, preflight, stop_motors)

SEND_RATE = 50.0  # Hz, the same rate the controller manager commands the motors


def format_status(status, args):
    return (f"CAN ID {status.can_id}: "
            f"{status.output_velocity(args.pole_pairs, args.gear_ratio):+7.2f} rad/s  "
            f"{status.position_deg:+8.1f} deg (motor side)  "
            f"{status.current_a:+6.2f} A  {status.temperature_c:3d} C  {status.error_text}")


def cmd_check(args):
    try:
        heard = preflight(args.interface, args.ids, args.timeout, args.bitrate)
    except CanCheckError as err:
        print(f"FAIL: {err}", file=sys.stderr)
        return 1
    for can_id in args.ids:
        print(format_status(heard[can_id], args))
    print(f"OK: {args.interface} is up and motors {', '.join(map(str, args.ids))} respond")
    return 0


def cmd_monitor(args):
    check_interface(args.interface, args.bitrate)
    latest = {}
    frames = Counter()
    period = 1.0 / args.rate
    window_start = time.monotonic()
    with CanBus(args.interface, status_only=True) as bus:
        while True:
            frame = bus.recv(period / 4.0)
            status = decode_status(*frame) if frame else None
            if status and (not args.ids or status.can_id in args.ids):
                latest[status.can_id] = status
                frames[status.can_id] += 1
            now = time.monotonic()
            if now - window_start >= period:
                elapsed = now - window_start
                for can_id in sorted(latest):
                    # The frame rate shows the motor's configured status rate
                    print(f"{format_status(latest[can_id], args)}  "
                          f"{frames[can_id] / elapsed:5.0f} frames/s")
                if not latest:
                    print(f"no status frames on {args.interface}")
                print(flush=True)
                latest.clear()
                frames.clear()
                window_start = now


def send_for(bus, command, duration, can_id, on_status=None):
    """Send a command at SEND_RATE for duration seconds, passing status frames to on_status."""
    period = 1.0 / SEND_RATE
    end = time.monotonic() + duration
    next_send = time.monotonic()
    while (now := time.monotonic()) < end:
        if now >= next_send:
            bus.send(*command)
            next_send += period
        frame = bus.recv(max(0.0, min(next_send, end) - time.monotonic()))
        status = decode_status(*frame) if frame else None
        if on_status and status and status.can_id == can_id:
            on_status(status)


def cmd_jog(args):
    if abs(args.velocity) > args.max_velocity:
        print(f"Refusing {args.velocity} rad/s: above --max-velocity {args.max_velocity}",
              file=sys.stderr)
        return 1
    check_interface(args.interface, args.bitrate)
    erpm = args.velocity * erpm_per_rad_s(args.pole_pairs, args.gear_ratio)
    command = encode_speed(args.id, erpm)
    last_print = [0.0]

    def show(status):
        if time.monotonic() - last_print[0] >= 0.25:
            print(format_status(status, args), flush=True)
            last_print[0] = time.monotonic()

    print(f"Jogging CAN ID {args.id} at {args.velocity:+.2f} rad/s ({erpm:+.0f} ERPM) "
          f"for {args.duration:g} s. Ctrl-C stops early.")
    with CanBus(args.interface, status_only=True) as bus:
        try:
            send_for(bus, command, args.duration, args.id, show)
        except KeyboardInterrupt:
            print("Stopping")
        finally:
            # Brake to zero speed, then release so the wheel turns freely
            stop_motors(bus, [args.id])
    print("A positive velocity should turn the wheel so that it drives the robot forward.\n"
          "If it does, use direction 1 for this motor, otherwise -1 "
          "(left_direction / right_direction in robot.launch.py).")
    return 0


def cmd_stop(args):
    # iot-robot.service runs this after every stop of the robot. Without a CAN link
    # nothing can reach the motors, which then rely on their own CAN timeout
    if (interface_type(args.interface) != ARPHRD_CAN
            or not interface_ioctl(args.interface, SIOCGIFFLAGS) & IFF_UP):
        print(f"CAN interface {args.interface} is not up; no stop frames sent")
        return 0
    with CanBus(args.interface, status_only=True) as bus:
        stop_motors(bus, args.ids, args.brake_time)
    print(f"Motors {', '.join(map(str, args.ids))} on {args.interface}: "
          f"zero speed for {args.brake_time:g} s, then released")
    return 0


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--pole-pairs", type=int, default=POLE_PAIRS)
    parser.add_argument("--gear-ratio", type=int, default=GEAR_RATIO)
    commands = parser.add_subparsers(dest="command", required=True)

    check = commands.add_parser("check", help="interface up and both motors reporting")
    check.add_argument("--ids", type=int, nargs="+", default=[1, 2])
    check.add_argument("--timeout", type=float, default=1.0)
    check.set_defaults(func=cmd_check)

    monitor = commands.add_parser("monitor", help="print decoded status frames")
    monitor.add_argument("--ids", type=int, nargs="*", default=[],
                         help="only these CAN IDs (default: all)")
    monitor.add_argument("--rate", type=float, default=2.0, help="printouts per second")
    monitor.set_defaults(func=cmd_monitor)

    jog = commands.add_parser("jog", help="spin one motor at a fixed output speed")
    jog.add_argument("--id", type=int, required=True, help="motor CAN ID")
    jog.add_argument("--velocity", type=float, default=1.0,
                     help="gearbox output speed in rad/s (default 1.0)")
    jog.add_argument("--duration", type=float, default=2.0, help="seconds (default 2)")
    jog.add_argument("--max-velocity", type=float, default=3.0,
                     help="safety cap in rad/s (default 3.0)")
    jog.set_defaults(func=cmd_jog)

    stop = commands.add_parser(
        "stop", help="zero speed, then release (what the robot does when it stops)")
    stop.add_argument("--ids", type=int, nargs="+", default=[1, 2])
    stop.add_argument("--brake-time", type=float, default=0.3,
                      help="seconds of zero speed before the release (default 0.3)")
    stop.set_defaults(func=cmd_stop)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        return args.func(args)
    except CanCheckError as err:
        print(f"FAIL: {err}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 0
    except OSError as err:
        print(f"CAN error on {args.interface}: {err}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
