"""Pretend to be the Nucleo sensor board on a pseudo-terminal, for testing stm32_bridge.

Streams D1/D2 lines like the firmware, and links the pty to a stable path so the bridge
can be pointed at it:

    ros2 run iot_robot_drivers fake_stm32 --battery
    ros2 run iot_robot_drivers stm32_bridge --ros-args -p port:=/tmp/fake_stm32

The board accepts no commands. The fake prints every line it receives as
``received: '<line>'``, so the tests can check that the bridge sends nothing.
"""

import argparse
import math
import os
import select
import signal
import sys
import time
import tty

BANNER = "# iot-stm32 1.2.0 (fake)\r\n"
# The firmware's defaults: warn after this many failed pings in a row (~1 s), then
# repeat every FAULT_REPEAT_PINGS (~10 s). Each sensor is pinged once per line
FAULT_WARN_PINGS = 15
FAULT_REPEAT_PINGS = 150


def fake_distances(t, no_echo):
    """Slowly varying distances in cm. The right sensor loses its echo for 0.5 s every 3 s."""
    left = 60.0 + 40.0 * math.sin(2.0 * math.pi * t / 8.0)
    right = 120.0 + 80.0 * math.sin(2.0 * math.pi * t / 5.0)
    if t % 3.0 < 0.5:
        right = no_echo
    return left, right


def fault_warning_due(failed_pings):
    """True when the firmware prints its warning after this many failed pings in a row."""
    return (failed_pings >= FAULT_WARN_PINGS
            and (failed_pings - FAULT_WARN_PINGS) % FAULT_REPEAT_PINGS == 0)


def format_reading(left_cm, right_cm, volts=None):
    line = f"D1:{left_cm:.1f},D2:{right_cm:.1f}"
    if volts is not None:
        line += f",V:{volts:.2f}"
    return line + "\r\n"


def link_pty(link, target):
    if os.path.lexists(link):
        if not os.path.islink(link):
            raise SystemExit(f"{link} exists and is not a symlink; pass --link elsewhere")
        os.unlink(link)
    os.symlink(target, link)


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--link", default="/tmp/fake_stm32",
                        help="symlink to create for the pty ('' for none)")
    parser.add_argument("--rate", type=float, default=15.0,
                        help="lines per second (default 15, like the firmware)")
    parser.add_argument("--battery", action="store_true",
                        help="append V:<volts>, as firmware built with battery sensing does")
    parser.add_argument("--zero-no-echo", action="store_true",
                        help="report no echo as 0.0 like the first firmware, instead of -1.0")
    parser.add_argument("--fault", choices=("left", "right"),
                        help="simulate an unplugged sensor on that side: -1.0 and the "
                             "firmware's '# warning: <side> sensor not responding' line")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)

    master, slave = os.openpty()
    # Raw mode: no echo of our own output back to us, no CR/LF translation
    tty.setraw(slave)
    # Keeping our slave descriptor open means the pty survives the bridge closing
    # and reopening it. A full buffer drops lines, as a UART with nobody reading would
    os.set_blocking(master, False)
    path = os.ttyname(slave)
    if args.link:
        link_pty(args.link, path)
        print(f"Fake STM32 on {path}, linked from {args.link}", flush=True)
    else:
        print(f"Fake STM32 on {path}", flush=True)

    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)

    no_echo = 0.0 if args.zero_no_echo else -1.0
    period = 1.0 / args.rate
    start = time.monotonic()
    next_send = start
    received = b""
    failed_pings = 0
    try:
        os.write(master, BANNER.encode("ascii"))
        while True:
            now = time.monotonic()
            readable, _, _ = select.select([master], [], [], max(0.0, next_send - now))
            if readable:
                try:
                    received += os.read(master, 1024)
                except (BlockingIOError, InterruptedError):
                    pass
                while b"\n" in received:
                    raw, received = received.split(b"\n", 1)
                    line = raw.decode("ascii", errors="replace").strip()
                    print(f"received: {line!r}", flush=True)

            now = time.monotonic()
            if now >= next_send:
                t = now - start
                volts = max(21.0, 25.0 - 0.002 * t) if args.battery else None
                left, right = fake_distances(t, no_echo)
                line = ""
                if args.fault:
                    # The firmware sends a failed ping as -1.0 whatever no_echo is
                    if args.fault == "left":
                        left = -1.0
                    else:
                        right = -1.0
                    failed_pings += 1
                    if fault_warning_due(failed_pings):
                        line = f"# warning: {args.fault} sensor not responding\r\n"
                line += format_reading(left, right, volts)
                try:
                    os.write(master, line.encode("ascii"))
                except BlockingIOError:
                    pass
                # Catch up without a burst if we were stalled
                next_send = max(next_send + period, now)
    except KeyboardInterrupt:
        pass
    finally:
        if args.link and os.path.islink(args.link) and os.readlink(args.link) == path:
            os.unlink(args.link)
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    main()
