"""Measure how fast person following can run on this computer, e.g. the robot's Pi 4.

    pixi run -e robot bench-person
    pixi run -e robot bench-person --threads 2 4 --people 2 --seconds 10

Times every stage of the person detector with the same code and models the robot uses,
then estimates the detection rate. Needs no camera, motors or ROS graph. Stop the robot
service first (`sudo systemctl stop iot-robot`) so nothing else competes for the CPU, and
run it with the case closed, since a hot Pi 4 slows down.
"""

import argparse
import os
import statistics
import subprocess
import time

import cv2
import numpy as np
import onnxruntime as ort

from iot_robot_perception.inference import onnx_session
from iot_robot_perception.person_detector import letterbox
from iot_robot_perception.reid import ReidEncoder

# The web page's video qualities (iot_robot_web/config/web_controller.yaml)
WEB_QUALITIES = (("high", 640, 70), ("medium", 480, 60), ("low", 320, 50))


def camera_frame(path):
    """A 640 x 480 BGR frame: the image at `path`, or a busy synthetic one."""
    if path:
        image = cv2.imread(path)
        if image is None:
            raise SystemExit(f"Cannot read {path}")
        return cv2.resize(image, (640, 480), interpolation=cv2.INTER_AREA)
    rng = np.random.default_rng(0)
    y, x = np.mgrid[0:480, 0:640]
    gradient = np.dstack([(x * 0.35) % 255, (y * 0.5) % 255, ((x + y) * 0.2) % 255])
    noise = cv2.GaussianBlur(rng.integers(0, 255, (480, 640, 3), dtype=np.uint8), (0, 0), 1.2)
    return cv2.addWeighted(gradient.astype(np.uint8), 0.6, noise, 0.4, 0)


def measure(run, seconds, warmup=0.5):
    """Run `run()` repeatedly for `seconds`. Returns (times in ms, CPU cores kept busy)."""
    start = time.perf_counter()
    while time.perf_counter() - start < warmup:
        run()
    times = []
    cpu = time.process_time()
    start = time.perf_counter()
    while time.perf_counter() - start < seconds or len(times) < 3:
        began = time.perf_counter()
        run()
        times.append(1000.0 * (time.perf_counter() - began))
    wall = time.perf_counter() - start
    return times, (time.process_time() - cpu) / wall


def paced(run, rate, seconds):
    """CPU cores kept busy while running `run()` `rate` times a second, as the detector does
    between camera frames. Spinning threads show up here as busy time between runs."""
    period = 1.0 / rate
    cpu = time.process_time()
    start = time.perf_counter()
    due = start
    while time.perf_counter() - start < seconds:
        run()
        due += period
        delay = due - time.perf_counter()
        if delay > 0.0:
            time.sleep(delay)
    return (time.process_time() - cpu) / (time.perf_counter() - start)


def p90(times):
    return sorted(times)[int(0.9 * (len(times) - 1))]


def machine():
    try:
        with open("/proc/device-tree/model") as file:
            name = file.read().strip("\x00\n")
    except OSError:
        name = os.uname().machine
    return f"{name}, {os.cpu_count()} cores, onnxruntime {ort.__version__}"


def temperature():
    """CPU temperature and, on a Raspberry Pi, whether it has throttled since boot."""
    text = "temperature unknown"
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as file:
            text = f"CPU {int(file.read()) / 1000:.1f} °C"
    except (OSError, ValueError):
        pass
    try:
        flags = int(subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                                   text=True, timeout=2).stdout.split("=")[1], 16)
        if flags & 0xF:
            text += ", THROTTLING NOW (heat or undervoltage)"
        elif flags & 0xF0000:
            text += ", throttled earlier since boot"
        else:
            text += ", not throttled"
    except (OSError, IndexError, ValueError, subprocess.SubprocessError):
        pass
    return text


def main():
    root = os.environ.get("PIXI_PROJECT_ROOT", os.getcwd())
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--model", default=os.path.join(root, "models", "yolo26n-pose.onnx"))
    parser.add_argument("--reid-model",
                        default=os.path.join(root, "models", "osnet_x0_25_msmt17.onnx"))
    parser.add_argument("--threads", type=int, nargs="+",
                        default=sorted({1, 2, 3, os.cpu_count() or 4}))
    parser.add_argument("--people", type=int, default=2,
                        help="most people in view to estimate the rate for")
    parser.add_argument("--seconds", type=float, default=5.0, help="per measurement")
    parser.add_argument("--image", help="a camera picture to use instead of a synthetic one")
    args = parser.parse_args()
    for path in (args.model, args.reid_model):
        if not os.path.exists(path):
            raise SystemExit(f"{path} not found. Export the models first (README, "
                             "Exporting the model) or pass --model / --reid-model.")

    cv2.setNumThreads(1)  # as the detector does
    frame = camera_frame(args.image)
    print(machine())
    print(f"Before: {temperature()}\n")

    rows = []

    def row(stage, threads, times, busy):
        rows.append((stage, threads, statistics.median(times), p90(times), busy))
        print(f"{stage:<28} {threads:>7} {statistics.median(times):>10.1f} "
              f"{p90(times):>8.1f} {busy:>11.1f}", flush=True)

    print(f"{'stage':<28} {'threads':>7} {'median ms':>10} {'p90 ms':>8} {'cores busy':>11}")
    probe = onnx_session(args.model, 1)
    size = probe.get_inputs()[0].shape[2]
    del probe
    times, busy = measure(lambda: letterbox(frame, size), args.seconds / 2)
    row(f"letterbox 640x480 -> {size}", 1, times, busy)
    blob = letterbox(frame, size)[0]
    best = {}
    for threads in args.threads:
        session = onnx_session(args.model, threads)
        name = session.get_inputs()[0].name
        times, busy = measure(lambda: session.run(None, {name: blob}), args.seconds)
        row("pose model", threads, times, busy)
        best[threads] = statistics.median(times)
    # A standing person filling about half of the picture height
    box = np.array([250.0, 120.0, 390.0, 470.0])
    reid = {}
    for threads in args.threads:
        encoder = ReidEncoder(args.reid_model, threads)
        times, busy = measure(lambda: encoder.embed(frame, box), args.seconds / 2)
        row("ReID, per person", threads, times, busy)
        reid[threads] = statistics.median(times)
    for quality, width, jpeg_quality in WEB_QUALITIES:
        def encode(width=width, jpeg_quality=jpeg_quality):
            # As the web page's encoder does
            interpolation = cv2.INTER_AREA if 2 * width <= 640 else cv2.INTER_LINEAR
            small = frame if width >= 640 else cv2.resize(
                frame, (width, width * 3 // 4), interpolation=interpolation)
            cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        times, busy = measure(encode, args.seconds / 2)
        row(f"web video {quality} ({width} px)", 1, times, busy)

    prepare = rows[0][2]
    # Fewer threads when more barely help: the spare cores run the camera, the motor
    # controllers and the web page
    threads = min(best, key=lambda n: (best[n] > 1.05 * min(best.values()), n))
    reid_threads = min(reid, key=lambda n: (reid[n] > 1.05 * min(reid.values()), n))
    print(f"\nSuggested settings: num_threads: {threads}, reid_threads: {reid_threads}")
    print("Estimated person detector rate with them (the camera and other nodes not counted):")
    for people in range(args.people + 1):
        period = prepare + best[threads] + people * reid[reid_threads]
        label = "nobody in view" if people == 0 else f"{people} " + (
            "person" if people == 1 else "people")
        print(f"  {label:<15} {1000.0 / period:5.1f} Hz  ({period:.0f} ms a frame)")

    # What spinning threads cost between frames, at about the rate the detector would run
    rate = 0.8 * 1000.0 / (prepare + best[threads] + reid[reid_threads])
    # With one thread there is no pool to spin
    spin_threads = threads if threads > 1 else max(args.threads)
    print(f"\nPose model at {rate:.1f} runs a second, {spin_threads} threads:")
    for spinning in (False, True):
        session = onnx_session(args.model, spin_threads, spinning)
        name = session.get_inputs()[0].name
        times = []

        def run():
            began = time.perf_counter()
            session.run(None, {name: blob})
            times.append(1000.0 * (time.perf_counter() - began))
        busy = paced(run, rate, args.seconds)
        setting = "spin_threads true" if spinning else "spin_threads false (default)"
        print(f"  {setting:<30} {statistics.median(times):6.1f} ms a run, "
              f"{busy:.2f} CPU cores kept busy")
    print(f"\nAfter: {temperature()}")


if __name__ == "__main__":
    main()
