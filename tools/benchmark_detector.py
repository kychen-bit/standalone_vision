"""Measure Jetson recognition latency on a saved frame or the live camera.

Examples:
  python3 tools/benchmark_detector.py --image frame.jpg \
      --mode COLOR --scene TURNTABLE --target 1
  python3 tools/benchmark_detector.py --camera /dev/video0 \
      --mode RING --scene ROUGH --target 2 --iterations 300
"""

import argparse
import json
from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import cv2

from jetson_recognition.camera import UVCCamera
from jetson_recognition.run import build_engine, load_config


def mode_arguments(arguments):
    mode = arguments.mode.upper()
    if mode in ("COLOR", "RING"):
        return arguments.target, arguments.scene
    if mode == "STACK":
        return arguments.target, arguments.scene, arguments.ring
    return (arguments.scene,)


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return 0.0
    index = int(round((len(ordered) - 1) * fraction))
    return float(ordered[index])


def summary(values):
    return {
        "mean": round(sum(values) / max(len(values), 1), 3),
        "p50": round(percentile(values, 0.50), 3),
        "p95": round(percentile(values, 0.95), 3),
        "max": round(max(values) if values else 0.0, 3),
    }


def parser():
    root = argparse.ArgumentParser(description="benchmark Jetson recognition latency")
    root.add_argument("--config", default="config/jetson.json")
    source = root.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path)
    source.add_argument("--camera", help="camera index or /dev path")
    root.add_argument(
        "--mode",
        choices=("COLOR", "RING", "STACK", "STATION", "TURNTABLE"),
        required=True,
    )
    root.add_argument("--scene", required=True)
    root.add_argument("--target", default="1")
    root.add_argument("--ring", default="1")
    root.add_argument("--iterations", type=int, default=300)
    root.add_argument("--warmup", type=int, default=20)
    root.add_argument(
        "--disable-tracking",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    return root


def main():
    arguments = parser().parse_args()
    if arguments.iterations < 1 or arguments.warmup < 0:
        raise SystemExit("iterations must be positive and warmup must be non-negative")
    config = load_config(arguments.config)
    engine = build_engine(config)
    detector_args = mode_arguments(arguments)
    camera = None
    if arguments.image:
        frame = cv2.imread(str(arguments.image))
        if frame is None:
            raise SystemExit("cannot read image: %s" % arguments.image)
        expected_size = (
            int(config["camera"]["height"]),
            int(config["camera"]["width"]),
        )
        if frame.shape[:2] != expected_size:
            raise SystemExit(
                "image resolution %dx%d does not match configured %dx%d"
                % (
                    frame.shape[1],
                    frame.shape[0],
                    expected_size[1],
                    expected_size[0],
                )
            )

        def next_frame():
            return frame, 0.0
    else:
        camera = UVCCamera(config["camera"], PROJECT_ROOT, arguments.camera)

        def next_frame():
            started = time.perf_counter()
            captured = camera.read()
            return captured, (time.perf_counter() - started) * 1000.0

    detect_ms = []
    read_ms = []
    successes = 0
    total_started = None
    try:
        for index in range(arguments.warmup + arguments.iterations):
            if index == arguments.warmup:
                total_started = time.perf_counter()
            frame, capture_elapsed = next_frame()
            if frame is None:
                continue
            started = time.perf_counter()
            measurement = engine.detect(frame, arguments.mode, detector_args)
            detect_elapsed = (time.perf_counter() - started) * 1000.0
            if index < arguments.warmup:
                continue
            read_ms.append(capture_elapsed)
            detect_ms.append(detect_elapsed)
            successes += int(measurement is not None)
    finally:
        if camera is not None:
            camera.close()
    if total_started is None:
        total_started = time.perf_counter()
    total_elapsed = time.perf_counter() - total_started
    measured = len(detect_ms)
    report = {
        "state": "BENCHMARK",
        "mode": arguments.mode,
        "scene": arguments.scene,
        "iterations_requested": arguments.iterations,
        "iterations_measured": measured,
        "detections": successes,
        "roi_mode": "FIXED_ROI" if arguments.mode in ("COLOR", "STACK") else None,
        "detection_rate": round(successes / measured, 4) if measured else 0.0,
        "detect_ms": summary(detect_ms),
        "read_ms": summary(read_ms) if camera is not None else None,
        "camera_settings": camera.actual_settings if camera is not None else None,
        "wall_fps": round(measured / total_elapsed, 3) if total_elapsed > 0 else 0.0,
    }
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    return 0 if measured == arguments.iterations else 2


if __name__ == "__main__":
    raise SystemExit(main())
