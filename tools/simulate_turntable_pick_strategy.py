"""Test all-color recognition and turntable inference without an MCU."""

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jetson_recognition.detectors import TopViewDetector
from jetson_recognition.turntable_strategy import TurntablePickStrategy


SYNTHETIC_HSV = {
    "1": (0, 220, 220),
    "2": (30, 220, 220),
    "3": (120, 240, 200),
    "4": (70, 220, 180),
    "5": None,
    "6": (98, 180, 180),
}


def color_frame(color_id):
    background = 255 if color_id == "5" else 0
    frame = np.full((720, 1280, 3), background, dtype=np.uint8)
    if color_id == "5":
        bgr = (20, 20, 20)
    else:
        hsv = np.asarray([[SYNTHETIC_HSV[color_id]]], dtype=np.uint8)
        bgr = tuple(int(value) for value in cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0])
    cv2.circle(frame, (640, 360), 80, bgr, -1)
    return frame


def main():
    parser = argparse.ArgumentParser(
        description="simulate three-position turntable classification/inference"
    )
    parser.add_argument("--target", required=True, choices=tuple("123456"))
    parser.add_argument(
        "--round-colors", nargs=3, required=True, choices=tuple("123456")
    )
    parser.add_argument(
        "--observed-order", nargs="+", required=True, choices=tuple("123456"),
        help="materials presented at inspection positions; one or two is sufficient",
    )
    arguments = parser.parse_args()
    config = json.loads(
        (PROJECT_ROOT / "config" / "jetson.json").read_text(encoding="utf-8")
    )
    detector = TopViewDetector(config, PROJECT_ROOT)
    strategy = TurntablePickStrategy(arguments.target, arguments.round_colors)
    transcript = []
    for position, actual_color in enumerate(arguments.observed_order, 1):
        measurement = detector.detect_any_color(
            color_frame(actual_color), "TURNTABLE", arguments.round_colors
        )
        if measurement is None:
            raise RuntimeError("position %d produced no color result" % position)
        decision = strategy.observe(measurement.target_id)
        item = {
            "position": position,
            "actual": actual_color,
            "detected": measurement.target_id,
            "center_px": [measurement.pixel_x, measurement.pixel_y],
            "confidence": measurement.confidence,
            "decision": decision,
        }
        transcript.append(item)
        print(json.dumps(item, ensure_ascii=False), flush=True)
        if decision["action"] != "MOVE_NEXT_AND_CLASSIFY":
            break
    if transcript[-1]["decision"]["action"] == "MOVE_NEXT_AND_CLASSIFY":
        raise RuntimeError("another observed color is required to finish the decision")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
