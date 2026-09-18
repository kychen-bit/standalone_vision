"""Headless version of the live tool's ``d`` key: dump one frame + masks.

Useful when the Jetson has no display, or when a diagnosis has to be kept as an
artefact. It writes the (white-balanced) frame and a mask montage next to the
JSON report, so "only red is detected" can be checked by eye instead of guessed.

    python3 tools/diag_frame.py
    python3 tools/diag_frame.py --camera-profile fill_light_locked --fill-light
"""

import argparse
import json
from pathlib import Path
import sys

import cv2

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jetson_recognition.camera import UVCCamera, apply_capture_profile  # noqa: E402
from jetson_recognition.color_profiles import apply_color_profile  # noqa: E402
from jetson_recognition.run import build_engine  # noqa: E402
from tools.test_turntable_color_live import COLOR_NAMES, diagnose  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/jetson.json")
    parser.add_argument("--camera-profile", default=None)
    parser.add_argument("--capture-profile", default=None)
    parser.add_argument("--fill-light", dest="fill_light", action="store_true",
                        default=None)
    parser.add_argument("--no-fill-light", dest="fill_light", action="store_false")
    parser.add_argument("--frame", default=None, help="read this image instead")
    parser.add_argument("--out", default="logs/diag")
    parser.add_argument("--settle", type=int, default=14)
    arguments = parser.parse_args()

    config = json.loads((PROJECT_ROOT / arguments.config).read_text(encoding="utf-8"))
    if arguments.fill_light is not None:
        config["lighting"]["fill_light"] = arguments.fill_light
    camera_config = config["camera"]
    if arguments.camera_profile:
        camera_config["v4l2_control_profile"] = arguments.camera_profile
    apply_capture_profile(camera_config, arguments.capture_profile)
    apply_color_profile(config, PROJECT_ROOT, quiet=True)

    engine = build_engine(config)
    detector = engine.detector
    detector.set_color_debug(True)

    if arguments.frame:
        frame = cv2.imread(str(arguments.frame))
        if frame is None:
            raise SystemExit("cannot read %s" % arguments.frame)
    else:
        camera = UVCCamera(camera_config, PROJECT_ROOT)
        try:
            for _ in range(arguments.settle):
                camera.read()
            frame = None
            for _ in range(8):
                current = camera.read()
                if current is not None:
                    frame = current
        finally:
            camera.close()
        if frame is None:
            raise SystemExit("no frame from the camera")

    colors = sorted(detector.color_ranges)
    # Run the real decision first: diagnose() reports what the detector
    # rejected, so it needs last_materials_debug to be populated.
    detector.detect_materials(frame, "TURNTABLE", colors)
    diagnose(
        detector, frame, colors, COLOR_NAMES, PROJECT_ROOT / arguments.out,
    )
    print(json.dumps({
        "state": "DIAG_IMAGES_WRITTEN",
        "directory": str(PROJECT_ROOT / arguments.out),
        "camera_profile": camera_config.get("v4l2_control_profile"),
        "fill_light": detector.fill_light,
    }, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
