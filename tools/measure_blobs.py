"""Report the raw blob sizes per colour, with the area gates bypassed.

This is the tool that answers "how big is the material on the table right now",
which is what ``scenes.*.min_area`` / ``max_area`` must be set from. It reads one
frame with the configured camera profile and prints, for every colour, the mask
pixel count, the three largest contour areas and the median HSV inside the mask,
plus the area band recommended by the same rule the live tool's ``d`` key uses.

    python3 tools/measure_blobs.py
    python3 tools/measure_blobs.py --camera-profile dim_room_locked --frames 20
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jetson_recognition.camera import UVCCamera, apply_capture_profile  # noqa: E402
from jetson_recognition.color_profiles import apply_color_profile  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/jetson.json")
    parser.add_argument("--camera-profile", default=None,
                        help="override camera.v4l2_control_profile")
    parser.add_argument("--capture-profile", default=None)
    parser.add_argument("--frame", default=None,
                        help="use this image file instead of the camera")
    parser.add_argument("--colors", nargs="+", default=None,
                        help="defaults to every colour defined in the config")
    parser.add_argument("--frames", type=int, default=14,
                        help="discard this many frames before measuring")
    arguments = parser.parse_args()

    import cv2

    config = json.loads((PROJECT_ROOT / arguments.config).read_text(encoding="utf-8"))
    apply_color_profile(config, PROJECT_ROOT)
    camera_config = config["camera"]
    apply_capture_profile(camera_config, arguments.capture_profile)
    if arguments.camera_profile:
        camera_config["v4l2_control_profile"] = arguments.camera_profile

    from jetson_recognition.detectors import TopViewDetector

    detector = TopViewDetector(config, PROJECT_ROOT)

    if arguments.frame:
        frame = cv2.imread(arguments.frame)
        if frame is None:
            raise SystemExit("cannot read %s" % arguments.frame)
        print(json.dumps({"state": "BLOB_SOURCE", "file": arguments.frame}))
    else:
        camera = UVCCamera(camera_config, PROJECT_ROOT)
        try:
            for _ in range(arguments.frames):
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

    balanced = detector._work_frame(frame)
    hsv = cv2.cvtColor(balanced, cv2.COLOR_BGR2HSV)
    print(json.dumps({
        "state": "ILLUMINATION",
        "colors": "white-balanced frame" if balanced is not frame else "raw frame",
        **(detector.last_illumination or {}),
    }, ensure_ascii=False))

    rows = []
    color_ids = arguments.colors or sorted(detector.color_ranges)
    for color_id in color_ids:
        mask = detector._clean_mask(detector._make_color_mask(hsv, str(color_id)))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas = sorted((float(cv2.contourArea(c)) for c in contours), reverse=True)
        inside = mask > 0
        row = {
            "color": color_id,
            "mask_px": int(np.count_nonzero(mask)),
            "contours": len(contours),
            "top_areas": [int(value) for value in areas[:3]],
        }
        if inside.any():
            median = np.median(hsv[inside], axis=0)
            row["mask_hsv_median"] = [int(value) for value in median]
        rows.append(row)
        print(json.dumps(row, ensure_ascii=False))

    largest = max((row["top_areas"][0] for row in rows if row["top_areas"]), default=0)
    if largest:
        radius = float(np.sqrt(largest / np.pi))
        print(json.dumps({
            "state": "AREA_BAND",
            "largest_top_area": int(largest),
            "equivalent_diameter": round(2.0 * radius, 1),
            "min_area": int(largest * 0.35),
            "max_area": int(largest * 2.5),
            "note": "min_area 用来滤杂斑；max_area 只是保险，不要按机位收紧",
        }, ensure_ascii=False))


if __name__ == "__main__":
    main()
