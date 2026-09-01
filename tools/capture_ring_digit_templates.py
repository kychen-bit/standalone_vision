"""Interactively capture normalized 1/2/3 ring-digit templates.

Move/click the selection onto one detected ring, then press 1, 2 or 3 to save
that ring's central digit using the actual camera, lighting and printed font.
"""

import argparse
import json
from pathlib import Path
import sys
import time

import cv2


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jetson_recognition.camera import UVCCamera
from jetson_recognition.detectors import TopViewDetector
from jetson_recognition.run import draw_circle_debug, load_config


class Selection:
    def __init__(self):
        self.position = None

    def on_mouse(self, event, x, y, _flags, _userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.position = (int(x), int(y))

    def choose(self, clusters, frame_shape):
        if not clusters:
            return None
        position = self.position or (frame_shape[1] // 2, frame_shape[0] // 2)
        return min(
            clusters,
            key=lambda cluster: (
                (cluster["center"][0] - position[0]) ** 2
                + (cluster["center"][1] - position[1]) ** 2
            ),
        )


def next_template_path(directory, digit):
    directory.mkdir(parents=True, exist_ok=True)
    index = 1
    while True:
        path = directory / ("%s_%02d.png" % (digit, index))
        if not path.exists():
            return path
        index += 1


def parser():
    command = argparse.ArgumentParser(
        description="capture real ring digit templates; click a ring and press 1/2/3"
    )
    command.add_argument("--config", default="config/jetson.json")
    command.add_argument("--camera")
    command.add_argument("--scene", default="ROUGH", choices=("ROUGH", "STORAGE"))
    command.add_argument(
        "--output-dir", default="config/ring_templates"
    )
    return command


def main():
    arguments = parser().parse_args()
    config = load_config(arguments.config)
    detector = TopViewDetector(config, PROJECT_ROOT)
    detector.set_circle_debug(True)
    scene = arguments.scene.upper()
    scene_config = config["scenes"][scene]
    search_roi = scene_config.get("ring_search_roi", scene_config["object_roi"])
    output_directory = Path(arguments.output_dir)
    if not output_directory.is_absolute():
        output_directory = PROJECT_ROOT / output_directory

    camera = UVCCamera(config["camera"], PROJECT_ROOT, arguments.camera)
    selection = Selection()
    window = "Ring Template Capture"
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, selection.on_mouse)
    print(
        "Click the wanted ring; press 1/2/3 to save its digit template; q exits.",
        flush=True,
    )
    last_saved = None
    try:
        while True:
            frame = camera.read()
            if frame is None:
                time.sleep(0.01)
                continue
            clusters = detector._detect_ring_contours_in_roi(
                frame, search_roi, config["ring_detection"]
            )
            selected = selection.choose(clusters, frame.shape)
            debug = detector.last_circle_debug
            if debug is not None:
                debug["kind"] = "RING"
                debug["groups"] = clusters
                debug["selected"] = selected
            canvas = draw_circle_debug(frame.copy(), debug, True)
            if selected is not None:
                center = tuple(int(round(value)) for value in selected["center"])
                cv2.putText(
                    canvas,
                    "SELECTED - press 1/2/3",
                    (max(0, center[0] - 100), max(50, center[1] - 80)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
            if last_saved:
                cv2.putText(
                    canvas,
                    last_saved,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
            cv2.imshow(window, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key in (ord("1"), ord("2"), ord("3")) and selected is not None:
                digit = chr(key)
                _guess, _score, _bbox, normalized = detector._classify_ring_digit(
                    frame, selected
                )
                if normalized is None:
                    last_saved = "digit crop is empty"
                    continue
                path = next_template_path(output_directory, digit)
                if not cv2.imwrite(str(path), normalized):
                    raise RuntimeError("failed to save %s" % path)
                last_saved = "saved %s" % path.name
                print(last_saved, flush=True)
    finally:
        camera.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
