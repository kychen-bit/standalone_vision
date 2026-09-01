"""Interactive capture-zone calibrator for the turntable pickup gate.

This is the visual counterpart of ``PickupSession``: ``pickup.capture_zone_px``
must be measured before the ``pickup`` mode can run, and this tool lets you
place that window on a real frame instead of guessing numbers.

Left-drag draws the outer capture zone (yellow). The inner safe zone (green)
is derived automatically from ``pickup.capture_margin_px``. The configured
target-colour detection result is overlaid so you can verify the window really
matches the physical gripper projection and that the requested colour is the
only thing detected inside it.

Keys while the window is shown:
  s          save capture_zone_px into config/jetson.json and exit
  r          reset the rectangle
  q / Esc    quit without saving

Live camera (also works from Jetson):
  python tools/capture_zone_calibrator.py --camera 0 --target 1 --scene TURNTABLE

Single image with a window (needs a GUI):
  python tools/capture_zone_calibrator.py --image frame.jpg --target 1 --scene TURNTABLE

Headless mode (SSH, no desktop): the rectangle comes from the CLI instead of
the mouse and is written straight into the config:
  python tools/capture_zone_calibrator.py --image frame.jpg --target 1 --scene TURNTABLE \\
      --x 340 --y 70 --w 600 --h 600 --save
"""

import argparse
import json
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import cv2

from jetson_recognition.detectors import TopViewDetector


def load_config(path):
    config_path = Path(path)
    if not config_path.is_absolute():
        config_path = PROJECT_ROOT / config_path
    return json.loads(config_path.read_text(encoding="utf-8"))


class CaptureZoneCalibrator:
    def __init__(self, config, target_id, scene, frame_shape):
        self.config = config
        self.target_id = str(target_id)
        self.scene = str(scene).upper()
        self.margin = float(config["pickup"].get("capture_margin_px", 12))
        self.height, self.width = frame_shape[:2]
        self.rect = None  # (x, y, w, h)
        self.drag_start = None
        existing = config["pickup"].get("capture_zone_px")
        if existing and len(existing) == 4:
            self.rect = tuple(int(value) for value in existing)
        self.detector = TopViewDetector(config, PROJECT_ROOT)

    # -- mouse -------------------------------------------------------------
    def on_mouse(self, event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_start is not None:
            x0, y0 = self.drag_start
            self.rect = self._normalise(x0, y0, x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            if self.drag_start is not None:
                x0, y0 = self.drag_start
                self.rect = self._normalise(x0, y0, x, y)
            self.drag_start = None

    def _normalise(self, x0, y0, x1, y1):
        x0 = max(0, min(x0, self.width - 1))
        x1 = max(0, min(x1, self.width - 1))
        y0 = max(0, min(y0, self.height - 1))
        y1 = max(0, min(y1, self.height - 1))
        return (min(x0, x1), min(y0, y1), abs(x1 - x0), abs(y1 - y0))

    # -- drawing -----------------------------------------------------------
    def safe_zone(self):
        if self.rect is None:
            return None
        x, y, w, h = self.rect
        return (x + self.margin, y + self.margin, w - 2 * self.margin, h - 2 * self.margin)

    def draw(self, frame, measurement):
        canvas = frame.copy()
        if self.rect is not None:
            x, y, w, h = self.rect
            cv2.rectangle(canvas, (x, y), (x + w, y + h), (0, 200, 255), 2)
        safe = self.safe_zone()
        if safe is not None:
            sx, sy, sw, sh = safe
            if sw > 0 and sh > 0:
                cv2.rectangle(
                    canvas,
                    (int(sx), int(sy)),
                    (int(sx + sw), int(sy + sh)),
                    (0, 255, 0),
                    2,
                )
        if measurement is not None:
            px = measurement.pixel_x if measurement.pixel_x is not None else measurement.x
            py = measurement.pixel_y if measurement.pixel_y is not None else measurement.y
            colour = (0, 255, 0) if self._inside(px, py) else (0, 220, 255)
            cv2.drawMarker(
                canvas, (int(round(px)), int(round(py))), colour,
                cv2.MARKER_CROSS, 26, 2,
            )
        return canvas

    def _inside(self, px, py):
        safe = self.safe_zone()
        if safe is None:
            return False
        sx, sy, sw, sh = safe
        return sx <= px <= sx + sw and sy <= py <= sy + sh

    # -- save --------------------------------------------------------------
    def save(self, config_path):
        if self.rect is None:
            raise ValueError("no capture zone drawn; nothing to save")
        x, y, w, h = self.rect
        if w <= 2 * self.margin or h <= 2 * self.margin:
            raise ValueError(
                "capture zone %.0fx%.0f too small for margin %.0f; no safe zone left"
                % (w, h, self.margin)
            )
        self.config["pickup"]["capture_zone_px"] = [int(v) for v in self.rect]
        config_path = Path(config_path)
        if not config_path.is_absolute():
            config_path = PROJECT_ROOT / config_path
        config_path.write_text(
            json.dumps(self.config, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        print(
            "saved capture_zone_px=%s to %s" % ([int(v) for v in self.rect], config_path),
            flush=True,
        )


def text_overlay(canvas, calibrator, measurement, interactive):
    lines = []
    if calibrator.rect is not None:
        x, y, w, h = calibrator.rect
        lines.append("capture_zone=(%d,%d %dx%d) margin=%.0f" % (x, y, w, h, calibrator.margin))
    else:
        lines.append("capture_zone=(not set) margin=%.0f" % calibrator.margin)
    if measurement is not None:
        lines.append(
            "target=%s detected at (%.1f,%.1f) inside=%s q=%.2f"
            % (
                calibrator.target_id,
                measurement.pixel_x or measurement.x,
                measurement.pixel_y or measurement.y,
                calibrator._inside(
                    measurement.pixel_x or measurement.x,
                    measurement.pixel_y or measurement.y,
                ),
                measurement.confidence,
            )
        )
    else:
        lines.append("target=%s not detected (only this colour is searched)" % calibrator.target_id)
    if interactive:
        lines.append("drag to draw  |  s=save  r=reset  q=quit")
    bar_h = 20 + 18 * len(lines)
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], bar_h), (0, 0, 0), -1)
    for index, line in enumerate(lines):
        cv2.putText(
            canvas, line, (10, 20 + 18 * index), cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (255, 255, 255), 1, cv2.LINE_AA,
        )
    return canvas


def run_interactive(args, config, calibrator):
    if args.image:
        frame = cv2.imread(str(args.image))
        if frame is None:
            raise SystemExit("cannot read image: %s" % args.image)
        calibrator.height, calibrator.width = frame.shape[:2]
        window_name = "capture zone calibrator (single image)"
        cv2.imshow(window_name, frame)
        cv2.setMouseCallback(window_name, calibrator.on_mouse)
        print(
            "left-drag to draw the capture zone around the gripper projection; "
            "s=save r=reset q=quit",
            flush=True,
        )
        while True:
            measurement = calibrator.detector.detect_color(
                frame, calibrator.target_id, calibrator.scene
            )
            canvas = calibrator.draw(frame, measurement)
            canvas = text_overlay(canvas, calibrator, measurement, True)
            cv2.imshow(window_name, canvas)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("s"), ord("S")):
                calibrator.save(args.config)
                break
            if key == ord("r"):
                calibrator.rect = None
            if key in (ord("q"), 27):
                break
        cv2.destroyAllWindows()
        return 0

    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        raise SystemExit("cannot open camera: %s" % args.camera)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, int(config["camera"]["width"]))
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, int(config["camera"]["height"]))
    if config["camera"].get("fourcc"):
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config["camera"]["fourcc"]))
    window_name = "capture zone calibrator (live)"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, calibrator.on_mouse)
    print(
        "left-drag to draw the capture zone; s=save r=reset q=quit",
        flush=True,
    )
    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                print('{"state":"CAMERA_EMPTY"}', flush=True)
                break
            calibrator.height, calibrator.width = frame.shape[:2]
            measurement = calibrator.detector.detect_color(
                frame, calibrator.target_id, calibrator.scene
            )
            canvas = calibrator.draw(frame, measurement)
            canvas = text_overlay(canvas, calibrator, measurement, True)
            cv2.imshow(window_name, canvas)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("s"), ord("S")):
                calibrator.save(args.config)
                break
            if key == ord("r"):
                calibrator.rect = None
            if key in (ord("q"), 27):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()
    return 0


def run_headless(args, config, calibrator):
    if args.image:
        frame = cv2.imread(str(args.image))
        if frame is None:
            raise SystemExit("cannot read image: %s" % args.image)
        calibrator.height, calibrator.width = frame.shape[:2]
    else:
        camera = cv2.VideoCapture(args.camera)
        if not camera.isOpened():
            raise SystemExit("cannot open camera: %s" % args.camera)
        ok, frame = camera.read()
        camera.release()
        if not ok:
            raise SystemExit("camera returned no frame")
        calibrator.height, calibrator.width = frame.shape[:2]
    calibrator.rect = (args.x, args.y, args.w, args.h)
    if args.save:
        calibrator.save(args.config)
    else:
        print(
            "capture_zone_px would be %s (pass --save to write it)"
            % [int(v) for v in calibrator.rect],
            flush=True,
        )
    return 0


def parser():
    root = argparse.ArgumentParser(description="place the turntable capture zone")
    root.add_argument("--config", default="config/jetson.json")
    root.add_argument("--target", default="1", help="colour ID to search while placing the window")
    root.add_argument("--scene", default="TURNTABLE")
    source = root.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path, help="single BGR frame file")
    source.add_argument("--camera", help="camera index or /dev/v4l/by-id path")
    root.add_argument("--x", type=int, default=0, help="headless rectangle X")
    root.add_argument("--y", type=int, default=0, help="headless rectangle Y")
    root.add_argument("--w", type=int, default=0, help="headless rectangle W")
    root.add_argument("--h", type=int, default=0, help="headless rectangle H")
    root.add_argument("--headless", action="store_true", help="use CLI rectangle, no GUI")
    root.add_argument("--save", action="store_true", help="write the rectangle into the config")
    return root


def main():
    args = parser().parse_args()
    config = load_config(args.config)
    calibrator = CaptureZoneCalibrator(config, args.target, args.scene, (720, 1280, 3))
    if args.headless or args.x or args.y or args.w or args.h:
        return run_headless(args, config, calibrator)
    return run_interactive(args, config, calibrator)


if __name__ == "__main__":
    main()
