"""Draw one scene ROI on a live (or saved) frame and write it into the config.

This is the missing first half of the calibration flow: everything downstream
(ring search, object search, the turntable disc) is configured as a pixel
rectangle, and until now those rectangles had to be hand-edited in
``config/jetson.json``.

Examples:

    # live camera, draw scenes.TURNTABLE.object_roi, then write it
    python3 tools/pick_roi.py --scene TURNTABLE --key object_roi --write

    # from a saved frame, draw one ring box
    python3 tools/pick_roi.py --image frame.jpg --scene ROUGH --key ring_rois.2 --write

    # measure the field: select an object whose real length is known
    python3 tools/pick_roi.py --known-mm 300

Keys after picking:  r = pick again,  w = write to the config,  q = quit.
"""

import argparse
import json
from pathlib import Path
import sys

import cv2


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jetson_recognition.camera import UVCCamera


KEYS = (
    "object_roi",
    "turntable_roi",
    "ring_search_roi",
    "ring_rois.1",
    "ring_rois.2",
    "ring_rois.3",
)
DICT_KEYS = ("object_roi", "ring_search_roi")


def parser():
    root = argparse.ArgumentParser(description=__doc__,
                                   formatter_class=argparse.RawDescriptionHelpFormatter)
    root.add_argument("--config", default="config/jetson.json")
    root.add_argument("--scene", help="scene name, e.g. TURNTABLE / ROUGH / STORAGE")
    root.add_argument("--key", choices=KEYS, help="which ROI to draw")
    root.add_argument("--camera", help="camera device or index; config value when omitted")
    root.add_argument("--image", type=Path, help="use a saved frame instead of the camera")
    root.add_argument("--write", action="store_true",
                      help="write the picked ROI into the config (keeps a backup)")
    root.add_argument("--known-mm", type=float,
                      help="real length of the selected feature: also print mm/px and field size")
    root.add_argument("--value", nargs=4, type=int, metavar=("X", "Y", "W", "H"),
                      help="skip the mouse and use this rectangle (headless)")
    return root


def load_config(path):
    return json.loads(path.read_text(encoding="utf-8"))


def roi_node(config, scene, key):
    """Return (container, final_key) for a dotted key, creating nothing yet."""
    scenes = config.setdefault("scenes", {})
    if scene not in scenes:
        raise SystemExit("unknown scene in config: %s" % scene)
    if key.startswith("ring_rois."):
        ring_id = key.split(".", 1)[1]
        return scenes[scene].setdefault("ring_rois", {}), ring_id
    return scenes[scene], key


def current_value(config, scene, key):
    container, final_key = roi_node(config, scene, key)
    return container.get(final_key)


def as_payload(key, box):
    """Match the style already used in config/jetson.json."""
    x, y, width, height = [int(value) for value in box]
    if key in DICT_KEYS:
        return {"x": x, "y": y, "width": width, "height": height}
    return [x, y, width, height]


def save_config(path, config):
    from tools.capture_camera_profile import save_config as write_with_backup

    return write_with_backup(path, config)


def describe(box, frame_shape, known_mm=None):
    x, y, width, height = [int(value) for value in box]
    message = {
        "state": "ROI_PICKED",
        "roi": [x, y, width, height],
        "size_px": [width, height],
        "frame": [int(frame_shape[1]), int(frame_shape[0])],
        "inside_frame": bool(
            x >= 0 and y >= 0
            and x + width <= frame_shape[1]
            and y + height <= frame_shape[0]
        ),
    }
    if known_mm:
        reference = max(width, height)
        mm_per_px = float(known_mm) / float(max(reference, 1))
        message["mm_per_px"] = round(mm_per_px, 5)
        message["field_mm"] = [
            round(frame_shape[1] * mm_per_px, 1),
            round(frame_shape[0] * mm_per_px, 1),
        ]
        message["note"] = "known length taken along the longer side of the box"
    return message


def pick_with_mouse(frame):
    window = "pick ROI  (drag a box, then SPACE/ENTER)"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    box = cv2.selectROI(window, frame, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow(window)
    return [int(value) for value in box]


def main():
    arguments = parser().parse_args()
    if arguments.value is None and not (arguments.scene and arguments.key):
        raise SystemExit(
            "give --value X Y W H, or --scene and --key to pick with the mouse"
        )
    config_path = Path(arguments.config)
    if not config_path.is_absolute():
        config_path = PROJECT_ROOT / config_path
    config = load_config(config_path)
    frame_shape = (
        int(config["camera"]["height"]), int(config["camera"]["width"]), 3,
    )

    if arguments.scene and arguments.key:
        print(json.dumps({"state": "ROI_CURRENT", "scene": arguments.scene,
                          "key": arguments.key,
                          "value": current_value(config, arguments.scene, arguments.key)},
                         ensure_ascii=False), flush=True)

    def store(box):
        container, final_key = roi_node(config, arguments.scene, arguments.key)
        container[final_key] = as_payload(arguments.key, box)
        backup = save_config(config_path, config)
        print(json.dumps({"state": "ROI_SAVED", "scene": arguments.scene,
                          "key": arguments.key, "backup": backup,
                          "config": str(config_path)}, ensure_ascii=False), flush=True)

    # Headless path: no camera, no window, just validate and report the numbers.
    if arguments.value is not None:
        box = [int(value) for value in arguments.value]
        print(json.dumps(describe(box, frame_shape, arguments.known_mm),
                         ensure_ascii=False), flush=True)
        if arguments.scene and arguments.key:
            print(json.dumps({"state": "ROI_JSON", "scene": arguments.scene,
                              "key": arguments.key,
                              "value": as_payload(arguments.key, box)},
                             ensure_ascii=False), flush=True)
            if arguments.write:
                store(box)
        return 0

    frame = None
    camera = None
    if arguments.image:
        frame = cv2.imread(str(arguments.image))
        if frame is None:
            raise SystemExit("cannot read image: %s" % arguments.image)
    else:
        camera = UVCCamera(config["camera"], PROJECT_ROOT, arguments.camera)
        for _ in range(40):
            candidate = camera.read()
            if candidate is not None:
                frame = candidate
    if frame is None:
        raise SystemExit("no frame available")

    try:
        box = None
        while True:
            if box is None:
                box = pick_with_mouse(frame)
                if box[2] <= 0 or box[3] <= 0:
                    print('{"state":"ROI_CANCELLED"}', flush=True)
                    break
            print(json.dumps(describe(box, frame.shape, arguments.known_mm),
                             ensure_ascii=False), flush=True)
            if arguments.scene and arguments.key:
                print(json.dumps({"state": "ROI_JSON", "scene": arguments.scene,
                                  "key": arguments.key,
                                  "value": as_payload(arguments.key, box)},
                                 ensure_ascii=False), flush=True)
            canvas = frame.copy()
            cv2.rectangle(canvas, (box[0], box[1]),
                          (box[0] + box[2], box[1] + box[3]), (0, 255, 0), 2)
            cv2.putText(canvas, "r=redo  w=write  q=quit", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow("roi preview", canvas)
            key = cv2.waitKey(0) & 0xFF
            if key == ord("r"):
                box = None
                continue
            if key == ord("w"):
                if not (arguments.scene and arguments.key):
                    print('{"state":"ROI_WRITE_SKIPPED","reason":"NO_SCENE_OR_KEY"}',
                          flush=True)
                    continue
                store(box)
                break
            if key in (ord("q"), 27):
                print('{"state":"ROI_DISCARDED"}', flush=True)
                break
    except KeyboardInterrupt:
        pass
    finally:
        if camera is not None:
            camera.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
