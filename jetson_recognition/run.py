"""Single-purpose Jetson test entry: live camera or saved image."""

import argparse
from collections import deque
import json
from pathlib import Path
import time

import cv2
import numpy as np

from .camera import UVCCamera
from .coordinator import run_coordinator
from .detectors import TopViewDetector
from .engine import RecognitionEngine, stable_measurement
from .output import ResultOutput
from .pickup import PickupSession
from .protocol import decode_frame


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def load_config(path):
    config_path = Path(path)
    if not config_path.is_absolute():
        config_path = PROJECT_ROOT / config_path
    return json.loads(config_path.read_text(encoding="utf-8"))


def mode_arguments(arguments):
    mode = arguments.mode.upper()
    if mode in ("COLOR", "RING"):
        return (arguments.target, arguments.scene)
    if mode == "STACK":
        return (arguments.target, arguments.scene, arguments.ring)
    return (arguments.scene,)


class RollingPerformance:
    def __init__(self, window=120):
        self.timestamps = deque(maxlen=max(2, int(window)))
        self.detect_ms_values = deque(maxlen=max(2, int(window)))

    def update(self, detect_ms, roi_mode=None):
        self.timestamps.append(time.monotonic())
        self.detect_ms_values.append(float(detect_ms))

    @property
    def fps(self):
        if len(self.timestamps) < 2:
            return 0.0
        elapsed = self.timestamps[-1] - self.timestamps[0]
        return (len(self.timestamps) - 1) / elapsed if elapsed > 0 else 0.0

    @property
    def detect_ms(self):
        if not self.detect_ms_values:
            return 0.0
        return sum(self.detect_ms_values) / len(self.detect_ms_values)

class HSVProbe:
    """Show raw BGR/HSV statistics around the mouse without using a mask."""

    def __init__(self, radius=15):
        self.radius = max(1, int(radius))
        self.position = None

    def on_mouse(self, event, x, y, flags, userdata):
        if event in (cv2.EVENT_MOUSEMOVE, cv2.EVENT_LBUTTONDOWN):
            self.position = (int(x), int(y))

    def sample(self, frame):
        height, width = frame.shape[:2]
        if self.position is None:
            center_x, center_y = width // 2, height // 2
        else:
            center_x = min(max(self.position[0], 0), width - 1)
            center_y = min(max(self.position[1], 0), height - 1)
        left = max(0, center_x - self.radius)
        top = max(0, center_y - self.radius)
        right = min(width, center_x + self.radius + 1)
        bottom = min(height, center_y + self.radius + 1)
        bgr_pixels = frame[top:bottom, left:right].reshape(-1, 3)
        hsv_pixels = cv2.cvtColor(
            frame[top:bottom, left:right], cv2.COLOR_BGR2HSV
        ).reshape(-1, 3)
        return {
            "center": (center_x, center_y),
            "rect": (left, top, right, bottom),
            "bgr50": np.percentile(bgr_pixels, 50, axis=0),
            "hsv05": np.percentile(hsv_pixels, 5, axis=0),
            "hsv50": np.percentile(hsv_pixels, 50, axis=0),
            "hsv95": np.percentile(hsv_pixels, 95, axis=0),
        }

    def draw(self, frame, canvas):
        sample = self.sample(frame)
        left, top, right, bottom = sample["rect"]
        cv2.rectangle(canvas, (left, top), (right, bottom), (255, 0, 255), 2)
        bgr = sample["bgr50"]
        low = sample["hsv05"]
        median = sample["hsv50"]
        high = sample["hsv95"]
        line1 = "PROBE x=%d y=%d  BGR50=%d,%d,%d" % (
            sample["center"][0],
            sample["center"][1],
            int(round(bgr[0])),
            int(round(bgr[1])),
            int(round(bgr[2])),
        )
        line2 = "HSV H=%d[%d..%d] S=%d[%d..%d] V=%d[%d..%d]" % (
            int(round(median[0])),
            int(round(low[0])),
            int(round(high[0])),
            int(round(median[1])),
            int(round(low[1])),
            int(round(high[1])),
            int(round(median[2])),
            int(round(low[2])),
            int(round(high[2])),
        )
        y0 = canvas.shape[0] - 54
        cv2.rectangle(
            canvas, (0, max(0, y0 - 22)), (canvas.shape[1], canvas.shape[0]),
            (0, 0, 0), -1
        )
        cv2.putText(
            canvas, line1, (10, y0), cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (255, 0, 255), 1, cv2.LINE_AA
        )
        cv2.putText(
            canvas, line2, (10, y0 + 25), cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (255, 0, 255), 1, cv2.LINE_AA
        )
        return canvas


def visualization_options(arguments, config):
    configured = config.get("visualization", {})
    return {
        "draw_contours": bool(
            getattr(arguments, "draw_contours", False)
            or configured.get("draw_color_contours", False)
        ),
        "draw_search_roi": bool(
            getattr(arguments, "show_roi", False)
            or configured.get("draw_search_roi", True)
        ),
        "show_fps": bool(
            getattr(arguments, "show_fps", False)
            or configured.get("show_fps", False)
        ),
        "show_mask": bool(
            getattr(arguments, "show_mask", False)
            or configured.get("show_mask", False)
        ),
        "show_hsv": bool(
            getattr(arguments, "show_hsv", False)
            or configured.get("show_hsv_probe", False)
        ),
    }


def draw_color_debug(canvas, debug, draw_search_roi=True):
    if not debug:
        return canvas
    if draw_search_roi:
        base_x, base_y, base_width, base_height = debug.get(
            "base_roi", debug["roi"]
        )
        cv2.rectangle(
            canvas,
            (base_x, base_y),
            (base_x + base_width, base_y + base_height),
            (255, 160, 0),
            2,
        )
        x, y, width, height = debug["roi"]
        if [x, y, width, height] != [base_x, base_y, base_width, base_height]:
            cv2.rectangle(
                canvas, (x, y), (x + width, y + height), (255, 255, 0), 1
            )
    selected = debug.get("selected")
    for candidate in debug.get("candidates", []):
        x, y, width, height = candidate["bbox"]
        is_selected = candidate is selected
        color = (0, 255, 0) if is_selected else (0, 220, 255)
        thickness = 2 if is_selected else 1
        cv2.rectangle(
            canvas, (x, y), (x + width, y + height), color, thickness
        )
        cv2.drawContours(canvas, [candidate["contour"]], -1, color, thickness)
        label = "%s A=%.0f" % (debug.get("color_name", "COLOR"), candidate["area"])
        if is_selected:
            label = "TARGET " + label
        cv2.putText(
            canvas,
            label,
            (x, max(48, y - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )
    return canvas


def draw_circle_debug(canvas, debug, draw_search_roi=True):
    if not debug:
        return canvas
    if draw_search_roi:
        x, y, width, height = debug["roi"]
        cv2.rectangle(
            canvas, (x, y), (x + width, y + height), (255, 160, 0), 2
        )
    selected = debug.get("selected")
    for candidate in debug.get("candidates", []):
        center = tuple(int(round(value)) for value in candidate["center"])
        is_selected = bool(candidate.get("selected", candidate is selected))
        color = (0, 255, 0) if is_selected else (0, 220, 255)
        thickness = 2 if is_selected else 1
        if "contour" in candidate:
            cv2.drawContours(
                canvas, [candidate["contour"]], -1, color, thickness
            )
            x, y, width, height = candidate["bbox"]
            cv2.rectangle(
                canvas, (x, y), (x + width, y + height), color, thickness
            )
            label = "A=%.0f C=%.2f" % (
                candidate["area"], candidate["circularity"]
            )
            label_position = (x, max(48, y - 5))
        else:
            radius = int(round(candidate["radius"]))
            cv2.circle(canvas, center, radius, color, thickness)
            label = "R=%.1f Q=%.2f" % (
                candidate["radius"], candidate["quality"]
            )
            label_position = (
                center[0] - radius,
                max(48, center[1] - radius - 5),
            )
        if is_selected:
            cv2.putText(
                canvas,
                label,
                label_position,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )
    for group in debug.get("groups", []):
        center = tuple(int(round(value)) for value in group["center"])
        is_selected = group is selected
        color = (0, 255, 0) if is_selected else (0, 220, 255)
        cv2.drawMarker(
            canvas, center, color, cv2.MARKER_CROSS, 20, 2
        )
        bbox = group.get("digit_bbox")
        if bbox:
            x, y, width, height = bbox
            cv2.rectangle(
                canvas, (x, y), (x + width, y + height), color, 1
            )
        cv2.putText(
            canvas,
            "ID=%s S=%.2f" % (
                group.get("ring_id", "UNKNOWN"),
                float(group.get("digit_score", 0.0)),
            ),
            (center[0] - 45, center[1] + 72),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )
    return canvas


def draw_result(
    frame,
    measurement,
    state,
    count,
    color_debug=None,
    circle_debug=None,
    performance=None,
    draw_search_roi=True,
):
    canvas = frame.copy()
    if measurement is None:
        text = state
        color = (0, 0, 255)
    else:
        color_name = (
            color_debug.get("color_name")
            if color_debug and measurement.kind == "COLOR"
            else measurement.kind + ":" + measurement.target_id
        )
        text = "%s %s center=(%.1f,%.1f) %s confidence=%.2f frames=%d" % (
            state,
            color_name,
            measurement.x,
            measurement.y,
            measurement.unit,
            measurement.confidence,
            count,
        )
        color = (0, 255, 0) if state == "STABLE" else (0, 220, 255)
        point_x = measurement.pixel_x if measurement.pixel_x is not None else measurement.x
        point_y = measurement.pixel_y if measurement.pixel_y is not None else measurement.y
        cv2.drawMarker(
            canvas, (int(round(point_x)), int(round(point_y))),
            color, cv2.MARKER_CROSS, 28, 2,
        )
    canvas = draw_color_debug(canvas, color_debug, draw_search_roi)
    canvas = draw_circle_debug(canvas, circle_debug, draw_search_roi)
    if performance is not None:
        text += "  FPS=%.1f DET=%.2fms" % (
            performance.fps,
            performance.detect_ms,
        )
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 38), (0, 0, 0), -1)
    cv2.putText(canvas, text, (10, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)
    return canvas


def draw_pickup(frame, update, pickup_session):
    canvas = frame.copy()
    outer = tuple(int(round(value)) for value in pickup_session.capture_zone)
    inner = tuple(int(round(value)) for value in pickup_session.safe_zone)
    cv2.rectangle(
        canvas,
        (outer[0], outer[1]),
        (outer[0] + outer[2], outer[1] + outer[3]),
        (255, 180, 0),
        2,
    )
    cv2.rectangle(
        canvas,
        (inner[0], inner[1]),
        (inner[0] + inner[2], inner[1] + inner[3]),
        (0, 255, 0),
        2,
    )
    measurement = update.measurement
    if measurement is not None:
        pixel_x = measurement.pixel_x if measurement.pixel_x is not None else measurement.x
        pixel_y = measurement.pixel_y if measurement.pixel_y is not None else measurement.y
        cv2.drawMarker(
            canvas,
            (int(round(pixel_x)), int(round(pixel_y))),
            (0, 255, 0) if update.state == "GRASP_READY" else (0, 220, 255),
            cv2.MARKER_CROSS,
            28,
            2,
        )
    text = "%s target=%s speed=%.1f stable=%d final=%d" % (
        update.state,
        pickup_session.target_id,
        update.speed_px_s,
        update.stable_count,
        update.final_count,
    )
    color = (0, 255, 0) if update.state == "GRASP_READY" else (0, 220, 255)
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 38), (0, 0, 0), -1)
    cv2.putText(canvas, text, (10, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, cv2.LINE_AA)
    return canvas


def build_engine(config):
    cv2.setUseOptimized(True)
    opencv_threads = int(config.get("runtime", {}).get("opencv_threads", 0))
    if opencv_threads > 0:
        cv2.setNumThreads(opencv_threads)
    runtime = dict(config["runtime"])
    runtime["color_stability"] = dict(config.get("stability", {}))
    return RecognitionEngine(TopViewDetector(config, PROJECT_ROOT), runtime)


def run_image(arguments, config, engine, output):
    visual = visualization_options(arguments, config)
    color_image = arguments.mode.upper() == "COLOR"
    if color_image and (arguments.show or arguments.save):
        visual["draw_contours"] = True
        visual["draw_search_roi"] = True
    engine.detector.set_color_debug(
        color_image
        or visual["draw_contours"]
        or visual["show_mask"]
        or visual["draw_search_roi"]
        or bool(arguments.save_mask)
    )
    circle_image = arguments.mode.upper() in ("RING", "STATION", "TURNTABLE")
    engine.detector.set_circle_debug(
        circle_image
        and (arguments.show or arguments.save or arguments.save_mask or visual["show_mask"])
    )
    frame = cv2.imread(str(arguments.image))
    if frame is None:
        raise SystemExit("cannot read image: %s" % arguments.image)
    measurement = engine.detect(frame, arguments.mode, mode_arguments(arguments))
    if measurement is None:
        print('{"state":"NOT_FOUND"}')
    else:
        output.emit(measurement, "SINGLE_FRAME", 1)
    if arguments.save:
        cv2.imwrite(
            str(arguments.save),
            draw_result(
                frame,
                measurement,
                "SINGLE_FRAME",
                int(measurement is not None),
                engine.detector.last_color_debug,
                engine.detector.last_circle_debug,
                draw_search_roi=visual["draw_search_roi"],
            ),
        )
    debug = engine.detector.last_color_debug
    if arguments.save_mask:
        debug_image = debug.get("mask") if debug else None
        if debug_image is None and engine.detector.last_circle_debug:
            debug_image = engine.detector.last_circle_debug.get("processed")
        if debug_image is None:
            raise SystemExit("no mask/preprocessed ROI is available for this mode")
        cv2.imwrite(str(arguments.save_mask), debug_image)
    if arguments.show or visual["show_mask"]:
        if arguments.show:
            cv2.imshow(
                "standalone-vision image test",
                draw_result(
                    frame,
                    measurement,
                    "SINGLE_FRAME",
                    int(measurement is not None),
                    debug,
                    engine.detector.last_circle_debug,
                    draw_search_roi=visual["draw_search_roi"],
                ),
            )
        if visual["show_mask"] and debug and debug.get("mask") is not None:
            cv2.imshow("Mask", debug["mask"])
        elif visual["show_mask"] and engine.detector.last_circle_debug:
            cv2.imshow(
                "Ring Mask"
                if engine.detector.last_circle_debug.get("kind") == "RING"
                else "Preprocessed ROI",
                engine.detector.last_circle_debug["processed"],
            )
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return 0 if measurement is not None else 2


def run_live(arguments, config, engine, output):
    visual = visualization_options(arguments, config)
    if arguments.headless and (arguments.show_mask or arguments.show_hsv):
        raise SystemExit("--show-mask/--show-hsv require a graphical desktop")
    if arguments.headless:
        # Graphical defaults in the configuration must not prevent an SSH run.
        visual["show_mask"] = False
        visual["show_hsv"] = False
    color_gui = arguments.mode.upper() == "COLOR" and not arguments.headless
    if color_gui:
        visual["draw_contours"] = True
        visual["draw_search_roi"] = True
    engine.detector.set_color_debug(
        color_gui or visual["show_mask"] or arguments.debug
    )
    circle_gui = (
        arguments.mode.upper() in ("RING", "STATION", "TURNTABLE")
        and not arguments.headless
    )
    engine.detector.set_circle_debug(
        circle_gui or visual["show_mask"] or arguments.debug
    )
    camera_config = dict(config["camera"])
    if arguments.camera_profile:
        camera_config["v4l2_control_profile"] = arguments.camera_profile
    if arguments.camera_backend:
        camera_config["backend"] = arguments.camera_backend
    active_profile = camera_config.get("v4l2_control_profile")
    camera = UVCCamera(camera_config, PROJECT_ROOT, arguments.camera)
    print(
        json.dumps(
            {
                "state": "CAMERA_READY",
                "profile": active_profile,
                "settings": camera.actual_settings,
                "requested_backend": camera_config.get("backend"),
                "startup_controls": camera.startup_controls,
                "requested_fps": float(camera_config["fps"]),
                "debug": visual,
            }
        ),
        flush=True,
    )
    session = engine.new_session(arguments.mode, mode_arguments(arguments))
    frame_number = 0
    print_every = max(1, int(arguments.print_every))
    last_state = None
    last_target_id = None
    performance = RollingPerformance()
    window_name = "Detection"
    hsv_probe = HSVProbe() if visual["show_hsv"] else None
    if not arguments.headless:
        cv2.namedWindow(window_name)
        if hsv_probe is not None:
            cv2.setMouseCallback(window_name, hsv_probe.on_mouse)
    print("Jetson standalone test started; press q or Ctrl+C to stop.", flush=True)
    camera_empty_reported = False
    try:
        while True:
            frame = camera.read()
            if frame is None:
                if not camera_empty_reported:
                    print('{"state":"CAMERA_EMPTY"}', flush=True)
                    camera_empty_reported = True
                time.sleep(0.02)
                continue
            if camera_empty_reported:
                print('{"state":"CAMERA_RECOVERED"}', flush=True)
                camera_empty_reported = False
            frame_number += 1
            detect_started = time.perf_counter()
            measurement, stable, count = session.update(frame)
            roi_runtime = engine.detector.last_color_runtime or {}
            performance.update(
                (time.perf_counter() - detect_started) * 1000.0,
                roi_runtime.get("mode"),
            )
            if stable is not None:
                state = "STABLE"
                reported_measurement = stable_measurement(measurement, stable)
            elif measurement is not None:
                state = "UNSTABLE"
                reported_measurement = measurement
            else:
                state = "LOST"
                reported_measurement = None
            state_changed = state != last_state
            target_changed = (
                measurement is not None and measurement.target_id != last_target_id
            )
            if reported_measurement is not None and (
                state_changed or target_changed
                or (arguments.debug and frame_number % print_every == 0)
            ):
                output.emit(reported_measurement, state, count)
                last_target_id = reported_measurement.target_id
            elif state == "LOST" and state_changed:
                print(
                    json.dumps(
                        {"state": "LOST", "previous_target_id": last_target_id},
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
            last_state = state
            if not arguments.headless:
                canvas = draw_result(
                    frame,
                    measurement,
                    state,
                    count,
                    engine.detector.last_color_debug,
                    engine.detector.last_circle_debug,
                    performance if visual["show_fps"] else None,
                    visual["draw_search_roi"],
                )
                if hsv_probe is not None:
                    canvas = hsv_probe.draw(frame, canvas)
                cv2.imshow(window_name, canvas)
                debug = engine.detector.last_color_debug
                if visual["show_mask"] and debug and debug.get("mask") is not None:
                    cv2.imshow("Mask", debug["mask"])
                elif visual["show_mask"] and engine.detector.last_circle_debug:
                    cv2.imshow(
                        "Ring Mask"
                        if engine.detector.last_circle_debug.get("kind") == "RING"
                        else "Preprocessed ROI",
                        engine.detector.last_circle_debug["processed"],
                    )
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
            if visual["show_fps"] and frame_number % max(1, arguments.stats_every) == 0:
                color_runtime = engine.detector.last_color_runtime or {}
                debug = engine.detector.last_color_debug or {}
                debug_candidates = debug.get("candidates", [])
                filter_stats = color_runtime.get("filter_stats") or {}
                representative = (
                    max(debug_candidates, key=lambda item: item.get("area", 0.0))
                    if debug_candidates
                    else None
                )
                candidate_summary = None
                if representative is not None:
                    candidate_summary = {
                        "bbox": representative.get("bbox"),
                        "area": round(float(representative.get("area", 0.0)), 1),
                        "confidence": round(
                            float(representative.get("quality", 0.0)), 3
                        ),
                    }
                perf_details = {
                    "roi_mode": color_runtime.get("mode"),
                    "roi": color_runtime.get("roi"),
                    "filter_stats": filter_stats,
                    "candidate_count": int(
                        filter_stats.get("accepted", len(debug_candidates))
                    ),
                    "largest_candidate": candidate_summary,
                }
                if arguments.mode.upper() == "RING":
                    ring_debug = engine.detector.last_circle_debug or {}
                    ring_candidates = ring_debug.get("candidates", [])
                    largest_ring = (
                        max(ring_candidates, key=lambda item: item["area"])
                        if ring_candidates
                        else None
                    )
                    perf_details = {
                        "roi": ring_debug.get("roi"),
                        "threshold": ring_debug.get("threshold"),
                        "template_source": ring_debug.get("template_source"),
                        "candidate_count": ring_debug.get("candidate_count", 0),
                        "cluster_count": ring_debug.get("cluster_count", 0),
                        "selected_cluster": ring_debug.get("selected"),
                        "largest_candidate": (
                            {
                                "area": round(largest_ring["area"], 1),
                                "circularity": round(
                                    largest_ring["circularity"], 3
                                ),
                                "aspect_ratio": round(
                                    largest_ring["aspect_ratio"], 3
                                ),
                            }
                            if largest_ring is not None
                            else None
                        ),
                    }
                print(
                    json.dumps(
                        {
                            "state": "PERF",
                            "fps": round(performance.fps, 2),
                            "capture_fps": round(camera.capture_fps, 2),
                            "detect_ms": round(performance.detect_ms, 3),
                            "frames": frame_number,
                            **perf_details,
                        }
                    ),
                    flush=True,
                )
    finally:
        camera.close()
        cv2.destroyAllWindows()
    return 0


def run_pickup(arguments, config, engine, output):
    pickup_config = dict(config.get("pickup", {}))
    if not pickup_config.get("enabled", False):
        raise SystemExit(
            "legacy pickup gate is disabled; coordinator PICK uses configured COLOR stability"
        )
    if arguments.capture_zone is not None:
        pickup_config["capture_zone_px"] = list(arguments.capture_zone)
    if arguments.timeout is not None:
        pickup_config["search_timeout_s"] = float(arguments.timeout)
    session = PickupSession(engine, arguments.target, arguments.scene, pickup_config)
    camera = UVCCamera(config["camera"], PROJECT_ROOT, arguments.camera)
    timeout_s = float(pickup_config.get("search_timeout_s", 20.0))
    started = time.monotonic()
    frame_number = 0
    print_every = max(1, int(arguments.print_every))
    ready_emit_every = max(
        1, int(pickup_config.get("ready_emit_every_frames", 3))
    )
    last_state = None
    exit_code = 0
    print(
        "Pickup waiting test started; blue=outer zone, green=safe zone; press q to stop.",
        flush=True,
    )
    try:
        while True:
            if time.monotonic() - started >= timeout_s:
                print(
                    json.dumps(
                        {
                            "state": "TARGET_TIMEOUT",
                            "target_id": str(arguments.target),
                            "timeout_s": timeout_s,
                        }
                    ),
                    flush=True,
                )
                exit_code = 3
                break
            frame = camera.read()
            if frame is None:
                print('{"state":"CAMERA_EMPTY"}', flush=True)
                time.sleep(0.02)
                continue
            frame_number += 1
            update = session.update(frame)
            should_emit = (
                update.state != last_state
                or (update.ready and frame_number % ready_emit_every == 0)
                or frame_number % print_every == 0
            )
            if should_emit:
                output.emit_pickup(update)
            last_state = update.state
            if not arguments.headless:
                cv2.imshow(
                    "standalone-vision pickup waiting test",
                    draw_pickup(frame, update, session),
                )
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
            if update.ready and arguments.exit_on_ready:
                break
    finally:
        camera.close()
        cv2.destroyAllWindows()
    return exit_code


def run_bridge(arguments):
    """Receive MaixCAM Pro frames on one UART and forward/print them.

    This is the receive side of the Maix -> Jetson link. Each frame is
    CRC-checked, printed as JSON, and optionally re-emitted on a second UART
    so a computer or downstream MCU sees the raw frame.
    """
    import serial

    port = serial.Serial(arguments.in_serial, baudrate=arguments.baud, timeout=0.2)
    out_serial = None
    if arguments.out_serial:
        out_serial = serial.Serial(
            arguments.out_serial,
            baudrate=arguments.baud,
            timeout=0,
            write_timeout=0.2,
        )
    print('{"state":"BRIDGE_READY","in":"%s"}' % arguments.in_serial, flush=True)
    buffer = b""
    try:
        while True:
            chunk = port.read(256)
            if not chunk:
                continue
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                decoded = decode_frame(line)
                if decoded is None:
                    continue
                name, fields = decoded
                print(
                    json.dumps({"frame": name, "fields": fields}, ensure_ascii=False),
                    flush=True,
                )
                if out_serial is not None:
                    out_serial.write(line + b"\n")
    finally:
        port.close()
        if out_serial is not None:
            out_serial.close()
    return 0


def run_maix_network_receiver(arguments):
    """Receive and CRC-check Maix QR frames over the USB TCP link."""
    import socket

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((arguments.host, int(arguments.port)))
    server.listen(1)
    print(
        json.dumps(
            {
                "state": "MAIX_TCP_LISTENING",
                "host": arguments.host,
                "port": int(arguments.port),
            }
        ),
        flush=True,
    )
    try:
        while True:
            connection, address = server.accept()
            print(
                json.dumps(
                    {
                        "state": "MAIX_TCP_CONNECTED",
                        "peer": "%s:%s" % address,
                    }
                ),
                flush=True,
            )
            buffer = b""
            try:
                while True:
                    chunk = connection.recv(1024)
                    if not chunk:
                        break
                    buffer += chunk
                    if len(buffer) > 4096 and b"\n" not in buffer:
                        print('{"state":"MAIX_TCP_BUFFER_RESET"}', flush=True)
                        buffer = b""
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        decoded = decode_frame(line)
                        if decoded is None:
                            print(
                                json.dumps(
                                    {
                                        "state": "MAIX_TCP_INVALID_FRAME",
                                        "raw": line.decode("ascii", "replace"),
                                    }
                                ),
                                flush=True,
                            )
                            continue
                        name, fields = decoded
                        print(
                            json.dumps(
                                {
                                    "state": "MAIX_TCP_FRAME",
                                    "frame": name,
                                    "fields": fields,
                                    "peer": address[0],
                                },
                                ensure_ascii=False,
                            ),
                            flush=True,
                        )
            except (ConnectionError, OSError) as error:
                print(
                    json.dumps(
                        {
                            "state": "MAIX_TCP_CONNECTION_ERROR",
                            "peer": address[0],
                            "error": str(error),
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
            finally:
                connection.close()
                print(
                    json.dumps(
                        {
                            "state": "MAIX_TCP_DISCONNECTED",
                            "peer": address[0],
                        }
                    ),
                    flush=True,
                )
    except KeyboardInterrupt:
        print('{"state":"MAIX_TCP_STOPPED"}', flush=True)
    finally:
        server.close()
    return 0


def parser():
    root = argparse.ArgumentParser(description="Jetson recognition-only tester (serial is optional)")
    root.add_argument("--config", default="config/jetson.json")
    root.add_argument("--serial", help="optional result-output serial device")
    root.add_argument("--baud", type=int, default=115200)
    # JetPack 4 on many Jetson Nano boards still uses Python 3.6, where the
    # ``required`` keyword for subparsers is unavailable.
    subparsers = root.add_subparsers(dest="source")
    for name in ("live", "image"):
        command = subparsers.add_parser(name)
        command.add_argument("--mode", choices=("COLOR", "RING", "STACK", "STATION", "TURNTABLE"), required=True)
        command.add_argument("--scene", required=True)
        command.add_argument(
            "--target", default="1",
            help="color ID/name (1..6, red, yellow, blue, green, black, light_blue) or ring ID",
        )
        command.add_argument("--ring", default="1", help="ring ID for STACK")
        command.add_argument(
            "--draw-contours",
            action="store_true",
            help="draw accepted color contours and bounding boxes",
        )
        command.add_argument(
            "--show-mask",
            action="store_true",
            help="show the HSV mask or preprocessed circle ROI in a second window",
        )
        command.add_argument(
            "--show-roi",
            action="store_true",
            help="draw the configured fixed color ROI",
        )
    live = subparsers.choices["live"]
    live.add_argument("--camera", help="camera index or /dev/v4l/by-id path; config value is used when omitted")
    live.add_argument(
        "--camera-profile",
        help="temporarily override camera.v4l2_control_profile",
    )
    live.add_argument(
        "--camera-backend",
        choices=("gstreamer_nv", "opencv_v4l2"),
        help="temporarily override camera.backend",
    )
    live.add_argument("--headless", action="store_true")
    live.add_argument(
        "--debug", action="store_true",
        help="periodically print detection details; default prints state changes only",
    )
    live.add_argument("--print-every", type=int, default=10)
    live.add_argument("--show-fps", action="store_true")
    live.add_argument(
        "--show-hsv",
        action="store_true",
        help="show raw BGR/HSV statistics around the mouse pointer",
    )
    live.add_argument(
        "--stats-every",
        type=int,
        default=60,
        help="headless PERF output interval in frames",
    )
    image = subparsers.choices["image"]
    image.add_argument("image", type=Path)
    image.add_argument("--show", action="store_true")
    image.add_argument("--save", type=Path)
    image.add_argument("--save-mask", type=Path)
    pickup = subparsers.add_parser(
        "pickup", help="wait for one requested color in a safe turntable capture zone"
    )
    pickup.add_argument("--scene", default="TURNTABLE")
    pickup.add_argument("--target", required=True, help="requested color ID 1..6")
    pickup.add_argument("--camera", help="camera index or /dev/v4l/by-id path")
    pickup.add_argument(
        "--capture-zone",
        type=int,
        nargs=4,
        metavar=("X", "Y", "W", "H"),
        help="temporary outer capture zone override in pixels",
    )
    pickup.add_argument("--headless", action="store_true")
    pickup.add_argument("--print-every", type=int, default=10)
    pickup.add_argument("--timeout", type=float)
    pickup.add_argument("--exit-on-ready", action="store_true")
    bridge = subparsers.add_parser(
        "bridge",
        help="receive MaixCAM Pro frames on one UART and optionally forward them",
    )
    bridge.add_argument(
        "--in-serial", required=True, help="UART that receives Maix frames"
    )
    bridge.add_argument(
        "--out-serial", help="optional second UART to re-emit raw frames"
    )
    bridge.add_argument("--baud", type=int, default=115200)
    maix_network = subparsers.add_parser(
        "maix-net",
        help="receive MaixCAM Pro QR frames over the USB virtual network",
    )
    maix_network.add_argument(
        "--host", default="0.0.0.0", help="local listen address"
    )
    maix_network.add_argument("--port", type=int, default=5000)
    coordinator = subparsers.add_parser(
        "coordinator",
        help="MCU-driven vision service: task code from Maix, results to the MCU",
    )
    coordinator.add_argument(
        "--maix-transport",
        choices=("tcp", "serial"),
        help="Maix input transport; defaults to config coordinator.maix_transport",
    )
    coordinator.add_argument(
        "--maix-serial", help="UART receiving Maix frames"
    )
    coordinator.add_argument(
        "--maix-tcp-host", help="local address for the Maix TCP listener"
    )
    coordinator.add_argument(
        "--maix-tcp-port", type=int, help="Maix TCP listener port"
    )
    coordinator.add_argument(
        "--mcu-serial", help="bidirectional UART to the electronics"
    )
    coordinator.add_argument(
        "--camera", help="camera index or /dev/v4l/by-id path"
    )
    coordinator.add_argument(
        "--gui",
        action="store_true",
        help="show Detection and Mask windows for coordinator debugging",
    )
    coordinator.add_argument("--baud", type=int, default=115200)
    return root


def resolve_serial(arguments, config):
    """Serial output comes from the CLI or, when enabled, from the config."""
    serial_cfg = config.get("serial", {})
    if arguments.serial:
        return arguments.serial, int(arguments.baud)
    if serial_cfg.get("enabled"):
        return serial_cfg.get("device"), int(serial_cfg.get("baud", arguments.baud))
    return None, int(arguments.baud)


def main():
    cli_parser = parser()
    arguments = cli_parser.parse_args()
    if arguments.source is None:
        cli_parser.error(
            "choose one test source: live, image, pickup, bridge, maix-net or coordinator"
        )
    config = load_config(arguments.config)
    if arguments.source == "bridge":
        raise SystemExit(run_bridge(arguments))
    if arguments.source == "maix-net":
        raise SystemExit(run_maix_network_receiver(arguments))
    engine = build_engine(config)
    if arguments.source == "coordinator":
        raise SystemExit(
            run_coordinator(
                arguments,
                config,
                engine,
                PROJECT_ROOT,
                draw_result_fn=draw_result,
            )
        )
    serial_device, serial_baud = resolve_serial(arguments, config)
    color_names = {
        str(color_id): definition["name"]
        for color_id, definition in config.get("colors", {}).items()
    }
    output = ResultOutput(serial_device, serial_baud, color_names)
    try:
        if arguments.source == "image":
            raise SystemExit(run_image(arguments, config, engine, output))
        if arguments.source == "pickup":
            raise SystemExit(run_pickup(arguments, config, engine, output))
        raise SystemExit(run_live(arguments, config, engine, output))
    finally:
        output.close()


if __name__ == "__main__":
    main()
