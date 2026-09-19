"""Live all-color turntable test; report offsets only for the requested target.

The round carries three materials of three different colours, so the test first
labels every material in the frame and then solves the batch assignment instead
of asking "is this the colour I want" once per colour.
"""

import argparse
from collections import deque
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jetson_recognition.camera import UVCCamera, apply_capture_profile
from jetson_recognition.color_batch import assign_distinct, assignment_margin
from jetson_recognition.color_profiles import apply_color_profile, save_override
from jetson_recognition.run import HSVProbe, RollingPerformance, build_engine
from jetson_recognition.stability import SimpleColorStability
from jetson_recognition.task_code import COLOR_NAMES, decode_task_code


def parse_colors(arguments):
    if arguments.all_colors:
        return list("123456")
    if arguments.task_code:
        plan = decode_task_code(arguments.task_code)
        return [item["color_id"] for item in plan["batches"][arguments.batch - 1]]
    if not arguments.round_colors:
        raise SystemExit("provide --task-code or --round-colors")
    colors = [str(value) for value in arguments.round_colors]
    if len(set(colors)) != 3:
        raise SystemExit("--round-colors must contain three distinct colors")
    return colors


def nearest_material(entries, position):
    """Index of the material under the mouse, or the largest one."""
    if not entries:
        return None
    if position is None:
        return max(range(len(entries)), key=lambda index: entries[index]["area"])
    return min(
        range(len(entries)),
        key=lambda index: (
            (entries[index]["center"][0] - position[0]) ** 2
            + (entries[index]["center"][1] - position[1]) ** 2
        ),
    )


def save_prototypes(config_path, prototypes, project_root=None, profile=None):
    """Write colors.<id>.prototype_hsv where the active mode keeps its values.

    The fill-light set lives in config/jetson.json (that is what was tuned with
    a light on); the no-fill-light set lives in config/color_profiles.json, so
    sampling in ambient mode never overwrites the fill-light calibration.
    """
    payload = {
        color_id: {
            "prototype_hsv": [round(float(value), 1) for value in values]
        }
        for color_id, values in sorted(prototypes.items())
    }
    if profile == "ambient" and project_root is not None:
        return "%s (backup %s)" % (
            "config/color_profiles.json",
            save_override(project_root, "ambient", "colors", payload),
        )
    from tools.capture_camera_profile import save_config

    config = json.loads(config_path.read_text(encoding="utf-8"))
    for color_id, block in payload.items():
        config.setdefault("colors", {}).setdefault(color_id, {}).update(block)
    return save_config(config_path, config)


def channel_excess(detector, color_id, hue, saturation, value):
    """How far a measured colour sits outside colors.<id>, per channel.

    This is the actionable part of the diagnosis: it says which bound to move
    (for example ``{"V": 45}`` means "v_min is 45 too high").
    """
    best = None
    for lower, upper in detector.color_ranges.get(color_id, ()):
        excess = {}
        for name, sample, lo, hi in (
            ("H", hue, lower[0], upper[0]),
            ("S", saturation, lower[1], upper[1]),
            ("V", value, lower[2], upper[2]),
        ):
            over = max(float(lo) - sample, sample - float(hi))
            if over > 0:
                excess[name] = round(over, 1)
        if best is None or sum(excess.values()) < sum(best.values()):
            best = excess
    return best or {}


def jsonable(value):
    """Make a nested debug structure safe for json.dumps.

    The material debug entries carry numpy arrays (contours, scores), which are
    useful in the window but cannot be serialised.
    """
    if isinstance(value, dict):
        return {str(key): jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(item) for item in value]
    if isinstance(value, np.ndarray):
        return "<ndarray %s>" % (tuple(value.shape),)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.bool_):
        return bool(value)
    return value


def diagnose(detector, frame, colors, color_names, output_dir):
    """Print why each colour does or does not produce a material.

    This answers "only red is detected" without guessing: it reports the mask
    size, the median HSV *inside* the mask (compare it with colors.<id>.hsv_
    ranges), which filter rejected the blobs, and the area band to put into
    scenes.*.min_area/max_area. A frame and a mask montage are written next to
    the log so the images can be shared.
    """
    # Threshold the frame the detector actually sees: in no-fill-light mode that
    # is the white-balanced one, and reporting the raw frame's masks would
    # describe pixels that never reach the colour decision.
    work = detector._work_frame(frame)
    hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)
    report = {"state": "DIAGNOSE", "frame_px": [int(work.shape[1]), int(work.shape[0])]}
    report["thresholded_frame"] = "white_balanced" if work is not frame else "raw"
    report["illumination"] = detector.last_illumination
    report["lighting"] = {
        "fill_light": detector.fill_light,
        "illumination_enabled": detector.illumination_enabled,
    }
    tiles = []
    colours_report = {}
    for color_id in colors:
        mask = detector._clean_mask(detector._make_color_mask(hsv, color_id))
        count = int(np.count_nonzero(mask))
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        areas = sorted(
            (float(cv2.contourArea(contour)) for contour in contours), reverse=True
        )[:3]
        entry = {"mask_px": count, "top_areas": [int(value) for value in areas]}
        if count:
            # Boolean masking: a uint8 0/255 array would index instead of mask.
            inside = mask > 0
            entry["mask_hsv_median"] = [
                int(np.median(hsv[..., 0][inside])),
                int(np.median(hsv[..., 1][inside])),
                int(np.median(hsv[..., 2][inside])),
            ]
            entry["mask_v_percentiles"] = [
                int(value)
                for value in np.percentile(hsv[..., 2][inside], [10, 50, 90])
            ]
            entry["mask_hue_percentiles"] = [
                int(value)
                for value in np.percentile(hsv[..., 0][inside], [10, 50, 90])
            ]
        colours_report[color_id] = entry
        tiles.append(
            cv2.cvtColor(
                cv2.resize(mask, (426, 240), interpolation=cv2.INTER_AREA),
                cv2.COLOR_GRAY2BGR,
            )
        )
    report["colors"] = colours_report
    debug = detector.last_materials_debug or {}
    materials = list(debug.get("materials", []))
    for entry in materials:
        core = entry.get("core_hsv")
        if not core:
            continue
        entry["outside_ranges"] = {
            color_id: excess
            for color_id, excess in (
                (color_id, channel_excess(detector, color_id, core[0], core[1], core[2]))
                for color_id in colors
            )
            if excess
        }
    report["materials"] = materials
    report["rings_explained"] = debug.get("rings_explained", [])
    report["filter_stats"] = debug.get("filter_stats", {})
    report["expected_shape"] = {
        "notice": "物料是圆台：俯视=填充圆，φ30 顶面在内、φ50 底面在外",
        "mass_ratio_min": detector.color_min_mass_ratio,
        "hole_ratio_max": detector.color_max_hole_ratio,
        "contour_circularity_min": detector.color_min_contour_circularity,
    }
    diameters = [
        float(entry["equivalent_diameter"]) for entry in report["materials"]
    ]
    if diameters:
        radius = max(diameters) / 2.0
        area = float(np.pi) * radius * radius
        report["recommended_area"] = {
            "min_area": int(area * 0.35),
            "max_area": int(area * 2.5),
            "note": "把这两个数写进 scenes.<当前场景>.min_area/max_area",
        }
    print(json.dumps(jsonable(report), ensure_ascii=False), flush=True)

    if output_dir is not None:
        try:
            output_dir.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d-%H%M%S")
            frame_path = output_dir / ("diag_%s_frame.png" % stamp)
            cv2.imwrite(str(frame_path), frame)
            rows = []
            for index in range(0, len(tiles), 3):
                row = list(tiles[index:index + 3])
                while len(row) < 3:
                    row.append(np.zeros_like(tiles[0]))
                rows.append(np.hstack(row))
            montage = np.vstack(rows)
            mask_path = output_dir / ("diag_%s_masks.png" % stamp)
            cv2.imwrite(str(mask_path), montage)
            print(
                json.dumps(
                    {"state": "DIAGNOSE_SAVED", "frame": str(frame_path),
                     "masks": str(mask_path),
                     "layout": "1-6 masks, then the frame"},
                    ensure_ascii=False,
                ),
                flush=True,
            )
        except OSError as error:
            print(
                json.dumps({"state": "DIAGNOSE_SAVE_FAILED", "error": str(error)}),
                flush=True,
            )


def main():
    parser = argparse.ArgumentParser(
        description="recognise all colors in one round and report target dx/dy"
    )
    parser.add_argument("--config", default="config/jetson.json")
    parser.add_argument("--camera")
    parser.add_argument(
        "--dump-every",
        type=int,
        default=0,
        help="print the full colour diagnosis every N frames (0 = only on the d key)",
    )
    parser.add_argument(
        "--dump-dir",
        default="logs",
        help="where the diagnosis frame and mask montage are written",
    )
    parser.add_argument(
        "--camera-profile",
        help="override camera.v4l2_control_profile, e.g. fill_light_locked / dim_room_locked",
    )
    parser.add_argument(
        "--capture-profile",
        help="override camera.capture_profile, e.g. wide_4_3 to compare fields of view",
    )
    parser.add_argument(
        "--camera-backend",
        choices=("opencv_v4l2", "gstreamer_nv"),
        help="override camera.backend exactly as jetson_recognition.run live does",
    )
    parser.add_argument("--target", required=True, choices=tuple("123456"))
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--task-code")
    source.add_argument("--round-colors", nargs=3, choices=tuple("123456"))
    source.add_argument("--all-colors", action="store_true")
    parser.add_argument("--batch", type=int, choices=(1, 2), default=1)
    parser.add_argument(
        "--reference-x",
        type=float,
        help="defaults to half the configured frame width",
    )
    parser.add_argument(
        "--reference-y",
        type=float,
        help="defaults to half the configured frame height",
    )
    parser.add_argument("--print-every", type=int, default=15)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--no-batch",
        action="store_true",
        help="use the legacy per-colour masks instead of whole-blob batch labels",
    )
    parser.add_argument(
        "--no-illumination",
        action="store_true",
        help="disable the per-frame white balance to compare against the old path",
    )
    parser.add_argument(
        "--vote-frames",
        type=int,
        default=8,
        help="frames that must agree on the same round batch",
    )
    lighting = parser.add_mutually_exclusive_group()
    lighting.add_argument(
        "--fill-light",
        dest="fill_light",
        action="store_true",
        help="downward fill light present: disable the per-frame white balance",
    )
    lighting.add_argument(
        "--no-fill-light",
        dest="fill_light",
        action="store_false",
        help="no fill light (competition rule): enable the white balance",
    )
    parser.set_defaults(fill_light=None)
    arguments = parser.parse_args()

    config_path = Path(arguments.config)
    if not config_path.is_absolute():
        config_path = PROJECT_ROOT / config_path
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if arguments.fill_light is not None:
        config.setdefault("lighting", {})["fill_light"] = bool(arguments.fill_light)
    # Follow the configured frame size instead of a hard-coded 1280x720 centre,
    # so a capture-mode change does not silently move the reference point.
    apply_capture_profile(config["camera"], arguments.capture_profile)
    if arguments.reference_x is None:
        arguments.reference_x = float(config["camera"]["width"]) / 2.0
    if arguments.reference_y is None:
        arguments.reference_y = float(config["camera"]["height"]) / 2.0
    colors = parse_colors(arguments)
    if arguments.target not in colors:
        raise SystemExit("--target must be one of this round's colors")

    engine = build_engine(config)
    detector = engine.detector
    detector.set_color_debug(True)
    profile_info = apply_color_profile(config, PROJECT_ROOT, quiet=True)
    active_profile = profile_info.get("profile")
    if arguments.no_illumination:
        detector.illumination.enabled = False
        detector.illumination_enabled = False
        print('{"state":"ILLUMINATION_DISABLED","reason":"A/B comparison"}', flush=True)
    print(
        json.dumps(
            {
                "state": "LIGHTING_MODE",
                "fill_light": detector.fill_light,
                "illumination_enabled": detector.illumination_enabled,
                "color_profile": active_profile,
                "prototypes_configured": sorted(detector.color_prototypes),
                "hint": "press 1..6 to lock the material under the mouse as that colour's prototype, w to save",
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    batch_mode = not arguments.no_batch
    vote_frames = max(1, int(arguments.vote_frames))
    dump_every = max(0, int(arguments.dump_every))
    dump_dir = PROJECT_ROOT / arguments.dump_dir
    batch_history = deque(maxlen=vote_frames)
    hsv_probe = HSVProbe()
    window_name = "Turntable all-color target test"
    camera_config = dict(config["camera"])
    if arguments.camera_profile:
        # Applied after build_engine, so an explicit command line still beats
        # the camera section that the active lighting profile selects.
        camera_config["v4l2_control_profile"] = arguments.camera_profile
    if arguments.camera_backend:
        camera_config["backend"] = arguments.camera_backend
    print(
        json.dumps(
            {
                "state": "CAMERA_OPENING",
                "backend": camera_config.get("backend"),
                "device": arguments.camera or camera_config.get("device"),
                "profile": camera_config.get("v4l2_control_profile"),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    camera = UVCCamera(camera_config, PROJECT_ROOT, arguments.camera)
    controls = dict(camera.startup_controls)
    performance = RollingPerformance()
    task_text = arguments.task_code or (
        "ALL_COLORS" if arguments.all_colors else "MANUAL:" + ",".join(colors)
    )
    stability_config = config.get("stability", {})
    target_window = SimpleColorStability(
        required_frames=stability_config.get("stable_frames", 8),
        max_center_delta_px=stability_config.get("max_center_delta_px", 6.0),
        min_duration_ms=stability_config.get("min_stable_time_ms", 250),
    )
    frame_number = 0
    last_signature = None
    last_report_time = 0.0
    print_every = max(1, arguments.print_every)
    print(
        json.dumps(
            {
                "state": "TURNTABLE_COLOR_TEST_READY",
                "round_colors": colors,
                "target": arguments.target,
                "target_name": COLOR_NAMES[arguments.target],
                "task_code": arguments.task_code,
                "reference_px": [arguments.reference_x, arguments.reference_y],
                "camera_controls": controls,
                "camera_backend": camera.backend_name,
                "mode": "ROUND_BATCH" if batch_mode else "PER_COLOUR_MASK",
                "illumination_enabled": detector.illumination_enabled,
                "vote_frames": vote_frames,
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    if not arguments.headless:
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, hsv_probe.on_mouse)
    no_frame_started = time.monotonic()
    no_frame_reported = False
    try:
        while True:
            frame = camera.read()
            if frame is None:
                if not no_frame_reported:
                    print(
                        json.dumps(
                            {
                                "state": "CAMERA_EMPTY",
                                "message": "waiting for the next frame",
                            },
                            ensure_ascii=False,
                        ),
                        flush=True,
                    )
                    no_frame_reported = True
                if not arguments.headless:
                    waiting = np.zeros(
                        (int(config["camera"]["height"]),
                         int(config["camera"]["width"]), 3),
                        dtype=np.uint8,
                    )
                    cv2.putText(
                        waiting,
                        "CAMERA_NO_FRAMES - waiting for UVC stream...",
                        (40, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 0, 255), 2,
                    )
                    cv2.imshow(window_name, waiting)
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        break
                time.sleep(0.02)
                continue
            if no_frame_reported:
                print('{"state":"CAMERA_RECOVERED"}', flush=True)
            no_frame_started = time.monotonic()
            no_frame_reported = False
            frame_number += 1
            started = time.perf_counter()
            if batch_mode:
                measurements = detector.detect_materials(frame, "TURNTABLE", colors)
                entries = (detector.last_materials_debug or {}).get("materials", [])
            else:
                measurements = detector.detect_colors(frame, "TURNTABLE", colors)
                entries = []
            # The detector works on the white-balanced frame; every readout and
            # every saved diagnosis image uses the same frame.
            base = detector.last_balanced_frame
            if base is None:
                base = frame
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            performance.update(elapsed_ms)
            if frame_number % 60 == 0:
                controls = camera._read_current_controls(camera.source)
            # Three materials, three different colours: solve the whole
            # assignment instead of trusting three independent blob labels.
            assignment = None
            assignment_gap = None
            if entries:
                rows = [entry.get("scores", {}) for entry in entries]
                assignment, _total = assign_distinct(rows, colors)
                if assignment is not None:
                    assignment_gap = assignment_margin(rows, colors)
            if assignment is None:
                assignment = [item.target_id for item in measurements]
            target = None
            target_label = None
            for index, label in enumerate(assignment):
                if label == arguments.target and index < len(measurements):
                    target = measurements[index]
                    target_label = label
                    break
            stable = None
            if target is None:
                target_window.miss()
            else:
                stable = target_window.update(
                    "COLOR:" + target_label,
                    target.pixel_x,
                    target.pixel_y,
                    target.x,
                    target.y,
                    target.confidence,
                )
            batch_key = (
                tuple(sorted(assignment)) if len(assignment) == len(colors) else None
            )
            batch_history.append(batch_key)
            votes = sum(1 for item in batch_history if item == batch_key)
            agreement = votes / float(len(batch_history))
            batch_stable = (
                batch_key is not None
                and len(batch_history) >= batch_history.maxlen
                and agreement >= 1.0
            )
            signature = (
                tuple(sorted(assignment)),
                target is not None,
                stable is not None,
                batch_stable,
            )
            now = time.monotonic()
            # Flickering masks used to produce synchronous JSON output on
            # almost every frame. Keep state changes observable without
            # allowing terminal I/O to throttle the camera loop.
            state_changed = signature != last_signature
            periodic = frame_number % print_every == 0
            if periodic or (state_changed and now - last_report_time >= 0.25):
                report = {
                    "state": "TARGET_STABLE" if stable else (
                        "TARGET_VISIBLE" if target is not None else "TARGET_LOST"
                    ),
                    "visible_materials": [
                        {"id": label, "name": COLOR_NAMES[label]}
                        for label in assignment
                    ],
                    "round_batch": list(assignment),
                    "batch_agreement": round(agreement, 3),
                    "batch_stable": batch_stable,
                    "target": arguments.target,
                    "target_name": COLOR_NAMES[arguments.target],
                    "detect_ms": round(elapsed_ms, 3),
                    "fps": round(performance.fps, 2),
                    "capture_fps": round(camera.capture_fps, 2),
                    "camera_controls": controls,
                    "illumination": detector.last_illumination,
                }
                if entries:
                    report["materials"] = [
                        {
                            "blob_id": entry["color_id"],
                            "assigned": assignment[index],
                            "center_px": [
                                round(value, 1) for value in entry["center"]
                            ],
                            "confidence": round(entry["confidence"], 3),
                            "distance": round(entry["distance"], 3),
                            "margin": round(entry["margin"], 3),
                            "area": round(entry["area"], 1),
                            "core_hsv": entry["core_hsv"],
                            "mean_hsv": entry["mean_hsv"],
                            "solidity": entry["solidity"],
                            "mass_ratio": entry["mass_ratio"],
                            "fill_ratio": entry["fill_ratio"],
                            "circularity": entry["circularity"],
                        }
                        for index, entry in enumerate(entries)
                    ]
                if assignment_gap is not None:
                    report["assignment_gap"] = round(float(assignment_gap), 3)
                if target is not None:
                    report["target_center_px"] = [target.pixel_x, target.pixel_y]
                    report["target_delta_px"] = [
                        round(target.pixel_x - arguments.reference_x, 3),
                        round(target.pixel_y - arguments.reference_y, 3),
                    ]
                    report["confidence"] = round(target.confidence, 3)
                print(json.dumps(report, ensure_ascii=False), flush=True)
                last_report_time = now
            last_signature = signature

            if dump_every and frame_number % dump_every == 0:
                diagnose(
                    detector, base, colors, COLOR_NAMES,
                    PROJECT_ROOT / arguments.dump_dir,
                )

            if not arguments.headless:
                # Show the white-balanced frame: it is what the detector sees,
                # and the probe then reports the values the thresholds use.
                canvas = base.copy()
                reference = (int(arguments.reference_x), int(arguments.reference_y))
                cv2.drawMarker(canvas, reference, (255, 0, 255), cv2.MARKER_CROSS, 28, 2)
                if batch_mode:
                    for index, entry in enumerate(entries):
                        label = (
                            assignment[index]
                            if index < len(assignment) else entry["color_id"]
                        )
                        is_target = label == arguments.target
                        color = (0, 255, 0) if is_target else (0, 200, 255)
                        for contour in entry.get("contours", []):
                            cv2.drawContours(canvas, [contour], -1, color, 1)
                        x, y, width, height = entry["bbox"]
                        cv2.rectangle(canvas, (x, y), (x + width, y + height), color, 1)
                        center = (
                            int(round(entry["center"][0])),
                            int(round(entry["center"][1])),
                        )
                        cv2.drawMarker(canvas, center, color, cv2.MARKER_CROSS, 24, 2)
                        cv2.putText(
                            canvas,
                            "%s%s d=%.2f q=%.2f" % (
                                "TARGET " if is_target else "",
                                COLOR_NAMES[label],
                                entry["distance"],
                                entry["confidence"],
                            ),
                            (center[0] + 10, center[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA,
                        )
                else:
                    for debug in detector.last_colors_debug:
                        for candidate in debug.get("candidates", []):
                            x, y, width, height = candidate["bbox"]
                            color_id = str(debug.get("requested_id", "?"))
                            is_target = color_id == arguments.target
                            color = (0, 255, 0) if is_target else (0, 200, 255)
                            cv2.drawContours(canvas, [candidate["contour"]], -1, color, 1)
                            cv2.rectangle(canvas, (x, y), (x + width, y + height), color, 1)
                if target is not None:
                    center = (int(round(target.pixel_x)), int(round(target.pixel_y)))
                    cv2.arrowedLine(canvas, reference, center, (0, 255, 0), 2)
                    cv2.putText(
                        canvas,
                        "dx=%.1f dy=%.1f px" % (
                            target.pixel_x - arguments.reference_x,
                            target.pixel_y - arguments.reference_y,
                        ),
                        (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
                    )
                light = detector.last_illumination or {}
                info_lines = [
                    "TASK %s  BATCH=%d  ROUND=%s" % (
                        task_text, arguments.batch, ",".join(colors)
                    ),
                    "TARGET %s(%s)  BATCH=%s %s" % (
                        arguments.target,
                        COLOR_NAMES[arguments.target],
                        ",".join(assignment) or "NONE",
                        "STABLE" if batch_stable else "agree=%.2f" % agreement,
                    ),
                    "FPS=%.1f CAP=%.1f DET=%.2fms" % (
                        performance.fps, camera.capture_fps, performance.detect_ms
                    ),
                    "EXP=%s FOCUS=%s WB=%s AUTO_EXP=%s AUTO_FOCUS=%s AUTO_WB=%s" % (
                        controls.get("exposure_time_absolute", "?"),
                        controls.get("focus_absolute", "?"),
                        controls.get("white_balance_temperature", "?"),
                        controls.get("auto_exposure", "?"),
                        controls.get("focus_automatic_continuous", "?"),
                        controls.get("white_balance_automatic", "?"),
                    ),
                    "LIGHT %s ref=%s gain=%s n=%.3f %s" % (
                        light.get("reason", "OFF"),
                        light.get("white_ref_bgr"),
                        light.get("gain"),
                        float(light.get("neutral_ratio", 0.0)),
                        light.get("exposure_hint", ""),
                    ),
                    "PROTO %s  [1-6] lock under mouse, [w] save json" % (
                        ",".join(
                            "%s:%s" % (
                                color_id,
                                ",".join("%.0f" % value for value in values),
                            )
                            for color_id, values in sorted(
                                detector.color_prototypes.items()
                            )
                        ) or "none",
                    ),
                ]
                for index, text in enumerate(info_lines):
                    y = 75 + index * 27
                    cv2.putText(canvas, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX,
                                0.58, (0, 255, 0), 2, cv2.LINE_AA)
                canvas = hsv_probe.draw(base, canvas)
                cv2.imshow(window_name, canvas)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("d"):
                    diagnose(
                        detector, base, colors, COLOR_NAMES,
                        PROJECT_ROOT / arguments.dump_dir,
                    )
                if key == ord("w"):
                    try:
                        backup = save_prototypes(
                            config_path, detector.color_prototypes,
                            PROJECT_ROOT, active_profile,
                        )
                        print(
                            json.dumps(
                                {
                                    "state": "PROTOTYPES_SAVED",
                                    "prototypes": detector.color_prototypes,
                                    "backup": backup,
                                    "config": str(config_path),
                                },
                                ensure_ascii=False,
                            ),
                            flush=True,
                        )
                    except (OSError, ValueError, KeyError) as error:
                        print(
                            json.dumps(
                                {"state": "PROTOTYPES_SAVE_FAILED",
                                 "error": str(error)},
                                ensure_ascii=False,
                            ),
                            flush=True,
                        )
                elif 32 <= key < 127 and chr(key) in "123456":
                    color_id = chr(key)
                    index = nearest_material(entries, hsv_probe.position)
                    if index is None:
                        print(
                            json.dumps(
                                {"state": "PROTOTYPE_SKIPPED",
                                 "reason": "NO_MATERIAL_VISIBLE"},
                                ensure_ascii=False,
                            ),
                            flush=True,
                        )
                    else:
                        values = [float(value) for value in entries[index]["core_hsv"]]
                        detector.color_prototypes[color_id] = values
                        print(
                            json.dumps(
                                {
                                    "state": "PROTOTYPE_LOCKED",
                                    "color": color_id,
                                    "color_name": COLOR_NAMES[color_id],
                                    "core_hsv": [round(value, 1) for value in values],
                                    "blob_id": entries[index]["color_id"],
                                    "center_px": [
                                        round(value, 1)
                                        for value in entries[index]["center"]
                                    ],
                                    "press": "w to save",
                                },
                                ensure_ascii=False,
                            ),
                            flush=True,
                        )
    except KeyboardInterrupt:
        pass
    finally:
        camera.close()
        if not arguments.headless:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
