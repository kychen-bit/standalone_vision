"""Derive colors.<id> thresholds from the real material under the real light.

Nothing here is guessed: the recommended numbers come from the pixels of the
material itself plus the pixels of the background that shares its hue.

    # put ONE material of the colour you are calibrating in the ROI, light on
    python3 tools/calibrate_color_ranges.py --color 1 --fill-light \
        --camera-profile fill_light_locked
    # happy with the proposal? write it into the fill-light set
    ... --write

Why the background matters: pure "material S minus margin" is not enough. Red
failed exactly because the warm table had S~101 while the material had S~255, so
any s_min below ~101 let the whole table into the red mask. The rule therefore
raises s_min above what the same-hue background can reach.

For black (colour 5) the knobs are different: black must be dark *and*
achromatic, so v_max comes from the material's own V and s_max from its own S.

Both lighting modes are supported through the same profile mechanism as the live
tool: the default target is config/jetson.json (the fill-light set), and
--profile ambient writes into config/color_profiles.json instead.
"""

import argparse
from collections import deque
import json
from pathlib import Path
import sys

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jetson_recognition.camera import UVCCamera, apply_capture_profile  # noqa: E402
from jetson_recognition.color_profiles import apply_color_profile  # noqa: E402
from jetson_recognition.detectors import _crop  # noqa: E402


def hue_delta(hue, reference):
    difference = np.abs(float(hue) - float(reference))
    return float(min(difference, 180.0 - difference))


def circular_mean(hue):
    angle = np.deg2rad(np.asarray(hue, dtype=np.float64) * 2.0)
    return float(
        (np.rad2deg(np.arctan2(np.sin(angle).mean(), np.cos(angle).mean())) % 360.0)
        / 2.0
    )


def material_pixels(hsv, mask, core_ratio, inner_ratio):
    """Pixels of the biggest blob, sampled like the classifier samples them.

    Returns (hue, saturation, value arrays, stats) for the annulus core of the
    largest blob, or None when the mask holds nothing usable.
    """
    contours, _hierarchy = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    hull = cv2.convexHull(contour)
    hull_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    cv2.drawContours(hull_mask, [hull], -1, 255, -1)
    # Hull AND mask: the hull spans the glare hole, but those pixels are not
    # material pixels - including them would hand black a v_max of 200.
    inside = (hull_mask > 0) & (mask > 0)
    count = int(np.count_nonzero(inside))
    if count < 64:
        return None
    rows, columns = np.nonzero(inside)
    center_x, center_y = float(columns.mean()), float(rows.mean())
    core_radius = max(2.0, float(np.sqrt(count / np.pi)) * core_ratio)
    distance_squared = (columns - center_x) ** 2 + (rows - center_y) ** 2
    core = distance_squared <= core_radius * core_radius
    if inner_ratio > 0.0:
        inner = core_radius * inner_ratio
        core &= distance_squared >= inner * inner
    if int(np.count_nonzero(core)) < 32:
        core = np.ones(count, dtype=bool)
    hue = hsv[..., 0][inside][core].astype(np.float32)
    saturation = hsv[..., 1][inside][core].astype(np.float32)
    value = hsv[..., 2][inside][core].astype(np.float32)
    return hue, saturation, value, {
        "hull_px": count,
        "core_px": int(np.count_nonzero(core)),
        "equivalent_diameter": round(
            2.0 * float(np.sqrt(count / np.pi)), 1
        ),
        "inside": inside,
    }


def background_saturation(hsv, inside, center_hue, hue_band):
    """Saturation percentiles of background pixels that share the hue.

    This is what makes s_min safe: the material's saturation alone says nothing
    about whether the table also satisfies the range.
    """
    pixels_hue = hsv[..., 0].astype(np.float32)
    delta = np.abs(pixels_hue - float(center_hue))
    delta = np.minimum(delta, 180.0 - delta)
    same_hue = (delta <= hue_band) & (~inside)
    total = int(np.count_nonzero(same_hue))
    if total < 256:
        return None, total
    saturation = hsv[..., 1][same_hue].astype(np.float32)
    return (
        {
            "p50": float(np.percentile(saturation, 50.0)),
            "p98": float(np.percentile(saturation, 98.0)),
            "px": total,
        },
        total,
    )


def recommend_ranges(samples):
    hue, saturation, value, stats, saturation_scale, value_scale, background = samples
    center_hue = circular_mean(hue)
    deviations = np.asarray([hue_delta(item, center_hue) for item in hue])
    half_width = max(
        float(np.percentile(deviations, 98.0)) + 4.0,
        int(stats.get("min_half_width", 6)),
    )
    proposed_s_min = max(
        40.0,
        round(float(np.percentile(saturation, 2.0)) - saturation_scale, 1),
        round((background or {}).get("p98", 0.0) + 10.0, 1),
    )
    proposed_v_min = max(25.0, round(float(np.percentile(value, 2.0)) - value_scale, 1))
    low = center_hue - half_width
    high = center_hue + half_width
    ranges = []
    if low < 0.0:
        ranges.append({
            "h_min": 0, "h_max": int(round(high)),
            "s_min": int(round(proposed_s_min)), "s_max": 255,
            "v_min": int(round(proposed_v_min)), "v_max": 255,
        })
        ranges.append({
            "h_min": int(round(180.0 + low)), "h_max": 179,
            "s_min": int(round(proposed_s_min)), "s_max": 255,
            "v_min": int(round(proposed_v_min)), "v_max": 255,
        })
    elif high > 179.0:
        ranges.append({
            "h_min": int(round(low)) % 180, "h_max": 179,
            "s_min": int(round(proposed_s_min)), "s_max": 255,
            "v_min": int(round(proposed_v_min)), "v_max": 255,
        })
        ranges.append({
            "h_min": 0, "h_max": int(round(high)) - 180,
            "s_min": int(round(proposed_s_min)), "s_max": 255,
            "v_min": int(round(proposed_v_min)), "v_max": 255,
        })
    else:
        ranges.append({
            "h_min": int(round(low)), "h_max": int(round(high)),
            "s_min": int(round(proposed_s_min)), "s_max": 255,
            "v_min": int(round(proposed_v_min)), "v_max": 255,
        })
    measured = {
        "center_hue": round(center_hue, 1),
        "half_width": round(half_width, 1),
        "h_percentiles": [
            round(float(np.percentile(hue, 2.0)), 1),
            round(float(np.percentile(hue, 50.0)), 1),
            round(float(np.percentile(hue, 98.0)), 1),
        ],
        "s_percentiles": [
            round(float(np.percentile(saturation, 2.0)), 1),
            round(float(np.percentile(saturation, 50.0)), 1),
            round(float(np.percentile(saturation, 98.0)), 1),
        ],
        "v_percentiles": [
            round(float(np.percentile(value, 2.0)), 1),
            round(float(np.percentile(value, 50.0)), 1),
            round(float(np.percentile(value, 98.0)), 1),
        ],
        "background_same_hue": background,
        "hull_px": stats["hull_px"],
        "core_px": stats["core_px"],
        "equivalent_diameter": stats["equivalent_diameter"],
    }
    return ranges, measured


def recommend_black(samples):
    _hue, saturation, value, stats, _saturation_scale, _value_scale, _background = samples
    v_max = int(round(min(float(np.percentile(value, 95.0)) + 10.0, 200.0)))
    s_max = int(round(max(float(np.percentile(saturation, 95.0)) + 20.0, 60.0)))
    s_max = int(min(s_max, 180))
    measured = {
        "v_percentiles": [
            round(float(np.percentile(value, 2.0)), 1),
            round(float(np.percentile(value, 50.0)), 1),
            round(float(np.percentile(value, 98.0)), 1),
        ],
        "s_percentiles": [
            round(float(np.percentile(saturation, 2.0)), 1),
            round(float(np.percentile(saturation, 50.0)), 1),
            round(float(np.percentile(saturation, 98.0)), 1),
        ],
        "hull_px": stats["hull_px"],
        "core_px": stats["core_px"],
        "equivalent_diameter": stats["equivalent_diameter"],
    }
    return {"v_max": v_max, "s_min": 0, "s_max": s_max}, measured


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/jetson.json")
    parser.add_argument("--color", required=True, help="1..6")
    parser.add_argument("--scene", default="TURNTABLE")
    parser.add_argument("--frames", type=int, default=40,
                        help="frames to average (the best one is used per aspect)")
    parser.add_argument("--fill-light", dest="fill_light", action="store_true",
                        default=None)
    parser.add_argument("--no-fill-light", dest="fill_light", action="store_false")
    parser.add_argument("--camera-profile", default=None)
    parser.add_argument("--capture-profile", default=None)
    parser.add_argument("--profile", choices=("fill", "ambient"), default=None,
                        help="write target: fill = config/jetson.json, ambient = "
                             "config/color_profiles.json (defaults to the active mode)")
    parser.add_argument("--saturation-margin", type=float, default=25.0)
    parser.add_argument("--value-margin", type=float, default=25.0)
    parser.add_argument("--hue-band", type=float, default=15.0,
                        help="hue window used to sample the background")
    parser.add_argument("--min-hue-half-width", type=float, default=6.0)
    parser.add_argument("--write", action="store_true",
                        help="write the proposal into the config (with a .bak)")
    arguments = parser.parse_args()

    config_path = Path(arguments.config)
    if not config_path.is_absolute():
        config_path = PROJECT_ROOT / config_path
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if arguments.fill_light is not None:
        config.setdefault("lighting", {})["fill_light"] = bool(arguments.fill_light)
    if arguments.camera_profile:
        config["camera"]["v4l2_control_profile"] = arguments.camera_profile
    apply_capture_profile(config["camera"], arguments.capture_profile)
    apply_color_profile(config, PROJECT_ROOT, quiet=True)

    color_id = str(arguments.color)
    if color_id not in config["colors"]:
        raise SystemExit("unknown color %s" % color_id)
    detector_config = config["color_detection"]
    label = detector_config["label_classifier"]

    from jetson_recognition.detectors import TopViewDetector

    detector = TopViewDetector(config, PROJECT_ROOT)
    scene = config["scenes"][arguments.scene]
    roi = scene["object_roi"]

    camera_config = config["camera"]
    camera = UVCCamera(camera_config, PROJECT_ROOT)
    try:
        for _ in range(12):
            camera.read()
        cores = deque(maxlen=arguments.frames)
        for _ in range(arguments.frames):
            frame = camera.read()
            if frame is None:
                continue
            work = detector._work_frame(frame)
            crop, _offset_x, _offset_y = _crop(work, roi)
            hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            mask = detector._clean_mask(detector._make_color_mask(hsv, color_id))
            sample = material_pixels(
                hsv, mask,
                float(label.get("prototype_core_ratio", 0.6)),
                float(label.get("prototype_core_inner_ratio", 0.3)),
            )
            if sample is None:
                continue
            hue, saturation, value, stats = sample
            background, _total = background_saturation(
                hsv, stats.pop("inside"), circular_mean(hue), arguments.hue_band
            )
            stats["min_half_width"] = arguments.min_hue_half_width
            cores.append(
                (
                    hue, saturation, value, stats,
                    arguments.saturation_margin, arguments.value_margin, background,
                )
            )
    finally:
        camera.close()

    report = {
        "state": "CALIBRATE",
        "color": color_id,
        "color_name": config["colors"][color_id].get("name"),
        "scene": arguments.scene,
        "fill_light": config.get("lighting", {}).get("fill_light"),
        "camera_profile": camera_config.get("v4l2_control_profile"),
        "frames_used": len(cores),
        "frames_requested": arguments.frames,
    }
    if not cores:
        report["error"] = "NO_MATERIAL_PIXELS"
        report["hint"] = (
            "把该颜色的物料放进 ROI（去掉其它颜色的干扰），再确认当前阈值至少能框住它；"
            "若掩码为空，先手动把 colors.%s 的 v_min 暂时降一点再采一次" % color_id
        )
        print(json.dumps(report, ensure_ascii=False), flush=True)
        return 1

    # Average the per-frame measurements instead of picking one frame, so a
    # single noisy capture cannot move the recommendation.
    def collect(index):
        values = []
        for entry in cores:
            values.append(entry[index])
        return values

    median_stats = {
        key: float(np.median([entry[3][key] for entry in cores]))
        for key in ("hull_px", "core_px", "equivalent_diameter")
    }
    median_stats["min_half_width"] = arguments.min_hue_half_width
    backgrounds = [entry[6] for entry in cores if entry[6]]
    background = None
    if backgrounds:
        background = {
            "p50": round(float(np.median([item["p50"] for item in backgrounds])), 1),
            "p98": round(float(np.median([item["p98"] for item in backgrounds])), 1),
            "px": int(np.median([item["px"] for item in backgrounds])),
        }
        background["hue_band"] = arguments.hue_band
    pooled = (
        np.concatenate(collect(0)), np.concatenate(collect(1)),
        np.concatenate(collect(2)), median_stats,
        arguments.saturation_margin, arguments.value_margin, background,
    )
    if color_id == "5":
        proposal, measured = recommend_black(pooled)
    else:
        proposal, measured = recommend_ranges(pooled)

    report["measured"] = measured
    report["proposal"] = proposal
    report["current"] = {
        key: config["colors"][color_id][key]
        for key in ("hsv_ranges", "black_threshold")
        if key in config["colors"][color_id]
    }
    report["area_band"] = {
        "min_area": int(round(median_stats["hull_px"] * 0.35)),
        "max_area": int(round(median_stats["hull_px"] * 2.5)),
        "measured_hull_px": int(median_stats["hull_px"]),
    }

    if arguments.write:
        payload = {color_id: proposal if color_id == "5" else {"hsv_ranges": proposal}}
        if color_id == "5":
            payload = {color_id: {"black_threshold": proposal}}
        target = arguments.profile or "ambient" if config["lighting"].get(
            "fill_light"
        ) is False else (arguments.profile or "fill")
        if target == "ambient":
            from jetson_recognition.color_profiles import save_override

            report["backup"] = save_override(
                PROJECT_ROOT, "ambient", "colors", payload
            )
            report["written_to"] = "config/color_profiles.json (ambient)"
        else:
            from tools.capture_camera_profile import save_config

            stored = json.loads(config_path.read_text(encoding="utf-8"))
            block = stored.setdefault("colors", {}).setdefault(color_id, {})
            block.update(payload[color_id])
            report["backup"] = save_config(config_path, stored)
            report["written_to"] = str(config_path)
    print(json.dumps(report, ensure_ascii=False), flush=True)
    if not arguments.write:
        print(
            "[calibrate] 这是【只测不写】。确认诊断掩码贴合物料后再加 --write 落盘。",
            file=sys.stderr,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
