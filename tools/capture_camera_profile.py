"""Let UVC auto controls settle, then save their current values as a lock profile."""

import argparse
from collections import deque
from datetime import datetime
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import cv2
import numpy as np

from jetson_recognition.camera import UVCCamera
from jetson_recognition.run import load_config


CONTROL_NAMES = (
    "white_balance_temperature",
    "exposure_time_absolute",
    "focus_absolute",
    "saturation",
    "gain",
    "power_line_frequency",
)
VERIFY_CONTROL_NAMES = (
    "white_balance_automatic",
    "white_balance_temperature",
    "auto_exposure",
    "exposure_time_absolute",
    "gain",
    "focus_automatic_continuous",
    "focus_absolute",
    "saturation",
    "power_line_frequency",
)
AUTO_CONTROLS = {
    "white_balance_automatic": 1,
    "auto_exposure": 3,
    "focus_automatic_continuous": 1,
}
EXTRA_CONTROLS = (
    "brightness", "contrast", "hue", "gamma", "sharpness",
    "backlight_compensation", "zoom_absolute",
)
ALL_CONTROLS = VERIFY_CONTROL_NAMES + EXTRA_CONTROLS


def automatic_config(config):
    """Always enable all three algorithms, even if the saved auto preset changed."""
    result = dict(config)
    profiles = dict(config.get("v4l2_control_profiles", {}))
    profile = dict(profiles.get("camera_default_auto", {}))
    for name in ("white_balance_temperature", "exposure_time_absolute", "focus_absolute"):
        profile.pop(name, None)
    profile.update(AUTO_CONTROLS)
    profile["_replace_base"] = True
    profile.setdefault("saturation", 60)
    profiles["camera_default_auto"] = profile
    result.update(v4l2_control_profiles=profiles,
                  v4l2_control_profile="camera_default_auto",
                  v4l2_controls_strict=True, threaded_capture=True)
    return result


def frame_features(frame):
    small = cv2.resize(frame, (160, 120), interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    # Spatial tiles catch lighting/composition changes hidden by a global mean.
    tiles = small.reshape(4, 30, 4, 40, 3).mean(axis=(1, 3))
    return {"tiles": tiles, "sharpness": float(cv2.Laplacian(gray, cv2.CV_64F).var())}


def feature_difference(features):
    tiles = np.asarray([f["tiles"] for f in features])
    sharpness = np.asarray([f["sharpness"] for f in features])
    return {
        "image_span": float(np.ptp(tiles, axis=0).max()),
        "sharpness_span": float(np.ptp(sharpness) / max(10.0, float(np.mean(sharpness)))),
    }


class ConvergenceWindow:
    """Require a full, regularly sampled window; stability is not optimality."""

    def __init__(self, seconds, image_tolerance=8.0, sharpness_tolerance=0.2):
        self.seconds = seconds
        self.image_tolerance = image_tolerance
        self.sharpness_tolerance = sharpness_tolerance
        self.samples = deque()

    def add(self, now, controls, features):
        if self.samples and now - self.samples[-1][0] > 1.5:
            self.samples.clear()
        self.samples.append((now, dict(controls), features))
        while len(self.samples) > 1 and self.samples[1][0] <= now - self.seconds:
            self.samples.popleft()
        differences = feature_difference([sample[2] for sample in self.samples])
        reasons = []
        if len(self.samples) < 4 or now - self.samples[0][0] < self.seconds:
            reasons.append("window_incomplete")
        for name in CONTROL_NAMES:
            values = [sample[1][name] for sample in self.samples]
            tolerance = {"white_balance_temperature": 100, "focus_absolute": 5,
                         "gain": 2}.get(name, 0)
            if name == "exposure_time_absolute":
                tolerance = max(2, 0.05 * float(np.median(values)))
            if max(values) - min(values) > tolerance:
                reasons.append(name)
        if differences["image_span"] > self.image_tolerance:
            reasons.append("image_changing")
        if differences["sharpness_span"] > self.sharpness_tolerance:
            reasons.append("focus_or_texture_changing")
        return not reasons, dict(differences, reasons=reasons)

    def reference(self):
        return {"tiles": np.mean([s[2]["tiles"] for s in self.samples], axis=0),
                "sharpness": float(np.mean([s[2]["sharpness"] for s in self.samples]))}


def restore_controls(device, original):
    # Set manual dependents while automatic algorithms are disabled, then
    # restore the original modes last (which may themselves be automatic).
    manual = dict(original)
    manual.update(white_balance_automatic=0, auto_exposure=1,
                  focus_automatic_continuous=0)
    apply_profile(device, manual)
    for name in AUTO_CONTROLS:
        run_v4l2(device, "--set-ctrl=%s=%s" % (name, original[name]))


def save_config(path, config):
    raw = path.read_text(encoding="utf-8")
    backup = path.with_name(path.name + "." + datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".bak")
    backup.write_text(raw, encoding="utf-8")
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)
    return str(backup)


def parse_controls(output):
    values = {}
    for line in output.splitlines():
        match = re.match(r"\s*([a-z0-9_]+)\s*:\s*(-?\d+)(?:\s+.*)?$", line)
        if match:
            values[match.group(1)] = int(match.group(2))
    return values


def run_v4l2(device, option):
    command = [
        "v4l2-ctl",
        "-d",
        device,
        option,
    ]
    try:
        completed = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            timeout=5.0,
        )
    except OSError as error:
        raise RuntimeError("cannot execute v4l2-ctl: %s" % error)
    if completed.returncode != 0:
        raise RuntimeError(
            "v4l2-ctl failed: %s"
            % (completed.stderr or completed.stdout).strip()
        )
    return completed.stdout


def read_controls(device, names=CONTROL_NAMES):
    output = run_v4l2(device, "--get-ctrl=" + ",".join(names))
    values = parse_controls(output)
    missing = [name for name in names if name not in values]
    if missing:
        raise RuntimeError("camera did not report controls: %s" % ",".join(missing))
    return values


def apply_profile(device, profile):
    automatic = (
        "white_balance_automatic",
        "auto_exposure",
        "focus_automatic_continuous",
    )
    names = list(automatic)
    names.extend(
        name for name in profile if name not in automatic and not name.startswith("_")
    )
    for name in names:
        run_v4l2(device, "--set-ctrl=%s=%s" % (name, profile[name]))


def verify_profile(device, profile):
    names = tuple(name for name in profile if not name.startswith("_"))
    actual = read_controls(device, names)
    expected = {
        name: int(value)
        for name, value in profile.items()
        if name in names
    }
    mismatches = {
        name: {"expected": value, "actual": actual.get(name)}
        for name, value in expected.items()
        if actual.get(name) != value
    }
    if mismatches:
        raise RuntimeError(
            "locked camera controls did not verify: %s"
            % json.dumps(mismatches, ensure_ascii=False, sort_keys=True)
        )
    return actual


def locked_profile(values):
    return {
        "_replace_base": True,
        "white_balance_automatic": 0,
        "auto_exposure": 1,
        "focus_automatic_continuous": 0,
        "white_balance_temperature": values["white_balance_temperature"],
        "exposure_time_absolute": values["exposure_time_absolute"],
        "focus_absolute": values["focus_absolute"],
        "saturation": values["saturation"],
        "gain": values["gain"],
        "power_line_frequency": values["power_line_frequency"],
    }


def parser():
    root = argparse.ArgumentParser(
        description="capture settled automatic UVC values into a locked profile"
    )
    root.add_argument("--config", default="config/jetson.json")
    root.add_argument("--camera", help="Linux /dev/video* or /dev/v4l/by-id path")
    root.add_argument("--profile", default="paper_locked")
    root.add_argument("--settle-seconds", type=float, default=10.0,
                      help="minimum automatic observation time")
    root.add_argument("--stable-seconds", type=float, default=3.0,
                      help="required continuous stable window (seconds)")
    root.add_argument("--timeout-seconds", type=float, default=45.0,
                      help="maximum automatic observation time; failure does not save")
    root.add_argument("--image-tolerance", type=float, default=8.0,
                      help="maximum BGR tile mean range (0..255 scale)")
    root.add_argument("--sharpness-tolerance", type=float, default=0.2,
                      help="maximum relative sharpness range")
    root.add_argument(
        "--white-balance",
        type=int,
        help="manual temperature override (2800..6500) when auto WB is unreliable",
    )
    root.add_argument(
        "--exposure",
        type=int,
        help="manual exposure_time_absolute override (1..10000)",
    )
    root.add_argument(
        "--focus", type=int, help="manual focus_absolute override (0..1023)"
    )
    root.add_argument("--gain", type=int, help="manual gain override (0..255)")
    root.add_argument("--headless", action="store_true")
    root.add_argument(
        "--activate",
        action="store_true",
        help="also make the new locked profile the active camera profile",
    )
    root.add_argument("--no-save", action="store_true")
    return root


def wait_for_stability(camera, device, arguments, automatic, expected=None):
    window = ConvergenceWindow(arguments.stable_seconds, arguments.image_tolerance,
                               arguments.sharpness_tolerance)
    started = time.monotonic()
    next_sample = started
    # Allow the same bounded observation budget after manual overrides too.
    deadline = started + arguments.timeout_seconds
    minimum = arguments.settle_seconds if automatic else arguments.stable_seconds + 1.0
    report = {"reasons": ["no_frames"]}
    last_controls = None
    while time.monotonic() < deadline:
        frame = camera.read()
        now = time.monotonic()
        if frame is None:
            continue
        if not arguments.headless:
            canvas = frame.copy()
            cv2.putText(canvas, "%s %.1fs - hold view fixed; q=cancel" %
                        ("AUTO" if automatic else "LOCK VERIFY", now - started),
                        (10, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            cv2.imshow("capture locked camera profile", canvas)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                raise KeyboardInterrupt
        if now < next_sample:
            continue
        last_controls = read_controls(device, ALL_CONTROLS)
        if automatic:
            if any(last_controls[k] != v for k, v in AUTO_CONTROLS.items()):
                raise RuntimeError("automatic controls changed or failed to enable: %s" % last_controls)
        elif any(last_controls[k] != v for k, v in expected.items() if not k.startswith("_")):
            raise RuntimeError("locked controls changed during image verification")
        # Ignore transient queued frames and lens movement just after locking.
        if not automatic and now - started < 1.0:
            next_sample = now + 0.5
            continue
        stable, report = window.add(now, last_controls, frame_features(frame))
        print(json.dumps({"state": "CAMERA_AUTO_OBSERVING" if automatic else "CAMERA_LOCK_OBSERVING",
                          "elapsed_s": round(now - started, 2), "stable": stable,
                          "controls": last_controls, "stability": report}), flush=True)
        if stable and now - started >= minimum:
            return last_controls, window.reference(), report
        next_sample = time.monotonic() + 0.5
    raise RuntimeError("camera did not stabilize before timeout: %s" % json.dumps(report))


def main():
    arguments = parser().parse_args()
    for name in ('settle_seconds', 'stable_seconds', 'timeout_seconds',
                 'image_tolerance', 'sharpness_tolerance'):
        value = getattr(arguments, name)
        if not math.isfinite(value) or value <= 0:
            raise SystemExit('--%s must be finite and positive' % name.replace('_', '-'))
    if arguments.settle_seconds < 1 or arguments.stable_seconds < 2:
        raise SystemExit('--settle-seconds >= 1 and --stable-seconds >= 2 are required')
    if arguments.timeout_seconds <= max(arguments.settle_seconds, arguments.stable_seconds + 1):
        raise SystemExit('--timeout-seconds must exceed settle time and stable window + 1')
    if arguments.profile == 'camera_default_auto':
        raise SystemExit('camera_default_auto is reserved; choose a locked profile name')
    limits = {
        "--white-balance": (arguments.white_balance, 2800, 6500),
        "--exposure": (arguments.exposure, 1, 10000),
        "--focus": (arguments.focus, 0, 1023),
        "--gain": (arguments.gain, 0, 255),
    }
    for label, (value, minimum, maximum) in limits.items():
        if value is not None and not minimum <= value <= maximum:
            raise SystemExit("%s must be in [%d,%d]" % (label, minimum, maximum))
    config_path = Path(arguments.config)
    if not config_path.is_absolute():
        config_path = PROJECT_ROOT / config_path
    config = load_config(str(config_path))
    camera_config = automatic_config(config["camera"])
    source = arguments.camera or camera_config.get("device")
    if not isinstance(source, str) or not source.startswith("/dev/"):
        raise SystemExit("a Linux /dev camera path is required")

    original = read_controls(source, ALL_CONTROLS)
    camera = None
    succeeded = False
    try:
        camera = UVCCamera(camera_config, PROJECT_ROOT, arguments.camera)
        # Some backends may reset controls while opening/negotiating the stream.
        # Reassert the auto preset after open and verify it throughout observation.
        apply_profile(source, camera_config["v4l2_control_profiles"]["camera_default_auto"])
        values, reference, stability = wait_for_stability(camera, source, arguments, True)
        auto_values = dict(values)
        if (arguments.white_balance is None
                and values["white_balance_temperature"] in (2800, 6500)):
            raise RuntimeError("automatic white balance reached the camera limit; "
                               "use a neutral gray reference or --white-balance")
        overrides = {
            "white_balance_temperature": arguments.white_balance,
            "exposure_time_absolute": arguments.exposure,
            "focus_absolute": arguments.focus,
            "gain": arguments.gain,
        }
        overrides = {k: v for k, v in overrides.items() if v is not None}
        values.update(overrides)
        profile = locked_profile(values)
        profile.update({name: values[name] for name in EXTRA_CONTROLS})
        apply_profile(source, profile)
        verify_profile(source, profile)
        verified, locked_reference, locked_stability = wait_for_stability(
            camera, source, arguments, False, profile)
        difference = feature_difference([reference, locked_reference])
        # Explicit manual overrides intentionally change the image; still require
        # stable post-lock frames and exact control readback in that case.
        if not overrides and (difference["image_span"] > arguments.image_tolerance
                              or difference["sharpness_span"] > arguments.sharpness_tolerance):
            raise RuntimeError("image changed after locking; auto readback may be stale: %s" %
                               json.dumps(difference))
        result = {
            "state": "CAMERA_PROFILE_CAPTURED",
            "profile": arguments.profile,
            "source_profile": "camera_default_auto",
            "automatic_controls": auto_values,
            "controls": profile,
            "verified_controls": verified,
            "stability": stability,
            "locked_stability": locked_stability,
            "lock_image_difference": difference,
            "lock_image_comparison": "skipped_manual_overrides" if overrides else "passed",
            "saved": not arguments.no_save,
            "activated": bool(arguments.activate and not arguments.no_save),
        }
        if not arguments.no_save:
            camera_section = config["camera"]
            profiles = camera_section.setdefault("v4l2_control_profiles", {})
            profiles[arguments.profile] = profile
            if arguments.activate:
                camera_section["v4l2_control_profile"] = arguments.profile
            result["backup"] = save_config(config_path, config)
        succeeded = True
        print(json.dumps(result, ensure_ascii=False, sort_keys=True), flush=True)
    finally:
        try:
            if not succeeded:
                try:
                    restore_controls(source, original)
                    print(json.dumps({"state": "CAMERA_CONTROLS_RESTORED"}), flush=True)
                except Exception as error:
                    print(json.dumps({"state": "CAMERA_RESTORE_FAILED", "error": str(error)}), flush=True)
        finally:
            if camera is not None:
                camera.close()
            if not arguments.headless:
                cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(2)
    except (RuntimeError, OSError, subprocess.TimeoutExpired) as error:
        print(json.dumps({"state": "CAMERA_PROFILE_FAILED", "error": str(error)}), flush=True)
        raise SystemExit(1)
