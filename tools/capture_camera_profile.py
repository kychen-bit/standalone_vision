"""Let UVC auto controls settle, then save their current values as a lock profile."""

import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import cv2

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
    actual = read_controls(device, VERIFY_CONTROL_NAMES)
    expected = {
        name: int(value)
        for name, value in profile.items()
        if name in VERIFY_CONTROL_NAMES
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
    root.add_argument("--camera", help="camera index or /dev/v4l/by-id path")
    root.add_argument("--profile", default="paper_locked")
    root.add_argument("--settle-seconds", type=float, default=8.0)
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


def main():
    arguments = parser().parse_args()
    if arguments.settle_seconds < 1.0:
        raise SystemExit("--settle-seconds must be at least 1")
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
    camera_config = dict(config["camera"])
    camera_config["v4l2_control_profile"] = "camera_default_auto"
    source = arguments.camera or camera_config.get("device")
    if not isinstance(source, str) or not source.startswith("/dev/"):
        raise SystemExit("a Linux /dev camera path is required")

    camera = UVCCamera(camera_config, PROJECT_ROOT, arguments.camera)
    started = time.monotonic()
    last_frame = None
    result = None
    try:
        while time.monotonic() - started < arguments.settle_seconds:
            frame = camera.read()
            if frame is None:
                continue
            last_frame = frame
            if not arguments.headless:
                canvas = frame.copy()
                remaining = max(0.0, arguments.settle_seconds - (time.monotonic() - started))
                cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 42), (0, 0, 0), -1)
                cv2.putText(
                    canvas,
                    "AUTO settling %.1fs - keep final view fixed; q=cancel" % remaining,
                    (10, 29),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("capture locked camera profile", canvas)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    return 2
        if last_frame is None:
            raise RuntimeError("camera returned no frames while settling")
        values = read_controls(source)
        if (
            arguments.white_balance is None
            and values["white_balance_temperature"] in (2800, 6500)
        ):
            raise RuntimeError(
                "automatic white balance reached the camera limit (%d); "
                "use a neutral gray reference or provide --white-balance"
                % values["white_balance_temperature"]
            )
        overrides = {
            "white_balance_temperature": arguments.white_balance,
            "exposure_time_absolute": arguments.exposure,
            "focus_absolute": arguments.focus,
            "gain": arguments.gain,
        }
        for name, value in overrides.items():
            if value is not None:
                values[name] = value
        profile = locked_profile(values)
        apply_profile(source, profile)
        verified = verify_profile(source, profile)
        result = {
            "state": "CAMERA_PROFILE_CAPTURED",
            "profile": arguments.profile,
            "source_profile": "camera_default_auto",
            "controls": profile,
            "verified_controls": verified,
            "saved": not arguments.no_save,
            "activated": bool(arguments.activate and not arguments.no_save),
        }
        if not arguments.no_save:
            camera_section = config.setdefault("camera", {})
            profiles = camera_section.setdefault("v4l2_control_profiles", {})
            profiles[arguments.profile] = profile
            if arguments.activate:
                camera_section["v4l2_control_profile"] = arguments.profile
            config_path.write_text(
                json.dumps(config, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        # Print and persist the verified result before native GUI/camera
        # cleanup, so an unexpected backend shutdown cannot leave an
        # apparently successful but unsaved calibration.
        print(json.dumps(result, ensure_ascii=False, sort_keys=True), flush=True)
    finally:
        cv2.destroyAllWindows()
        camera.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
