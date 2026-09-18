"""Sweep exposure/gain/fps and report what the camera actually accepts.

The number you may *set* and the number the sensor can *use* are different: a
frame that lasts 1/fps seconds cannot integrate longer than that, and the UVC
driver silently clamps (or refuses) anything above it. This tool prints both so
the ceiling for a given fps is visible instead of assumed.

    python3 tools/sweep_exposure.py                 # exposure sweep at the config fps
    python3 tools/sweep_exposure.py --fps 15        # buy exposure time by slowing down
    python3 tools/sweep_exposure.py --gain-sweep 400
"""

import argparse
import json
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jetson_recognition.camera import UVCCamera, apply_capture_profile
from jetson_recognition.illumination import WhiteBalanceNormalizer


def read_back(device, name):
    completed = subprocess.run(
        ["v4l2-ctl", "-d", device, "--get-ctrl=%s" % name],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True,
    )
    text = (completed.stdout or completed.stderr).strip()
    return text.split(":")[-1].strip() if ":" in text else text


def set_control(device, name, value):
    error = UVCCamera._set_control(device, name, value)
    return error, read_back(device, name)


def measure(camera, normalizer, settle=14, samples=6):
    # A fresh estimate per setting: holding the previous white reference would
    # report the previous exposure's numbers back at us.
    normalizer.reset()
    frame = None
    for _ in range(settle):
        frame = camera.read()
    for _ in range(samples):
        current = camera.read()
        if current is None:
            break
        frame = current
        balanced, info = normalizer.apply(frame)
    if frame is None:
        return None
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    values = hsv[:, :, 2].astype(np.float32)
    return {
        "info": info,
        "p50v": float(np.percentile(values, 50.0)),
        "p99v": float(np.percentile(values, 99.0)),
        "p999v": float(np.percentile(values, 99.9)),
        "meanv": float(values.mean()),
        "sat": float((hsv[:, :, 1] > 60).mean()) * 100.0,
        "fps": float(camera.capture_fps),
    }


def report(label, result):
    if result is None:
        print("%-22s NO_FRAME" % label)
        return
    info = result["info"]
    white_ref = info["white_ref_bgr"]
    white_text = (
        "[%s]" % ",".join("%.0f" % v for v in white_ref) if white_ref else "none"
    )
    print(
        "%-22s ref=%-14s wb=%-6.3f %-16s neutral=%-6.3f p99V=%-5.0f p999V=%-5.0f "
        "sat=%-5.1f meanV=%-6.1f fps=%.1f"
        % (
            label, white_text, float(np.mean(info["gain"])), info["exposure_hint"],
            info["neutral_ratio"], result["p99v"], result["p999v"], result["sat"],
            result["meanv"], result["fps"],
        )
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fps", type=float, default=None,
                        help="override camera fps before sweeping")
    parser.add_argument("--gain", type=int, default=0, help="fixed gain for the sweep")
    parser.add_argument("--exposures", type=int, nargs="+",
                        default=[100, 200, 300, 400, 666, 1000, 2000])
    parser.add_argument("--gain-sweep", type=int, default=None,
                        help="fix this exposure and sweep gain instead")
    parser.add_argument("--gains", type=int, nargs="+",
                        default=[0, 8, 16, 32, 64, 128])
    arguments = parser.parse_args()

    config = json.loads((PROJECT_ROOT / "config" / "jetson.json").read_text("utf-8"))
    apply_capture_profile(config["camera"], None)
    if arguments.fps:
        config["camera"]["fps"] = arguments.fps
    device = UVCCamera._v4l2_device(config["camera"].get("device"))
    normalizer = WhiteBalanceNormalizer(
        config.get("color_detection", {}).get("illumination")
    )
    camera = UVCCamera(config["camera"], PROJECT_ROOT)
    try:
        print("device=%s %s %s" % (device, camera.actual_settings,
                                    getattr(camera, "backend_name", "")))
        error, actual = set_control(device, "auto_exposure", 1)
        print("auto_exposure(manual) -> %s %s" % (actual, error or "ok"))
        set_control(device, "gain", arguments.gain)
        print("unit: exposure_time_absolute counts 100 us, so value*100us must stay "
              "below the frame time 1/fps\n")
        if arguments.gain_sweep is not None:
            set_control(device, "exposure_time_absolute", arguments.gain_sweep)
            print("=== gain sweep at exposure=%d (actual %s) ===" % (
                arguments.gain_sweep, read_back(device, "exposure_time_absolute")))
            for gain in arguments.gains:
                error, actual = set_control(device, "gain", gain)
                report("gain=%d (-> %s)" % (gain, actual),
                       measure(camera, normalizer))
                if error:
                    print("    set error: %s" % error)
        else:
            print("=== exposure sweep at gain=%d ===" % arguments.gain)
            for exposure in arguments.exposures:
                error, actual = set_control(device, "exposure_time_absolute", exposure)
                report("expo=%d (-> %s)" % (exposure, actual),
                       measure(camera, normalizer))
                if error:
                    print("    set error: %s" % error)
    finally:
        camera.close()


if __name__ == "__main__":
    main()
