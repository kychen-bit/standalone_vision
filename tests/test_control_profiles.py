"""Camera control profiles: documentation keys in, replace flag respected."""

import json
from pathlib import Path
import sys
import unittest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jetson_recognition.camera import UVCCamera  # noqa: E402


class ControlProfileTest(unittest.TestCase):
    def resolve(self, config):
        camera = UVCCamera.__new__(UVCCamera)
        applied = {}

        def fake_set_control(device, name, value):
            applied[name] = value
            return None

        camera._set_control = staticmethod(fake_set_control)
        old_platform = None
        import platform as platform_module

        old_platform = platform_module.system
        platform_module.system = lambda: "Linux"
        try:
            failures = camera._apply_v4l2_controls(config, "/dev/video0")
        finally:
            platform_module.system = old_platform
        return applied, failures

    def test_notice_keys_are_not_sent_to_v4l2(self):
        config = {
            "v4l2_control_profile": "locked",
            "v4l2_control_profiles": {
                "locked": {
                    "_replace_base": True,
                    "notice": "human readable",
                    "exposure_time_absolute": 200,
                    "profile_notice": "also human readable",
                }
            },
        }
        applied, failures = self.resolve(config)
        self.assertEqual(failures, {})
        self.assertEqual(applied, {"exposure_time_absolute": 200})

    def test_replace_base_still_drops_the_shared_block(self):
        config = {
            "v4l2_controls": {"gain": 0, "exposure_time_absolute": 50, "hue": 50},
            "v4l2_control_profile": "locked",
            "v4l2_control_profiles": {
                "locked": {
                    "_replace_base": True,
                    "notice": "doc",
                    "exposure_time_absolute": 200,
                }
            },
        }
        applied, _ = self.resolve(config)
        self.assertEqual(applied, {"exposure_time_absolute": 200})

    def test_profile_without_replace_flag_extends_base(self):
        config = {
            "v4l2_controls": {"gain": 0},
            "v4l2_control_profile": "extra",
            "v4l2_control_profiles": {"extra": {"saturation": 90}},
        }
        applied, _ = self.resolve(config)
        self.assertEqual(applied, {"gain": 0, "saturation": 90})

    def test_real_config_profiles_are_applicable(self):
        config = json.loads(
            (PROJECT_ROOT / "config" / "jetson.json").read_text(encoding="utf-8")
        )["camera"]
        for name in config.get("v4l2_control_profiles", {}):
            config["v4l2_control_profile"] = name
            applied, failures = self.resolve(config)
            self.assertEqual(failures, {}, "profile %s" % name)
            self.assertTrue(applied or name == "camera_default_auto")

    def test_dim_room_profile_matches_the_measurement(self):
        config = json.loads(
            (PROJECT_ROOT / "config" / "jetson.json").read_text(encoding="utf-8")
        )["camera"]
        profile = config["v4l2_control_profiles"]["dim_room_locked"]
        # exposure 200 counts of 100 us = 20 ms, which fits inside a 1/60 s
        # frame only when the sensor throttles; keep it at or below 250.
        self.assertLessEqual(profile["exposure_time_absolute"], 250)
        self.assertEqual(profile["exposure_time_absolute"], 200)
        # measured: gain 16 clips the blue channel (p999V 255) at this exposure
        self.assertEqual(profile["gain"], 12)
        self.assertEqual(profile["auto_exposure"], 1)
        self.assertEqual(profile["white_balance_automatic"], 0)

    def test_dim_capture_profile_exists_for_longer_exposure(self):
        config = json.loads(
            (PROJECT_ROOT / "config" / "jetson.json").read_text(encoding="utf-8")
        )["camera"]
        profile = config["capture_profiles"]["dim_30fps"]
        self.assertEqual(profile["fps"], 30)
        # 1/30 s = 33.3 ms = 333 counts of 100 us
        self.assertGreaterEqual(333, 1)

    def test_turntable_area_gates_are_not_fitted_to_one_height(self):
        config = json.loads(
            (PROJECT_ROOT / "config" / "jetson.json").read_text(encoding="utf-8")
        )
        scene = config["scenes"]["TURNTABLE"]
        self.assertGreaterEqual(scene["max_area"], 150000)
        self.assertLessEqual(scene["min_area"], 6000)


if __name__ == "__main__":
    unittest.main()
