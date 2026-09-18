"""The fill-light and no-fill-light colour parameter sets must not collide."""

import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jetson_recognition.color_profiles import (  # noqa: E402
    apply_color_profile,
    resolve_profile_name,
    save_override,
)


def base_config():
    return {
        "lighting": {"fill_light": False},
        "colors": {
            "1": {"name": "red", "hsv_ranges": [{"h_min": 0, "h_max": 8}],
                  "prototype_hsv": [3.0, 200.0, 200.0]},
            "2": {"name": "yellow", "hsv_ranges": [{"h_min": 20, "h_max": 34}]},
        },
        "color_detection": {
            "geometry": {"min_mass_ratio": 0.35, "max_hole_ratio": 0.3},
        },
    }


class ResolveProfileNameTest(unittest.TestCase):
    def test_follows_fill_light_switch(self):
        fill = {"lighting": {"fill_light": True}}
        ambient = {"lighting": {"fill_light": False}}
        self.assertEqual(resolve_profile_name(fill), "fill")
        self.assertEqual(resolve_profile_name(ambient), "ambient")

    def test_no_switch_configured(self):
        self.assertIsNone(resolve_profile_name({}))

    def test_explicit_override_wins(self):
        self.assertEqual(
            resolve_profile_name({"lighting": {"fill_light": True}}, "ambient"),
            "ambient",
        )


class ApplyColorProfileTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        (self.root / "config").mkdir()

    def tearDown(self):
        self.temporary.cleanup()

    def write_profile(self, payload):
        (self.root / "config" / "color_profiles.json").write_text(
            json.dumps(payload, ensure_ascii=False), encoding="utf-8"
        )

    def test_ambient_only_overrides_what_it_specifies(self):
        self.write_profile(
            {
                "ambient": {
                    "colors": {"1": {"prototype_hsv": [2.0, 150.0, 90.0]}},
                    "color_detection": {"geometry": {"min_mass_ratio": 0.5}},
                }
            }
        )
        config = base_config()
        info = apply_color_profile(config, self.root, quiet=True)
        self.assertEqual(info["profile"], "ambient")
        self.assertEqual(info["colors_overridden"], ["1"])
        # overridden
        self.assertEqual(config["colors"]["1"]["prototype_hsv"], [2.0, 150.0, 90.0])
        self.assertEqual(config["color_detection"]["geometry"]["min_mass_ratio"], 0.5)
        # untouched siblings survive the deep merge
        self.assertEqual(config["colors"]["1"]["name"], "red")
        self.assertEqual(config["colors"]["1"]["hsv_ranges"][0]["h_min"], 0)
        self.assertEqual(config["colors"]["2"]["hsv_ranges"][0]["h_max"], 34)
        self.assertEqual(config["color_detection"]["geometry"]["max_hole_ratio"], 0.3)

    def test_fill_mode_keeps_the_base_values(self):
        self.write_profile({"ambient": {"colors": {"1": {"prototype_hsv": [9, 9, 9]}}}})
        config = base_config()
        config["lighting"]["fill_light"] = True
        before = copy.deepcopy(config["colors"])
        info = apply_color_profile(config, self.root, quiet=True)
        self.assertEqual(info["profile"], "fill")
        self.assertEqual(info["sections"], [])
        self.assertEqual(config["colors"], before)

    def test_apply_is_idempotent(self):
        self.write_profile({"ambient": {"colors": {"1": {"prototype_hsv": [2, 150, 90]}}}})
        config = base_config()
        apply_color_profile(config, self.root, quiet=True)
        once = copy.deepcopy(config)
        apply_color_profile(config, self.root, quiet=True)
        self.assertEqual(config, once)

    def test_missing_file_is_not_fatal(self):
        config = base_config()
        info = apply_color_profile(config, self.root, quiet=True)
        self.assertEqual(info["sections"], [])
        self.assertEqual(config["colors"]["1"]["name"], "red")

    def test_unknown_section_is_rejected(self):
        with self.assertRaises(ValueError):
            save_override(self.root, "ambient", "camera", {})

    def test_save_then_apply_round_trip(self):
        self.write_profile({"ambient": {}, "fill": {}})
        backup = save_override(
            self.root, "ambient", "colors", {"3": {"prototype_hsv": [120.0, 180.0, 120.0]}}
        )
        self.assertTrue(backup and Path(backup).exists())
        config = base_config()
        info = apply_color_profile(config, self.root, quiet=True)
        self.assertEqual(info["colors_overridden"], ["3"])
        self.assertEqual(config["colors"]["3"]["prototype_hsv"], [120.0, 180.0, 120.0])
        # the previously written block is not lost by a second save
        save_override(self.root, "ambient", "colors", {"4": {"prototype_hsv": [0, 0, 0]}})
        stored = json.loads((self.root / "config" / "color_profiles.json").read_text("utf-8"))
        self.assertEqual(sorted(stored["ambient"]["colors"]), ["3", "4"])

    def test_real_config_and_profile_file_load(self):
        real_root = PROJECT_ROOT
        config = json.loads(
            (real_root / "config" / "jetson.json").read_text(encoding="utf-8")
        )
        before = copy.deepcopy(config["colors"])
        info = apply_color_profile(config, real_root, quiet=True)
        self.assertEqual(info["profile"], "ambient")
        # the shipped ambient section is empty, so nothing may change yet
        self.assertEqual(config["colors"], before)


if __name__ == "__main__":
    unittest.main()
