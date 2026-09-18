"""Colour thresholds on the real materials.

Two measured facts drive everything here (2026-09-18, fill light, real parts):

* The red part's core is fully saturated (S=255) while the warm table
  background sits at S~101, so red needs ``s_min`` above the background.
* The black part is *not* achromatic: its matte-textured top face reflects
  saturated light (measured H=123, S median 112, S p95 255), and brightness and
  chroma are anti-correlated across the face (V<40 grooves: S~220; V>210
  highlights: S~60). Requiring low chroma for black caps it at 0.4% of the
  face, so black must be defined by darkness alone.
"""

import json
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from jetson_recognition.detectors import TopViewDetector


def load_config():
    return json.loads((ROOT / "config" / "jetson.json").read_text(encoding="utf-8"))


def detector_for(config):
    return TopViewDetector(config, ROOT)


def mask_for(detector, color_id, hsv_triple):
    """Mask of a one-pixel HSV patch, without morphology."""
    hsv = np.zeros((1, 1, 3), dtype=np.uint8)
    hsv[0, 0] = hsv_triple
    return detector._make_color_mask(hsv, color_id)


class BlackIsDarkNotAchromaticTest(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.detector = detector_for(self.config)

    def test_black_gate_only_limits_brightness(self):
        black = self.config["colors"]["5"]["black_threshold"]
        self.assertLessEqual(black["v_max"], 120)
        # Deliberate: a glossy black part reflects saturated light.
        self.assertGreaterEqual(
            black["s_max"], 200,
            "a chroma gate on black cannot cover a glossy black part; measured "
            "coverage drops from 47.9% to 0.4% of the face",
        )

    def test_dark_saturated_black_face_is_in_the_black_mask(self):
        """The measured black part: dark groove with S=220."""
        groove = (123, 220, 30)
        self.assertEqual(int(mask_for(self.detector, "5", groove)[0, 0]), 255)
        # ...and it is not claimed by the blue range it is hue-close to,
        # because blue requires S>=210 *and* V>=85.
        self.assertEqual(int(mask_for(self.detector, "3", groove)[0, 0]), 0)

    def test_real_black_is_black_and_not_red(self):
        pitch_black = (0, 0, 10)
        self.assertEqual(int(mask_for(self.detector, "5", pitch_black)[0, 0]), 255)
        self.assertEqual(int(mask_for(self.detector, "1", pitch_black)[0, 0]), 0)

    def test_bright_colour_is_never_black(self):
        """What actually keeps colours out of the black mask is brightness."""
        for color_id, pixel in (("1", (5, 255, 200)), ("3", (118, 250, 180)),
                                ("4", (75, 250, 180)), ("6", (95, 140, 200))):
            self.assertEqual(
                int(mask_for(self.detector, "5", pixel)[0, 0]), 0,
                "colour %s at its working brightness must not be black" % color_id,
            )

    def test_shadow_darkness_overlaps_black_by_design(self):
        """Documented trade-off: a very dark colour pixel does look like black.

        Both a shadowed red pixel and a glossy black groove are dark and
        saturated, so no per-pixel rule separates them. The round's distinct
        colour assignment plus each colour's v_min are the guard, not s_max.
        """
        shadowed_red = (5, 255, 60)
        self.assertEqual(int(mask_for(self.detector, "5", shadowed_red)[0, 0]), 255)
        self.assertEqual(int(mask_for(self.detector, "1", shadowed_red)[0, 0]), 0)


class ValueFloorTest(unittest.TestCase):
    """v_min is a shadow tolerance: above the noise floor, below nominal."""

    def test_value_floors_stay_above_the_noise_floor(self):
        config = load_config()
        for color_id, definition in config["colors"].items():
            if color_id == "5":
                continue
            for entry in definition["hsv_ranges"]:
                self.assertGreaterEqual(
                    entry["v_min"], 60,
                    "colors.%s.v_min=%s lets amplified chroma noise into the "
                    "mask" % (color_id, entry["v_min"]),
                )
                self.assertLessEqual(
                    entry["v_min"], 120,
                    "colors.%s.v_min=%s is a bright-scene-only floor; a "
                    "shadowed material would be ignored" % (color_id, entry["v_min"]),
                )

    def test_measured_shadow_case_reaches_its_own_mask(self):
        """A material at half nominal brightness must still be recognised."""
        detector = detector_for(load_config())
        # V=95 is above every configured floor and well inside normal lighting.
        for color_id, hue in (("1", 5), ("2", 40), ("3", 120), ("4", 80), ("6", 95)):
            definition = load_config()["colors"][color_id]
            saturation = definition["hsv_ranges"][0]["s_min"] + 20
            pixel = (hue, min(255, saturation + 60), 95)
            self.assertEqual(
                int(mask_for(detector, color_id, pixel)[0, 0]), 255,
                "colour %s at V=95 must be inside its own mask" % color_id,
            )


class ShadowCannotBeShapedAwayTest(unittest.TestCase):
    def test_a_solid_round_shadow_passes_every_shape_gate(self):
        """Documents the limit: shape gates reject hollow/fragmented blobs only."""
        config = load_config()
        config["scenes"]["TURNTABLE"].update({"min_area": 300, "max_area": 500000})
        detector = detector_for(config)
        frame = np.full((720, 1280, 3), 200, dtype=np.uint8)
        # A pitch-black disc cast by a point-source lamp: perfectly solid+round.
        cv2.circle(frame, (640, 360), 120, (0, 0, 0), -1)
        # It is detected unless an area/position gate rejects it, which is why
        # the config carries shadow_notice instead of more shape rules.
        self.assertIsNotNone(detector.detect_color(frame, "5", "TURNTABLE"))


if __name__ == "__main__":
    unittest.main()
