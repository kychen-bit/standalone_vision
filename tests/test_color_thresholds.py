"""Colour thresholds must separate black (achromatic) from shadowed colours.

The live failure this file pins down: black's threshold had ``s_max: 255``, i.e.
no chroma gate at all, so every pixel with V <= 80 counted as black. When the
arm shadowed the red material its V fell below 80 while its saturation stayed at
255, so the shadowed red material *became* the black mask - measured on a real
frame, the largest black blob was 128 084 px with a median of H=0 S=255 V=18.
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


class BlackIsAchromaticTest(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.detector = detector_for(self.config)

    def test_black_threshold_has_a_chroma_gate(self):
        black = self.config["colors"]["5"]["black_threshold"]
        self.assertLessEqual(black["s_max"], 120)
        self.assertLess(black["v_max"], 120)

    def test_shadowed_red_is_not_black(self):
        """The regression: dark but saturated red must stay red."""
        shadowed_red = (5, 255, 90)
        self.assertNotEqual(
            int(mask_for(self.detector, "5", shadowed_red)[0, 0]), 255,
            "a saturated dark red pixel must not fall into the black mask",
        )
        self.assertEqual(
            int(mask_for(self.detector, "1", shadowed_red)[0, 0]), 255,
            "a saturated dark red pixel must still fall into the red mask",
        )

    def test_below_the_noise_floor_is_deliberately_undecided(self):
        """Documented limitation: at 10x under nominal exposure, nothing wins.

        Refusing to guess beats poisoning the round assignment with a wrong
        colour; the cure for this state is exposure or light, not a threshold.
        """
        unusable = (5, 255, 40)
        for color_id in ("1", "5"):
            self.assertEqual(int(mask_for(self.detector, color_id, unusable)[0, 0]), 0)

    def test_real_black_is_black_and_not_red(self):
        pitch_black = (0, 0, 10)
        self.assertEqual(int(mask_for(self.detector, "5", pitch_black)[0, 0]), 255)
        self.assertEqual(int(mask_for(self.detector, "1", pitch_black)[0, 0]), 0)

    def test_shadowed_blue_and_green_stay_chromatic(self):
        for color_id, pixel in (("3", (118, 250, 95)), ("4", (75, 250, 95))):
            self.assertEqual(
                int(mask_for(self.detector, "5", pixel)[0, 0]), 0,
                "colour %s must not be swallowed by the black mask" % color_id,
            )
            self.assertEqual(
                int(mask_for(self.detector, color_id, pixel)[0, 0]), 255,
                "colour %s must survive its own mask when darkened" % color_id,
            )

    def test_saturated_dark_noise_is_not_black(self):
        """Chroma noise lifts S, so dark saturated pixels must not become black.

        Dark *neutral* pixels (low S, low V) are indistinguishable from a black
        material by colour alone - that is what the area band is for.
        """
        saturated_noise = (100, 200, 40)
        self.assertEqual(int(mask_for(self.detector, "5", saturated_noise)[0, 0]), 0)


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
        config["colors"]["5"]["black_threshold"] = {"v_max": 80, "s_min": 0, "s_max": 90}
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
