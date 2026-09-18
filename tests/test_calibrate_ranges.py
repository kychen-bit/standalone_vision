"""The range-calibration rules: the red fix generalised to every colour.

Red was fixed by two measured numbers, not by taste:

* the material's own saturation (255) set the upper reference, and
* the *same-hue background* (the warm table, S~101) set the floor, so s_min went
  to 140 to keep the table out of the red mask.

These tests pin that rule down, plus the different rule black needs.
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

from tools.calibrate_color_ranges import (  # noqa: E402
    background_saturation,
    circular_mean,
    material_pixels,
    recommend_black,
    recommend_ranges,
)


def hsv_frame(pairs):
    """Build an HSV image from (region_mask, (h, s, v)) pairs."""
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    for region, values in pairs:
        frame[region] = values
    return frame


def disc(center, radius):
    mask = np.zeros((240, 320), dtype=bool)
    cv2.circle(
        mask.view(np.uint8).reshape(240, 320), center, radius, 1, -1
    )
    return mask.astype(bool)


class MaterialPixelTest(unittest.TestCase):
    def test_annulus_excludes_the_center_highlight(self):
        frame = hsv_frame([
            (np.ones((240, 320), dtype=bool), (0, 40, 40)),
        ])
        frame[disc((160, 120), 60)] = (170, 230, 200)
        frame[disc((160, 120), 15)] = (170, 90, 250)   # blown-out spot
        # The colour mask never covers the spot: it is a hole in the blob.
        mask = np.zeros((240, 320), dtype=np.uint8)
        cv2.circle(mask, (160, 120), 60, 255, -1)
        cv2.circle(mask, (160, 120), 16, 0, -1)

        hue, saturation, value, stats = material_pixels(
            frame, mask, core_ratio=0.6, inner_ratio=0.3
        )
        self.assertGreater(stats["hull_px"], 8_000)
        # The annulus sits inside the phi30 face and never touches the spot.
        self.assertAlmostEqual(float(np.median(saturation)), 230.0, delta=3.0)
        self.assertAlmostEqual(circular_mean(hue), 170.0, delta=1.0)
        self.assertGreater(float(np.percentile(value, 98.0)), 190.0)


class BackgroundRuleTest(unittest.TestCase):
    def test_s_min_clears_a_same_hue_background(self):
        """Reproduces the red case: material S=255 over a table with S~100."""
        frame = hsv_frame([
            (np.ones((240, 320), dtype=bool), (42, 100, 150)),   # warm table
        ])
        frame[disc((160, 120), 60)] = (35, 255, 150)             # material
        inside = disc((160, 120), 60)
        background, count = background_saturation(frame, inside, 35.0, 15.0)
        self.assertGreater(count, 1000)
        self.assertAlmostEqual(background["p98"], 100.0, delta=3.0)

        hue, saturation, value, stats = material_pixels(
            frame,
            np.where(inside, 255, 0).astype(np.uint8),
            core_ratio=0.6, inner_ratio=0.3,
        )
        stats["min_half_width"] = 6.0
        ranges, measured = recommend_ranges(
            (hue, saturation, value, stats, 25.0, 25.0, background)
        )
        # Floor is the background's p98 plus a margin, not the material's p2
        # (which would sit right on the table's saturation).
        self.assertTrue(ranges)
        for entry in ranges:
            self.assertGreaterEqual(entry["s_min"], 110)
            self.assertLess(entry["s_min"], 255)
        self.assertEqual(measured["background_same_hue"]["p98"], 100.0)

    def test_background_of_another_hue_does_not_inflate_s_min(self):
        frame = hsv_frame([
            (np.ones((240, 320), dtype=bool), (60, 220, 150)),   # green table
        ])
        inside = disc((160, 120), 60)
        frame[inside] = (120, 220, 150)                           # blue material
        background, count = background_saturation(frame, inside, 120.0, 15.0)
        self.assertLess(count, 256, "a differently coloured table must not count")
        hue, saturation, value, stats = material_pixels(
            frame, np.where(inside, 255, 0).astype(np.uint8),
            core_ratio=0.6, inner_ratio=0.3,
        )
        stats["min_half_width"] = 6.0
        ranges, _measured = recommend_ranges(
            (hue, saturation, value, stats, 25.0, 25.0, background)
        )
        self.assertEqual(len(ranges), 1)
        self.assertLessEqual(ranges[0]["s_min"], 200)
        self.assertGreaterEqual(ranges[0]["v_min"], 100)

    def test_hue_wrap_emits_two_ranges(self):
        frame = hsv_frame([(np.ones((240, 320), dtype=bool), (0, 20, 20))])
        inside = disc((160, 120), 60)
        frame[inside] = (178, 240, 200)     # a red right at the wrap point
        hue, saturation, value, stats = material_pixels(
            frame, np.where(inside, 255, 0).astype(np.uint8),
            core_ratio=0.6, inner_ratio=0.3,
        )
        stats["min_half_width"] = 6.0
        ranges, _measured = recommend_ranges(
            (hue, saturation, value, stats, 25.0, 25.0, None)
        )
        self.assertEqual(len(ranges), 2)
        self.assertTrue(any(entry["h_max"] == 179 for entry in ranges))
        self.assertTrue(any(entry["h_min"] == 0 for entry in ranges))


class BlackRuleTest(unittest.TestCase):
    def test_v_max_and_s_max_come_from_the_material(self):
        frame = hsv_frame([(np.ones((240, 320), dtype=bool), (0, 20, 20))])
        inside = disc((160, 120), 60)
        frame[inside] = (0, 35, 25)          # dark and nearly achromatic
        frame[disc((160, 120), 15)] = (10, 120, 200)   # tinted highlight
        # The mask only covers the part itself, never the highlight.
        mask = np.where(inside, 255, 0).astype(np.uint8)
        texture = np.zeros((240, 320), dtype=np.uint8)
        cv2.circle(texture, (160, 120), 60, 255, -1)
        cv2.circle(texture, (160, 120), 16, 0, -1)
        mask = cv2.bitwise_and(mask, texture)
        hue, saturation, value, stats = material_pixels(
            mask=mask, hsv=frame, core_ratio=0.6, inner_ratio=0.3,
        )
        proposal, measured = recommend_black(
            (hue, saturation, value, stats, 25.0, 25.0, None)
        )
        self.assertLessEqual(proposal["v_max"], 80)
        self.assertGreaterEqual(proposal["s_max"], 60)
        self.assertLessEqual(proposal["s_max"], 180)
        self.assertEqual(proposal["s_min"], 0)
        self.assertLessEqual(measured["v_percentiles"][2], 40)

    def test_black_stays_below_the_saturated_materials(self):
        """Black must not swallow a shadowed *saturated* colour.

        A saturated material keeps S>=140 even in shadow (S is invariant under
        pure illumination scaling), so black's chroma ceiling has to stay below
        that. Yellow and light blue ship with a low s_min on purpose - they are
        low-saturation colours, and hue is what separates them from black.
        """
        config = json.loads(
            (ROOT / "config" / "jetson.json").read_text(encoding="utf-8")
        )
        black = config["colors"]["5"]["black_threshold"]
        saturated_floors = [
            entry["s_min"]
            for color_id, definition in config["colors"].items()
            if color_id in ("1", "3", "4")
            for entry in definition["hsv_ranges"]
        ]
        self.assertLess(black["s_max"], min(saturated_floors))
        self.assertLessEqual(black["s_max"], 120)


if __name__ == "__main__":
    unittest.main()
