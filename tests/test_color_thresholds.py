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

    def test_every_colour_uses_the_same_schema(self):
        """One schema for all six: hsv_ranges with min and max on all three.

        Black is two ordinary ranges (dark + coloured-clipped), so nothing in
        the config or the detector treats it specially any more.
        """
        for color_id, definition in self.config["colors"].items():
            ranges = definition.get("hsv_ranges")
            self.assertTrue(ranges, "colors.%s has no hsv_ranges" % color_id)
            for entry in ranges:
                for key in ("h_min", "h_max", "s_min", "s_max", "v_min", "v_max"):
                    self.assertIn(key, entry, "colors.%s range %s" % (color_id, entry))
                self.assertLessEqual(entry["h_min"], entry["h_max"])
                self.assertLessEqual(entry["s_min"], entry["s_max"])
                self.assertLessEqual(entry["v_min"], entry["v_max"])
            self.assertNotIn("black_threshold", definition)

    def test_black_gate_is_a_dark_range(self):
        dark = self.config["colors"]["5"]["hsv_ranges"][0]
        self.assertEqual((dark["h_min"], dark["h_max"]), (0, 179))
        self.assertEqual((dark["s_min"], dark["s_max"]), (0, 255))
        self.assertEqual(dark["v_min"], 0)
        # Measured: the black face's V runs p25=42 p50=88 p75=164 with the table
        # at ~200. A gate at the median (80) leaves half the face outside, so
        # the blob halves and its centre flickers; above ~100 the whole face is
        # modelled and the centre is stable to a fraction of a pixel.
        self.assertGreaterEqual(
            dark["v_max"], 100,
            "a black gate at the face's own median makes the blob flicker "
            "(measured 7% area swing, 1.2 px centre jitter)",
        )
        self.assertLessEqual(dark["v_max"], 160)

    def test_clip_range_sits_above_the_table(self):
        """The clipped interval must not swallow the table.

        Measured table V p10..p90 = 139..170, so 245 leaves a wide margin; this
        interval is what makes a mirror-bright black face detectable at all.
        """
        clipped = self.config["colors"]["5"]["hsv_ranges"][1]
        self.assertGreaterEqual(clipped["v_min"], 220)
        self.assertLessEqual(clipped["v_min"], 254)
        self.assertEqual(clipped["v_max"], 255)
        # Saturation floor: a specular highlight keeps the lamp's colour while a
        # blown-out *diffuse* white surface desaturates, so without it the white
        # table would be read as black.
        self.assertGreaterEqual(clipped["s_min"], 60)

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

    def test_clipped_mirror_face_is_black(self):
        """Head-on under the fill light the black face clips instead of darkening.

        Measured: face V p10=148 p50=255 (half clipped) while the table is at
        142~170, so the material is *brighter* than the table and only the
        clipped interval separates them: V>=245 gave the whole face as one blob
        (bbox 286x285, aspect 1.00, hull circularity 1.00) with 3 px from the
        table.
        """
        face = (111, 125, 255)
        self.assertEqual(int(mask_for(self.detector, "5", face)[0, 0]), 255)
        # ...and it must also *score* as black, or the blob would be dropped by
        # the material distance gate after passing the mask.
        distances = self.detector._color_distances(
            np.asarray([111.0], dtype=np.float32),
            np.asarray([125.0], dtype=np.float32),
            np.asarray([255.0], dtype=np.float32),
            ["1", "5"],
        )
        self.assertLess(float(distances["5"][0]), 0.2)
        self.assertLess(float(distances["5"][0]), float(distances["1"][0]))

    def test_clipped_white_surface_is_not_black(self):
        """The saturation floor is what keeps a blown-out white surface out.

        A specular highlight keeps the lamp's colour (measured face S=125); a
        diffuse white surface that clips desaturates to S~0. Without this floor a
        white synthetic background enters the black mask and swallows everything.
        """
        for white in ((0, 0, 255), (0, 20, 250), (0, 60, 255)):
            self.assertEqual(
                int(mask_for(self.detector, "5", white)[0, 0]), 0,
                "clipped white %s must not be black" % (white,),
            )

    def test_mid_grey_table_is_not_black(self):
        """The table is the mid-grey that black must never claim."""
        table = (139, 72, 156)
        self.assertEqual(int(mask_for(self.detector, "5", table)[0, 0]), 0)

    def test_really_bright_colour_is_black_by_design(self):
        """A clipped *coloured* pixel is also not mid-grey.

        Accepted consequence of the two-interval rule: a glossy colour's small
        specular spots land in the black mask. They are far below min_area, and
        a colour whose whole face is clipped is unusable anyway.
        """
        blown = (5, 255, 255)
        self.assertEqual(int(mask_for(self.detector, "5", blown)[0, 0]), 255)
        # Working brightness of the colours stays out.
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
