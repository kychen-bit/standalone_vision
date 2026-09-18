"""Shape gates, per-colour calibration points and the fill-light switch."""

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
    config = json.loads((ROOT / "config" / "jetson.json").read_text(encoding="utf-8"))
    # The shipped turntable area limits are tuned to the real rig; these tests
    # are about shape gates, so make them size-independent.
    config["scenes"]["TURNTABLE"].update({"min_area": 300, "max_area": 500000})
    return config


def hsv_bgr(h, s=220, v=220):
    pixel = np.asarray([[[h, s, v]]], dtype=np.uint8)
    return tuple(int(value) for value in cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0, 0])


def empty_scene():
    return np.full((720, 1280, 3), 235, dtype=np.uint8)


class ShapeGateTests(unittest.TestCase):
    def test_solid_material_still_passes(self):
        detector = TopViewDetector(load_config(), ROOT)
        frame = empty_scene()
        cv2.circle(frame, (640, 360), 62, hsv_bgr(0), -1)
        self.assertIsNotNone(detector.detect_color(frame, "1", "TURNTABLE"))

    def test_hollow_colour_marking_is_rejected_by_a_shape_gate(self):
        config = load_config()
        frame = empty_scene()
        # A thin ring of colour: the outline is perfectly round, but hollow.
        cv2.circle(frame, (640, 360), 62, hsv_bgr(0), 8)
        detector = TopViewDetector(config, ROOT)
        self.assertIsNone(detector.detect_color(frame, "1", "TURNTABLE"))
        stats = detector._last_color_filter_stats
        self.assertGreater(stats["mass_rejected"] + stats["hole_rejected"], 0)

        # Only turning both hollow-shape gates off lets the ring through.
        config["color_detection"]["geometry"]["min_mass_ratio"] = 0.0
        config["color_detection"]["geometry"]["max_hole_ratio"] = 1.0
        permissive = TopViewDetector(config, ROOT)
        self.assertIsNotNone(permissive.detect_color(frame, "1", "TURNTABLE"))

    def test_frustum_cone_ring_is_rejected_by_the_hole_gate(self):
        """The conical face of the real part: mass is fine, only the hole shows it."""
        config = load_config()
        frame = empty_scene()
        # Annulus r=62 with an inner hole of r=37, i.e. the phi30/phi50 ratio.
        cv2.circle(frame, (640, 360), 62, hsv_bgr(0), -1)
        cv2.circle(frame, (640, 360), 37, (235, 235, 235), -1)
        detector = TopViewDetector(config, ROOT)
        detector.set_color_debug(True)
        self.assertIsNone(detector.detect_color(frame, "1", "TURNTABLE"))
        self.assertGreater(detector._last_color_filter_stats["hole_rejected"], 0)

        config["color_detection"]["geometry"]["max_hole_ratio"] = 1.0
        permissive = TopViewDetector(config, ROOT)
        self.assertIsNotNone(permissive.detect_color(frame, "1", "TURNTABLE"))

    def test_cross_shaped_marking_is_rejected_by_mass_ratio(self):
        config = load_config()
        frame = empty_scene()
        center = (640, 360)
        for angle in (45, 135):
            length, thickness = 200, 14
            dx = int(round(np.cos(np.deg2rad(angle)) * length / 2.0))
            dy = int(round(np.sin(np.deg2rad(angle)) * length / 2.0))
            cv2.line(
                frame,
                (center[0] - dx, center[1] - dy),
                (center[0] + dx, center[1] + dy),
                hsv_bgr(0),
                thickness,
            )
        detector = TopViewDetector(config, ROOT)
        self.assertIsNone(detector.detect_color(frame, "1", "TURNTABLE"))
        self.assertGreater(detector._last_color_filter_stats["mass_rejected"], 0)

    def test_shape_metrics_are_reported_for_every_material(self):
        detector = TopViewDetector(load_config(), ROOT)
        frame = empty_scene()
        cv2.circle(frame, (500, 360), 62, hsv_bgr(0), -1)
        cv2.circle(frame, (900, 360), 62, hsv_bgr(60), -1)
        detector.detect_materials(frame, "TURNTABLE", ["1", "4"])
        entries = detector.last_materials_debug["materials"]
        self.assertEqual(len(entries), 2)
        for entry in entries:
            self.assertGreater(entry["mass_ratio"], 0.9)
            self.assertGreater(entry["contour_circularity"], 0.8)
            self.assertAlmostEqual(entry["hole_ratio"], 0.0, places=2)
            # A phi50 disc at this synthetic scale: the dump reports the size
            # so the operator can set scenes.*.min_area/max_area from it.
            self.assertAlmostEqual(entry["equivalent_diameter"], 124.0, delta=4.0)
            self.assertEqual(len(entry["core_hsv"]), 3)
            self.assertGreater(entry["core_pixels"], 100)


class GlareTests(unittest.TestCase):
    """A hard lamp puts a blown-out spot on the top face of the material."""

    MATERIAL = (170, 230, 200)     # saturated red-pink, inside colors.1
    HIGHLIGHT = (170, 150, 250)    # same hue but washed out by the lamp

    def _frame_with_center_highlight(self):
        frame = empty_scene()
        cv2.circle(frame, (640, 360), 100, hsv_bgr(*self.MATERIAL), -1)
        cv2.circle(frame, (640, 360), 28, hsv_bgr(*self.HIGHLIGHT), -1)
        return frame

    def test_annulus_core_skips_the_center_highlight(self):
        """The disc core is contaminated by the highlight, the annulus is not."""
        frame = self._frame_with_center_highlight()
        config = load_config()
        config["color_detection"]["label_classifier"]["prototype_core_inner_ratio"] = 0.0
        disc = TopViewDetector(config, ROOT)
        disc.detect_materials(frame, "TURNTABLE", ["1"])
        disc_core = disc.last_materials_debug["materials"][0]["core_hsv"]

        config["color_detection"]["label_classifier"]["prototype_core_inner_ratio"] = 0.5
        annulus = TopViewDetector(config, ROOT)
        annulus.detect_materials(frame, "TURNTABLE", ["1"])
        annulus_core = annulus.last_materials_debug["materials"][0]["core_hsv"]

        # The highlight is smaller than the skipped inner radius, so the
        # annulus samples the material's own colour while the full disc core
        # averages the washed-out spot into it.
        self.assertGreater(annulus_core[1], disc_core[1] + 10.0)
        self.assertAlmostEqual(annulus_core[1], self.MATERIAL[1], delta=6.0)
        for core in (disc_core, annulus_core):
            self.assertGreater(core[0], 140.0)
            self.assertLess(core[0], 179.0)

    def test_a_mostly_glare_hole_is_not_rejected_as_a_ring(self):
        """A bright hole is a highlight; the frustum's coloured ring is not."""
        frame = empty_scene()
        cv2.circle(frame, (640, 360), 100, hsv_bgr(*self.MATERIAL), -1)
        # A big blown-out spot: (65/100)^2 = 42% of the face, over the 0.3 gate.
        cv2.circle(frame, (640, 360), 65, (255, 255, 255), -1)

        config = load_config()
        detector = TopViewDetector(config, ROOT)
        detector.detect_materials(frame, "TURNTABLE", ["1"])
        entries = detector.last_materials_debug["materials"]
        self.assertEqual(len(entries), 1, "the material must survive its own glare")
        self.assertGreater(entries[0]["hole_ratio"], 0.3)
        self.assertGreaterEqual(entries[0]["hole_glare_ratio"], 0.6)
        self.assertGreater(
            detector.last_materials_debug["filter_stats"]["1"]["glare_hole_excused"],
            0,
        )

        # Turning the excuse off must reject it again, which proves the excuse
        # (and not some other metric) is what accepted it.
        config["color_detection"]["geometry"]["glare_hole_ratio"] = 1.1
        strict = TopViewDetector(config, ROOT)
        strict.detect_materials(frame, "TURNTABLE", ["1"])
        self.assertEqual(strict.last_materials_debug["materials"], [])
        self.assertGreater(
            strict.last_materials_debug["filter_stats"]["1"]["hole_rejected"], 0
        )

    def test_a_coloured_ring_hole_is_still_rejected(self):
        """The cone face of the frustum must keep failing the hole gate."""
        frame = empty_scene()
        cv2.circle(frame, (640, 360), 100, hsv_bgr(*self.MATERIAL), -1)
        # Same geometry, but the hole holds another saturated colour.
        cv2.circle(frame, (640, 360), 65, hsv_bgr(60, 220, 200), -1)
        detector = TopViewDetector(load_config(), ROOT)
        detector.detect_materials(frame, "TURNTABLE", ["1"])
        self.assertEqual(detector.last_materials_debug["materials"], [])
        stats = detector.last_materials_debug["filter_stats"]["1"]
        self.assertEqual(stats["glare_hole_excused"], 0)
        self.assertGreater(stats["hole_rejected"], 0)


class PrototypeCalibrationTests(unittest.TestCase):
    CORE = (96.0, 175.0, 160.0)

    def _frame(self):
        frame = empty_scene()
        cv2.circle(frame, (640, 360), 62, hsv_bgr(98, 175, 160), -1)
        return frame

    def test_prototype_moves_the_distance_to_zero(self):
        hue = np.asarray([self.CORE[0]], dtype=np.float32)
        saturation = np.asarray([self.CORE[1]], dtype=np.float32)
        value = np.asarray([self.CORE[2]], dtype=np.float32)
        plain = TopViewDetector(load_config(), ROOT)
        configured = load_config()
        configured["colors"]["6"]["prototype_hsv"] = list(self.CORE)
        tuned = TopViewDetector(configured, ROOT)

        self.assertEqual(tuned.color_prototypes["6"], list(self.CORE))
        before = float(plain._color_distances(hue, saturation, value, ["6"])["6"][0])
        after = float(tuned._color_distances(hue, saturation, value, ["6"])["6"][0])
        self.assertGreater(before, 0.0)
        self.assertLess(after, 1e-6)

    def test_blob_scores_follow_the_configured_prototype(self):
        frame = self._frame()
        first = TopViewDetector(load_config(), ROOT)
        first.detect_materials(frame, "TURNTABLE", ["4", "6"])
        core = list(first.last_materials_debug["materials"][0]["core_hsv"])

        configured = load_config()
        configured["colors"]["6"]["prototype_hsv"] = core
        tuned = TopViewDetector(configured, ROOT)
        tuned.detect_materials(frame, "TURNTABLE", ["4", "6"])
        entry = tuned.last_materials_debug["materials"][0]
        self.assertAlmostEqual(entry["scores"]["6"], 1.0, places=6)
        self.assertGreater(entry["scores"]["6"], entry["scores"]["4"])

    def test_prototype_is_optional(self):
        detector = TopViewDetector(load_config(), ROOT)
        self.assertEqual(detector.color_prototypes, {})

    def test_invalid_prototype_is_rejected(self):
        configured = load_config()
        configured["colors"]["6"]["prototype_hsv"] = [96.0]
        with self.assertRaises(ValueError):
            TopViewDetector(configured, ROOT)


class FillLightSwitchTests(unittest.TestCase):
    def test_no_fill_light_enables_the_white_balance(self):
        config = load_config()
        config["lighting"] = {"fill_light": False}
        detector = TopViewDetector(config, ROOT)
        self.assertFalse(detector.fill_light)
        self.assertTrue(detector.illumination_enabled)

    def test_fill_light_disables_the_white_balance(self):
        config = load_config()
        config["lighting"] = {"fill_light": True}
        detector = TopViewDetector(config, ROOT)
        self.assertTrue(detector.fill_light)
        self.assertFalse(detector.illumination_enabled)
        # With the fill light on, the frame reaches the thresholds unchanged.
        frame = empty_scene()
        self.assertIs(detector._work_frame(frame), frame)
        self.assertIsNone(detector.last_illumination)

    def test_absent_switch_keeps_the_authored_setting(self):
        config = load_config()
        config.pop("lighting", None)
        detector = TopViewDetector(config, ROOT)
        self.assertIsNone(detector.fill_light)
        self.assertEqual(
            detector.illumination_enabled,
            config["color_detection"]["illumination"]["enabled"],
        )

    def test_shipped_config_defaults_to_the_rule_change(self):
        config = load_config()
        self.assertIn("lighting", config)
        self.assertFalse(config["lighting"]["fill_light"])
        self.assertGreater(
            config["color_detection"]["geometry"]["min_mass_ratio"], 0.0
        )


if __name__ == "__main__":
    unittest.main()
