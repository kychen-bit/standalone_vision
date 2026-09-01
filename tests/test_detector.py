import json
from pathlib import Path
import unittest

import cv2
import numpy as np

from jetson_recognition.detectors import TopViewDetector
from jetson_recognition.run import HSVProbe, build_engine


ROOT = Path(__file__).resolve().parents[1]


def load_config():
    return json.loads((ROOT / "config" / "jetson.json").read_text(encoding="utf-8"))


def hsv_bgr(h, s=220, v=220):
    pixel = np.asarray([[[h, s, v]]], dtype=np.uint8)
    return tuple(int(value) for value in cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0, 0])


class DetectorTests(unittest.TestCase):
    SCREEN_COLORS = {
        "1": hsv_bgr(0),
        "2": hsv_bgr(30),
        "3": hsv_bgr(120),
        "4": hsv_bgr(60),
        "5": (20, 20, 20),
        "6": hsv_bgr(98),
    }

    def test_six_screen_colors_are_detected(self):
        detector = TopViewDetector(load_config(), ROOT)
        for color_id, bgr in self.SCREEN_COLORS.items():
            background = 255 if color_id == "5" else 0
            frame = np.full((720, 1280, 3), background, dtype=np.uint8)
            cv2.rectangle(frame, (500, 300), (600, 400), bgr, -1)
            with self.subTest(color_id=color_id):
                result = detector.detect_color(frame, color_id, "PAPER_TEST")
                self.assertIsNotNone(result)
                self.assertAlmostEqual(result.pixel_x, 550.0, delta=2.0)
                self.assertAlmostEqual(result.pixel_y, 350.0, delta=2.0)

    def test_red_uses_both_hue_ends(self):
        detector = TopViewDetector(load_config(), ROOT)
        for hue in (2, 177):
            frame = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.rectangle(frame, (500, 300), (600, 400), hsv_bgr(hue), -1)
            with self.subTest(hue=hue):
                self.assertIsNotNone(detector.detect_color(frame, "red", "PAPER_TEST"))

    def test_blue_and_light_blue_use_independent_hsv_ranges(self):
        detector = TopViewDetector(load_config(), ROOT)
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.rectangle(frame, (300, 300), (400, 400), hsv_bgr(120), -1)
        cv2.rectangle(frame, (700, 300), (800, 400), hsv_bgr(98), -1)

        blue = detector.detect_color(frame, "blue", "PAPER_TEST")
        light_blue = detector.detect_color(frame, "light_blue", "PAPER_TEST")

        self.assertAlmostEqual(blue.pixel_x, 350.0, delta=2.0)
        self.assertAlmostEqual(light_blue.pixel_x, 750.0, delta=2.0)

    def test_fixed_roi_ignores_outside_and_returns_global_center(self):
        config = load_config()
        config["scenes"]["PAPER_TEST"]["object_roi"] = {
            "x": 400, "y": 200, "width": 400, "height": 300,
        }
        detector = TopViewDetector(config, ROOT)
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.rectangle(frame, (50, 50), (150, 150), self.SCREEN_COLORS["1"], -1)
        self.assertIsNone(detector.detect_color(frame, "1", "PAPER_TEST"))

        cv2.rectangle(frame, (500, 300), (600, 400), self.SCREEN_COLORS["1"], -1)
        result = detector.detect_color(frame, "1", "PAPER_TEST")
        self.assertAlmostEqual(result.pixel_x, 550.0, delta=2.0)
        self.assertAlmostEqual(result.pixel_y, 350.0, delta=2.0)
        self.assertEqual(detector.last_color_runtime["mode"], "FIXED_ROI")

    def test_dynamic_roi_starts_after_stable_and_falls_back_same_frame(self):
        config = load_config()
        engine = build_engine(config)
        session = engine.new_session("COLOR", ("red", "PAPER_TEST"))
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.rectangle(frame, (300, 300), (400, 400), self.SCREEN_COLORS["1"], -1)

        session.update(frame)
        session.update(frame)
        _measurement, stable, _count = session.update(frame)
        self.assertIsNotNone(stable)
        self.assertEqual(engine.detector.last_color_runtime["mode"], "FIXED_ROI")

        session.update(frame)
        runtime = engine.detector.last_color_runtime
        self.assertEqual(runtime["mode"], "DYNAMIC_ROI")
        self.assertLess(runtime["roi"][2], runtime["base_roi"][2])

        moved = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.rectangle(moved, (850, 300), (950, 400), self.SCREEN_COLORS["1"], -1)
        measurement, _stable, _count = session.update(moved)
        self.assertIsNotNone(measurement)
        self.assertEqual(engine.detector.last_color_runtime["mode"], "FIXED_ROI")
        self.assertAlmostEqual(measurement.pixel_x, 900.0, delta=2.0)

    def test_area_filter_rejects_small_noise(self):
        detector = TopViewDetector(load_config(), ROOT)
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.rectangle(frame, (500, 300), (505, 305), self.SCREEN_COLORS["1"], -1)
        self.assertIsNone(detector.detect_color(frame, "1", "PAPER_TEST"))

    def test_unknown_irregular_shape_is_accepted(self):
        detector = TopViewDetector(load_config(), ROOT)
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        points = np.asarray(
            [(460, 250), (610, 275), (570, 345), (650, 430), (505, 405), (430, 330)],
            dtype=np.int32,
        )
        cv2.fillPoly(frame, [points], self.SCREEN_COLORS["4"])
        result = detector.detect_color(frame, "green", "PAPER_TEST")
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.pixel_x, 535.0, delta=15.0)
        self.assertAlmostEqual(result.pixel_y, 337.0, delta=15.0)

    def test_black_uses_value_not_hue(self):
        detector = TopViewDetector(load_config(), ROOT)
        for hue in (0, 60, 120):
            frame = np.full((720, 1280, 3), 255, dtype=np.uint8)
            cv2.rectangle(frame, (500, 300), (600, 400), hsv_bgr(hue, 220, 40), -1)
            with self.subTest(hue=hue):
                self.assertIsNotNone(detector.detect_color(frame, "black", "PAPER_TEST"))

    def test_debug_contains_one_mask_roi_box_and_contour(self):
        detector = TopViewDetector(load_config(), ROOT)
        detector.set_color_debug(True)
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.rectangle(frame, (500, 300), (600, 400), self.SCREEN_COLORS["1"], -1)

        result = detector.detect_color(frame, "red", "PAPER_TEST")
        debug = detector.last_color_debug

        self.assertIsNotNone(result)
        self.assertEqual(debug["color_name"], "RED")
        self.assertEqual(debug["roi"], [160, 90, 960, 540])
        self.assertEqual(len(debug["candidates"]), 1)
        self.assertIsNotNone(debug["mask"])
        self.assertGreater(len(debug["selected"]["contour"]), 0)

    def test_hsv_probe_reads_raw_pixels(self):
        frame = np.full((100, 120, 3), (255, 0, 0), dtype=np.uint8)
        probe = HSVProbe(radius=5)
        probe.position = (60, 50)
        sample = probe.sample(frame)
        self.assertEqual(tuple(sample["bgr50"]), (255.0, 0.0, 0.0))
        self.assertEqual(tuple(sample["hsv50"]), (120.0, 255.0, 255.0))

    def test_invalid_roi_is_rejected_at_startup(self):
        config = load_config()
        config["scenes"]["ROUGH"]["object_roi"] = {
            "x": 1200, "y": 0, "width": 200, "height": 100,
        }
        with self.assertRaises(ValueError):
            TopViewDetector(config, ROOT)

    def test_default_config_contains_no_lab_classifier(self):
        config = load_config()
        self.assertNotIn("color_classifier", config)
        self.assertEqual(config["color_detection"]["algorithm"], "HSV_CONTOUR_V1")


if __name__ == "__main__":
    unittest.main()
