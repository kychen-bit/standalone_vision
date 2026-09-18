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


def draw_numbered_ring(frame, center, digit, axes=(55, 55)):
    cv2.ellipse(
        frame, center, axes, 0, 0, 360, (20, 20, 20), 8, cv2.LINE_AA
    )
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 1.5
    thickness = 4
    (width, height), _baseline = cv2.getTextSize(
        str(digit), font, scale, thickness
    )
    origin = (center[0] - width // 2, center[1] + height // 2)
    cv2.putText(
        frame, str(digit), origin, font, scale,
        (20, 20, 20), thickness, cv2.LINE_AA,
    )


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

    def test_multi_color_detection_converts_full_roi_to_hsv_once(self):
        detector = TopViewDetector(load_config(), ROOT)
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.circle(frame, (350, 350), 50, self.SCREEN_COLORS["1"], -1)
        cv2.circle(frame, (700, 350), 50, self.SCREEN_COLORS["4"], -1)
        original = cv2.cvtColor
        calls = []

        def counting_cvt_color(image, code):
            if code == cv2.COLOR_BGR2HSV:
                calls.append(image.shape)
            return original(image, code)

        cv2.cvtColor = counting_cvt_color
        try:
            results = detector.detect_colors(frame, "TURNTABLE", ["1", "4", "5"])
        finally:
            cv2.cvtColor = original

        # The search ROI is converted once for all colours. The illuminant
        # estimator converts its own downscaled copy separately, which is not
        # what this regression is about.
        roi_conversions = [shape for shape in calls if shape == (720, 1280, 3)]
        self.assertEqual(roi_conversions, [(720, 1280, 3)])
        self.assertEqual({result.target_id for result in results}, {"1", "4"})
        self.assertEqual(detector.last_color_runtime["mode"], "SHARED_FIXED_ROI")

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
        config["color_detection"]["dynamic_roi"]["enabled"] = True
        engine = build_engine(config)
        session = engine.new_session("COLOR", ("red", "PAPER_TEST"))
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.rectangle(frame, (300, 300), (400, 400), self.SCREEN_COLORS["1"], -1)

        required_frames = int(config["stability"]["stable_frames"])
        duration_ms = float(config["stability"]["min_stable_time_ms"])
        update_count = max(required_frames, int(duration_ms // 20.0) + 2)
        stable = None
        for index in range(update_count):
            _measurement, stable, _count = session.update(
                frame, timestamp=index * 0.02
            )
        self.assertIsNotNone(stable)
        self.assertEqual(engine.detector.last_color_runtime["mode"], "FIXED_ROI")

        session.update(frame, timestamp=update_count * 0.02)
        runtime = engine.detector.last_color_runtime
        self.assertEqual(runtime["mode"], "DYNAMIC_ROI")
        self.assertLess(runtime["roi"][2], runtime["base_roi"][2])

        moved = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.rectangle(moved, (850, 300), (950, 400), self.SCREEN_COLORS["1"], -1)
        measurement, _stable, _count = session.update(
            moved, timestamp=(update_count + 1) * 0.02
        )
        self.assertIsNotNone(measurement)
        self.assertEqual(engine.detector.last_color_runtime["mode"], "FIXED_ROI")
        self.assertAlmostEqual(measurement.pixel_x, 900.0, delta=2.0)

    def test_competition_config_keeps_full_frame_after_stable(self):
        config = load_config()
        self.assertFalse(config["color_detection"]["dynamic_roi"]["enabled"])
        engine = build_engine(config)
        session = engine.new_session("COLOR", ("red", "TURNTABLE"))
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.rectangle(frame, (300, 300), (400, 400), self.SCREEN_COLORS["1"], -1)
        for index in range(20):
            session.update(frame, timestamp=index * 0.02)
        self.assertEqual(engine.detector.last_color_runtime["mode"], "FIXED_ROI")
        self.assertEqual(
            engine.detector.last_color_runtime["roi"], [0, 0, 1280, 720]
        )

    def test_area_filter_rejects_small_noise(self):
        detector = TopViewDetector(load_config(), ROOT)
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.rectangle(frame, (500, 300), (505, 305), self.SCREEN_COLORS["1"], -1)
        self.assertIsNone(detector.detect_color(frame, "1", "PAPER_TEST"))

    def test_color_candidate_touching_roi_border_is_rejected(self):
        detector = TopViewDetector(load_config(), ROOT)
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        # PAPER_TEST begins at x=160, so this otherwise valid red circle is
        # clipped by the search ROI and has an unsafe center estimate.
        cv2.circle(frame, (170, 350), 45, self.SCREEN_COLORS["1"], -1)
        self.assertIsNone(detector.detect_color(frame, "1", "PAPER_TEST"))

    def test_color_candidate_rejects_long_environment_strip(self):
        detector = TopViewDetector(load_config(), ROOT)
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.rectangle(frame, (350, 330), (800, 370), self.SCREEN_COLORS["1"], -1)
        self.assertIsNone(detector.detect_color(frame, "1", "PAPER_TEST"))

    def test_convex_hull_center_tolerates_reflective_edge_notch(self):
        detector = TopViewDetector(load_config(), ROOT)
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.circle(frame, (550, 350), 65, self.SCREEN_COLORS["1"], -1)
        # Simulate a highlight cutting a narrow notch through the color mask.
        cv2.rectangle(frame, (545, 330), (620, 370), (0, 0, 0), -1)

        result = detector.detect_color(frame, "1", "PAPER_TEST")

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.pixel_x, 550.0, delta=8.0)
        self.assertAlmostEqual(result.pixel_y, 350.0, delta=4.0)

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

    def test_stack_searches_storage_scene_roi_not_fixed_ring_box(self):
        detector = TopViewDetector(load_config(), ROOT)
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        # Ring 1's legacy box is near x=160..400. An arm-mounted camera can
        # observe the same stored red object elsewhere in the station ROI.
        cv2.rectangle(frame, (900, 300), (1020, 420), self.SCREEN_COLORS["1"], -1)

        result = detector.detect_stack(frame, "1", "STORAGE", "1")

        self.assertIsNotNone(result)
        self.assertEqual(result.kind, "STACK")
        self.assertEqual(result.target_id, "1")
        self.assertAlmostEqual(result.pixel_x, 960.0, delta=2.0)
        self.assertAlmostEqual(result.pixel_y, 360.0, delta=2.0)
        # Tracks config/jetson.json scenes.STORAGE.object_roi, which is now the
        # full frame; the ring 1 legacy box [160, 250, 240, 240] must still be
        # ignored for STACK.
        self.assertEqual(
            detector.last_color_runtime["base_roi"],
            [0, 0, 1280, 720],
        )

    def test_dark_achromatic_patch_is_black_for_any_hue(self):
        """Hue stays irrelevant for black; chroma is what changed.

        The patch is dark and nearly grey, i.e. a real black material with only
        the tiny hue offset a demosaic leaves behind.
        """
        detector = TopViewDetector(load_config(), ROOT)
        for hue in (0, 60, 120):
            frame = np.full((720, 1280, 3), 255, dtype=np.uint8)
            cv2.rectangle(frame, (500, 300), (600, 400), hsv_bgr(hue, 20, 40), -1)
            with self.subTest(hue=hue):
                self.assertIsNotNone(detector.detect_color(frame, "black", "PAPER_TEST"))

    def test_dark_saturated_patch_is_black_too(self):
        """Measured reversal: black's gate must not require low chroma.

        A glossy black part reflects saturated light: the real part measured
        H=123, S median 112, S p95 255 while a chroma gate of 90 covered only
        0.4% of its face. So a dark saturated patch counts as black, and what
        keeps *colours* out of the black mask is their brightness (v_min), not
        black's saturation limit.
        """
        detector = TopViewDetector(load_config(), ROOT)
        for hue in (0, 60, 120):
            frame = np.full((720, 1280, 3), 255, dtype=np.uint8)
            cv2.rectangle(frame, (500, 300), (600, 400), hsv_bgr(hue, 220, 40), -1)
            with self.subTest(hue=hue):
                self.assertIsNotNone(detector.detect_color(frame, "black", "PAPER_TEST"))

    def test_black_rejects_irregular_shadow(self):
        detector = TopViewDetector(load_config(), ROOT)
        frame = np.full((720, 1280, 3), 180, dtype=np.uint8)
        shadow = np.asarray(
            [(350, 250), (720, 290), (760, 330), (600, 355),
             (730, 410), (380, 440), (300, 350)],
            dtype=np.int32,
        )
        cv2.fillPoly(frame, [shadow], (35, 35, 35))
        self.assertIsNone(detector.detect_color(frame, "black", "PAPER_TEST"))

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

    def test_default_config_uses_white_balanced_hsv_ranges(self):
        config = load_config()
        self.assertNotIn("color_classifier", config)
        self.assertEqual(
            config["color_detection"]["algorithm"],
            "HSV_CONTOUR_V1_WHITE_BALANCED",
        )
        # The 2026 rule change removed the downward fill light, so the shipped
        # configuration must normalise the illuminant per frame.
        self.assertTrue(config["color_detection"]["illumination"]["enabled"])

    def test_ring_detection_selects_requested_digit_and_global_coordinates(self):
        detector = TopViewDetector(load_config(), ROOT)
        frame = np.full((720, 1280, 3), 200, dtype=np.uint8)
        for center_x, digit in ((280, "1"), (640, "2"), (1000, "3")):
            draw_numbered_ring(frame, (center_x, 370), digit)

        for ring_id, expected_x in (("1", 280), ("2", 640), ("3", 1000)):
            with self.subTest(ring_id=ring_id):
                result = detector.detect_ring(frame, ring_id, "ROUGH")
                self.assertIsNotNone(result)
                self.assertAlmostEqual(result.pixel_x, expected_x, delta=3.0)
                self.assertAlmostEqual(result.pixel_y, 370.0, delta=3.0)

    def test_ring_debug_exposes_mask_contours_and_selected_cluster(self):
        config = load_config()
        # Keep the debug-coordinate fixture independent of field ROI tuning.
        config["scenes"]["ROUGH"]["ring_search_roi"] = {
            "x": 80, "y": 170, "width": 1120, "height": 430,
        }
        detector = TopViewDetector(config, ROOT)
        detector.set_circle_debug(True)
        frame = np.full((720, 1280, 3), 200, dtype=np.uint8)
        draw_numbered_ring(frame, (640, 370), "2")

        result = detector.detect_ring(frame, "2", "ROUGH")
        debug = detector.last_circle_debug

        self.assertIsNotNone(result)
        self.assertEqual(debug["roi"], [80, 170, 1120, 430])
        self.assertEqual(debug["kind"], "RING")
        self.assertEqual(debug["target_id"], "2")
        self.assertEqual(debug["template_source"], "BUILTIN_SCREEN_TEST")
        self.assertGreater(len(debug["candidates"]), 0)
        self.assertIsNotNone(debug["selected"])
        self.assertGreaterEqual(debug["selected"]["members"], 2)
        self.assertAlmostEqual(debug["selected"]["center"][0], 640.0, delta=3.0)
        self.assertAlmostEqual(debug["selected"]["center"][1], 370.0, delta=3.0)
        self.assertTrue(
            all("circularity" in candidate for candidate in debug["candidates"])
        )
        self.assertEqual(debug["processed"].ndim, 2)

    def test_ring_session_uses_its_own_configured_stability(self):
        config = load_config()
        engine = build_engine(config)
        session = engine.new_session("RING", ("2", "ROUGH"))
        frame = np.full((720, 1280, 3), 200, dtype=np.uint8)
        draw_numbered_ring(frame, (640, 370), "2")

        required = int(config["ring_detection"]["stability"]["stable_frames"])
        duration_ms = float(
            config["ring_detection"]["stability"]["min_stable_time_ms"]
        )
        update_count = max(required, int(duration_ms // 20.0) + 2)
        stable = None
        for index in range(update_count):
            _measurement, stable, count = session.update(
                frame, timestamp=index * 0.02
            )
            if index < update_count - 1:
                self.assertIsNone(stable)
        self.assertIsNotNone(stable)
        self.assertGreaterEqual(count, required)

    def test_ring_contour_tolerates_moderate_camera_tilt(self):
        detector = TopViewDetector(load_config(), ROOT)
        frame = np.full((720, 1280, 3), 200, dtype=np.uint8)
        draw_numbered_ring(frame, (640, 370), "2", axes=(55, 42))

        result = detector.detect_ring(frame, "2", "ROUGH")

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.pixel_x, 640.0, delta=3.0)
        self.assertAlmostEqual(result.pixel_y, 370.0, delta=3.0)

    def test_ring_default_path_has_no_hough_parameters(self):
        ring_config = load_config()["ring_detection"]
        self.assertEqual(
            ring_config["algorithm"], "DYNAMIC_DIGIT_TEMPLATE_RING_V1"
        )
        self.assertNotIn("center_threshold", ring_config)
        self.assertNotIn("min_radius_px", ring_config)

    def test_ring_identity_follows_digit_not_fixed_position(self):
        detector = TopViewDetector(load_config(), ROOT)
        frame = np.full((720, 1280, 3), 200, dtype=np.uint8)
        draw_numbered_ring(frame, (280, 370), "3")
        draw_numbered_ring(frame, (640, 370), "1")
        draw_numbered_ring(frame, (1000, 370), "2")

        result = detector.detect_ring(frame, "2", "ROUGH")

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.pixel_x, 1000.0, delta=3.0)

    def test_ring_target_does_not_accept_a_different_digit(self):
        detector = TopViewDetector(load_config(), ROOT)
        frame = np.full((720, 1280, 3), 200, dtype=np.uint8)
        draw_numbered_ring(frame, (730, 420), "1")

        self.assertIsNone(detector.detect_ring(frame, "2", "ROUGH"))

    def test_single_target_ring_can_move_inside_search_roi(self):
        detector = TopViewDetector(load_config(), ROOT)
        for center in ((220, 260), (730, 420), (1080, 520)):
            frame = np.full((720, 1280, 3), 200, dtype=np.uint8)
            draw_numbered_ring(frame, center, "2")
            with self.subTest(center=center):
                result = detector.detect_ring(frame, "2", "ROUGH")
                self.assertIsNotNone(result)
                self.assertAlmostEqual(result.pixel_x, center[0], delta=3.0)
                self.assertAlmostEqual(result.pixel_y, center[1], delta=3.0)


if __name__ == "__main__":
    unittest.main()
