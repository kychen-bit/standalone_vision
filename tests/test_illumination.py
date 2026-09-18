"""The rule change removed the fill light: colour must survive the light.

These tests build the same material scene, shine several plausible competition
illuminants on it and require the same six labels every time. The last test
pins down the regression this work is about: with the unnormalised thresholds the
dim scene loses most of the materials.
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
from jetson_recognition.illumination import WhiteBalanceNormalizer


FIELD_LEVEL = 235
BACKGROUND = (FIELD_LEVEL, FIELD_LEVEL, FIELD_LEVEL)
POSITIONS = {
    "1": (200, 200),
    "2": (500, 200),
    "3": (800, 200),
    "4": (1100, 200),
    "5": (350, 520),
    "6": (850, 520),
}


def load_config():
    config = json.loads((ROOT / "config" / "jetson.json").read_text(encoding="utf-8"))
    # The shipped turntable area limits are tuned to the real rig. These tests
    # are about colour and illumination, so make them size-independent.
    config["scenes"]["TURNTABLE"].update({"min_area": 300, "max_area": 500000})
    return config


def hsv_bgr(h, s, v):
    pixel = np.asarray([[[h, s, v]]], dtype=np.uint8)
    return tuple(
        int(value) for value in cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0, 0]
    )


def material_bgr(config, color_id):
    """The centre of the configured range: what this config calls that colour."""
    definition = config["colors"][color_id]
    if color_id == "5":
        return (40, 40, 40)
    threshold = definition["hsv_ranges"][0]
    return hsv_bgr(
        0.5 * (threshold["h_min"] + threshold["h_max"]),
        0.5 * (threshold["s_min"] + threshold["s_max"]),
        0.5 * (threshold["v_min"] + threshold["v_max"]),
    )


def build_scene(config, radius=60):
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :] = BACKGROUND
    for color_id, center in POSITIONS.items():
        cv2.circle(frame, center, radius, material_bgr(config, color_id), -1)
    return frame


def illuminate(frame, gains, shading=None):
    """Apply a per-channel illuminant and an optional horizontal shading ramp."""
    result = frame.astype(np.float32)
    for channel, gain in enumerate(gains):
        result[:, :, channel] *= float(gain)
    if shading is not None:
        left, right = shading
        ramp = np.linspace(left, right, frame.shape[1], dtype=np.float32)
        result *= ramp[None, :, None]
    return np.clip(result, 0.0, 255.0).astype(np.uint8)


class WhiteBalanceNormalizerTests(unittest.TestCase):
    def test_neutral_surface_lands_on_the_canonical_level(self):
        normalizer = WhiteBalanceNormalizer(
            {"enabled": True, "canonical_level": 235.0}
        )
        frame = np.full((120, 160, 3), 120, dtype=np.uint8)
        balanced, info = normalizer.apply(frame)
        self.assertTrue(info["reliable"])
        self.assertAlmostEqual(int(balanced[10, 10, 0]), 235, delta=4)
        self.assertAlmostEqual(int(balanced[10, 10, 2]), 235, delta=4)
        self.assertAlmostEqual(info["gain"][0], 235.0 / 120.0, delta=0.05)

    def test_tinted_light_is_neutralised_per_channel(self):
        normalizer = WhiteBalanceNormalizer({"enabled": True})
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        frame[:, :] = (150, 170, 200)  # warm illuminant on a white field
        balanced, info = normalizer.apply(frame)
        self.assertTrue(info["reliable"])
        pixel = balanced[10, 10].astype(np.float32)
        self.assertAlmostEqual(float(pixel[0]), 235.0, delta=4)
        self.assertAlmostEqual(float(pixel[2]), 235.0, delta=4)

    def test_colourful_frame_without_a_white_reference_passes_through(self):
        normalizer = WhiteBalanceNormalizer({"enabled": True})
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        frame[:, :] = (200, 40, 40)
        balanced, info = normalizer.apply(frame)
        self.assertFalse(info["reliable"])
        self.assertEqual(info["reason"], "NO_NEUTRAL_REFERENCE")
        self.assertTrue(np.array_equal(balanced, frame))

    def test_abrupt_darkening_is_held_instead_of_followed(self):
        normalizer = WhiteBalanceNormalizer(
            {"enabled": True, "hold_smoothing": 0.0}
        )
        normalizer.apply(np.full((120, 160, 3), 200, dtype=np.uint8))
        frame = np.full((120, 160, 3), 60, dtype=np.uint8)
        _balanced, info = normalizer.apply(frame)
        self.assertTrue(info["held"])
        self.assertEqual(info["reason"], "HELD_ABRUPT_CHANGE")
        self.assertAlmostEqual(info["gain"][0], 235.0 / 200.0, delta=0.02)

    def test_disabled_normalizer_is_a_pass_through(self):
        normalizer = WhiteBalanceNormalizer({"enabled": False})
        frame = np.full((60, 80, 3), 90, dtype=np.uint8)
        balanced, info = normalizer.apply(frame)
        self.assertFalse(info["enabled"])
        self.assertTrue(np.array_equal(balanced, frame))


class MaterialSceneTests(unittest.TestCase):
    # Gains stay at or below 1.0 so the white field is not clipped: the venue
    # is dimmer than the tuning condition and tinted, which is what the rule
    # change is about. A blown-out field is checked separately below.
    ILLUMINANTS = {
        "warm_hall": (0.62, 0.75, 1.00),
        "cool_hall": (1.00, 0.76, 0.60),
        "dim_room": (0.45, 0.45, 0.45),
        "flicker_dip": (0.80, 0.80, 0.80),
        "warm_and_dim": (0.35, 0.42, 0.56),
    }

    def test_all_six_materials_are_labelled_on_the_reference_scene(self):
        config = load_config()
        detector = TopViewDetector(config, ROOT)
        materials = detector.detect_materials(build_scene(config), "TURNTABLE")
        self.assertEqual({item.target_id for item in materials}, set(POSITIONS))
        self.assertTrue(detector.last_illumination["reliable"])

    def test_all_six_materials_survive_illuminant_changes(self):
        config = load_config()
        reference = build_scene(config)
        for name, gains in self.ILLUMINANTS.items():
            with self.subTest(illuminant=name):
                # A fresh detector: the reference is estimated from scratch.
                detector = TopViewDetector(config, ROOT)
                materials = detector.detect_materials(
                    illuminate(reference, gains), "TURNTABLE"
                )
                self.assertEqual(
                    {item.target_id for item in materials},
                    set(POSITIONS),
                    "lost under %s (%s)" % (name, gains),
                )

    def test_shading_ramp_still_locates_every_material(self):
        config = load_config()
        detector = TopViewDetector(config, ROOT)
        frame = illuminate(build_scene(config), (1.0, 1.0, 1.0), shading=(0.82, 1.0))
        materials = detector.detect_materials(frame, "TURNTABLE")
        self.assertEqual({item.target_id for item in materials}, set(POSITIONS))

    def test_unnormalised_thresholds_lose_materials_in_dim_light(self):
        config = load_config()
        config["color_detection"]["illumination"]["enabled"] = False
        detector = TopViewDetector(config, ROOT)
        dim = illuminate(build_scene(config), (0.45, 0.45, 0.45))
        found = {item.target_id for item in detector.detect_materials(dim, "TURNTABLE")}
        self.assertLess(len(found), len(POSITIONS))

        normalised = TopViewDetector(load_config(), ROOT)
        repaired = {
            item.target_id
            for item in normalised.detect_materials(dim, "TURNTABLE")
        }
        self.assertEqual(repaired, set(POSITIONS))

    def test_overexposed_field_is_reported(self):
        config = load_config()
        detector = TopViewDetector(config, ROOT)
        frame = illuminate(build_scene(config), (1.35, 1.35, 1.35))
        detector.detect_materials(frame, "TURNTABLE")
        self.assertGreater(detector.last_illumination["overexposed_ratio"], 0.05)

    def test_material_scores_cover_every_configured_colour(self):
        config = load_config()
        detector = TopViewDetector(config, ROOT)
        materials = detector.detect_materials(build_scene(config), "TURNTABLE")
        debug = detector.last_materials_debug
        self.assertEqual(len(debug["materials"]), len(materials))
        for entry in debug["materials"]:
            self.assertEqual(set(entry["scores"]), set("123456"))
        # The requested colour must also win its own blob's score list.
        by_center = {
            (round(item["center"][0]), round(item["center"][1])): item
            for item in debug["materials"]
        }
        for color_id, center in POSITIONS.items():
            entry = min(
                by_center.values(),
                key=lambda item: abs(item["center"][0] - center[0])
                + abs(item["center"][1] - center[1]),
            )
            self.assertEqual(entry["color_id"], color_id)
            self.assertLess(float(entry["distance"]), 0.5)


if __name__ == "__main__":
    unittest.main()
