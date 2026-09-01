import unittest

from jetson_recognition.engine import RecognitionEngine
from jetson_recognition.model import Measurement
from jetson_recognition.stability import SimpleColorStability


class FakeDetector:
    def detect_color(self, frame, color_id, scene):
        return Measurement("COLOR", color_id, 10.0, 20.0, 0.0, 0.9, "PX")


class EngineTests(unittest.TestCase):
    def setUp(self):
        self.engine = RecognitionEngine(
            FakeDetector(),
            {
                "stable_frames": 3,
                "stable_spread_px": 4.0,
                "stable_spread_mm": 2.0,
                "stable_yaw_deg": 1.0,
                "min_confidence": 0.5,
                "reset_after_misses": 2,
            },
        )

    def test_session_becomes_stable_after_required_frames(self):
        session = self.engine.new_session("COLOR", ("1", "ROUGH"))
        self.assertIsNone(session.update(object())[1])
        self.assertIsNone(session.update(object())[1])
        measurement, stable, count = session.update(object())
        self.assertEqual(measurement.kind, "COLOR")
        self.assertIsNotNone(stable)
        self.assertEqual(count, 3)

    def test_bad_argument_count_is_rejected_before_detection(self):
        with self.assertRaises(ValueError):
            self.engine.detect(object(), "COLOR", ("1",))

    def test_auxiliary_mode_can_be_disabled_by_config(self):
        runtime = dict(self.engine.runtime)
        runtime["enabled_modes"] = ["COLOR", "RING", "STACK"]
        engine = RecognitionEngine(FakeDetector(), runtime)
        with self.assertRaises(ValueError):
            engine.detect(object(), "STATION", ("ROUGH",))

    def test_simple_color_stability_requires_three_nearby_frames(self):
        tracker = SimpleColorStability(required_frames=3, max_center_delta_px=5.0)
        self.assertIsNone(tracker.update("COLOR:1", 10, 10, 10, 10, 1.0))
        self.assertIsNone(tracker.update("COLOR:1", 12, 11, 12, 11, 1.0))
        self.assertIsNotNone(tracker.update("COLOR:1", 13, 12, 13, 12, 1.0))

        self.assertIsNone(tracker.update("COLOR:1", 30, 30, 30, 30, 1.0))
        self.assertEqual(len(tracker.samples), 1)
        tracker.miss()
        self.assertEqual(tracker.samples, [])


if __name__ == "__main__":
    unittest.main()
