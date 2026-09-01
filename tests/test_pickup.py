import unittest

from jetson_recognition.engine import RecognitionEngine
from jetson_recognition.model import Measurement
from jetson_recognition.pickup import PickupSession


class SequenceDetector:
    def __init__(self, measurements):
        self.measurements = list(measurements)

    def detect_color(self, frame, color_id, scene):
        if not self.measurements:
            return None
        return self.measurements.pop(0)


def measurement(pixel_x, pixel_y, confidence=0.9, unit="PX"):
    x = pixel_x if unit == "PX" else -12.5
    y = pixel_y if unit == "PX" else 8.0
    return Measurement(
        "COLOR",
        "1",
        x,
        y,
        0.0,
        confidence,
        unit,
        0.0,
        pixel_x,
        pixel_y,
    )


def pickup_config():
    return {
        "capture_zone_px": [50, 50, 100, 100],
        "capture_margin_px": 10,
        "stable_frames": 3,
        "stable_spread_px": 3.0,
        "min_confidence": 0.5,
        "reset_after_misses": 2,
        "motion_window_frames": 3,
        "max_target_speed_px_s": 20.0,
        "low_speed_hold_frames": 2,
        "final_recheck_frames": 2,
        "ready_emit_every_frames": 2,
        "result_valid_ms": 200,
    }


def engine_for(measurements):
    return RecognitionEngine(SequenceDetector(measurements), {})


class PickupSessionTests(unittest.TestCase):
    def test_requires_zone_stability_and_final_recheck(self):
        values = [measurement(100, 100) for _ in range(8)]
        session = PickupSession(engine_for(values), "1", "TURNTABLE", pickup_config())
        states = [session.update(object(), index * 0.1) for index in range(8)]
        self.assertEqual(
            [item.state for item in states],
            [
                "SETTLING",
                "SETTLING",
                "SETTLING",
                "SETTLING",
                "FINAL_CHECK",
                "FINAL_CHECK",
                "GRASP_READY",
                "GRASP_READY",
            ],
        )
        self.assertFalse(states[5].ready)
        self.assertTrue(states[6].ready)
        self.assertTrue(states[6].ready_rising_edge)
        self.assertFalse(states[7].ready_rising_edge)
        self.assertEqual(states[6].valid_for_ms, 200)

    def test_target_outside_inner_zone_cannot_be_ready(self):
        session = PickupSession(
            engine_for([measurement(55, 100)]), "1", "TURNTABLE", pickup_config()
        )
        update = session.update(object(), 0.0)
        self.assertEqual(update.state, "TRACKING")
        self.assertEqual(update.reason, "OUTSIDE_CAPTURE_ZONE")

    def test_moving_target_resets_confirmation(self):
        values = [measurement(80, 100), measurement(90, 100)]
        session = PickupSession(engine_for(values), "1", "TURNTABLE", pickup_config())
        self.assertEqual(session.update(object(), 0.0).state, "SETTLING")
        update = session.update(object(), 0.1)
        self.assertEqual(update.state, "MOVING")
        self.assertGreater(update.speed_px_s, 20.0)

    def test_raw_pixels_are_used_after_metric_calibration(self):
        values = [measurement(100, 100, unit="MM") for _ in range(8)]
        session = PickupSession(engine_for(values), "1", "TURNTABLE", pickup_config())
        update = None
        for index in range(8):
            update = session.update(object(), index * 0.1)
        self.assertEqual(update.state, "GRASP_READY")
        self.assertEqual(update.measurement.unit, "MM")
        self.assertEqual(update.measurement.pixel_x, 100)

    def test_missing_target_breaks_final_recheck(self):
        # Walk into FINAL_CHECK, then lose the target: the recheck must
        # restart from the low-speed hold once the target is seen again.
        values = (
            [measurement(100, 100) for _ in range(6)]
            + [None]
            + [measurement(100, 100) for _ in range(4)]
        )
        session = PickupSession(engine_for(values), "1", "TURNTABLE", pickup_config())
        updates = [session.update(object(), index * 0.1) for index in range(11)]
        self.assertEqual(updates[5].state, "FINAL_CHECK")
        self.assertEqual(updates[6].state, "SEARCHING")
        self.assertEqual(updates[6].reason, "TARGET_LOST_TEMPORARILY")
        self.assertEqual(updates[7].state, "SETTLING")
        self.assertEqual(updates[7].reason, "TARGET_SLOWING")
        self.assertFalse(updates[7].ready)
        self.assertFalse(updates[8].ready)

    def test_ready_grant_revoked_after_target_loss(self):
        # Once GRASP_READY is granted, a single missed frame must revoke it.
        values = (
            [measurement(100, 100) for _ in range(7)]
            + [None]
            + [measurement(100, 100) for _ in range(3)]
        )
        session = PickupSession(engine_for(values), "1", "TURNTABLE", pickup_config())
        updates = [session.update(object(), index * 0.1) for index in range(11)]
        self.assertEqual(updates[6].state, "GRASP_READY")
        self.assertTrue(updates[6].ready)
        self.assertTrue(updates[6].ready_rising_edge)
        self.assertEqual(updates[7].state, "SEARCHING")
        self.assertEqual(updates[7].reason, "TARGET_LOST_AFTER_READY")
        self.assertFalse(updates[7].ready)

    def test_low_speed_hold_requires_consecutive_frames(self):
        # Speed below the limit but not held long enough: no accumulation.
        config = pickup_config()
        config["low_speed_hold_frames"] = 3
        session = PickupSession(
            engine_for([measurement(100, 100) for _ in range(4)]),
            "1",
            "TURNTABLE",
            config,
        )
        updates = [session.update(object(), index * 0.1) for index in range(4)]
        self.assertEqual(
            [item.reason for item in updates],
            [
                "TARGET_SLOWING",
                "TARGET_SLOWING",
                "TARGET_SLOWING",
                "BUILDING_STABILITY",
            ],
        )

    def test_temporary_loss_keeps_accumulation(self):
        # A single missed frame before authorisation does not wipe the
        # accumulated samples; recovery resumes instead of restarting.
        config = pickup_config()
        config["low_speed_hold_frames"] = 1
        values = [
            measurement(100, 100),
            measurement(100, 100),
            None,
            measurement(100, 100),
            measurement(100, 100),
            measurement(100, 100),
        ]
        session = PickupSession(engine_for(values), "1", "TURNTABLE", config)
        updates = [session.update(object(), index * 0.1) for index in range(6)]
        self.assertEqual(updates[1].stable_count, 1)
        self.assertEqual(updates[2].state, "SEARCHING")
        # Recovery needs one low-speed hold frame, then resumes accumulation
        # from the retained sample (1 -> 2) instead of restarting from zero.
        self.assertEqual(updates[3].state, "SETTLING")
        self.assertEqual(updates[3].reason, "TARGET_SLOWING")
        self.assertEqual(updates[4].stable_count, 2)
        self.assertEqual(updates[5].state, "FINAL_CHECK")

    def test_unconfigured_zone_is_rejected(self):
        config = pickup_config()
        config["capture_zone_px"] = None
        with self.assertRaises(ValueError):
            PickupSession(engine_for([]), "1", "TURNTABLE", config)


if __name__ == "__main__":
    unittest.main()
