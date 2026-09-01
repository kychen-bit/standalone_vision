import unittest

from jetson_recognition.coordinator import VisionCoordinator
from jetson_recognition.engine import RecognitionEngine
from jetson_recognition.model import Measurement


def runtime_config():
    return {
        "stable_frames": 3,
        "stable_spread_px": 3.0,
        "stable_spread_mm": 2.0,
        "stable_yaw_deg": 0.8,
        "min_confidence": 0.4,
        "reset_after_misses": 2,
    }


def coord_config():
    return {
        "scene": "TURNTABLE",
        "timeout_s": 10.0,
        "max_attempts": 2,
        "target_report_every": 0,
    }


def measurement(pixel_x, pixel_y, confidence=0.9, unit="PX", kind="COLOR", target="1"):
    x = pixel_x if unit == "PX" else -12.5
    y = pixel_y if unit == "PX" else 8.0
    return Measurement(
        kind, target, x, y, 0.0, confidence, unit, 0.0, pixel_x, pixel_y
    )


class FakeDetector:
    def __init__(self, sequences):
        self.sequences = dict(sequences)
        self.index = {}

    def reset(self, key):
        self.index[key] = 0

    def _next(self, key):
        seq = self.sequences.get(key, [])
        i = self.index.get(key, 0)
        if i >= len(seq):
            return None
        self.index[key] = i + 1
        return seq[i]

    def detect_color(self, frame, color_id, scene):
        return self._next(("COLOR", str(color_id)))

    def detect_ring(self, frame, ring_id, scene):
        return self._next(("RING", str(ring_id)))

    def detect_stack(self, frame, color_id, scene, ring_id):
        return self._next(("STACK", str(color_id)))


def fresh_coordinator(sequences=None):
    detector = FakeDetector(sequences or {})
    engine = RecognitionEngine(detector, runtime_config())
    return detector, engine, VisionCoordinator(engine, coord_config())


TASK = "156+123+516+231"


class CoordinatorTests(unittest.TestCase):
    def test_start_and_task_plan(self):
        _, _, coord = fresh_coordinator()
        coord.on_mcu_frame("START", ["RUN001"])
        self.assertEqual(coord.drain(), [("READY", ["RUN001"])])
        self.assertEqual(coord.state, "WAIT_TASK")
        coord.accept_task_code(TASK)
        frames = coord.drain()
        self.assertEqual(frames[0][0], "TASK_PLAN")
        self.assertEqual(frames[0][1][0], TASK)
        self.assertEqual(frames[0][1][1], "1,5,6,5,1,6")
        self.assertEqual(coord.state, "TASK_READY")

    def test_bad_task_code_is_rejected(self):
        _, _, coord = fresh_coordinator()
        coord.accept_task_code("111+123+456+231")
        self.assertEqual(coord.drain(), [("ERROR", ["", "BAD_TASK_CODE"])])

    def test_req_pick_uses_queue_and_grants(self):
        reds = [measurement(100, 100) for _ in range(8)]
        detector, _, coord = fresh_coordinator({("COLOR", "1"): reds})
        coord.on_mcu_frame("START", ["R"])
        coord.drain()
        coord.accept_task_code(TASK)
        coord.drain()
        coord.on_mcu_frame("REQ", ["001", "PICK"])
        self.assertEqual(coord.drain(), [("ACCEPTED", ["001", "PICK"])])
        self.assertEqual(coord.target, "1")
        for index in range(8):
            coord.update(object(), index * 0.1)
        frames = coord.drain()
        granted = [frame for frame in frames if frame[0] == "GRASP_READY"]
        self.assertEqual(len(granted), 1)
        self.assertEqual(granted[0][1][0], "001")
        self.assertEqual(granted[0][1][1], "1")
        self.assertEqual(coord.state, "WAIT_DONE")

    def test_done_ok_advances_queue(self):
        reds = [measurement(100, 100) for _ in range(8)]
        detector, _, coord = fresh_coordinator({("COLOR", "1"): reds})
        coord.on_mcu_frame("START", ["R"])
        coord.drain()
        coord.accept_task_code(TASK)
        coord.drain()
        coord.on_mcu_frame("REQ", ["001", "PICK"])
        coord.drain()
        for index in range(8):
            coord.update(object(), index * 0.1)
        coord.drain()
        coord.on_mcu_frame("DONE", ["001", "OK"])
        self.assertEqual(coord.drain(), [("DONE_ACK", ["001", "OK"])])
        self.assertEqual(coord.state, "TASK_READY")
        # next auto target is the second color of batch one: black (5)
        detector.sequences[("COLOR", "5")] = [measurement(100, 100) for _ in range(8)]
        coord.on_mcu_frame("REQ", ["002", "PICK"])
        self.assertEqual(coord.drain(), [("ACCEPTED", ["002", "PICK"])])
        self.assertEqual(coord.target, "5")

    def test_done_fail_retries_same_target(self):
        reds = [measurement(100, 100) for _ in range(8)]
        detector, _, coord = fresh_coordinator({("COLOR", "1"): reds})
        coord.on_mcu_frame("START", ["R"])
        coord.drain()
        coord.accept_task_code(TASK)
        coord.drain()
        coord.on_mcu_frame("REQ", ["001", "PICK"])
        coord.drain()
        for index in range(8):
            coord.update(object(), index * 0.1)
        coord.drain()
        coord.on_mcu_frame("DONE", ["001", "FAIL"])
        self.assertEqual(coord.drain(), [("DONE_ACK", ["001", "RETRY"])])
        self.assertEqual(coord.state, "BUSY")
        self.assertEqual(coord.target, "1")
        # retry still grants the same target on the second pass
        detector.reset(("COLOR", "1"))
        for index in range(8):
            coord.update(object(), index * 0.1)
        frames = coord.drain()
        granted = [frame for frame in frames if frame[0] == "GRASP_READY"]
        self.assertEqual(len(granted), 1)
        self.assertEqual(granted[0][1][1], "1")

    def test_explicit_color_does_not_consume_queue(self):
        _, _, coord = fresh_coordinator()
        coord.on_mcu_frame("START", ["R"])
        coord.drain()
        coord.accept_task_code(TASK)
        coord.drain()
        coord.on_mcu_frame("REQ", ["005", "PICK", "TURNTABLE", "5"])
        self.assertEqual(coord.drain(), [("ACCEPTED", ["005", "PICK"])])
        self.assertEqual(coord.target, "5")
        self.assertEqual(coord.queue_index, 0)

    def test_timeout_emits_error(self):
        _, _, coord = fresh_coordinator()
        coord.on_mcu_frame("START", ["R"])
        coord.drain()
        coord.accept_task_code(TASK)
        coord.drain()
        coord.on_mcu_frame("REQ", ["001", "PICK"])
        coord.drain()
        coord.update(object(), 0.0)
        coord.drain()
        coord.update(object(), 11.0)
        self.assertIn(("ERROR", ["001", "TIMEOUT"]), coord.drain())
        self.assertEqual(coord.state, "TASK_READY")

    def test_req_place_aligns_ring(self):
        ring = measurement(100, 100, kind="RING", target="2")
        _, _, coord = fresh_coordinator({("RING", "2"): [ring] * 6})
        coord.on_mcu_frame("START", ["R"])
        coord.drain()
        coord.on_mcu_frame("REQ", ["003", "PLACE", "ROUGH", "2"])
        self.assertEqual(coord.drain(), [("ACCEPTED", ["003", "PLACE"])])
        self.assertEqual(coord.session is not None, True)
        for index in range(6):
            coord.update(object(), index * 0.1)
        frames = coord.drain()
        self.assertIn(
            ("ALIGN_READY", ["003", "PLACE", "2", "100.000", "100.000", "0.000", "PX"]),
            frames,
        )
        self.assertEqual(coord.state, "WAIT_DONE")

    def test_abort_resets(self):
        _, _, coord = fresh_coordinator()
        coord.on_mcu_frame("START", ["R"])
        coord.drain()
        coord.on_mcu_frame("REQ", ["004", "PICK", "TURNTABLE", "1"])
        coord.drain()
        self.assertEqual(coord.state, "BUSY")
        coord.on_mcu_frame("ABORT", ["R"])
        self.assertEqual(coord.state, "IDLE")
        self.assertIsNone(coord.seq)
        self.assertIsNone(coord.target)

    def test_stale_done_is_ignored(self):
        _, _, coord = fresh_coordinator()
        coord.on_mcu_frame("DONE", ["999", "OK"])
        self.assertEqual(coord.drain(), [])


if __name__ == "__main__":
    unittest.main()
