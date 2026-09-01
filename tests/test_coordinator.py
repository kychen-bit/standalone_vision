import unittest

from jetson_recognition.coordinator import VisionCoordinator
from jetson_recognition.engine import RecognitionEngine
from jetson_recognition.model import Measurement
from jetson_recognition.protocol import encode_frame


def runtime_config():
    return {
        "stable_frames": 5,
        "stable_spread_px": 3.0,
        "stable_spread_mm": 2.0,
        "stable_yaw_deg": 0.8,
        "min_confidence": 0.4,
        "reset_after_misses": 2,
        "color_stability": {
            "stable_frames": 5,
            "max_center_delta_px": 6.0,
        },
    }


def coord_config():
    return {
        "scene": "TURNTABLE",
        "timeout_s": 10.0,
        "action_timeout_s": 20.0,
        "result_valid_ms": 250,
        "revoke_after_misses": 2,
        "require_exec_ack": True,
        "max_attempts": 2,
        "target_report_every": 0,
    }


def measurement(
    pixel_x, pixel_y, confidence=0.9, unit="PX", kind="COLOR", target="1"
):
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
        sequence = self.sequences.get(key, [])
        index = self.index.get(key, 0)
        if index >= len(sequence):
            return None
        self.index[key] = index + 1
        return sequence[index]

    def detect_color(self, frame, color_id, scene):
        return self._next(("COLOR", str(color_id)))

    def detect_ring(self, frame, ring_id, scene):
        return self._next(("RING", str(ring_id)))

    def detect_stack(self, frame, color_id, scene, ring_id):
        return self._next(("STACK", str(color_id)))


def fresh_coordinator(sequences=None, overrides=None):
    detector = FakeDetector(sequences or {})
    engine = RecognitionEngine(detector, runtime_config())
    config = coord_config()
    config.update(overrides or {})
    return detector, engine, VisionCoordinator(engine, config)


TASK = "156+123+516+231"


def start_with_task(coordinator):
    coordinator.on_mcu_frame("START", ["RUN001"])
    ready = coordinator.drain()
    coordinator.accept_task_code(TASK)
    plan = coordinator.drain()
    return ready, plan


def feed_frames(coordinator, count=5, start=0.0, step=0.02):
    for index in range(count):
        coordinator.update(object(), start + index * step)
    return coordinator.drain()


class CoordinatorTests(unittest.TestCase):
    def test_start_and_task_plan_are_serial_encodable(self):
        _, _, coordinator = fresh_coordinator()
        ready, plan = start_with_task(coordinator)
        self.assertEqual(ready, [("READY", ["RUN001"])])
        self.assertEqual(
            plan,
            [("TASK_PLAN", [TASK, "1", "5", "6", "5", "1", "6"])],
        )
        for name, fields in ready + plan:
            self.assertTrue(encode_frame(name, *fields).startswith(b"@"))
        self.assertEqual(coordinator.state, "TASK_READY")

    def test_bad_task_code_error_has_no_empty_serial_field(self):
        _, _, coordinator = fresh_coordinator()
        coordinator.on_mcu_frame("START", ["RUN001"])
        coordinator.drain()
        coordinator.accept_task_code("111+123+456+231")
        frames = coordinator.drain()
        self.assertEqual(frames, [("ERROR", ["NONE", "BAD_TASK_CODE"])])
        encode_frame(frames[0][0], *frames[0][1])

    def test_pick_grant_requires_five_frames_and_exec(self):
        reds = [measurement(100, 100) for _ in range(12)]
        _, _, coordinator = fresh_coordinator({("COLOR", "1"): reds})
        start_with_task(coordinator)
        coordinator.on_mcu_frame("REQ", ["001", "PICK"])
        self.assertEqual(coordinator.drain(), [("ACCEPTED", ["001", "PICK"])])
        self.assertEqual(coordinator.queue_index, 0)
        self.assertEqual(feed_frames(coordinator, 4), [])
        frames = feed_frames(coordinator, 1, start=0.08)
        self.assertEqual(len(frames), 1)
        name, fields = frames[0]
        self.assertEqual(name, "GRASP_READY")
        self.assertEqual(fields[0:2], ["001", "1"])
        self.assertEqual(fields[8], "5")
        self.assertEqual(fields[9], "250")
        encode_frame(name, *fields)
        self.assertEqual(coordinator.state, "WAIT_EXEC")
        coordinator.on_mcu_frame("EXEC", ["001"])
        self.assertEqual(coordinator.drain(), [("EXEC_ACK", ["001"])])
        self.assertEqual(coordinator.state, "WAIT_DONE")

    def test_done_ok_advances_auto_queue_only_after_exec(self):
        reds = [measurement(100, 100) for _ in range(12)]
        detector, _, coordinator = fresh_coordinator({("COLOR", "1"): reds})
        start_with_task(coordinator)
        coordinator.on_mcu_frame("REQ", ["001", "PICK"])
        coordinator.drain()
        feed_frames(coordinator)
        coordinator.on_mcu_frame("DONE", ["001", "OK"])
        self.assertEqual(
            coordinator.drain(),
            [("ERROR", ["001", "EXEC_REQUIRED", "WAIT_EXEC"])],
        )
        self.assertEqual(coordinator.queue_index, 0)
        coordinator.on_mcu_frame("EXEC", ["001"])
        coordinator.drain()
        coordinator.on_mcu_frame("DONE", ["001", "OK"])
        self.assertEqual(coordinator.drain(), [("DONE_ACK", ["001", "OK"])])
        self.assertEqual(coordinator.queue_index, 1)
        detector.sequences[("COLOR", "5")] = [
            measurement(100, 100, target="5") for _ in range(8)
        ]
        coordinator.on_mcu_frame("REQ", ["002", "PICK"])
        self.assertEqual(coordinator.drain(), [("ACCEPTED", ["002", "PICK"])])
        self.assertEqual(coordinator.target, "5")

    def test_done_fail_retries_same_queue_target(self):
        reds = [measurement(100, 100) for _ in range(20)]
        detector, _, coordinator = fresh_coordinator({("COLOR", "1"): reds})
        start_with_task(coordinator)
        coordinator.on_mcu_frame("REQ", ["001", "PICK"])
        coordinator.drain()
        feed_frames(coordinator)
        coordinator.on_mcu_frame("EXEC", ["001"])
        coordinator.drain()
        coordinator.on_mcu_frame("DONE", ["001", "FAIL"])
        self.assertEqual(coordinator.drain(), [("DONE_ACK", ["001", "RETRY"])])
        self.assertEqual(coordinator.state, "BUSY")
        self.assertEqual(coordinator.target, "1")
        self.assertEqual(coordinator.queue_index, 0)
        detector.reset(("COLOR", "1"))
        frames = feed_frames(coordinator)
        self.assertEqual(frames[0][0], "GRASP_READY")

    def test_explicit_color_never_consumes_auto_queue(self):
        blacks = [measurement(100, 100, target="5") for _ in range(8)]
        _, _, coordinator = fresh_coordinator({("COLOR", "5"): blacks})
        start_with_task(coordinator)
        coordinator.on_mcu_frame("REQ", ["005", "PICK", "TURNTABLE", "5"])
        coordinator.drain()
        feed_frames(coordinator)
        coordinator.on_mcu_frame("EXEC", ["005"])
        coordinator.drain()
        coordinator.on_mcu_frame("DONE", ["005", "OK"])
        self.assertEqual(coordinator.drain(), [("DONE_ACK", ["005", "OK"])])
        self.assertEqual(coordinator.queue_index, 0)

    def test_search_timeout_does_not_consume_queue(self):
        _, _, coordinator = fresh_coordinator()
        start_with_task(coordinator)
        coordinator.on_mcu_frame("REQ", ["001", "PICK"])
        coordinator.drain()
        coordinator.update(object(), 0.0)
        coordinator.update(object(), 11.0)
        self.assertEqual(coordinator.drain(), [("ERROR", ["001", "TIMEOUT"])])
        self.assertEqual(coordinator.state, "TASK_READY")
        self.assertEqual(coordinator.queue_index, 0)

    def test_grasp_is_revoked_after_two_missing_frames(self):
        sequence = [measurement(100, 100) for _ in range(5)] + [None, None]
        _, _, coordinator = fresh_coordinator({("COLOR", "1"): sequence})
        start_with_task(coordinator)
        coordinator.on_mcu_frame("REQ", ["001", "PICK"])
        coordinator.drain()
        feed_frames(coordinator)
        coordinator.update(object(), 0.10)
        self.assertEqual(coordinator.drain(), [])
        coordinator.update(object(), 0.12)
        self.assertEqual(
            coordinator.drain(), [("GRASP_REVOKED", ["001", "LOST"])]
        )
        self.assertEqual(coordinator.state, "BUSY")

    def test_grasp_expires_without_exec(self):
        reds = [measurement(100, 100) for _ in range(8)]
        _, _, coordinator = fresh_coordinator({("COLOR", "1"): reds})
        start_with_task(coordinator)
        coordinator.on_mcu_frame("REQ", ["001", "PICK"])
        coordinator.drain()
        feed_frames(coordinator)
        coordinator.update(object(), 0.40)
        self.assertEqual(
            coordinator.drain(), [("GRASP_REVOKED", ["001", "EXPIRED"])]
        )
        self.assertEqual(coordinator.state, "BUSY")

    def test_new_request_cannot_overwrite_waiting_action(self):
        reds = [measurement(100, 100) for _ in range(8)]
        _, _, coordinator = fresh_coordinator({("COLOR", "1"): reds})
        start_with_task(coordinator)
        coordinator.on_mcu_frame("REQ", ["001", "PICK"])
        coordinator.drain()
        feed_frames(coordinator)
        coordinator.on_mcu_frame("REQ", ["002", "PICK", "TURNTABLE", "5"])
        self.assertEqual(
            coordinator.drain(),
            [("ERROR", ["002", "BAD_STATE", "WAIT_EXEC"])],
        )
        self.assertEqual(coordinator.seq, "001")

    def test_place_alignment_uses_common_state_gate(self):
        rings = [measurement(100, 100, kind="RING", target="2") for _ in range(8)]
        _, _, coordinator = fresh_coordinator({("RING", "2"): rings})
        start_with_task(coordinator)
        coordinator.on_mcu_frame("REQ", ["003", "PLACE", "ROUGH", "2"])
        self.assertEqual(coordinator.drain(), [("ACCEPTED", ["003", "PLACE"])])
        frames = feed_frames(coordinator)
        self.assertEqual(frames[0][0], "ALIGN_READY")
        self.assertEqual(frames[0][1][-2:], ["0.900", "5"])
        encode_frame(frames[0][0], *frames[0][1])
        self.assertEqual(coordinator.state, "WAIT_DONE")

    def test_abort_resets_and_acknowledges(self):
        _, _, coordinator = fresh_coordinator()
        start_with_task(coordinator)
        coordinator.on_mcu_frame("REQ", ["004", "PICK", "TURNTABLE", "1"])
        coordinator.drain()
        coordinator.on_mcu_frame("ABORT", ["RUN001"])
        self.assertEqual(coordinator.drain(), [("ABORTED", ["RUN001"])])
        self.assertEqual(coordinator.state, "IDLE")
        self.assertEqual(coordinator.queue, [])

    def test_start_requires_nonempty_run_id(self):
        _, _, coordinator = fresh_coordinator()
        coordinator.on_mcu_frame("START", [])
        self.assertEqual(coordinator.drain(), [("ERROR", ["NONE", "BAD_START"])])
        self.assertEqual(coordinator.state, "IDLE")


if __name__ == "__main__":
    unittest.main()
