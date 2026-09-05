"""Simulate the complete two-batch competition flow without hardware.

The six materials pass through every vision stage from the rules:

* six pickups from the turntable;
* six placements and six pickups at the rough-processing station;
* three first-batch placements at storage;
* three second-batch same-color STACK alignments at storage.

The real detector, stability windows, CRC framing, task parser and coordinator
state machine are used. Camera images and MCU/Maix transports are synthetic.
"""

import json
from pathlib import Path
import sys

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jetson_recognition.coordinator import FrameBuffer, VisionCoordinator
from jetson_recognition.protocol import decode_frame, encode_frame
from jetson_recognition.run import build_engine, load_config
from jetson_recognition.task_code import decode_task_code
from tools.simulate_ring_control_loop import draw_numbered_ring


TASK_CODE = "156+123+516+231"
ALTERNATE_TASK_CODE = "426+213+436+231"
SCREEN_BGR = {
    "1": (0, 0, 255),
    "2": (0, 255, 255),
    "3": (255, 0, 0),
    "4": (0, 255, 0),
    "5": (20, 20, 20),
    "6": (255, 255, 0),
}


def _color_frame(config, color_id, center=(640, 370)):
    height = int(config["camera"]["height"])
    width = int(config["camera"]["width"])
    background = 255 if str(color_id) == "5" else 0
    frame = np.full((height, width, 3), background, dtype=np.uint8)
    half_size = 60
    cv2.rectangle(
        frame,
        (center[0] - half_size, center[1] - half_size),
        (center[0] + half_size, center[1] + half_size),
        SCREEN_BGR[str(color_id)],
        -1,
    )
    return frame


def _ring_frame(config):
    frame = np.full(
        (int(config["camera"]["height"]), int(config["camera"]["width"]), 3),
        200,
        dtype=np.uint8,
    )
    for center_x, digit in ((280, "1"), (640, "2"), (1000, "3")):
        draw_numbered_ring(frame, (center_x, 370), digit)
    return frame


def run_simulation(emit=True):
    config = load_config("config/jetson.json")
    engine = build_engine(config)
    coordinator = VisionCoordinator(engine, config["coordinator"])
    task = decode_task_code(TASK_CODE)
    transcript = []
    completed_actions = []
    clock_s = [0.0]

    def record(direction, wire):
        decoded = decode_frame(wire)
        if decoded is None:
            raise RuntimeError("simulation produced an invalid CRC frame")
        item = {
            "direction": direction,
            "wire": wire.decode("ascii").strip(),
            "name": decoded[0],
            "fields": decoded[1],
        }
        transcript.append(item)
        if emit:
            print(json.dumps(item, ensure_ascii=False), flush=True)
        return decoded

    def mcu_send(name, *fields):
        decoded = record("MCU_TO_JETSON", encode_frame(name, *fields))
        coordinator.on_mcu_frame(*decoded)

    def maix_tcp_send(payload):
        wire = encode_frame(
            "MAIX_QR", payload, "320", "240", "120", "4", "STABLE"
        )
        parser = FrameBuffer()
        frames = parser.feed(wire[:11]) + parser.feed(wire[11:])
        if len(frames) != 1:
            raise RuntimeError("simulated TCP stream did not produce one frame")
        decoded = record("MAIX_TCP_TO_JETSON", wire)
        return coordinator.accept_task_code(decoded[1][0])

    def receive_all():
        received = []
        for name, fields in coordinator.drain():
            record("JETSON_TO_MCU", encode_frame(name, *fields))
            received.append((name, fields))
        return received

    def expect_one(received, expected_name, expected_target=None):
        matches = [item for item in received if item[0] == expected_name]
        if len(matches) != 1:
            raise RuntimeError(
                "expected one %s, received %r" % (expected_name, received)
            )
        fields = matches[0][1]
        if expected_target is not None:
            target_index = 1 if expected_name == "GRASP_READY" else 2
            if fields[target_index] != str(expected_target):
                raise RuntimeError(
                    "%s returned target %s, expected %s"
                    % (expected_name, fields[target_index], expected_target)
                )
        return fields

    def feed_until_stable(frame, mode):
        if mode == "RING":
            stability = config["ring_detection"]["stability"]
        elif mode == "COLOR":
            stability = config["stability"]
        else:
            stability = config["runtime"]
        required = int(stability["stable_frames"])
        duration_frames = int(
            float(stability.get("min_stable_time_ms", 0)) // 20.0
        ) + 2
        update_count = max(required, duration_frames)
        received = []
        for _index in range(update_count):
            coordinator.update(frame, clock_s[0])
            clock_s[0] += 0.02
            received.extend(receive_all())
        return received

    def perform_pick(seq, color_id, scene, use_task_queue):
        fields = [seq, "PICK", scene]
        if not use_task_queue:
            fields.append(color_id)
        mcu_send("REQ", *fields)
        expect_one(receive_all(), "ACCEPTED")
        ready = feed_until_stable(_color_frame(config, color_id), "COLOR")
        expect_one(ready, "GRASP_READY", color_id)
        mcu_send("EXEC", seq)
        expect_one(receive_all(), "EXEC_ACK")
        mcu_send("DONE", seq, "OK")
        expect_one(receive_all(), "DONE_ACK")
        completed_actions.append((seq, "PICK", scene, color_id))

    def perform_place(seq, ring_id, scene):
        mcu_send("REQ", seq, "PLACE", scene, ring_id)
        expect_one(receive_all(), "ACCEPTED")
        ready = feed_until_stable(_ring_frame(config), "RING")
        expect_one(ready, "ALIGN_READY", ring_id)
        mcu_send("DONE", seq, "OK")
        expect_one(receive_all(), "DONE_ACK")
        completed_actions.append((seq, "PLACE", scene, ring_id))

    def perform_stack(seq, color_id, storage_ring_id):
        mcu_send("REQ", seq, "STACK", "STORAGE", color_id, storage_ring_id)
        expect_one(receive_all(), "ACCEPTED")
        # Deliberately place it away from the legacy fixed ring box. A wrist
        # camera must search for the same-color first-batch material.
        ready = feed_until_stable(
            _color_frame(config, color_id, center=(760, 370)), "STACK"
        )
        expect_one(ready, "ALIGN_READY", color_id)
        mcu_send("DONE", seq, "OK")
        expect_one(receive_all(), "DONE_ACK")
        completed_actions.append(
            (seq, "STACK", "STORAGE", color_id, storage_ring_id)
        )

    mcu_send("START", "SIM_FULL")
    expect_one(receive_all(), "READY")
    if maix_tcp_send(TASK_CODE) != "ACCEPTED":
        raise RuntimeError("first task code was not accepted")
    expect_one(receive_all(), "TASK_PLAN")
    # Reproduce the real run that previously replaced one plan with another.
    if maix_tcp_send(ALTERNATE_TASK_CODE) != "LOCKED":
        raise RuntimeError("different task code did not hit the task lock")
    if receive_all() or coordinator.task_raw != TASK_CODE:
        raise RuntimeError("locked task plan was modified")

    first_batch = task["batches"][0]
    second_batch = task["batches"][1]
    storage_ring_by_color = {
        item["color_id"]: item["ring_id"] for item in first_batch
    }

    # First batch: turntable -> robot -> rough -> robot -> storage.
    for index, item in enumerate(first_batch, 1):
        perform_pick("B1_TP_PICK_%d" % index, item["color_id"], "TURNTABLE", True)
    for index, item in enumerate(first_batch, 1):
        perform_place("B1_ROUGH_PLACE_%d" % index, item["ring_id"], "ROUGH")
    for index, item in enumerate(first_batch, 1):
        perform_pick("B1_ROUGH_PICK_%d" % index, item["color_id"], "ROUGH", False)
    for index, item in enumerate(first_batch, 1):
        perform_place("B1_STORAGE_PLACE_%d" % index, item["ring_id"], "STORAGE")

    # Second batch follows the same path, then stacks onto the first batch's
    # same-color material (storage ring comes from the first batch mapping).
    for index, item in enumerate(second_batch, 1):
        perform_pick("B2_TP_PICK_%d" % index, item["color_id"], "TURNTABLE", True)
    for index, item in enumerate(second_batch, 1):
        perform_place("B2_ROUGH_PLACE_%d" % index, item["ring_id"], "ROUGH")
    for index, item in enumerate(second_batch, 1):
        perform_pick("B2_ROUGH_PICK_%d" % index, item["color_id"], "ROUGH", False)
    for index, item in enumerate(second_batch, 1):
        perform_stack(
            "B2_STORAGE_STACK_%d" % index,
            item["color_id"],
            storage_ring_by_color[item["color_id"]],
        )

    outbound = [
        item for item in transcript if item["direction"] == "JETSON_TO_MCU"
    ]
    counts = {
        name: sum(item["name"] == name for item in outbound)
        for name in (
            "TASK_PLAN",
            "ACCEPTED",
            "GRASP_READY",
            "EXEC_ACK",
            "ALIGN_READY",
            "DONE_ACK",
        )
    }
    expected_counts = {
        "TASK_PLAN": 1,
        "ACCEPTED": 24,
        "GRASP_READY": 12,
        "EXEC_ACK": 12,
        "ALIGN_READY": 12,
        "DONE_ACK": 24,
    }
    if (
        counts != expected_counts
        or len(completed_actions) != 24
        or coordinator.queue_index != 6
        or coordinator.state != "TASK_READY"
    ):
        raise RuntimeError(
            "full competition loop failed: counts=%r actions=%d queue=%d state=%s"
            % (
                counts,
                len(completed_actions),
                coordinator.queue_index,
                coordinator.state,
            )
        )
    if emit:
        print(
            json.dumps(
                {
                    "state": "FULL_COMPETITION_SIMULATION_OK",
                    "task_code": TASK_CODE,
                    "materials": 6,
                    "vision_actions": len(completed_actions),
                    "pick_results": counts["GRASP_READY"],
                    "align_results": counts["ALIGN_READY"],
                    "stack_results": 3,
                    "queue_index": coordinator.queue_index,
                    "coordinator_state": coordinator.state,
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
    return transcript


if __name__ == "__main__":
    run_simulation()
