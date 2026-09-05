"""Run an MCU -> requested ring digit -> ALIGN_READY -> DONE CRC loop."""

import json
from pathlib import Path
import sys

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jetson_recognition.coordinator import VisionCoordinator
from jetson_recognition.protocol import decode_frame, encode_frame
from jetson_recognition.run import build_engine, load_config


TASK_CODE = "156+123+516+231"


def draw_numbered_ring(frame, center, digit):
    cv2.circle(frame, center, 55, (20, 20, 20), 8, cv2.LINE_AA)
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 1.5
    thickness = 4
    (width, height), _baseline = cv2.getTextSize(
        str(digit), font, scale, thickness
    )
    cv2.putText(
        frame,
        str(digit),
        (center[0] - width // 2, center[1] + height // 2),
        font,
        scale,
        (20, 20, 20),
        thickness,
        cv2.LINE_AA,
    )


def run_simulation(emit=True):
    config = load_config("config/jetson.json")
    engine = build_engine(config)
    coordinator = VisionCoordinator(engine, config["coordinator"])
    transcript = []

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

    def receive_all():
        for name, fields in coordinator.drain():
            record("JETSON_TO_MCU", encode_frame(name, *fields))

    frame = np.full(
        (int(config["camera"]["height"]), int(config["camera"]["width"]), 3),
        200,
        dtype=np.uint8,
    )
    # Deliberately swap positions: the requested digit 2 is on the right.
    draw_numbered_ring(frame, (280, 370), "3")
    draw_numbered_ring(frame, (640, 370), "1")
    draw_numbered_ring(frame, (1000, 370), "2")

    mcu_send("START", "SIM_RING_RUN")
    receive_all()
    coordinator.accept_task_code(TASK_CODE)
    receive_all()
    mcu_send("REQ", "RING001", "PLACE", "ROUGH", "2")
    receive_all()

    stable_frames = int(
        config["ring_detection"]["stability"]["stable_frames"]
    )
    duration_frames = int(
        float(
            config["ring_detection"]["stability"].get(
                "min_stable_time_ms", 0
            )
        )
        // 20.0
    ) + 2
    update_count = max(stable_frames, duration_frames)
    for index in range(update_count):
        coordinator.update(frame, index * 0.02)
        receive_all()

    mcu_send("DONE", "RING001", "OK")
    receive_all()

    outbound = [
        item for item in transcript if item["direction"] == "JETSON_TO_MCU"
    ]
    names = [item["name"] for item in outbound]
    expected = ["READY", "TASK_PLAN", "ACCEPTED", "ALIGN_READY", "DONE_ACK"]
    alignment = next(
        (item for item in outbound if item["name"] == "ALIGN_READY"), None
    )
    if (
        names != expected
        or alignment is None
        or alignment["fields"][2] != "2"
        or abs(float(alignment["fields"][3]) - 1000.0) > 3.0
        or coordinator.state != "TASK_READY"
    ):
        raise RuntimeError(
            "ring loop failed: outbound=%r alignment=%r state=%s"
            % (names, alignment, coordinator.state)
        )
    if emit:
        print(
            json.dumps(
                {
                    "state": "RING_SIMULATION_OK",
                    "requested_ring_id": "2",
                    "detected_center_x": float(alignment["fields"][3]),
                    "required_stable_frames": stable_frames,
                    "min_stable_time_ms": config["ring_detection"][
                        "stability"
                    ].get("min_stable_time_ms", 0),
                    "coordinator_state": coordinator.state,
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
    return transcript


if __name__ == "__main__":
    run_simulation()
