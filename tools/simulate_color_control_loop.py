"""Run a camera/serial-free MCU -> color recognition -> DONE protocol loop."""

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

    def maix_send(name, *fields):
        decoded = record("MAIX_TO_JETSON", encode_frame(name, *fields))
        if decoded[0] != "MAIX_QR" or not decoded[1]:
            raise RuntimeError("unexpected simulated Maix frame")
        coordinator.accept_task_code(decoded[1][0])

    def receive_all():
        for name, fields in coordinator.drain():
            record("JETSON_TO_MCU", encode_frame(name, *fields))

    frame = np.zeros(
        (int(config["camera"]["height"]), int(config["camera"]["width"]), 3),
        dtype=np.uint8,
    )
    # A synthetic red material fully inside the configured TURNTABLE ROI.
    cv2.rectangle(frame, (520, 300), (660, 440), (0, 0, 255), -1)

    mcu_send("START", "SIM_RUN")
    receive_all()
    maix_send("MAIX_QR", TASK_CODE, "320", "240", "120", "5", "STABLE")
    receive_all()
    mcu_send("REQ", "SIM001", "PICK", "TURNTABLE", "1")
    receive_all()

    stable_frames = int(config["stability"]["stable_frames"])
    for index in range(stable_frames):
        coordinator.update(frame, index * 0.02)
        receive_all()

    mcu_send("EXEC", "SIM001")
    receive_all()
    mcu_send("DONE", "SIM001", "OK")
    receive_all()

    outbound = [
        item["name"]
        for item in transcript
        if item["direction"] == "JETSON_TO_MCU"
    ]
    expected = [
        "READY",
        "TASK_PLAN",
        "ACCEPTED",
        "GRASP_READY",
        "EXEC_ACK",
        "DONE_ACK",
    ]
    if outbound != expected or coordinator.state != "TASK_READY":
        raise RuntimeError(
            "closed loop failed: outbound=%r state=%s"
            % (outbound, coordinator.state)
        )
    if emit:
        print(
            json.dumps(
                {
                    "state": "SIMULATION_OK",
                    "stable_frames": stable_frames,
                    "coordinator_state": coordinator.state,
                    "outbound": outbound,
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
    return transcript


if __name__ == "__main__":
    run_simulation()
