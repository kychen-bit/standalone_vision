"""Console and optional serial adapters for observable recognition results."""

import json
import time

from .protocol import encode_frame


def measurement_dict(measurement, state, stable_frames, color_names=None):
    result = {
        "time_ms": int(time.time() * 1000),
        "state": state,
        "kind": measurement.kind,
        "target_id": measurement.target_id,
        "x": round(float(measurement.x), 3),
        "y": round(float(measurement.y), 3),
        "yaw": round(float(measurement.yaw), 3),
        "confidence": round(float(measurement.confidence), 3),
        "unit": measurement.unit,
        "residual": round(float(measurement.residual), 3),
        "stable_frames": int(stable_frames),
    }
    if measurement.kind == "COLOR" and color_names:
        result["color"] = color_names.get(
            str(measurement.target_id), str(measurement.target_id)
        )
        result["center_x"] = result["x"]
        result["center_y"] = result["y"]
    return result


class ResultOutput:
    """Always prints JSON; opens serial only when a device is explicitly supplied."""

    def __init__(self, serial_device=None, baud=115200, color_names=None):
        self.serial = None
        self.color_names = dict(color_names or {})
        if serial_device:
            import serial

            self.serial = serial.Serial(serial_device, baudrate=baud, timeout=0, write_timeout=0.2)

    def emit(self, measurement, state, stable_frames):
        result = measurement_dict(
            measurement, state, stable_frames, self.color_names
        )
        print(json.dumps(result, ensure_ascii=False), flush=True)
        if self.serial is not None:
            self.serial.write(
                encode_frame(
                    "VISION_RESULT",
                    state,
                    measurement.kind,
                    measurement.target_id,
                    "%.3f" % measurement.x,
                    "%.3f" % measurement.y,
                    "%.3f" % measurement.yaw,
                    "%.3f" % measurement.confidence,
                    measurement.unit,
                    stable_frames,
                )
            )

    def emit_pickup(self, update):
        measurement = update.measurement
        result = {
            "time_ms": update.observed_at_ms,
            "state": update.state,
            "reason": update.reason,
            "target_id": measurement.target_id if measurement else None,
            "x": round(float(measurement.x), 3) if measurement else None,
            "y": round(float(measurement.y), 3) if measurement else None,
            "unit": measurement.unit if measurement else None,
            "pixel_x": (
                round(float(measurement.pixel_x), 3)
                if measurement and measurement.pixel_x is not None
                else None
            ),
            "pixel_y": (
                round(float(measurement.pixel_y), 3)
                if measurement and measurement.pixel_y is not None
                else None
            ),
            "confidence": (
                round(float(measurement.confidence), 3) if measurement else None
            ),
            "speed_px_s": round(float(update.speed_px_s), 3),
            "stable_frames": int(update.stable_count),
            "final_recheck": int(update.final_count),
            "valid_for_ms": int(update.valid_for_ms),
            "ready_rising_edge": bool(update.ready_rising_edge),
        }
        print(json.dumps(result, ensure_ascii=False), flush=True)
        if self.serial is not None and measurement is not None:
            frame_name = "GRASP_READY" if update.ready else "PICKUP_STATE"
            pixel_x = (
                measurement.pixel_x
                if measurement.pixel_x is not None
                else measurement.x
            )
            pixel_y = (
                measurement.pixel_y
                if measurement.pixel_y is not None
                else measurement.y
            )
            self.serial.write(
                encode_frame(
                    frame_name,
                    update.state,
                    measurement.target_id,
                    "%.3f" % measurement.x,
                    "%.3f" % measurement.y,
                    measurement.unit,
                    "%.3f" % pixel_x,
                    "%.3f" % pixel_y,
                    "%.3f" % measurement.confidence,
                    "%.3f" % update.speed_px_s,
                    update.stable_count,
                    update.final_count,
                    update.valid_for_ms,
                    int(update.ready_rising_edge),
                )
            )

    def close(self):
        if self.serial is not None:
            self.serial.close()
