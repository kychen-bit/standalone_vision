"""Recognition orchestration with no camera or serial dependency."""

from .model import Measurement
from .stability import SimpleColorStability, StableWindow


MODE_ARGUMENTS = {
    "COLOR": 2,
    "RING": 2,
    "STACK": 3,
    "STATION": 1,
    "TURNTABLE": 1,
}


class RecognitionEngine:
    """One interface for all Jetson recognition modes."""

    def __init__(self, detector, runtime_config):
        self.detector = detector
        self.runtime = runtime_config
        enabled = runtime_config.get("enabled_modes")
        self.enabled_modes = (
            {str(mode).upper() for mode in enabled}
            if enabled is not None
            else set(MODE_ARGUMENTS)
        )
        unknown = self.enabled_modes.difference(MODE_ARGUMENTS)
        if unknown:
            raise ValueError("unknown enabled recognition modes: %s" % sorted(unknown))

    def validate(self, mode, args):
        mode = str(mode).upper()
        expected = MODE_ARGUMENTS.get(mode)
        if expected is None:
            raise ValueError("unsupported recognition mode: %s" % mode)
        if mode not in self.enabled_modes:
            raise ValueError("recognition mode is disabled by config: %s" % mode)
        if len(args) != expected:
            raise ValueError(
                "%s expects %d arguments, got %d" % (mode, expected, len(args))
            )
        return mode, tuple(str(value) for value in args)

    def detect(self, frame, mode, args):
        mode, args = self.validate(mode, args)
        if mode == "COLOR":
            return self.detector.detect_color(frame, args[0], args[1])
        if mode == "RING":
            return self.detector.detect_ring(frame, args[0], args[1])
        if mode == "STACK":
            return self.detector.detect_stack(frame, args[0], args[1], args[2])
        if mode == "STATION":
            return self.detector.detect_station(frame, args[0])
        return self.detector.detect_turntable(frame, args[0])

    def new_session(self, mode, args):
        mode, args = self.validate(mode, args)
        return RecognitionSession(mode, args, self)


class RecognitionSession:
    def __init__(self, mode, args, engine):
        self.mode = mode
        self.args = args
        self.engine = engine
        self.window = None
        self.unit = None

    def update(self, frame):
        measurement = self.engine.detect(frame, self.mode, self.args)
        if measurement is None:
            if self.window is not None:
                self.window.miss()
            return None, None, 0

        runtime = self.engine.runtime
        if self.mode == "COLOR":
            if self.window is None:
                stability = runtime.get("color_stability", {})
                self.window = SimpleColorStability(
                    required_frames=stability.get("stable_frames", 3),
                    max_center_delta_px=stability.get("max_center_delta_px", 6.0),
                )
            stable = self.window.update(
                measurement.kind + ":" + measurement.target_id,
                measurement.pixel_x if measurement.pixel_x is not None else measurement.x,
                measurement.pixel_y if measurement.pixel_y is not None else measurement.y,
                measurement.x,
                measurement.y,
                measurement.confidence,
            )
            if stable is not None and hasattr(
                self.engine.detector, "activate_dynamic_roi"
            ):
                self.engine.detector.activate_dynamic_roi(
                    measurement.kind, measurement.target_id, self.args[1]
                )
            return measurement, stable, len(self.window.samples)
        if self.window is None or self.unit != measurement.unit:
            self.unit = measurement.unit
            spread = (
                runtime["stable_spread_mm"]
                if self.unit == "MM"
                else runtime["stable_spread_px"]
            )
            self.window = StableWindow(
                required_frames=runtime["stable_frames"],
                max_spread_xy=spread,
                max_spread_yaw=runtime["stable_yaw_deg"],
                min_quality=runtime["min_confidence"],
                reset_after_misses=runtime["reset_after_misses"],
            )
        stable = self.window.update(
            measurement.kind + ":" + measurement.target_id,
            measurement.x,
            measurement.y,
            measurement.yaw,
            measurement.confidence,
        )
        return measurement, stable, len(self.window.samples)


def stable_measurement(measurement, stable):
    if measurement is None or stable is None:
        return None
    x, y, yaw, confidence, _ = stable
    pixel_x = x if measurement.unit == "PX" else measurement.pixel_x
    pixel_y = y if measurement.unit == "PX" else measurement.pixel_y
    return Measurement(
        measurement.kind,
        measurement.target_id,
        x,
        y,
        yaw,
        confidence,
        measurement.unit,
        measurement.residual,
        pixel_x,
        pixel_y,
    )
