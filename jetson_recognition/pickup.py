"""Target-driven turntable waiting and safe visual-ready gating."""

from collections import namedtuple
import math
import time

from .stability import StableWindow


PickupUpdate = namedtuple(
    "PickupUpdate",
    (
        "state measurement stable_count speed_px_s ready reason final_count "
        "observed_at_ms valid_for_ms ready_rising_edge"
    ),
)


class PickupSession:
    """Convert target-color observations into a one-shot GRASP_READY event.

    This module never moves the arm or closes the gripper. It only grants a
    short-lived visual permission after the target is inside the configured
    pixel capture zone, stationary, stable and finally rechecked.
    """

    STATES = (
        "SEARCHING",
        "TRACKING",
        "LOW_CONFIDENCE",
        "MOVING",
        "SETTLING",
        "FINAL_CHECK",
        "GRASP_READY",
    )

    def __init__(self, engine, target_id, scene, config):
        self.engine = engine
        self.target_id = str(target_id)
        self.scene = str(scene).upper()
        self.config = dict(config)
        zone = self.config.get("capture_zone_px")
        if zone is None or len(zone) != 4:
            raise ValueError(
                "pickup.capture_zone_px is not configured; measure it on the live image"
            )
        self.capture_zone = tuple(float(value) for value in zone)
        x, y, width, height = self.capture_zone
        margin = float(self.config.get("capture_margin_px", 0))
        if x < 0 or y < 0 or width <= 0 or height <= 0:
            raise ValueError("pickup capture zone is invalid")
        if width <= 2 * margin or height <= 2 * margin:
            raise ValueError("pickup capture margin leaves no safe inner zone")
        self.margin = margin
        self.final_required = int(self.config.get("final_recheck_frames", 3))
        if self.final_required < 1:
            raise ValueError("pickup final_recheck_frames must be positive")
        self.max_speed = float(self.config.get("max_target_speed_px_s", 30.0))
        self.valid_for_ms = int(self.config.get("result_valid_ms", 200))
        self.motion_window_frames = max(
            2, int(self.config.get("motion_window_frames", 5))
        )
        self.window = StableWindow(
            required_frames=int(self.config.get("stable_frames", 5)),
            max_spread_xy=float(self.config.get("stable_spread_px", 4.0)),
            max_spread_yaw=1.0,
            min_quality=float(self.config.get("min_confidence", 0.45)),
            reset_after_misses=int(self.config.get("reset_after_misses", 2)),
        )
        self.low_speed_hold = max(
            1, int(self.config.get("low_speed_hold_frames", 3))
        )
        self.speed_hold_count = 0
        self.misses = 0
        self.motion_samples = []
        self.base_stable = False
        self.final_count = 0
        self.ready_latched = False
        self.state = "SEARCHING"

    @property
    def safe_zone(self):
        x, y, width, height = self.capture_zone
        return (
            x + self.margin,
            y + self.margin,
            width - 2 * self.margin,
            height - 2 * self.margin,
        )

    def _pixel_point(self, measurement):
        if measurement.pixel_x is not None and measurement.pixel_y is not None:
            return float(measurement.pixel_x), float(measurement.pixel_y)
        if measurement.unit == "PX":
            return float(measurement.x), float(measurement.y)
        raise ValueError("pickup gating requires raw pixel coordinates")

    def _inside_safe_zone(self, x, y):
        zone_x, zone_y, width, height = self.safe_zone
        return (
            zone_x <= x <= zone_x + width
            and zone_y <= y <= zone_y + height
        )

    def _reset_confirmation(self, clear_motion=True):
        self.window.clear()
        self.base_stable = False
        self.final_count = 0
        self.ready_latched = False
        if clear_motion:
            self.motion_samples = []

    def _append_motion(self, timestamp, x, y):
        self.motion_samples.append((float(timestamp), float(x), float(y)))
        if len(self.motion_samples) > self.motion_window_frames:
            self.motion_samples.pop(0)
        if len(self.motion_samples) < 2:
            return 0.0
        first = self.motion_samples[0]
        last = self.motion_samples[-1]
        elapsed = last[0] - first[0]
        if elapsed <= 1e-6:
            return 0.0
        return math.hypot(last[1] - first[1], last[2] - first[2]) / elapsed

    def _result(self, state, measurement, count, speed, ready, reason, rising_edge=False):
        self.state = state
        return PickupUpdate(
            state,
            measurement,
            int(count),
            float(speed),
            bool(ready),
            reason,
            int(self.final_count),
            int(time.time() * 1000),
            self.valid_for_ms,
            bool(rising_edge),
        )

    def _miss(self, reason):
        """One frame without a target. Authorised grants are revoked at once."""
        self.misses += 1
        self.speed_hold_count = 0
        if self.ready_latched:
            # Already authorised, then the target vanished: revoke the grant
            # immediately. A single missed frame after GRASP_READY is unsafe.
            self._reset_confirmation()
            return self._result(
                "SEARCHING", None, 0, 0.0, False, "TARGET_LOST_AFTER_READY"
            )
        # Not yet authorised: tolerate a few consecutive misses (one-frame
        # glare / occlusion) instead of throwing away the accumulated samples.
        self.base_stable = False
        self.final_count = 0
        if self.window is not None:
            self.window.miss()
        if self.misses >= int(self.config.get("reset_after_misses", 2)):
            return self._result("SEARCHING", None, 0, 0.0, False, reason)
        return self._result(
            "SEARCHING", None, 0, 0.0, False, "TARGET_LOST_TEMPORARILY"
        )

    def update(self, frame, timestamp=None):
        """Process one frame. ``timestamp`` is monotonic seconds for testing."""
        if timestamp is None:
            timestamp = time.monotonic()
        measurement = self.engine.detect(
            frame, "COLOR", (self.target_id, self.scene)
        )
        if measurement is None:
            return self._miss("TARGET_NOT_VISIBLE")

        self.misses = 0
        pixel_x, pixel_y = self._pixel_point(measurement)
        if not self._inside_safe_zone(pixel_x, pixel_y):
            self._reset_confirmation()
            return self._result(
                "TRACKING", measurement, 0, 0.0, False, "OUTSIDE_CAPTURE_ZONE"
            )

        if measurement.confidence < float(self.config.get("min_confidence", 0.45)):
            self._reset_confirmation()
            return self._result(
                "LOW_CONFIDENCE", measurement, 0, 0.0, False, "QUALITY_BELOW_LIMIT"
            )

        speed = self._append_motion(timestamp, pixel_x, pixel_y)
        if speed > self.max_speed:
            # Turntable is still moving: discard stability, restart the
            # low-speed hold counter and never feed moving samples into the
            # stable window.
            self.speed_hold_count = 0
            self._reset_confirmation(clear_motion=False)
            return self._result(
                "MOVING", measurement, 0, speed, False, "TARGET_MOVING"
            )

        # Speed is below the limit, but require a few consecutive low-speed
        # frames before accumulating. This stops moving/settling flapping
        # while the turntable decelerates to a stop. The window is NOT cleared
        # here: after a MOVING frame it is already empty, and after a brief
        # target loss it keeps the retained stationary samples.
        if self.speed_hold_count < self.low_speed_hold:
            self.speed_hold_count += 1
            self.base_stable = False
            self.final_count = 0
            self.ready_latched = False
            return self._result(
                "SETTLING", measurement, 0, speed, False, "TARGET_SLOWING"
            )

        stable = self.window.update(
            "COLOR:" + self.target_id,
            pixel_x,
            pixel_y,
            0.0,
            measurement.confidence,
        )
        count = len(self.window.samples)
        if stable is None:
            self.base_stable = False
            self.final_count = 0
            self.ready_latched = False
            return self._result(
                "SETTLING", measurement, count, speed, False, "BUILDING_STABILITY"
            )

        if not self.base_stable:
            self.base_stable = True
            self.final_count = 0
            return self._result(
                "FINAL_CHECK", measurement, count, speed, False, "BASE_STABLE"
            )

        if not self.ready_latched:
            self.final_count += 1
            if self.final_count < self.final_required:
                return self._result(
                    "FINAL_CHECK", measurement, count, speed, False, "RECHECKING"
                )
            self.ready_latched = True
            return self._result(
                "GRASP_READY",
                measurement,
                count,
                speed,
                True,
                "FINAL_RECHECK_PASSED",
                rising_edge=True,
            )

        return self._result(
            "GRASP_READY", measurement, count, speed, True, "READY_HELD"
        )
