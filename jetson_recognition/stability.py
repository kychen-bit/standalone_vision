"""Small multi-frame stability gates; no prediction or motion model."""

import math
import time


class SimpleColorStability:
    """Declare stable after N consecutive nearby detections of one color."""

    def __init__(
        self,
        required_frames=3,
        max_center_delta_px=6.0,
        min_duration_ms=0,
    ):
        self.required_frames = max(1, int(required_frames))
        self.max_center_delta_px = float(max_center_delta_px)
        self.min_duration_s = max(0.0, float(min_duration_ms) / 1000.0)
        self.clear()

    def clear(self):
        self.label = None
        self.samples = []

    def miss(self):
        self.clear()

    def update(
        self,
        label,
        pixel_x,
        pixel_y,
        output_x,
        output_y,
        quality,
        timestamp=None,
    ):
        timestamp = time.monotonic() if timestamp is None else float(timestamp)
        sample = (
            float(pixel_x), float(pixel_y),
            float(output_x), float(output_y), float(quality),
            timestamp,
        )
        if self.label != label:
            self.label = label
            self.samples = [sample]
            return None
        if self.samples and timestamp < self.samples[-1][5]:
            self.samples = [sample]
            return None
        if self.samples:
            previous = self.samples[-1]
            delta = math.hypot(
                sample[0] - previous[0], sample[1] - previous[1]
            )
            if delta > self.max_center_delta_px:
                self.samples = [sample]
                return None
        self.samples.append(sample)
        if self.min_duration_s > 0:
            cutoff = timestamp - self.min_duration_s
            while len(self.samples) > 1 and self.samples[1][5] <= cutoff:
                self.samples.pop(0)
        else:
            while len(self.samples) > self.required_frames:
                self.samples.pop(0)
        if len(self.samples) < self.required_frames:
            return None
        if timestamp - self.samples[0][5] < self.min_duration_s:
            return None
        xs = [item[0] for item in self.samples]
        ys = [item[1] for item in self.samples]
        if (
            max(xs) - min(xs) > self.max_center_delta_px
            or max(ys) - min(ys) > self.max_center_delta_px
        ):
            return None
        count = len(self.samples)
        return (
            sum(item[2] for item in self.samples) / count,
            sum(item[3] for item in self.samples) / count,
            0.0,
            sum(item[4] for item in self.samples) / count,
            count,
        )


class StableWindow:
    def __init__(
        self,
        required_frames=5,
        max_spread_xy=3.0,
        max_spread_yaw=1.0,
        min_quality=0.4,
        reset_after_misses=2,
        min_duration_ms=0,
    ):
        self.required_frames = int(required_frames)
        self.max_spread_xy = float(max_spread_xy)
        self.max_spread_yaw = float(max_spread_yaw)
        self.min_quality = float(min_quality)
        self.reset_after_misses = int(reset_after_misses)
        self.min_duration_s = max(0.0, float(min_duration_ms) / 1000.0)
        self.clear()

    def clear(self):
        self.label = None
        self.samples = []
        self.misses = 0

    def miss(self):
        self.misses += 1
        if self.misses >= self.reset_after_misses:
            self.clear()

    def update(self, label, x, y, yaw, quality, timestamp=None):
        timestamp = time.monotonic() if timestamp is None else float(timestamp)
        quality = float(quality)
        if label is None or quality < self.min_quality:
            self.miss()
            return None
        self.misses = 0
        if self.label != label:
            self.label = label
            self.samples = []
        if self.samples and timestamp < self.samples[-1][4]:
            self.samples = []
        self.samples.append(
            (float(x), float(y), float(yaw), quality, timestamp)
        )
        if self.min_duration_s > 0:
            cutoff = timestamp - self.min_duration_s
            while len(self.samples) > 1 and self.samples[1][4] <= cutoff:
                self.samples.pop(0)
        else:
            while len(self.samples) > self.required_frames:
                self.samples.pop(0)
        if not self.is_stable():
            return None
        count = len(self.samples)
        return (
            sum(sample[0] for sample in self.samples) / count,
            sum(sample[1] for sample in self.samples) / count,
            sum(sample[2] for sample in self.samples) / count,
            sum(sample[3] for sample in self.samples) / count,
            count,
        )

    def is_stable(self):
        if len(self.samples) < self.required_frames:
            return False
        if self.samples[-1][4] - self.samples[0][4] < self.min_duration_s:
            return False
        xs = [sample[0] for sample in self.samples]
        ys = [sample[1] for sample in self.samples]
        yaws = [sample[2] for sample in self.samples]
        return (
            max(xs) - min(xs) <= self.max_spread_xy
            and max(ys) - min(ys) <= self.max_spread_xy
            and max(yaws) - min(yaws) <= self.max_spread_yaw
        )
