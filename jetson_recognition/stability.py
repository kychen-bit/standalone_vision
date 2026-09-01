"""Small multi-frame stability gates; no prediction or motion model."""

import math


class SimpleColorStability:
    """Declare stable after N consecutive nearby detections of one color."""

    def __init__(self, required_frames=3, max_center_delta_px=6.0):
        self.required_frames = max(1, int(required_frames))
        self.max_center_delta_px = float(max_center_delta_px)
        self.clear()

    def clear(self):
        self.label = None
        self.samples = []

    def miss(self):
        self.clear()

    def update(self, label, pixel_x, pixel_y, output_x, output_y, quality):
        sample = (
            float(pixel_x), float(pixel_y),
            float(output_x), float(output_y), float(quality),
        )
        if self.label != label:
            self.label = label
            self.samples = [sample]
            return None
        if self.samples:
            previous = self.samples[-1]
            delta = math.hypot(sample[0] - previous[0], sample[1] - previous[1])
            if delta > self.max_center_delta_px:
                self.samples = [sample]
                return None
        self.samples.append(sample)
        if len(self.samples) > self.required_frames:
            self.samples.pop(0)
        if len(self.samples) < self.required_frames:
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
    ):
        self.required_frames = int(required_frames)
        self.max_spread_xy = float(max_spread_xy)
        self.max_spread_yaw = float(max_spread_yaw)
        self.min_quality = float(min_quality)
        self.reset_after_misses = int(reset_after_misses)
        self.clear()

    def clear(self):
        self.label = None
        self.samples = []
        self.misses = 0

    def miss(self):
        self.misses += 1
        if self.misses >= self.reset_after_misses:
            self.clear()

    def update(self, label, x, y, yaw, quality):
        quality = float(quality)
        if label is None or quality < self.min_quality:
            self.miss()
            return None
        self.misses = 0
        if self.label != label:
            self.label = label
            self.samples = []
        self.samples.append((float(x), float(y), float(yaw), quality))
        if len(self.samples) > self.required_frames:
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
        xs = [sample[0] for sample in self.samples]
        ys = [sample[1] for sample in self.samples]
        yaws = [sample[2] for sample in self.samples]
        return (
            max(xs) - min(xs) <= self.max_spread_xy
            and max(ys) - min(ys) <= self.max_spread_xy
            and max(yaws) - min(yaws) <= self.max_spread_yaw
        )
