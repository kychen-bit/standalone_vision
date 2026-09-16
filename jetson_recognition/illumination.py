"""Per-frame illuminant estimation and white-balanced re-exposure.

The preliminary rules removed the downward fill light, so the same material is
now imaged under different colour temperatures and brightness levels inside one
competition. A fixed HSV table only holds for the light it was tuned under,
which is why the shipped parameters are still marked "TEMPORARY SCREEN TEST".

The field itself is the calibration target: the working area is matte white and
always visible around the turntable, so every frame contains a known neutral
surface. This module finds those neutral pixels, measures the illuminant per
channel (a white-patch estimate anchored on a real white surface instead of on
the brightest pixel of the frame) and rescales the frame so the neutral field
always lands on ``canonical_level``. The HSV of a material then follows its
reflectance rather than the ambient light, and one threshold set covers both a
dim hall and a sunlit one.

Nothing is applied when no trustworthy white reference is visible: the frame is
passed through unchanged and the diagnostics say why. The estimate is smoothed
over time and frozen when the scene changes abruptly (an arm reaching into the
frame), so a shadow is not mistaken for a new illuminant.
"""

import cv2
import numpy as np


class WhiteBalanceNormalizer:
    """Estimate the illuminant per frame and re-expose the frame for it."""

    def __init__(self, config=None):
        config = dict(config or {})
        self.enabled = bool(config.get("enabled", True))
        self.reference_percentile = min(
            100.0, max(50.0, float(config.get("reference_percentile", 88.0)))
        )
        self.neutral_max_chroma = max(
            0.0, float(config.get("neutral_max_chroma", 0.35))
        )
        self.neutral_max_chroma_relaxed = max(
            self.neutral_max_chroma,
            float(config.get("neutral_max_chroma_relaxed", 0.55)),
        )
        self.clip_level = min(255.0, float(config.get("clip_level", 250.0)))
        self.max_clipped_ratio = min(
            1.0, max(0.0, float(config.get("max_clipped_ratio", 0.5)))
        )
        self.min_level = max(1.0, float(config.get("min_level", 20.0)))
        self.max_level = min(255.0, float(config.get("max_level", 255.0)))
        self.min_neutral_ratio = max(
            0.0, float(config.get("min_neutral_ratio", 0.02))
        )
        self.smoothing = min(1.0, max(0.01, float(config.get("smoothing", 0.25))))
        self.hold_smoothing = min(
            1.0, max(0.0, float(config.get("hold_smoothing", 0.02)))
        )
        self.canonical_level = min(
            255.0, max(32.0, float(config.get("canonical_level", 235.0)))
        )
        self.max_gain = max(1.0, float(config.get("max_gain", 4.0)))
        self.gain_epsilon = min(
            0.5, max(0.0, float(config.get("gain_epsilon", 0.02)))
        )
        self.hold_jump_ratio = max(1.0, float(config.get("hold_jump_ratio", 1.6)))
        self.estimate_width = max(32, int(config.get("estimate_width", 256)))
        self.reference = None
        self.gain = np.ones(3, dtype=np.float32)
        self.last = self._initial_diagnostics()

    def _initial_diagnostics(self):
        return {
            "enabled": self.enabled,
            "reliable": False,
            "held": False,
            "reason": "NOT_RUN",
            "neutral_ratio": 0.0,
            "white_ref_bgr": None,
            "gain": [1.0, 1.0, 1.0],
            "canonical_level": self.canonical_level,
            "overexposed_ratio": 0.0,
            "gain_limited": False,
            "reference_mode": None,
            "clipped_ratio": 0.0,
        }

    def reset(self):
        self.reference = None
        self.gain = np.ones(3, dtype=np.float32)
        self.last = self._initial_diagnostics()

    def _estimate(self, small):
        """Return (per-channel white level, diagnostics) for one small frame.

        Everything runs on OpenCV primitives: the HSV conversion supplies the
        maximum channel and ``255*(max-min)/max``, which is exactly the chroma
        used to recognise the field, and the per-channel level comes from a
        masked histogram instead of a sorted percentile.
        """
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        saturation = hsv[..., 1]
        value = hsv[..., 2]
        overexposed = float(np.count_nonzero(value >= self.clip_level)) / float(
            max(value.size, 1)
        )
        info = {
            "neutral_ratio": 0.0,
            "overexposed_ratio": overexposed,
            "held": False,
            "gain_limited": False,
            "reference_mode": None,
            "clipped_ratio": 0.0,
        }
        neutral = None
        # A warm or cool lamp tints even a white surface, so the strict chroma
        # limit is relaxed once before giving up on the frame.
        for limit, mode in (
            (self.neutral_max_chroma, "STRICT"),
            (self.neutral_max_chroma_relaxed, "RELAXED"),
        ):
            candidate = (
                (saturation <= limit * 255.0)
                & (value >= self.min_level)
                & (value <= self.max_level)
            )
            ratio = float(np.count_nonzero(candidate)) / float(
                max(value.size, 1)
            )
            info["neutral_ratio"] = ratio
            if ratio >= self.min_neutral_ratio:
                neutral = candidate
                info["reference_mode"] = mode
                break
        if neutral is None:
            info["reason"] = "NO_NEUTRAL_REFERENCE"
            return None, info
        clipped = neutral & (value >= self.clip_level)
        info["clipped_ratio"] = float(np.count_nonzero(clipped)) / float(
            max(np.count_nonzero(neutral), 1)
        )
        if info["clipped_ratio"] > self.max_clipped_ratio:
            # A blown-out white field carries no colour information left; a
            # gain derived from it would shift hues. Leave the frame alone and
            # tell the operator to lower the exposure instead.
            info["reason"] = "WHITE_FIELD_CLIPPED"
            return None, info
        mask = neutral.astype(np.uint8) * 255
        reference = np.empty(3, dtype=np.float32)
        for channel in range(3):
            plane = np.ascontiguousarray(small[..., channel])
            histogram = cv2.calcHist([plane], [0], mask, [256], [0, 256])
            reference[channel] = self._histogram_percentile(
                histogram, self.reference_percentile
            )
        if float(reference.min()) < self.min_level:
            info["reason"] = "REFERENCE_TOO_DARK"
            return None, info
        info["reason"] = "OK"
        return reference, info

    @staticmethod
    def _histogram_percentile(histogram, percentile):
        total = float(histogram.sum())
        if total <= 0.0:
            return 0.0
        cumulative = np.cumsum(histogram.reshape(-1).astype(np.float64))
        index = int(np.searchsorted(cumulative, total * percentile / 100.0))
        return float(min(max(index, 0), 255))

    def apply(self, frame):
        """Return ``(balanced_frame, diagnostics)`` for one BGR frame."""
        if not self.enabled:
            self.last = self._initial_diagnostics()
            self.last["reason"] = "DISABLED"
            return frame, self.last
        if frame is None or frame.ndim != 3 or frame.size == 0:
            self.last = self._initial_diagnostics()
            self.last["reason"] = "BAD_FRAME"
            return frame, self.last

        height, width = frame.shape[:2]
        # A strided subsample is statistically enough for a scene-level white
        # estimate and costs almost nothing, unlike a full resize.
        step = max(1, int(round(float(width) / float(self.estimate_width))))
        small = frame[::step, ::step]
        if not small.flags["C_CONTIGUOUS"]:
            small = np.ascontiguousarray(small)
        if small.shape[0] < 8 or small.shape[1] < 8:
            small = frame
        reference, info = self._estimate(small)

        held = False
        if reference is None:
            # Keep the previous reference: a single dark or fully colourful
            # frame is an occlusion, not a new illuminant.
            if self.reference is None:
                self.last = dict(self.last)
                self.last.update(info)
                self.last["enabled"] = True
                self.last["reliable"] = False
                self.last["gain"] = [1.0, 1.0, 1.0]
                self.last["white_ref_bgr"] = None
                return frame, self.last
            held = True
        else:
            if self.reference is not None:
                ratio = reference / np.maximum(self.reference, 1e-3)
                if float(ratio.max()) > self.hold_jump_ratio or float(
                    ratio.min()
                ) < 1.0 / self.hold_jump_ratio:
                    # Abrupt change: drift slowly instead of following it.
                    held = True
                alpha = self.hold_smoothing if held else self.smoothing
                reference = (1.0 - alpha) * self.reference + alpha * reference
            self.reference = reference

        gain = self.canonical_level / np.maximum(self.reference, self.min_level)
        limited = bool(np.any(gain > self.max_gain))
        gain = np.clip(gain, 0.05, self.max_gain)
        self.gain = gain.astype(np.float32)
        balanced = self._balance(frame, self.gain)

        self.last = {
            "enabled": True,
            "reliable": not held,
            "held": held,
            "reason": info["reason"] if not held else "HELD_ABRUPT_CHANGE",
            "neutral_ratio": info["neutral_ratio"],
            "white_ref_bgr": [round(float(value), 2) for value in self.reference],
            "gain": [round(float(value), 4) for value in self.gain],
            "canonical_level": self.canonical_level,
            "overexposed_ratio": info["overexposed_ratio"],
            "gain_limited": limited,
            "reference_mode": info.get("reference_mode"),
            "clipped_ratio": info.get("clipped_ratio", 0.0),
        }
        return balanced, self.last

    def _balance(self, frame, gain):
        if float(np.abs(gain - 1.0).max()) <= self.gain_epsilon:
            # Well-lit venue: the frame already is what the thresholds expect,
            # so skip the full-frame lookup entirely.
            return frame
        lookup = np.empty((256, 1, 3), dtype=np.uint8)
        levels = np.arange(256, dtype=np.float32)
        for channel in range(3):
            lookup[:, 0, channel] = np.clip(
                levels * float(gain[channel]), 0.0, 255.0
            ).astype(np.uint8)
        return cv2.LUT(frame, lookup)
