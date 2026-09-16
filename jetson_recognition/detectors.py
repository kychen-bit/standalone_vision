"""Traditional eye-in-hand recognition reused from the main project."""

import json

import cv2
import numpy as np

from .geometry import apply_homography, estimate_rigid_pose
from .illumination import WhiteBalanceNormalizer
from .model import Measurement


class PlaneCalibration:
    """Map pixels to robot-plane millimetres for a fixed camera pose."""

    def __init__(self, path):
        data = json.loads(path.read_text(encoding="utf-8"))
        self.planes = data.get("planes", {})

    def map(self, scene, x_px, y_px):
        plane = self.planes.get(scene.upper(), {})
        matrix = plane.get("homography_px_to_robot_mm")
        if not plane.get("calibrated") or not matrix:
            return float(x_px), float(y_px), "PX"
        x_mm, y_mm = apply_homography(matrix, float(x_px), float(y_px))
        return x_mm, y_mm, "MM"


def _crop(frame, roi):
    if frame is None or frame.ndim != 3:
        raise ValueError("frame must be a BGR image")
    x, y, width, height = _roi_values(roi)
    if x < 0 or y < 0 or width <= 0 or height <= 0:
        raise ValueError("invalid ROI")
    if x + width > frame.shape[1] or y + height > frame.shape[0]:
        raise ValueError("ROI exceeds frame")
    return frame[y : y + height, x : x + width], x, y


def _roi_values(roi):
    """Accept the readable config form and the legacy four-item list."""
    if isinstance(roi, dict):
        return [
            int(roi["x"]),
            int(roi["y"]),
            int(roi["width"]),
            int(roi["height"]),
        ]
    return [int(value) for value in roi]


class TopViewDetector:
    """Recognise one requested target in one BGR frame."""

    def __init__(self, config, project_root):
        self.config = config
        self.project_root = project_root
        calibration_path = project_root / config["camera"].get(
            "calibration_file", "config/calibration.json"
        )
        self.calibration = PlaneCalibration(calibration_path)

        # COLOR_V1 intentionally stays small: ROI, HSV inRange, morphology,
        # convex outer contour and basic geometry. Complex classifiers belong
        # in a future optional detector, not in the competition path.
        self.color_ranges = {}
        self.color_names = {}
        self.black_threshold = None
        for color_id, definition in config["colors"].items():
            color_id = str(color_id)
            self.color_names[color_id] = str(definition["name"]).upper()
            if color_id == "5":
                black = definition["black_threshold"]
                self.black_threshold = {
                    "v_max": int(black["v_max"]),
                    "s_min": int(black.get("s_min", 0)),
                    "s_max": int(black.get("s_max", 255)),
                }
                self.color_ranges[color_id] = []
                continue
            ranges = []
            for threshold in definition.get("hsv_ranges", []):
                lower = np.asarray(
                    [threshold["h_min"], threshold["s_min"], threshold["v_min"]],
                    dtype=np.uint8,
                )
                upper = np.asarray(
                    [threshold["h_max"], threshold.get("s_max", 255), threshold.get("v_max", 255)],
                    dtype=np.uint8,
                )
                ranges.append((lower, upper))
            if not ranges:
                raise ValueError("missing HSV range for color %s" % color_id)
            self.color_ranges[color_id] = ranges
        color_config = config["color_detection"]
        morphology = color_config["morphology"]
        geometry = color_config["geometry"]
        morphology_size = self._odd_kernel(
            morphology.get("kernel_size", 3), "color morphology"
        )
        self.color_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (morphology_size, morphology_size)
        )
        self.color_open_iterations = max(
            0, int(morphology.get("open_iterations", 1))
        )
        self.color_close_iterations = max(
            0, int(morphology.get("close_iterations", 1))
        )
        self.color_min_area = float(geometry["min_area"])
        configured_max = geometry.get("max_area")
        self.color_max_area = (
            float(configured_max) if configured_max is not None else float("inf")
        )
        self.color_min_aspect_ratio = float(
            geometry.get("min_aspect_ratio", 0.65)
        )
        self.color_max_aspect_ratio = float(
            geometry.get("max_aspect_ratio", 1.55)
        )
        if not (
            0 < self.color_min_aspect_ratio <= self.color_max_aspect_ratio
        ):
            raise ValueError("invalid color aspect-ratio limits")
        self.color_border_margin = max(
            0, int(geometry.get("border_margin_px", 10))
        )
        self.black_min_circularity = float(
            geometry.get("black_min_circularity", 0.0)
        )
        if not 0.0 <= self.black_min_circularity <= 1.0:
            raise ValueError("black_min_circularity must be in [0, 1]")
        dynamic_roi = color_config.get("dynamic_roi", {})
        self.dynamic_roi_enabled = bool(dynamic_roi.get("enabled", False))
        self.dynamic_roi_margin = max(0, int(dynamic_roi.get("margin_px", 80)))
        self._dynamic_roi_active = set()
        self._last_color_boxes = {}
        self.color_debug_enabled = False
        self.last_color_debug = None
        self.last_colors_debug = []
        self.last_color_runtime = None
        self._last_color_filter_stats = None

        # Per-frame white balance. The preliminary rules removed the downward
        # fill light, so the colour thresholds have to stay valid while the
        # ambient light changes; see jetson_recognition/illumination.py.
        self.illumination = WhiteBalanceNormalizer(
            color_config.get("illumination", {})
        )
        self.illumination_enabled = self.illumination.enabled
        self.last_illumination = None
        self.last_balanced_frame = None
        # Blob-level colour decision for the round batch: every material is
        # classified once from the mean colour of its whole blob instead of
        # from six independent binary masks.
        label_config = color_config.get("label_classifier", {})
        self.label_hue_scale = float(label_config.get("hue_scale", 20.0))
        self.label_saturation_scale = float(
            label_config.get("saturation_scale", 60.0)
        )
        self.label_value_scale = float(label_config.get("value_scale", 50.0))
        self.label_center_weight = float(label_config.get("center_weight", 0.05))
        grouping = color_config.get("material_grouping", {})
        self.material_merge_distance = max(
            0.0, float(grouping.get("merge_distance_px", 40.0))
        )
        self.material_max_distance = max(
            0.05,
            float(
                grouping.get(
                    "max_label_distance",
                    label_config.get("accept_distance", 1.0),
                )
            ),
        )
        self.material_min_margin = max(
            0.0, float(grouping.get("min_label_margin", 0.0))
        )
        self.last_materials_debug = None

        ring_config = config["ring_detection"]
        ring_morphology = ring_config["morphology"]
        ring_kernel_size = self._odd_kernel(
            ring_morphology.get("kernel_size", 3), "ring morphology"
        )
        self.ring_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (ring_kernel_size, ring_kernel_size)
        )
        self.ring_open_iterations = max(
            0, int(ring_morphology.get("open_iterations", 0))
        )
        self.ring_close_iterations = max(
            0, int(ring_morphology.get("close_iterations", 1))
        )
        self.ring_digit_config = dict(ring_config.get("digit_template", {}))
        self.ring_digit_templates = {}
        self.ring_template_source = "DISABLED"
        if self.ring_digit_config.get("enabled", False):
            self._load_ring_digit_templates()

        # Hough remains available only for the independent turntable detector.
        turntable_config = config["turntable_detection"]
        grid_size = max(1, int(turntable_config.get("clahe_grid_size", 8)))
        self.circle_clahe = cv2.createCLAHE(
            clipLimit=float(turntable_config.get("clahe_clip_limit", 2.5)),
            tileGridSize=(grid_size, grid_size),
        )
        self.circle_debug_enabled = False
        self.last_circle_debug = None
        self._validate_rois()

    # -- illuminant normalisation -------------------------------------------
    def _work_frame(self, frame):
        """Re-expose the frame for the current illuminant before HSV work.

        The original frame is returned whenever no trustworthy white reference
        is visible, so a dark or unusually colourful scene can never make the
        detector worse than the plain unnormalised path.
        """
        if not self.illumination_enabled:
            self.last_balanced_frame = None
            return frame
        balanced, info = self.illumination.apply(frame)
        self.last_illumination = info
        # Kept only so a live window can show what the detector actually sees.
        self.last_balanced_frame = balanced if balanced is not frame else None
        return balanced

    # -- blob colour metric -------------------------------------------------
    @staticmethod
    def _channel_distance(values, low, high, scale, center_weight):
        """Distance to one channel box, plus a small pull towards its centre."""
        low = float(low)
        high = float(high)
        scale = max(float(scale), 1e-6)
        outside = np.maximum(0.0, np.maximum(low - values, values - high)) / scale
        center = 0.5 * (low + high)
        return outside + float(center_weight) * np.abs(values - center) / scale

    def _color_distances(self, hue, saturation, value, color_ids):
        """Metric distance from colour samples to each configured colour.

        The configured HSV ranges double as prototypes: a sample inside a range
        costs nothing, and the small centre term decides the cases where two
        ranges overlap (green vs light blue in hue, black vs light blue in
        value). No hand-labelled training data is needed, and the ranges stay
        the single place where colour behaviour is tuned.
        """
        distances = {}
        for color_id in [str(item) for item in color_ids]:
            if color_id == "5":
                black = self.black_threshold
                if black is None:
                    distances[color_id] = None
                    continue
                dv = self._channel_distance(
                    value, 0.0, black["v_max"],
                    self.label_value_scale, self.label_center_weight,
                )
                ds = self._channel_distance(
                    saturation, black["s_min"], black["s_max"],
                    self.label_saturation_scale, self.label_center_weight,
                )
                distances[color_id] = np.sqrt(dv * dv + ds * ds)
                continue
            best = None
            for lower, upper in self.color_ranges.get(color_id, ()):
                dh = self._channel_distance(
                    hue, lower[0], upper[0],
                    self.label_hue_scale, self.label_center_weight,
                )
                ds = self._channel_distance(
                    saturation, lower[1], upper[1],
                    self.label_saturation_scale, self.label_center_weight,
                )
                dv = self._channel_distance(
                    value, lower[2], upper[2],
                    self.label_value_scale, self.label_center_weight,
                )
                distance = np.sqrt(dh * dh + ds * ds + dv * dv)
                best = distance if best is None else np.minimum(best, distance)
            distances[color_id] = best
        return distances

    @staticmethod
    def _contour_mask(contour, box):
        """Fill one contour into a mask of the given box."""
        box_x, box_y, width, height = [int(value) for value in box]
        width = max(1, width)
        height = max(1, height)
        local = np.zeros((height, width), dtype=np.uint8)
        shifted = contour - np.asarray([[[box_x, box_y]]], dtype=contour.dtype)
        cv2.drawContours(local, [shifted], -1, 255, -1)
        return local > 0

    @staticmethod
    def _box_gap(box_a, box_b):
        """Gap between two boxes; 0 when they touch or overlap."""
        left_a, top_a, width_a, height_a = box_a
        left_b, top_b, width_b, height_b = box_b
        gap_x = max(0, max(left_b - (left_a + width_a), left_a - (left_b + width_b)))
        gap_y = max(0, max(top_b - (top_a + height_a), top_a - (top_b + height_b)))
        return float(np.hypot(gap_x, gap_y))

    def _group_summary(self, hsv, masks, members):
        """Mean colour of one material, measured over the union of its blobs.

        Different colour masks cover slightly different pixels of the same
        material, so classification uses the union of all of them rather than
        only the pixels that happened to match one range.
        """
        left = int(min(item["box"][0] for item in members))
        top = int(min(item["box"][1] for item in members))
        right = int(max(item["box"][0] + item["box"][2] for item in members))
        bottom = int(max(item["box"][1] + item["box"][3] for item in members))
        width = max(1, right - left)
        height = max(1, bottom - top)
        box = [left, top, width, height]
        union = np.zeros((height, width), dtype=bool)
        for item in members:
            union |= self._contour_mask(item["contour"], box)
        matched = np.zeros((height, width), dtype=bool)
        for color_id in sorted({item["color_id"] for item in members}):
            matched |= masks[color_id][top:top + height, left:left + width] > 0
        pixels = union & matched
        count = int(np.count_nonzero(pixels))
        if count == 0:
            return None
        block = hsv[top:top + height, left:left + width]
        hue = block[..., 0][pixels].astype(np.float32)
        saturation = block[..., 1][pixels].astype(np.float32)
        value = block[..., 2][pixels].astype(np.float32)
        # Circular mean: a red material can sit on both ends of the hue axis.
        angle = np.deg2rad(hue * 2.0)
        mean_hue = float(
            (
                np.rad2deg(
                    np.arctan2(np.sin(angle).mean(), np.cos(angle).mean())
                )
                % 360.0
            )
            / 2.0
        )
        rows, columns = np.nonzero(pixels)
        return {
            "mean_hsv": [mean_hue, float(saturation.mean()), float(value.mean())],
            "pixels": count,
            "inside_px": int(np.count_nonzero(union)),
            "center_px": [float(left + columns.mean()), float(top + rows.mean())],
            "box": box,
        }

    def _materials_in_roi(self, hsv, scene_config, color_ids):
        """One entry per material: merged blobs plus a per-colour score."""
        masks = {}
        members = []
        filter_stats = {}
        for color_id in color_ids:
            mask = self._clean_mask(self._make_color_mask(hsv, color_id))
            masks[color_id] = mask
            candidates, _debug, stats = self._filter_candidates(
                mask, color_id, scene_config, hsv.shape[:2]
            )
            filter_stats[color_id] = stats
            for candidate in candidates:
                members.append(
                    {
                        "color_id": color_id,
                        "area": candidate[1],
                        "box": (
                            candidate[4], candidate[5],
                            candidate[6], candidate[7],
                        ),
                        "contour": candidate[8],
                    }
                )
        # Blue and light blue describe nearly the same pixels of one material,
        # so merge the blobs of different colours that touch into one material.
        groups = []
        for member in sorted(members, key=lambda item: item["area"], reverse=True):
            for group in groups:
                gap = min(
                    self._box_gap(member["box"], item["box"]) for item in group
                )
                if gap <= self.material_merge_distance:
                    group.append(member)
                    break
            else:
                groups.append([member])

        materials = []
        for group in groups:
            summary = self._group_summary(hsv, masks, group)
            if summary is None:
                continue
            mean_hue, mean_saturation, mean_value = summary["mean_hsv"]
            distances = self._color_distances(
                np.asarray([mean_hue], dtype=np.float32),
                np.asarray([mean_saturation], dtype=np.float32),
                np.asarray([mean_value], dtype=np.float32),
                color_ids,
            )
            scores = {}
            for color_id, distance in distances.items():
                if distance is None:
                    continue
                scores[color_id] = 1.0 / (1.0 + float(distance[0]))
            if not scores:
                continue
            ordered = sorted(scores.items(), key=lambda item: item[1], reverse=True)
            best_id, best_score = ordered[0]
            best_distance = 1.0 / max(best_score, 1e-6) - 1.0
            margin = best_score - ordered[1][1] if len(ordered) > 1 else 1.0
            if best_distance > self.material_max_distance:
                continue
            if self.material_min_margin > 0.0 and margin < self.material_min_margin:
                continue
            area = float(sum(item["area"] for item in group))
            quality = min(1.0, area / max(self.color_min_area * 2.0, 1.0))
            materials.append(
                {
                    "color_id": best_id,
                    "color_name": self.color_names[best_id],
                    "confidence": quality * best_score,
                    "area": area,
                    "bbox": list(summary["box"]),
                    "center": summary["center_px"],
                    "mean_hsv": [round(value, 2) for value in summary["mean_hsv"]],
                    "distance": best_distance,
                    "margin": margin,
                    "scores": scores,
                    "pixels": summary["pixels"],
                    "mask_colors": sorted({item["color_id"] for item in group}),
                    "contours": [item["contour"] for item in group],
                }
            )
        materials.sort(key=lambda item: item["area"], reverse=True)
        return materials, {
            "filter_stats": filter_stats,
            "member_blobs": len(members),
            "material_count": len(materials),
            "materials": materials,
        }

    def detect_materials(self, frame, scene, allowed_ids=None, roi_override=None):
        """Classify every visible material once: the round-batch entry point.

        A raw-material round carries three materials of three different
        colours, so the question is not "is this the colour I asked for" but
        "which colours are these three". Each material is classified from the
        mean colour of its whole blob, which keeps a glossy edge or a few stray
        pixels from deciding the label, and ``last_materials_debug`` keeps the
        per-colour scores so a caller can still solve the batch assignment.
        """
        scene = str(scene).upper()
        scene_config = self.config["scenes"].get(scene)
        if scene_config is None:
            raise KeyError("unknown scene: %s" % scene)
        color_ids = (
            [self._resolve_color_id(value) for value in allowed_ids]
            if allowed_ids is not None
            else sorted(self.color_names)
        )
        work = self._work_frame(frame)
        base_roi = roi_override or scene_config["object_roi"]
        roi, offset_x, offset_y = _crop(work, base_roi)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        entries, debug = self._materials_in_roi(hsv, scene_config, color_ids)
        results = []
        for entry in entries:
            center_x = entry["center"][0] + offset_x
            center_y = entry["center"][1] + offset_y
            entry["center"] = [center_x, center_y]
            box = entry["bbox"]
            entry["bbox"] = [
                box[0] + offset_x, box[1] + offset_y, box[2], box[3],
            ]
            entry["contours"] = [
                contour + np.asarray([[[offset_x, offset_y]]], dtype=contour.dtype)
                for contour in entry.get("contours", [])
            ]
            x, y, unit = self.calibration.map(scene, center_x, center_y)
            results.append(
                Measurement(
                    "COLOR", entry["color_id"], x, y, 0.0,
                    entry["confidence"], unit, 0.0,
                    float(center_x), float(center_y),
                )
            )
        debug["scene"] = scene
        debug["roi"] = _roi_values(base_roi)
        debug["color_ids"] = list(color_ids)
        debug["illumination"] = self.last_illumination
        self.last_materials_debug = debug
        return results

    def set_color_debug(self, enabled):
        self.color_debug_enabled = bool(enabled)
        if not self.color_debug_enabled:
            self.last_color_debug = None

    def set_circle_debug(self, enabled):
        """Keep circle candidates and the processed ROI for live tuning."""
        self.circle_debug_enabled = bool(enabled)
        if not self.circle_debug_enabled:
            self.last_circle_debug = None

    @staticmethod
    def _odd_kernel(value, label):
        value = int(value)
        if value < 1 or value % 2 == 0:
            raise ValueError("%s kernel must be a positive odd integer" % label)
        return value

    @staticmethod
    def _foreground_mask(gray, invert=True):
        threshold_type = cv2.THRESH_BINARY_INV if invert else cv2.THRESH_BINARY
        _value, mask = cv2.threshold(
            gray, 0, 255, threshold_type | cv2.THRESH_OTSU
        )
        return mask

    @staticmethod
    def _normalize_digit(mask, size, padding):
        contours, _hierarchy = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        height, width = mask.shape[:2]
        minimum_area = max(2.0, float(height * width) * 0.003)
        usable = []
        fallback = []
        for contour in contours:
            if cv2.contourArea(contour) < minimum_area:
                continue
            x, y, box_width, box_height = cv2.boundingRect(contour)
            fallback.append((x, y, box_width, box_height))
            if (
                x > 0
                and y > 0
                and x + box_width < width - 1
                and y + box_height < height - 1
            ):
                usable.append((x, y, box_width, box_height))
        boxes = usable or fallback
        if not boxes:
            return None
        left = min(box[0] for box in boxes)
        top = min(box[1] for box in boxes)
        right = max(box[0] + box[2] for box in boxes)
        bottom = max(box[1] + box[3] for box in boxes)
        digit = mask[top:bottom, left:right]
        if digit.size == 0:
            return None
        available = max(1, int(size) - 2 * int(padding))
        scale = min(
            float(available) / max(digit.shape[1], 1),
            float(available) / max(digit.shape[0], 1),
        )
        resized_width = max(1, int(round(digit.shape[1] * scale)))
        resized_height = max(1, int(round(digit.shape[0] * scale)))
        resized = cv2.resize(
            digit,
            (resized_width, resized_height),
            interpolation=cv2.INTER_NEAREST,
        )
        normalized = np.zeros((int(size), int(size)), dtype=np.uint8)
        offset_x = (int(size) - resized_width) // 2
        offset_y = (int(size) - resized_height) // 2
        normalized[
            offset_y : offset_y + resized_height,
            offset_x : offset_x + resized_width,
        ] = resized
        return normalized

    @staticmethod
    def _rotate_binary(image, angle):
        if abs(float(angle)) < 1e-6:
            return image
        size = image.shape[0]
        matrix = cv2.getRotationMatrix2D(
            ((size - 1) / 2.0, (size - 1) / 2.0), float(angle), 1.0
        )
        return cv2.warpAffine(
            image,
            matrix,
            (size, size),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )

    def _builtin_digit_template(self, digit, size, padding):
        canvas = np.zeros((128, 128), dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 3.2
        thickness = 7
        (text_width, text_height), _baseline = cv2.getTextSize(
            str(digit), font, font_scale, thickness
        )
        origin = (
            (canvas.shape[1] - text_width) // 2,
            (canvas.shape[0] + text_height) // 2,
        )
        cv2.putText(
            canvas,
            str(digit),
            origin,
            font,
            font_scale,
            255,
            thickness,
            cv2.LINE_AA,
        )
        return self._normalize_digit(canvas, size, padding)

    def _load_ring_digit_templates(self):
        config = self.ring_digit_config
        size = max(24, int(config.get("template_size", 64)))
        padding = max(1, int(config.get("padding_px", 5)))
        angles = [float(value) for value in config.get("rotation_degrees", [0])]
        template_directory = self.project_root / config.get(
            "template_directory", "config/ring_templates"
        )
        loaded_digits = set()
        for digit in ("1", "2", "3"):
            base_templates = []
            if template_directory.exists():
                for path in sorted(template_directory.glob("%s*" % digit)):
                    gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
                    if gray is None:
                        continue
                    mask = self._foreground_mask(
                        gray, bool(config.get("template_invert", True))
                    )
                    if int(np.count_nonzero(mask)) > mask.size // 2:
                        mask = cv2.bitwise_not(mask)
                    normalized = self._normalize_digit(mask, size, padding)
                    if normalized is not None:
                        base_templates.append(normalized)
                        loaded_digits.add(digit)
            if not base_templates and config.get("allow_builtin_fallback", True):
                base_templates.append(
                    self._builtin_digit_template(digit, size, padding)
                )
            variants = []
            for template in base_templates:
                for angle in angles:
                    variants.append(self._rotate_binary(template, angle))
            if variants:
                self.ring_digit_templates[digit] = variants
        if set(self.ring_digit_templates) != {"1", "2", "3"}:
            raise ValueError(
                "ring digit templates 1/2/3 are incomplete in %s"
                % template_directory
            )
        if loaded_digits == {"1", "2", "3"}:
            self.ring_template_source = "FILES"
        elif loaded_digits:
            self.ring_template_source = "MIXED_FILES_AND_BUILTIN"
        else:
            self.ring_template_source = "BUILTIN_SCREEN_TEST"

    def _validate_rois(self):
        width = int(self.config["camera"]["width"])
        height = int(self.config["camera"]["height"])
        for scene_name, scene in self.config["scenes"].items():
            rectangles = []
            if "object_roi" in scene:
                rectangles.append(("object_roi", scene["object_roi"]))
            if "turntable_roi" in scene:
                rectangles.append(("turntable_roi", scene["turntable_roi"]))
            if "ring_search_roi" in scene:
                rectangles.append(("ring_search_roi", scene["ring_search_roi"]))
            for ring_id, rectangle in scene.get("ring_rois", {}).items():
                rectangles.append(("ring_rois.%s" % ring_id, rectangle))
            for name, rectangle in rectangles:
                x, y, roi_width, roi_height = _roi_values(rectangle)
                if (
                    x < 0
                    or y < 0
                    or roi_width <= 0
                    or roi_height <= 0
                    or x + roi_width > width
                    or y + roi_height > height
                ):
                    raise ValueError(
                        "%s.%s exceeds configured camera frame" % (scene_name, name)
                    )

    def _resolve_color_id(self, color):
        value = str(color)
        if value in self.color_names:
            return value
        wanted = value.upper()
        for color_id, name in self.color_names.items():
            if name == wanted:
                return color_id
        raise KeyError("unknown color: %s" % value)

    def activate_dynamic_roi(self, kind, color_id, scene):
        """Enable the small ROI only after the session reports STABLE."""
        if self.dynamic_roi_enabled:
            key = (str(kind).upper(), self._resolve_color_id(color_id), str(scene).upper())
            if key in self._last_color_boxes:
                self._dynamic_roi_active.add(key)

    def _expanded_dynamic_roi(self, base_roi, bbox):
        base_x, base_y, base_width, base_height = _roi_values(base_roi)
        box_x, box_y, box_width, box_height = [int(value) for value in bbox]
        left = max(base_x, box_x - self.dynamic_roi_margin)
        top = max(base_y, box_y - self.dynamic_roi_margin)
        right = min(
            base_x + base_width,
            box_x + box_width + self.dynamic_roi_margin,
        )
        bottom = min(
            base_y + base_height,
            box_y + box_height + self.dynamic_roi_margin,
        )
        return [left, top, right - left, bottom - top]

    def _make_color_mask(self, hsv, color_id):
        if color_id == "5":
            black = self.black_threshold
            # TODO(real material only): add an optional local brightness check
            # here only if fixed lighting still cannot separate black/shadow.
            return cv2.inRange(
                hsv,
                np.asarray([0, black["s_min"], 0], dtype=np.uint8),
                np.asarray([179, black["s_max"], black["v_max"]], dtype=np.uint8),
            )
        mask = None
        for lower, upper in self.color_ranges[color_id]:
            part = cv2.inRange(hsv, lower, upper)
            if mask is None:
                mask = part
            else:
                cv2.bitwise_or(mask, part, dst=mask)
        # TODO(real material only): an optional Lab second-stage hook may be
        # added after this mask if blue/light-blue HSV ranges truly overlap.
        return mask

    def _clean_mask(self, mask):
        """Apply the configured colour morphology to one binary mask."""
        if self.color_open_iterations:
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN, self.color_kernel,
                iterations=self.color_open_iterations,
            )
        if self.color_close_iterations:
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_CLOSE, self.color_kernel,
                iterations=self.color_close_iterations,
            )
        return mask

    def _filter_candidates(
        self, mask, color_id, scene_config, roi_shape,
        offset_x=0, offset_y=0,
    ):
        """Contour, hull and geometry filters shared by every colour path."""
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        minimum = float(scene_config.get("min_area", self.color_min_area))
        maximum_value = scene_config.get("max_area", self.color_max_area)
        maximum = float(maximum_value) if maximum_value is not None else float("inf")
        filter_stats = {
            "raw_contours": len(contours),
            "area_rejected": 0,
            "shape_rejected": 0,
            "border_rejected": 0,
            "accepted": 0,
            "area_limits_px": [minimum, maximum if np.isfinite(maximum) else None],
        }
        candidates = []
        debug_candidates = []
        for contour in contours:
            # The real material is rotationally symmetric. Using its convex
            # outer silhouette keeps the center stable when glare removes a
            # notch from the color mask, without adding contour clustering.
            hull = cv2.convexHull(contour)
            area = float(cv2.contourArea(hull))
            if area < minimum or area > maximum:
                filter_stats["area_rejected"] += 1
                continue
            box_x, box_y, width, height = cv2.boundingRect(hull)
            aspect_ratio = float(width) / max(float(height), 1.0)
            perimeter = float(cv2.arcLength(hull, True))
            circularity = (
                4.0 * np.pi * area / (perimeter * perimeter)
                if perimeter > 0.0 else 0.0
            )
            if not (
                self.color_min_aspect_ratio
                <= aspect_ratio
                <= self.color_max_aspect_ratio
            ) or (
                color_id == "5"
                and circularity < self.black_min_circularity
            ):
                filter_stats["shape_rejected"] += 1
                continue
            margin = self.color_border_margin
            if (
                box_x <= margin
                or box_y <= margin
                or box_x + width >= roi_shape[1] - margin
                or box_y + height >= roi_shape[0] - margin
            ):
                # A clipped target has an unreliable center and should never
                # be handed to the gripper, regardless of its color.
                filter_stats["border_rejected"] += 1
                continue
            moments = cv2.moments(hull)
            if moments["m00"] == 0:
                filter_stats["area_rejected"] += 1
                continue
            center_x = moments["m10"] / moments["m00"] + offset_x
            center_y = moments["m01"] / moments["m00"] + offset_y
            confidence = min(1.0, area / max(minimum * 2.0, 1.0))
            global_box = [box_x + offset_x, box_y + offset_y, width, height]
            candidate = (
                confidence, area, center_x, center_y,
                global_box[0], global_box[1], width, height, hull,
            )
            candidates.append(candidate)
            filter_stats["accepted"] += 1
            if self.color_debug_enabled:
                global_contour = hull + np.asarray(
                    [[[offset_x, offset_y]]], dtype=contour.dtype
                )
                debug_candidates.append(
                    {
                        "bbox": global_box,
                        "contour": global_contour,
                        "center": [float(center_x), float(center_y)],
                        "area": area,
                        "aspect_ratio": aspect_ratio,
                        "circularity": circularity,
                        "quality": confidence,
                        "accepted": True,
                    }
                )

        return candidates, debug_candidates, filter_stats

    def _color_candidate(
        self, frame, color_id, roi_rect, scene_config, prepared_hsv=None,
        mask_override=None,
    ):
        if prepared_hsv is None:
            roi, offset_x, offset_y = _crop(frame, roi_rect)
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        else:
            hsv, offset_x, offset_y = prepared_hsv
            # Only the dimensions are used below for border rejection.
            roi = hsv
        if mask_override is None:
            mask = self._make_color_mask(hsv, color_id)
        else:
            # Mutually exclusive label mask: another colour already claimed
            # these pixels, which is how blue/light-blue and black/shadow stop
            # producing two detections of the same material.
            mask = mask_override
        mask = self._clean_mask(mask)
        candidates, debug_candidates, filter_stats = self._filter_candidates(
            mask, color_id, scene_config, roi.shape[:2], offset_x, offset_y
        )
        self._last_color_filter_stats = filter_stats
        selected = max(candidates, key=lambda item: item[1]) if candidates else None
        if self.color_debug_enabled:
            selected_debug = None
            if selected is not None:
                selected_debug = min(
                    debug_candidates,
                    key=lambda item: (
                        abs(item["center"][0] - selected[2])
                        + abs(item["center"][1] - selected[3])
                    ),
                )
            self.last_color_debug = {
                "requested_id": color_id,
                "color_name": self.color_names[color_id],
                "roi": _roi_values(roi_rect),
                "mask": mask,
                "candidates": debug_candidates,
                "selected": selected_debug,
            }
        return selected

    def detect_color(self, frame, color_id, scene, roi_override=None, kind="COLOR"):
        scene = scene.upper()
        color_id = self._resolve_color_id(color_id)
        scene_config = self.config["scenes"].get(scene)
        if scene_config is None:
            raise KeyError("unknown scene: %s" % scene)
        work = self._work_frame(frame)
        base_roi = roi_override or scene_config["object_roi"]
        key = (str(kind).upper(), color_id, scene)
        candidate = None
        active_roi = base_roi
        roi_mode = "FIXED_ROI"
        if (
            self.dynamic_roi_enabled
            and key in self._dynamic_roi_active
            and key in self._last_color_boxes
        ):
            active_roi = self._expanded_dynamic_roi(
                base_roi, self._last_color_boxes[key]
            )
            candidate = self._color_candidate(
                work, color_id, active_roi, scene_config
            )
            if candidate is not None:
                roi_mode = "DYNAMIC_ROI"
        if candidate is None:
            if key in self._dynamic_roi_active:
                self._dynamic_roi_active.discard(key)
            active_roi = base_roi
            candidate = self._color_candidate(
                work, color_id, base_roi, scene_config
            )
        self.last_color_runtime = {
            "mode": roi_mode,
            "roi": _roi_values(active_roi),
            "base_roi": _roi_values(base_roi),
            "filter_stats": dict(self._last_color_filter_stats or {}),
        }
        if self.last_color_debug is not None:
            self.last_color_debug["base_roi"] = _roi_values(base_roi)
        if candidate is None:
            self._last_color_boxes.pop(key, None)
            return None
        confidence, _, x_px, y_px, box_x, box_y, width, height, _hull = candidate
        self._last_color_boxes[key] = [box_x, box_y, width, height]
        x, y, unit = self.calibration.map(scene, x_px, y_px)
        return Measurement(
            kind, color_id, x, y, 0.0, confidence, unit, 0.0,
            float(x_px), float(y_px),
        )

    def detect_any_color(self, frame, scene, allowed_ids=None):
        """Classify the strongest material among the allowed color IDs.

        This is intended for turntable inspection, where reporting a
        non-target color is meaningful.  Target-driven PICK continues to use
        ``detect_color`` and is unaffected.
        """
        color_ids = (
            [self._resolve_color_id(value) for value in allowed_ids]
            if allowed_ids is not None
            else sorted(self.color_ranges)
        )
        results = self.detect_colors(frame, scene, color_ids)
        if not results:
            return None
        return max(results, key=lambda item: item.confidence)

    def detect_colors(self, frame, scene, allowed_ids=None):
        """Return one strongest measurement for every visible allowed color."""
        scene = str(scene).upper()
        scene_config = self.config["scenes"].get(scene)
        if scene_config is None:
            raise KeyError("unknown scene: %s" % scene)
        color_ids = (
            [self._resolve_color_id(value) for value in allowed_ids]
            if allowed_ids is not None
            else sorted(self.color_ranges)
        )
        # A dynamic ROI can differ for each tracked color, so retain the
        # single-color path when that optional mode is active. Competition
        # uses the full search ROI: crop and convert it to HSV once, then
        # reuse it for every requested color.
        if self.dynamic_roi_enabled:
            results = []
            debug_results = []
            for color_id in color_ids:
                measurement = self.detect_color(frame, color_id, scene)
                if self.color_debug_enabled and self.last_color_debug is not None:
                    debug_results.append(self.last_color_debug)
                if measurement is not None:
                    results.append(measurement)
            self.last_colors_debug = debug_results
            return results

        base_roi = scene_config["object_roi"]
        work = self._work_frame(frame)
        roi, offset_x, offset_y = _crop(work, base_roi)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        prepared_hsv = (hsv, offset_x, offset_y)
        results = []
        debug_results = []
        filter_stats = {}
        for color_id in color_ids:
            candidate = self._color_candidate(
                work, color_id, base_roi, scene_config, prepared_hsv=prepared_hsv
            )
            if self.color_debug_enabled and self.last_color_debug is not None:
                debug_results.append(self.last_color_debug)
            filter_stats[color_id] = dict(self._last_color_filter_stats or {})
            key = ("COLOR", color_id, scene)
            if candidate is None:
                self._last_color_boxes.pop(key, None)
                continue
            confidence, _, x_px, y_px, box_x, box_y, width, height, _hull = candidate
            self._last_color_boxes[key] = [box_x, box_y, width, height]
            x, y, unit = self.calibration.map(scene, x_px, y_px)
            results.append(
                Measurement(
                    "COLOR", color_id, x, y, 0.0, confidence, unit, 0.0,
                    float(x_px), float(y_px),
                )
            )
        self.last_color_runtime = {
            "mode": "SHARED_FIXED_ROI",
            "roi": _roi_values(base_roi),
            "base_roi": _roi_values(base_roi),
            "filter_stats_by_color": filter_stats,
        }
        self.last_colors_debug = debug_results
        return results

    def _detect_circle_in_roi(self, frame, roi_rect, parameters):
        roi, offset_x, offset_y = _crop(frame, roi_rect)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        scale = float(parameters.get("processing_scale", 1.0))
        if not 0.25 <= scale <= 1.0:
            raise ValueError("circle processing_scale must be in [0.25, 1.0]")
        if scale < 1.0:
            gray = cv2.resize(
                gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
            )
        gray = self.circle_clahe.apply(gray)
        configured_blur = self._odd_kernel(
            parameters.get("blur_kernel_px", 7), "circle blur"
        )
        blur_size = max(1, int(round(configured_blur * scale)))
        if blur_size % 2 == 0:
            blur_size += 1
        gray = cv2.GaussianBlur(gray, (blur_size, blur_size), 1.5)
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=float(parameters.get("dp", 1.2)),
            minDist=float(parameters.get("min_distance_px", 30)) * scale,
            param1=float(parameters.get("edge_threshold", 100)),
            param2=float(parameters.get("center_threshold", 24)),
            minRadius=max(1, int(round(parameters["min_radius_px"] * scale))),
            maxRadius=max(2, int(round(parameters["max_radius_px"] * scale))),
        )
        debug = None
        if self.circle_debug_enabled:
            debug = {
                "roi": _roi_values(roi_rect),
                "processed": gray,
                "candidates": [],
                "selected": None,
            }
        if circles is None:
            self.last_circle_debug = debug
            return None
        expected = float(parameters.get("expected_radius_px", 0))
        candidates = []
        for center_x, center_y, radius in circles[0]:
            center_x = float(center_x) / scale
            center_y = float(center_y) / scale
            radius = float(radius) / scale
            radius_score = (
                max(0.0, 1.0 - abs(float(radius) - expected) / expected)
                if expected > 0
                else 0.7
            )
            border = min(
                center_x,
                center_y,
                roi.shape[1] - center_x,
                roi.shape[0] - center_y,
            )
            border_score = max(0.0, min(1.0, border / max(float(radius), 1.0)))
            quality = 0.7 * radius_score + 0.3 * border_score
            candidate = (quality, float(center_x), float(center_y), float(radius))
            candidates.append(candidate)
            if debug is not None:
                debug["candidates"].append(
                    {
                        "center": [center_x + offset_x, center_y + offset_y],
                        "radius": radius,
                        "quality": quality,
                    }
                )
        selected = max(candidates, key=lambda item: item[0])
        quality, center_x, center_y, radius = selected
        if debug is not None:
            selected_index = candidates.index(selected)
            debug["selected"] = debug["candidates"][selected_index]
            self.last_circle_debug = debug
        return center_x + offset_x, center_y + offset_y, radius, quality

    def _make_ring_mask(self, gray, threshold_config):
        mode = str(threshold_config.get("mode", "fixed")).lower()
        threshold_type = (
            cv2.THRESH_BINARY_INV
            if bool(threshold_config.get("invert", True))
            else cv2.THRESH_BINARY
        )
        if mode == "fixed":
            _value, mask = cv2.threshold(
                gray,
                int(threshold_config.get("value", 120)),
                255,
                threshold_type,
            )
            return mask
        if mode == "adaptive":
            block_size = self._odd_kernel(
                threshold_config.get("adaptive_block_size", 31),
                "ring adaptive threshold",
            )
            if block_size < 3:
                raise ValueError("ring adaptive threshold block must be at least 3")
            return cv2.adaptiveThreshold(
                gray,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                threshold_type,
                block_size,
                float(threshold_config.get("adaptive_c", 5)),
            )
        raise ValueError("ring threshold mode must be fixed or adaptive")

    @staticmethod
    def _merge_ring_centers(candidates, maximum_distance):
        """Greedily merge concentric contours without a clustering library."""
        groups = []
        for candidate in sorted(
            candidates, key=lambda item: item["area"], reverse=True
        ):
            matched = None
            for group in groups:
                center_x = sum(item["center"][0] for item in group) / len(group)
                center_y = sum(item["center"][1] for item in group) / len(group)
                distance = float(
                    np.hypot(
                        candidate["center"][0] - center_x,
                        candidate["center"][1] - center_y,
                    )
                )
                if distance <= maximum_distance:
                    matched = group
                    break
            if matched is None:
                groups.append([candidate])
            else:
                matched.append(candidate)
        return groups

    def _detect_ring_contours_in_roi(self, frame, roi_rect, parameters):
        roi, offset_x, offset_y = _crop(frame, roi_rect)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        mask = self._make_ring_mask(gray, parameters["threshold"])
        if self.ring_open_iterations:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                self.ring_kernel,
                iterations=self.ring_open_iterations,
            )
        if self.ring_close_iterations:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                self.ring_kernel,
                iterations=self.ring_close_iterations,
            )

        contours, _hierarchy = cv2.findContours(
            mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
        )
        contour_config = parameters["contour"]
        minimum_area = float(contour_config["min_area"])
        maximum_area = float(contour_config["max_area"])
        minimum_circularity = float(contour_config["min_circularity"])
        minimum_aspect = float(contour_config["min_aspect_ratio"])
        maximum_aspect = float(contour_config["max_aspect_ratio"])
        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < minimum_area or area > maximum_area:
                continue
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 0:
                continue
            circularity = float(4.0 * np.pi * area / (perimeter * perimeter))
            box_x, box_y, width, height = cv2.boundingRect(contour)
            aspect = float(width) / max(float(height), 1.0)
            if (
                circularity < minimum_circularity
                or aspect < minimum_aspect
                or aspect > maximum_aspect
            ):
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            center_x = float(moments["m10"] / moments["m00"] + offset_x)
            center_y = float(moments["m01"] / moments["m00"] + offset_y)
            global_contour = contour + np.asarray(
                [[[offset_x, offset_y]]], dtype=contour.dtype
            )
            candidates.append(
                {
                    "center": [center_x, center_y],
                    "area": area,
                    "circularity": circularity,
                    "aspect_ratio": aspect,
                    "bbox": [box_x + offset_x, box_y + offset_y, width, height],
                    "radius": (float(width) + float(height)) / 4.0,
                    "quality": min(1.0, circularity),
                    "contour": global_contour,
                    "selected": False,
                }
            )

        merge_distance = float(
            parameters["clustering"].get("center_merge_distance_px", 12)
        )
        groups = self._merge_ring_centers(candidates, merge_distance)
        minimum_members = max(
            1,
            int(parameters["clustering"].get("min_concentric_contours", 2)),
        )
        clusters = []
        for group in groups:
            if len(group) < minimum_members:
                continue
            center_x = sum(item["center"][0] for item in group) / len(group)
            center_y = sum(item["center"][1] for item in group) / len(group)
            quality = sum(item["quality"] for item in group) / len(group)
            radius = max(item["radius"] for item in group)
            group_index = len(clusters)
            for candidate in group:
                candidate["group_index"] = group_index
            clusters.append(
                {
                    "center": [center_x, center_y],
                    "radius": radius,
                    "quality": quality,
                    "members": len(group),
                    "group_index": group_index,
                    "ring_id": "UNKNOWN",
                    "digit_score": 0.0,
                    "digit_bbox": None,
                }
            )
        debug = None
        if self.circle_debug_enabled:
            debug = {
                "roi": _roi_values(roi_rect),
                "processed": mask,
                "candidates": candidates,
                "selected": None,
                "candidate_count": len(candidates),
                "cluster_count": len(clusters),
                "groups": clusters,
                "threshold": dict(parameters["threshold"]),
            }
        self.last_circle_debug = debug
        return clusters

    @staticmethod
    def _template_score(observed, template):
        observed_pixels = observed > 0
        template_pixels = template > 0
        total = int(observed_pixels.sum()) + int(template_pixels.sum())
        if total == 0:
            return 0.0
        intersection = int(np.logical_and(observed_pixels, template_pixels).sum())
        return float(2.0 * intersection / total)

    def _classify_ring_digit(self, frame, cluster):
        config = self.ring_digit_config
        center_x, center_y = cluster["center"]
        crop_ratio = float(config.get("crop_radius_ratio", 0.55))
        half_size = max(8, int(round(cluster["radius"] * crop_ratio)))
        left = max(0, int(round(center_x)) - half_size)
        top = max(0, int(round(center_y)) - half_size)
        right = min(frame.shape[1], int(round(center_x)) + half_size + 1)
        bottom = min(frame.shape[0], int(round(center_y)) + half_size + 1)
        if right - left < 8 or bottom - top < 8:
            return "UNKNOWN", 0.0, [left, top, right - left, bottom - top], None
        gray = cv2.cvtColor(frame[top:bottom, left:right], cv2.COLOR_BGR2GRAY)
        mask = self._foreground_mask(
            gray, bool(config.get("image_invert", True))
        )
        normalized = self._normalize_digit(
            mask,
            max(24, int(config.get("template_size", 64))),
            max(1, int(config.get("padding_px", 5))),
        )
        if normalized is None:
            return "UNKNOWN", 0.0, [left, top, right - left, bottom - top], None
        scores = {}
        for digit, templates in self.ring_digit_templates.items():
            scores[digit] = max(
                self._template_score(normalized, template)
                for template in templates
            )
        digit, score = max(scores.items(), key=lambda item: item[1])
        if score < float(config.get("min_score", 0.55)):
            digit = "UNKNOWN"
        return digit, float(score), [left, top, right - left, bottom - top], normalized

    def detect_ring(self, frame, ring_id, scene):
        scene = scene.upper()
        ring_id = str(ring_id)
        if ring_id not in ("1", "2", "3"):
            raise KeyError("unknown ring id: %s" % ring_id)
        scene_config = self.config["scenes"].get(scene)
        if scene_config is None:
            raise KeyError("ring scene is not configured")
        digit_enabled = bool(self.ring_digit_config.get("enabled", False))
        if digit_enabled:
            search_roi = scene_config.get(
                "ring_search_roi", scene_config.get("object_roi")
            )
        else:
            search_roi = scene_config.get("ring_rois", {}).get(ring_id)
        if search_roi is None:
            raise KeyError("ring search ROI is not configured")
        clusters = self._detect_ring_contours_in_roi(
            frame, search_roi, self.config["ring_detection"]
        )
        normalized_digits = {}
        if digit_enabled:
            for cluster in clusters:
                digit, score, bbox, normalized = self._classify_ring_digit(
                    frame, cluster
                )
                cluster["ring_id"] = digit
                cluster["digit_score"] = score
                cluster["digit_bbox"] = bbox
                if normalized is not None:
                    normalized_digits[cluster["group_index"]] = normalized
            matches = [
                cluster for cluster in clusters if cluster["ring_id"] == ring_id
            ]
        else:
            matches = clusters
        selected = (
            max(
                matches,
                key=lambda cluster: (
                    cluster["digit_score"] if digit_enabled else 1.0,
                    cluster["members"],
                    cluster["quality"],
                ),
            )
            if matches
            else None
        )
        if self.last_circle_debug is not None:
            self.last_circle_debug["kind"] = "RING"
            self.last_circle_debug["target_id"] = ring_id
            self.last_circle_debug["scene"] = scene
            self.last_circle_debug["template_source"] = self.ring_template_source
            self.last_circle_debug["normalized_digits"] = normalized_digits
            self.last_circle_debug["selected"] = selected
            if selected is not None:
                for candidate in self.last_circle_debug["candidates"]:
                    candidate["selected"] = (
                        candidate.get("group_index")
                        == selected["group_index"]
                    )
        if selected is None:
            return None
        x_px, y_px = selected["center"]
        quality = selected["quality"]
        if digit_enabled:
            quality = min(quality, selected["digit_score"])
        x, y, unit = self.calibration.map(scene, x_px, y_px)
        return Measurement(
            "RING",
            ring_id,
            x,
            y,
            0.0,
            quality,
            unit,
            0.0,
            float(x_px),
            float(y_px),
        )

    def detect_station(self, frame, scene):
        scene = scene.upper()
        scene_config = self.config["scenes"].get(scene)
        if scene_config is None:
            raise KeyError("scene is not configured")
        ring_ids = sorted(scene_config.get("ring_rois", {}).keys())
        if len(ring_ids) < 2:
            raise ValueError("station needs at least two configured rings")
        detections = []
        for ring_id in ring_ids:
            detected = self.detect_ring(frame, ring_id, scene)
            if detected is None:
                return None
            detections.append(detected)
        units = {detected.unit for detected in detections}
        if len(units) != 1:
            raise ValueError("mixed station units")
        unit = detections[0].unit
        reference_key = "station_reference_mm" if unit == "MM" else "station_reference_px"
        reference_map = scene_config.get(reference_key, {})
        if any(ring_id not in reference_map for ring_id in ring_ids):
            raise ValueError("missing %s" % reference_key)
        reference = [reference_map[ring_id] for ring_id in ring_ids]
        observed = [(detected.x, detected.y) for detected in detections]
        x, y, yaw, residual = estimate_rigid_pose(reference, observed)
        tolerance = float(scene_config.get("max_station_residual", 8.0 if unit == "MM" else 15.0))
        residual_score = max(0.0, 1.0 - residual / max(tolerance, 1e-6))
        quality = min(detected.confidence for detected in detections) * residual_score
        if residual > tolerance:
            return None
        return Measurement("STATION", scene, x, y, yaw, quality, unit, residual)

    def detect_turntable(self, frame, scene="TURNTABLE"):
        scene = scene.upper()
        scene_config = self.config["scenes"].get(scene)
        if scene_config is None:
            raise KeyError("turntable scene is not configured")
        circle = self._detect_circle_in_roi(
            frame,
            scene_config["turntable_roi"],
            self.config["turntable_detection"],
        )
        if circle is None:
            return None
        x_px, y_px, _, quality = circle
        x, y, unit = self.calibration.map(scene, x_px, y_px)
        return Measurement(
            "TURNTABLE",
            scene,
            x,
            y,
            0.0,
            quality,
            unit,
            0.0,
            float(x_px),
            float(y_px),
        )

    def detect_stack(self, frame, color_id, scene, ring_id):
        scene_config = self.config["scenes"].get(scene.upper())
        if scene_config is None or str(ring_id) not in scene_config.get("ring_rois", {}):
            raise KeyError("stack ring ROI is not configured")
        # The camera is mounted on the arm, so a stored object must not be
        # assumed to remain inside a hard-coded image-space ring rectangle.
        # Search the scene-level work area for the requested same-color object;
        # ring_id remains validated as task metadata for protocol consistency.
        return self.detect_color(
            frame,
            color_id,
            scene,
            kind="STACK",
        )
