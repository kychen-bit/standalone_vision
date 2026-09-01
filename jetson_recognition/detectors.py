"""Traditional eye-in-hand recognition reused from the main project."""

import json

import cv2
import numpy as np

from .geometry import apply_homography, estimate_rigid_pose
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
        calibration_path = project_root / config["camera"].get(
            "calibration_file", "config/calibration.json"
        )
        self.calibration = PlaneCalibration(calibration_path)

        # COLOR_V1 intentionally stays small: fixed ROI, HSV inRange,
        # morphology, contour area and moments. Complex classifiers belong in
        # a future optional detector, not in the default competition path.
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
        dynamic_roi = color_config.get("dynamic_roi", {})
        self.dynamic_roi_enabled = bool(dynamic_roi.get("enabled", False))
        self.dynamic_roi_margin = max(0, int(dynamic_roi.get("margin_px", 80)))
        self._dynamic_roi_active = set()
        self._last_color_boxes = {}
        self.color_debug_enabled = False
        self.last_color_debug = None
        self.last_color_runtime = None
        self._last_color_filter_stats = None

        ring_config = config["ring_detection"]
        grid_size = max(1, int(ring_config.get("clahe_grid_size", 8)))
        self.circle_clahe = cv2.createCLAHE(
            clipLimit=float(ring_config.get("clahe_clip_limit", 2.5)),
            tileGridSize=(grid_size, grid_size),
        )
        self._validate_rois()

    def set_color_debug(self, enabled):
        self.color_debug_enabled = bool(enabled)
        if not self.color_debug_enabled:
            self.last_color_debug = None

    @staticmethod
    def _odd_kernel(value, label):
        value = int(value)
        if value < 1 or value % 2 == 0:
            raise ValueError("%s kernel must be a positive odd integer" % label)
        return value

    def _validate_rois(self):
        width = int(self.config["camera"]["width"])
        height = int(self.config["camera"]["height"])
        for scene_name, scene in self.config["scenes"].items():
            rectangles = []
            if "object_roi" in scene:
                rectangles.append(("object_roi", scene["object_roi"]))
            if "turntable_roi" in scene:
                rectangles.append(("turntable_roi", scene["turntable_roi"]))
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

    def _color_candidate(
        self, frame, color_id, roi_rect, scene_config, reject_border=False
    ):
        roi, offset_x, offset_y = _crop(frame, roi_rect)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = self._make_color_mask(hsv, color_id)
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

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        minimum = float(scene_config.get("min_area", self.color_min_area))
        maximum_value = scene_config.get("max_area", self.color_max_area)
        maximum = float(maximum_value) if maximum_value is not None else float("inf")
        filter_stats = {
            "raw_contours": len(contours),
            "area_rejected": 0,
            "border_rejected": 0,
            "accepted": 0,
            "area_limits_px": [minimum, maximum if np.isfinite(maximum) else None],
        }
        candidates = []
        debug_candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < minimum or area > maximum:
                filter_stats["area_rejected"] += 1
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                filter_stats["area_rejected"] += 1
                continue
            center_x = moments["m10"] / moments["m00"] + offset_x
            center_y = moments["m01"] / moments["m00"] + offset_y
            box_x, box_y, width, height = cv2.boundingRect(contour)
            if reject_border and (
                box_x <= 1
                or box_y <= 1
                or box_x + width >= roi.shape[1] - 1
                or box_y + height >= roi.shape[0] - 1
            ):
                # A clipped contour has a biased center. Reject this fast pass
                # and retry the fixed ROI in the same frame.
                filter_stats["border_rejected"] += 1
                continue
            confidence = min(1.0, area / max(minimum * 2.0, 1.0))
            global_box = [box_x + offset_x, box_y + offset_y, width, height]
            candidate = (
                confidence, area, center_x, center_y,
                global_box[0], global_box[1], width, height,
            )
            candidates.append(candidate)
            filter_stats["accepted"] += 1
            if self.color_debug_enabled:
                global_contour = contour + np.asarray(
                    [[[offset_x, offset_y]]], dtype=contour.dtype
                )
                debug_candidates.append(
                    {
                        "bbox": global_box,
                        "contour": global_contour,
                        "center": [float(center_x), float(center_y)],
                        "area": area,
                        "quality": confidence,
                        "accepted": True,
                    }
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
                frame, color_id, active_roi, scene_config, reject_border=True
            )
            if candidate is not None:
                roi_mode = "DYNAMIC_ROI"
        if candidate is None:
            if key in self._dynamic_roi_active:
                self._dynamic_roi_active.discard(key)
            active_roi = base_roi
            candidate = self._color_candidate(
                frame, color_id, base_roi, scene_config
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
        confidence, _, x_px, y_px, box_x, box_y, width, height = candidate
        self._last_color_boxes[key] = [box_x, box_y, width, height]
        x, y, unit = self.calibration.map(scene, x_px, y_px)
        return Measurement(
            kind, color_id, x, y, 0.0, confidence, unit, 0.0,
            float(x_px), float(y_px),
        )

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
        if circles is None:
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
            candidates.append((quality, float(center_x), float(center_y), float(radius)))
        quality, center_x, center_y, radius = max(candidates, key=lambda item: item[0])
        return center_x + offset_x, center_y + offset_y, radius, quality

    def detect_ring(self, frame, ring_id, scene):
        scene = scene.upper()
        scene_config = self.config["scenes"].get(scene)
        if scene_config is None or str(ring_id) not in scene_config.get("ring_rois", {}):
            raise KeyError("ring ROI is not configured")
        circle = self._detect_circle_in_roi(
            frame,
            scene_config["ring_rois"][str(ring_id)],
            self.config["ring_detection"],
        )
        if circle is None:
            return None
        x_px, y_px, _, quality = circle
        x, y, unit = self.calibration.map(scene, x_px, y_px)
        return Measurement(
            "RING",
            str(ring_id),
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
        return self.detect_color(
            frame,
            color_id,
            scene,
            roi_override=scene_config["ring_rois"][str(ring_id)],
            kind="STACK",
        )
