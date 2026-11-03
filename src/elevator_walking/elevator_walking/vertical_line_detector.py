#!/usr/bin/env python3
"""Shared Canny/Hough vertical-line detector and preview renderer."""

import math
import statistics
from typing import List, NamedTuple, Tuple

import cv2
import numpy as np


DETECTOR_DEFAULTS = {
    "process_width": 960,
    "canny_low_threshold": 10,
    "canny_high_threshold": 220,
    "show_edges": True,
    "min_line_length_px": 60,
    "hough_threshold": 40,
    "max_line_gap_px": 12,
    "max_angle_from_vertical_deg": 8.0,
    "merge_tolerance_px": 20.0,
    "center_tolerance_px": 40.0,
    "roi_top_ratio": 0.05,
    "roi_bottom_ratio": 0.95,
}


class DetectionResult(NamedTuple):
    scale: float
    edges: np.ndarray
    candidates: List[Tuple[float, Tuple[int, int, int, int]]]
    clusters: list


def declare_detector_parameters(node):
    """Declare the same detector parameters on preview and movement nodes."""
    for name, default in DETECTOR_DEFAULTS.items():
        node.declare_parameter(name, default)


class VerticalLineDetector:
    """Detect near-vertical Hough segments and group them by image x position."""

    def __init__(self, **parameters):
        values = dict(DETECTOR_DEFAULTS)
        values.update(parameters)
        self.process_width = int(values["process_width"])
        self.canny_low_threshold = int(values["canny_low_threshold"])
        self.canny_high_threshold = int(values["canny_high_threshold"])
        self.show_edges = bool(values["show_edges"])
        self.min_line_length_px = int(values["min_line_length_px"])
        self.hough_threshold = int(values["hough_threshold"])
        self.max_line_gap_px = int(values["max_line_gap_px"])
        self.max_angle_from_vertical_deg = float(
            values["max_angle_from_vertical_deg"]
        )
        self.merge_tolerance_px = float(values["merge_tolerance_px"])
        self.center_tolerance_px = float(values["center_tolerance_px"])
        self.roi_top_ratio = float(values["roi_top_ratio"])
        self.roi_bottom_ratio = float(values["roi_bottom_ratio"])
        self._validate_parameters()

    @classmethod
    def from_node_parameters(cls, node):
        values = {
            name: node.get_parameter(name).value
            for name in DETECTOR_DEFAULTS
        }
        return cls(**values)

    def _validate_parameters(self):
        if self.process_width < 100:
            raise ValueError("process_width must be at least 100 pixels")
        if not (
            0 <= self.canny_low_threshold < self.canny_high_threshold <= 255
        ):
            raise ValueError(
                "Canny thresholds must satisfy 0 <= low < high <= 255"
            )
        if self.min_line_length_px < 1 or self.hough_threshold < 1:
            raise ValueError("line length and Hough threshold must be positive")
        if self.max_line_gap_px < 0:
            raise ValueError("max_line_gap_px cannot be negative")
        if not math.isfinite(self.max_angle_from_vertical_deg) or not (
            0.0 <= self.max_angle_from_vertical_deg < 45.0
        ):
            raise ValueError("max_angle_from_vertical_deg must be in [0, 45)")
        if (
            not math.isfinite(self.merge_tolerance_px)
            or not math.isfinite(self.center_tolerance_px)
            or self.merge_tolerance_px <= 0.0
            or self.center_tolerance_px <= 0.0
        ):
            raise ValueError("line tolerances must be finite and positive")
        if not (
            0.0 <= self.roi_top_ratio < self.roi_bottom_ratio <= 1.0
        ):
            raise ValueError("ROI ratios must satisfy 0 <= top < bottom <= 1")

    def detect(self, image):
        height, width = image.shape[:2]
        scale = min(1.0, self.process_width / float(width))
        small_width = max(1, int(round(width * scale)))
        small_height = max(1, int(round(height * scale)))
        small = cv2.resize(image, (small_width, small_height))

        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        edges = cv2.Canny(
            blurred,
            self.canny_low_threshold,
            self.canny_high_threshold,
        )

        roi_top = int(small_height * self.roi_top_ratio)
        roi_bottom = int(small_height * self.roi_bottom_ratio)
        edges[:roi_top, :] = 0
        edges[roi_bottom:, :] = 0

        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180.0,
            self.hough_threshold,
            minLineLength=self.min_line_length_px,
            maxLineGap=self.max_line_gap_px,
        )

        candidates = []
        if lines is not None:
            for line in lines:
                coordinates = np.asarray(line).reshape(-1)
                if coordinates.size < 4:
                    continue
                x1, y1, x2, y2 = [int(value) for value in coordinates[:4]]
                dx = x2 - x1
                dy = y2 - y1
                if dy == 0:
                    continue
                angle_from_vertical = math.degrees(
                    math.atan2(abs(dx), abs(dy))
                )
                if angle_from_vertical > self.max_angle_from_vertical_deg:
                    continue
                center_x = (x1 + x2) / 2.0
                candidates.append((center_x, (x1, y1, x2, y2)))

        clusters = self._cluster_lines(candidates, scale)
        return DetectionResult(scale, edges, candidates, clusters)

    def centered_clusters(self, image_width, detection):
        """Return line clusters within the configured band around image center."""
        center_x = image_width / 2.0
        return [
            cluster
            for cluster in detection.clusters
            if self._cluster_center_distance(cluster, detection, center_x)
            <= self.center_tolerance_px
        ]

    def nearest_center_distance_px(self, image_width, detection):
        """Return the closest candidate's x distance from image center."""
        if not detection.clusters:
            return None
        center_x = image_width / 2.0
        return min(
            self._cluster_center_distance(cluster, detection, center_x)
            for cluster in detection.clusters
        )

    @staticmethod
    def _cluster_center_distance(cluster, detection, image_center_x):
        cluster_x = statistics.median(cluster["x_values"]) / detection.scale
        return abs(cluster_x - image_center_x)

    def draw_preview(self, image, detection, status_text=None):
        height, width = image.shape[:2]
        preview = image.copy()
        if self.show_edges:
            full_size_edges = cv2.resize(
                detection.edges,
                (width, height),
                interpolation=cv2.INTER_NEAREST,
            )
            preview[full_size_edges > 0] = (0, 0, 255)

        center_x = width // 2
        tolerance = int(round(self.center_tolerance_px))
        cv2.rectangle(
            preview,
            (max(0, center_x - tolerance), 0),
            (min(width - 1, center_x + tolerance), height - 1),
            (40, 90, 40),
            1,
        )
        cv2.line(
            preview,
            (center_x, 0),
            (center_x, height - 1),
            (255, 0, 0),
            2,
        )

        for cluster in detection.clusters:
            line_x_small = int(round(statistics.median(cluster["x_values"])))
            line_x = int(round(line_x_small / detection.scale))
            centered = abs(line_x - center_x) <= tolerance
            color = (0, 255, 0) if centered else (0, 200, 255)
            for x1, y1, x2, y2 in cluster["segments"]:
                start = (
                    int(round(x1 / detection.scale)),
                    int(round(y1 / detection.scale)),
                )
                end = (
                    int(round(x2 / detection.scale)),
                    int(round(y2 / detection.scale)),
                )
                cv2.line(preview, start, end, color, 3, cv2.LINE_AA)
            cv2.line(preview, (line_x, 0), (line_x, height - 1), color, 1)
            cv2.putText(
                preview,
                f"x={line_x}",
                (max(4, min(width - 80, line_x + 4)), 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
                cv2.LINE_AA,
            )

        centered_one = (
            len(detection.clusters) == 1
            and len(self.centered_clusters(width, detection)) == 1
        )
        if status_text is None:
            status_text = (
                "ONE LINE NEAR CENTER" if centered_one else "SEARCHING"
            )
        summary = (
            f"vertical lines={len(detection.clusters)} "
            f"segments={len(detection.candidates)}  {status_text}"
        )
        cv2.rectangle(preview, (0, 0), (width, 36), (0, 0, 0), -1)
        cv2.putText(
            preview,
            summary,
            (12, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return preview

    def _cluster_lines(self, candidates, scale):
        tolerance = self.merge_tolerance_px * scale
        sorted_candidates = sorted(candidates, key=lambda item: item[0])
        clusters = []
        for center_x, segment in sorted_candidates:
            if (
                not clusters
                or center_x - statistics.median(clusters[-1]["x_values"])
                > tolerance
            ):
                clusters.append({"x_values": [center_x], "segments": [segment]})
            else:
                clusters[-1]["x_values"].append(center_x)
                clusters[-1]["segments"].append(segment)
        return clusters
