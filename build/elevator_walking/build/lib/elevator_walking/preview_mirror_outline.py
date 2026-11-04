#!/usr/bin/env python3
"""Preview large rectangle-like contours as possible mirror outlines."""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class MirrorOutlinePreview(Node):
    """Show convex quadrilateral contours for visual mirror-detection tests."""

    def __init__(self):
        super().__init__("preview_mirror_outline")
        self.declare_parameter("image_topic", "/camera/color/image_raw/compressed")
        self.declare_parameter(
            "preview_topic", "/camera/color/mirror_outline/compressed"
        )
        self.declare_parameter("process_width", 960)
        self.declare_parameter("canny_low_threshold", 10)
        self.declare_parameter("canny_high_threshold", 150)
        self.declare_parameter("show_edges", False)
        self.declare_parameter("close_kernel_size", 7)
        self.declare_parameter("polygon_epsilon_ratio", 0.02)
        self.declare_parameter("min_area_ratio", 0.03)
        self.declare_parameter("min_aspect_ratio", 0.15)
        self.declare_parameter("max_aspect_ratio", 2.0)
        self.declare_parameter("max_candidates", 5)
        self.declare_parameter("roi_top_ratio", 0.00)
        self.declare_parameter("roi_bottom_ratio", 0.5)
        self.declare_parameter("roi_left_ratio", 0.3)
        self.declare_parameter("roi_right_ratio", 0.7)

        self.image_topic = str(self.get_parameter("image_topic").value).strip()
        self.preview_topic = str(
            self.get_parameter("preview_topic").value
        ).strip()
        self.process_width = int(self.get_parameter("process_width").value)
        self.canny_low = int(self.get_parameter("canny_low_threshold").value)
        self.canny_high = int(self.get_parameter("canny_high_threshold").value)
        self.show_edges = bool(self.get_parameter("show_edges").value)
        self.close_kernel_size = int(
            self.get_parameter("close_kernel_size").value
        )
        self.polygon_epsilon_ratio = float(
            self.get_parameter("polygon_epsilon_ratio").value
        )
        self.min_area_ratio = float(self.get_parameter("min_area_ratio").value)
        self.min_aspect_ratio = float(
            self.get_parameter("min_aspect_ratio").value
        )
        self.max_aspect_ratio = float(
            self.get_parameter("max_aspect_ratio").value
        )
        self.max_candidates = int(self.get_parameter("max_candidates").value)
        self.roi_top_ratio = float(self.get_parameter("roi_top_ratio").value)
        self.roi_bottom_ratio = float(
            self.get_parameter("roi_bottom_ratio").value
        )
        self.roi_left_ratio = float(self.get_parameter("roi_left_ratio").value)
        self.roi_right_ratio = float(
            self.get_parameter("roi_right_ratio").value
        )
        self._validate_parameters()

        self._bad_image_reported = False
        self._last_summary_at = 0.0
        self.publisher = self.create_publisher(
            CompressedImage, self.preview_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Previewing possible mirror outlines from {self.image_topic} "
            f"on {self.preview_topic}; processing width={self.process_width}px."
        )

    def _validate_parameters(self):
        if self.process_width < 100:
            raise ValueError("process_width must be at least 100 pixels")
        if not 0 <= self.canny_low < self.canny_high <= 255:
            raise ValueError(
                "Canny thresholds must satisfy 0 <= low < high <= 255"
            )
        if self.close_kernel_size < 1 or self.close_kernel_size % 2 == 0:
            raise ValueError("close_kernel_size must be a positive odd number")
        if not 0.001 <= self.polygon_epsilon_ratio <= 0.2:
            raise ValueError("polygon_epsilon_ratio must be in [0.001, 0.2]")
        if not 0.0 < self.min_area_ratio < 1.0:
            raise ValueError("min_area_ratio must be in (0, 1)")
        if not (
            0.0 < self.min_aspect_ratio <= self.max_aspect_ratio
        ) or self.max_aspect_ratio > 10.0:
            raise ValueError("invalid aspect-ratio limits")
        if self.max_candidates < 1:
            raise ValueError("max_candidates must be at least 1")
        if not (
            0.0 <= self.roi_top_ratio < self.roi_bottom_ratio <= 1.0
            and 0.0 <= self.roi_left_ratio < self.roi_right_ratio <= 1.0
        ):
            raise ValueError(
                "ROI ratios must satisfy 0 <= top < bottom <= 1 and "
                "0 <= left < right <= 1; got "
                f"top={self.roi_top_ratio}, bottom={self.roi_bottom_ratio}, "
                f"left={self.roi_left_ratio}, right={self.roi_right_ratio}"
            )

    def _find_candidates(self, image):
        height, width = image.shape[:2]
        scale = min(1.0, self.process_width / float(width))
        small_width = max(1, int(round(width * scale)))
        small_height = max(1, int(round(height * scale)))
        small = cv2.resize(image, (small_width, small_height))
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)

        left = int(small_width * self.roi_left_ratio)
        right = int(small_width * self.roi_right_ratio)
        top = int(small_height * self.roi_top_ratio)
        bottom = int(small_height * self.roi_bottom_ratio)
        roi_mask = np.zeros_like(edges)
        roi_mask[top:bottom, left:right] = 255
        edges = cv2.bitwise_and(edges, roi_mask)

        kernel_size = self.close_kernel_size
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (kernel_size, kernel_size)
        )
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
        )

        roi_area = max(1, (right - left) * (bottom - top))
        candidates = []
        for contour in contours:
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0.0:
                continue
            polygon = cv2.approxPolyDP(
                contour,
                self.polygon_epsilon_ratio * perimeter,
                True,
            )
            if len(polygon) != 4 or not cv2.isContourConvex(polygon):
                continue

            area_ratio = cv2.contourArea(polygon) / float(roi_area)
            if area_ratio < self.min_area_ratio:
                continue
            _, _, box_width, box_height = cv2.boundingRect(polygon)
            if box_height <= 0:
                continue
            aspect_ratio = box_width / float(box_height)
            if not self.min_aspect_ratio <= aspect_ratio <= self.max_aspect_ratio:
                continue
            candidates.append((area_ratio, polygon, aspect_ratio))

        candidates.sort(key=lambda candidate: candidate[0], reverse=True)
        return edges, candidates[: self.max_candidates], scale

    @staticmethod
    def _box_iou(polygon_a, polygon_b):
        box_a = cv2.boundingRect(polygon_a)
        box_b = cv2.boundingRect(polygon_b)
        ax, ay, aw, ah = box_a
        bx, by, bw, bh = box_b
        left = max(ax, bx)
        top = max(ay, by)
        right = min(ax + aw, bx + bw)
        bottom = min(ay + ah, by + bh)
        intersection = max(0, right - left) * max(0, bottom - top)
        union = aw * ah + bw * bh - intersection
        return intersection / float(union) if union > 0 else 0.0

    def _draw_preview(self, image, edges, candidates, scale, roi):
        height, width = image.shape[:2]
        preview = image.copy()
        if self.show_edges:
            full_edges = cv2.resize(
                edges, (width, height), interpolation=cv2.INTER_NEAREST
            )
            preview[full_edges > 0] = (0, 0, 255)

        left, top, right, bottom = roi
        cv2.rectangle(preview, (left, top), (right, bottom), (255, 180, 0), 1)

        drawn = []
        label_count = 0
        for area_ratio, polygon, aspect_ratio in candidates:
            if any(self._box_iou(polygon, previous) > 0.8 for previous in drawn):
                continue
            drawn.append(polygon)
            points = np.rint(polygon.reshape(-1, 2) / scale).astype(np.int32)
            color = (0, 255, 0) if label_count == 0 else (0, 200, 255)
            cv2.polylines(preview, [points], True, color, 4, cv2.LINE_AA)
            x = max(4, min(width - 200, int(points[:, 0].min())))
            y = max(24, int(points[:, 1].min()) - 8)
            cv2.putText(
                preview,
                f"quad {label_count + 1}: area={area_ratio:.1%} w/h={aspect_ratio:.2f}",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )
            label_count += 1

        status = (
            f"rectangle candidates={label_count}"
            if label_count
            else "NO RECTANGLE CANDIDATE"
        )
        cv2.putText(
            preview,
            status,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return preview, label_count

    def _on_image(self, message):
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            if not self._bad_image_reported:
                self.get_logger().error("Could not decode compressed color image")
                self._bad_image_reported = True
            return

        edges, candidates, scale = self._find_candidates(image)
        height, width = image.shape[:2]
        roi = (
            int(width * self.roi_left_ratio),
            int(height * self.roi_top_ratio),
            int(width * self.roi_right_ratio),
            int(height * self.roi_bottom_ratio),
        )
        preview, count = self._draw_preview(image, edges, candidates, scale, roi)

        now = time.monotonic()
        if now - self._last_summary_at >= 2.0:
            summary = f"Found {count} rectangle candidate(s)."
            if candidates:
                summary += f" Largest area ratio: {candidates[0][0]:.1%}."
            self.get_logger().info(summary)
            self._last_summary_at = now

        success, preview_data = cv2.imencode(
            ".jpg", preview, [int(cv2.IMWRITE_JPEG_QUALITY), 85]
        )
        if not success:
            self.get_logger().warning("Could not encode mirror-outline preview")
            return

        output = CompressedImage()
        output.header = message.header
        output.format = "jpeg"
        output.data = preview_data.tobytes()
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MirrorOutlinePreview()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Could not start mirror-outline preview: {exc}")
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
