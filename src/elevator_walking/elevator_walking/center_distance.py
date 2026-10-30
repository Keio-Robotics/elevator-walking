#!/usr/bin/env python3
"""Publish the median depth around the center of a ROS image, in meters."""

import math
import statistics
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class CenterDistancePublisher(Node):
    """Read the center region of a depth image and publish one distance."""

    def __init__(self):
        super().__init__("center_distance_publisher")
        self.declare_parameter("image_topic", "/camera/depth/image_rect_raw")
        self.declare_parameter("distance_topic", "/camera/depth/center_distance")
        self.declare_parameter("roi_size", 31)
        self.declare_parameter("min_valid_pixels", 200)
        self.declare_parameter("depth_scale", 0.001)

        image_topic = self.get_parameter("image_topic").value
        distance_topic = self.get_parameter("distance_topic").value
        self.roi_size = int(self.get_parameter("roi_size").value)
        self.min_valid_pixels = int(
            self.get_parameter("min_valid_pixels").value
        )
        self.depth_scale = float(self.get_parameter("depth_scale").value)

        if self.roi_size < 1:
            raise ValueError("roi_size must be greater than zero")
        if self.roi_size % 2 == 0:
            self.roi_size += 1
            self.get_logger().warning(
                f"roi_size must be odd; using {self.roi_size} pixels instead"
            )
        if not 1 <= self.min_valid_pixels <= self.roi_size * self.roi_size:
            raise ValueError(
                "min_valid_pixels must be between 1 and roi_size squared"
            )
        if not math.isfinite(self.depth_scale) or self.depth_scale <= 0.0:
            raise ValueError("depth_scale must be a positive finite number")

        self._unsupported_encoding_reported = False
        self._bad_image_reported = False
        self.publisher = self.create_publisher(Float32, distance_topic, 10)
        self.subscription = self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Reading center depth from {image_topic}; publishing meters on "
            f"{distance_topic} (median {self.roi_size}x{self.roi_size} region)"
        )

    def _on_image(self, image):
        distance = self._center_distance_meters(image)
        message = Float32()
        message.data = distance
        self.publisher.publish(message)

    def _center_distance_meters(self, image):
        if image.encoding == "16UC1":
            pixel_format = "H"
            scale_to_meters = self.depth_scale
        elif image.encoding == "32FC1":
            pixel_format = "f"
            scale_to_meters = 1.0
        else:
            if not self._unsupported_encoding_reported:
                self.get_logger().error(
                    f"Unsupported depth encoding '{image.encoding}'. "
                    "Expected 16UC1 (millimeters) or 32FC1 (meters)."
                )
                self._unsupported_encoding_reported = True
            return float("nan")

        bytes_per_pixel = struct.calcsize(pixel_format)
        if (
            image.width < 1
            or image.height < 1
            or image.step < image.width * bytes_per_pixel
            or len(image.data) < image.step * image.height
        ):
            if not self._bad_image_reported:
                self.get_logger().error("Received a malformed depth image")
                self._bad_image_reported = True
            return float("nan")

        half = self.roi_size // 2
        center_x = image.width // 2
        center_y = image.height // 2
        left = max(0, center_x - half)
        right = min(image.width, center_x + half + 1)
        top = max(0, center_y - half)
        bottom = min(image.height, center_y + half + 1)
        byte_order = ">" if image.is_bigendian else "<"
        values = []

        for y in range(top, bottom):
            row_offset = y * image.step
            for x in range(left, right):
                offset = row_offset + x * bytes_per_pixel
                raw_value = struct.unpack_from(
                    byte_order + pixel_format, image.data, offset
                )[0]
                if raw_value > 0 and math.isfinite(raw_value):
                    values.append(raw_value * scale_to_meters)

        if len(values) < self.min_valid_pixels:
            return float("nan")
        return float(statistics.median(values))


def main(args=None):
    rclpy.init(args=args)
    node = CenterDistancePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
