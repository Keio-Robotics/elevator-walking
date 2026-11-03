#!/usr/bin/env python3
"""Publish a camera preview with candidate vertical lines overlaid."""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from elevator_walking.vertical_line_detector import (
    VerticalLineDetector,
    declare_detector_parameters,
)


class VerticalLinePreview(Node):
    """Visualize vertical Hough line candidates without commanding the robot."""

    def __init__(self):
        super().__init__("preview_vertical_lines")
        self.declare_parameter(
            "image_topic", "/camera/color/image_raw/compressed"
        )
        self.declare_parameter(
            "preview_topic", "/camera/color/vertical_lines/compressed"
        )
        declare_detector_parameters(self)

        image_topic = str(self.get_parameter("image_topic").value)
        preview_topic = str(self.get_parameter("preview_topic").value)
        self.detector = VerticalLineDetector.from_node_parameters(self)
        self._bad_image_reported = False
        self.publisher = self.create_publisher(
            CompressedImage, preview_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            CompressedImage,
            image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Previewing vertical lines from {image_topic} on {preview_topic}; "
            f"processing width={self.detector.process_width}px."
        )

    def _on_image(self, message):
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            if not self._bad_image_reported:
                self.get_logger().error("Could not decode compressed color image")
                self._bad_image_reported = True
            return

        detection = self.detector.detect(image)
        preview = self.detector.draw_preview(image, detection)
        success, preview_data = cv2.imencode(
            ".jpg", preview, [int(cv2.IMWRITE_JPEG_QUALITY), 85]
        )
        if not success:
            self.get_logger().warning("Could not encode vertical-line preview")
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
        node = VerticalLinePreview()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Could not start vertical-line preview: {exc}")
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
