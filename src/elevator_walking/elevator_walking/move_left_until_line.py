#!/usr/bin/env python3
"""Move left until the shared vertical-line detector finds a line."""

import json
import math
import time

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


SPORT_API_ID_STOPMOVE = 1003
SPORT_API_ID_MOVE = 1008


class MoveLeftUntilLine(Node):
    """Move in +Y until at least one vertical line cluster is detected."""

    def __init__(self):
        super().__init__("move_left_until_line")
        self.declare_parameter(
            "image_topic", "/camera/color/image_raw/compressed"
        )
        self.declare_parameter(
            "preview_topic", "/camera/color/move_left_until_line/compressed"
        )
        self.declare_parameter("vx", -0.05)
        self.declare_parameter("vy", 0.1)
        self.declare_parameter("approach_vy", 0.1)
        self.declare_parameter("slowdown_distance_px", 40.0)
        self.declare_parameter("command_period_s", 0.1)
        self.declare_parameter("startup_timeout_s", 5.0)
        self.declare_parameter("image_timeout_s", 0.5)
        self.declare_parameter("max_motion_s", 20.0)
        declare_detector_parameters(self)

        self.image_topic = str(self.get_parameter("image_topic").value).strip()
        self.preview_topic = str(
            self.get_parameter("preview_topic").value
        ).strip()
        self.vx = float(self.get_parameter("vx").value)
        self.vy = float(self.get_parameter("vy").value)
        self.approach_vy = float(self.get_parameter("approach_vy").value)
        self.slowdown_distance_px = float(
            self.get_parameter("slowdown_distance_px").value
        )
        self.command_period_s = float(
            self.get_parameter("command_period_s").value
        )
        self.startup_timeout_s = float(
            self.get_parameter("startup_timeout_s").value
        )
        self.image_timeout_s = float(
            self.get_parameter("image_timeout_s").value
        )
        self.max_motion_s = float(self.get_parameter("max_motion_s").value)
        self._validate_parameters()
        self.detector = VerticalLineDetector.from_node_parameters(self)

        try:
            from unitree_api.msg import Request
        except ImportError as exc:
            raise RuntimeError(
                "unitree_api messages are unavailable; source the Foxy "
                "Unitree ROS 2 workspace before running this node"
            ) from exc

        self._request_type = Request
        self._state = "waiting_image"
        self._created_at = time.monotonic()
        self._motion_started_at = None
        self._last_image_at = None
        self._last_move_command_at = None
        self._approach_mode = False
        self._command_vy = self.vy
        self._bad_image_reported = False

        self._sport_request_publisher = self.create_publisher(
            Request, "/api/sport/request", 10
        )
        self.preview_publisher = self.create_publisher(
            CompressedImage, self.preview_topic, qos_profile_sensor_data
        )
        self.image_subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.05, self._control_tick)
        self.get_logger().info(
            f"Waiting for images on {self.image_topic}; then moving left "
            f"with Move x={self.vx:+.3f}, y=+{self.vy:.3f} "
            f"(approach y=+{self.approach_vy:.3f} inside "
            f"{self.slowdown_distance_px:.0f}px) until a vertical "
            f"line is within +/-{self.detector.center_tolerance_px:.0f}px "
            f"of image center. Preview: {self.preview_topic}. "
            f"Maximum movement time: {self.max_motion_s:.1f} s."
        )

    def _validate_parameters(self):
        values = (
            self.vx,
            self.vy,
            self.approach_vy,
            self.slowdown_distance_px,
            self.command_period_s,
            self.startup_timeout_s,
            self.image_timeout_s,
            self.max_motion_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("All numeric parameters must be finite")
        if self.vy <= 0.0:
            raise ValueError("vy must be positive for leftward movement")
        if self.approach_vy <= 0.0 or self.approach_vy > self.vy:
            raise ValueError("approach_vy must be positive and no greater than vy")
        if self.slowdown_distance_px <= 0.0:
            raise ValueError("slowdown_distance_px must be positive")
        if min(
            self.command_period_s,
            self.startup_timeout_s,
            self.image_timeout_s,
            self.max_motion_s,
        ) <= 0.0:
            raise ValueError("period and timeout parameters must be positive")

    def _publish_sport_request(self, api_id, parameter=""):
        request = self._request_type()
        request.header.identity.api_id = api_id
        request.parameter = parameter
        request.binary = []
        self._sport_request_publisher.publish(request)

    def _publish_move(self):
        parameter = json.dumps(
            {"x": self.vx, "y": self._command_vy, "z": 0.0},
            separators=(",", ":"),
        )
        self._publish_sport_request(SPORT_API_ID_MOVE, parameter)
        self._last_move_command_at = time.monotonic()

    def _publish_preview(self, message, image, detection):
        if self._state == "done":
            status = "LINE FOUND - STOPPED"
        elif self._state == "stopped":
            status = "STOPPED - SAFETY"
        elif self._state == "moving":
            status = "SLOW APPROACH" if self._approach_mode else "MOVING LEFT"
        else:
            status = "WAITING FOR IMAGE"

        preview = self.detector.draw_preview(image, detection, status)
        success, data = cv2.imencode(
            ".jpg", preview, [int(cv2.IMWRITE_JPEG_QUALITY), 85]
        )
        if not success:
            return
        output = CompressedImage()
        output.header = message.header
        output.format = "jpeg"
        output.data = data.tobytes()
        self.preview_publisher.publish(output)

    def _on_image(self, message):
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            if not self._bad_image_reported:
                self.get_logger().error("Could not decode compressed color image")
                self._bad_image_reported = True
            if self._state == "moving":
                self._stop("invalid compressed image", completed=False)
            return

        now = time.monotonic()
        self._last_image_at = now
        detection = self.detector.detect(image)
        centered_lines = self.detector.centered_clusters(
            image.shape[1], detection
        )
        line_count = len(centered_lines)
        nearest_line_px = self.detector.nearest_center_distance_px(
            image.shape[1], detection
        )

        if (
            line_count == 0
            and self._state in ("waiting_image", "moving")
            and not self._approach_mode
            and nearest_line_px is not None
            and nearest_line_px <= self.slowdown_distance_px
        ):
            self._approach_mode = True
            self._command_vy = self.approach_vy
            self.get_logger().info(
                f"Vertical line candidate is {nearest_line_px:.0f}px from "
                f"image center; reducing lateral speed to "
                f"{self.approach_vy:.3f}."
            )

        if line_count > 0:
            if self._state == "moving":
                self._stop(
                    f"detected {line_count} vertical line cluster(s) "
                    "within the center band",
                    completed=True,
                )
            elif self._state == "waiting_image":
                self._state = "done"
                self.get_logger().info(
                    f"Already detected {line_count} vertical line cluster(s) "
                    "within the center band; not moving."
                )
        elif self._state == "waiting_image":
            self._state = "moving"
            self._motion_started_at = now
            try:
                self._publish_move()
            except Exception as exc:
                self._stop(f"could not publish Move request: {exc}", False)
                return
            self.get_logger().info(
                f"No vertical line detected; starting leftward movement "
                f"(vx={self.vx:+.3f}, vy=+{self._command_vy:.3f})."
            )

        self._publish_preview(message, image, detection)

    def _control_tick(self):
        if self._state in ("done", "stopped"):
            return

        now = time.monotonic()
        if self._state == "waiting_image":
            if now - self._created_at > self.startup_timeout_s:
                self._state = "stopped"
                self.get_logger().warning(
                    "Stopped without moving: no valid camera image received "
                    f"within {self.startup_timeout_s:.1f} s."
                )
            return

        if self._last_image_at is None or (
            now - self._last_image_at > self.image_timeout_s
        ):
            self._stop("camera image timed out", completed=False)
            return

        if now - self._motion_started_at >= self.max_motion_s:
            self._stop(
                f"maximum movement time ({self.max_motion_s:.1f} s) reached",
                completed=False,
            )
            return

        if (
            self._last_move_command_at is None
            or now - self._last_move_command_at >= self.command_period_s
        ):
            try:
                self._publish_move()
            except Exception as exc:
                self._stop(f"could not publish Move request: {exc}", False)

    def _stop(self, reason, completed):
        if self._state != "moving":
            return

        self._state = "done" if completed else "stopped"
        try:
            self._publish_sport_request(SPORT_API_ID_STOPMOVE)
        except Exception as exc:
            self.get_logger().error(f"Could not publish StopMove request: {exc}")

        if completed:
            self.get_logger().info(f"Stopped: {reason}.")
        else:
            self.get_logger().warning(f"Stopped for safety: {reason}.")

    def stop_if_moving(self):
        if self._state == "moving":
            self._stop("node shutdown", completed=False)


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = MoveLeftUntilLine()
        while rclpy.ok() and node._state not in ("done", "stopped"):
            rclpy.spin_once(node, timeout_sec=0.1)
        exit_code = 0 if node._state == "done" else 1
    except KeyboardInterrupt:
        if node is not None:
            node.stop_if_moving()
        exit_code = 130
    except Exception as exc:
        print(f"Could not start move-left-until-line node: {exc}")
    finally:
        if node is not None:
            node.stop_if_moving()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
