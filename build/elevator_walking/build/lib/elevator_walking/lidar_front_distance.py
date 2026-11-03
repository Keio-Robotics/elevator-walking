#!/usr/bin/env python3
"""Publish the forward distance estimated from Unitree's lidar point cloud."""

import math
import statistics
import struct
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32


FLOAT_FORMATS = {
    PointField.FLOAT32: ("f", 4),
    PointField.FLOAT64: ("d", 8),
}


class LidarFrontDistancePublisher(Node):
    """Estimate forward distance from fresh clouds and publish at 10 Hz."""

    def __init__(self):
        super().__init__("lidar_front_distance_publisher")
        self.declare_parameter("cloud_topic", "/utlidar/cloud")
        self.declare_parameter("distance_topic", "/utlidar/front_distance")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("window_s", 0.3)
        self.declare_parameter("cloud_timeout_s", 0.5)
        self.declare_parameter("min_x_m", 0.1)
        self.declare_parameter("max_x_m", 10.0)
        self.declare_parameter("horizontal_half_angle_deg", 10.0)
        self.declare_parameter("vertical_half_angle_deg", 15.0)
        self.declare_parameter("min_points", 5)

        cloud_topic = str(self.get_parameter("cloud_topic").value)
        distance_topic = str(self.get_parameter("distance_topic").value)
        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )
        self.window_s = float(self.get_parameter("window_s").value)
        self.cloud_timeout_s = float(
            self.get_parameter("cloud_timeout_s").value
        )
        self.min_x_m = float(self.get_parameter("min_x_m").value)
        self.max_x_m = float(self.get_parameter("max_x_m").value)
        horizontal_angle = float(
            self.get_parameter("horizontal_half_angle_deg").value
        )
        vertical_angle = float(
            self.get_parameter("vertical_half_angle_deg").value
        )
        self.min_points = int(self.get_parameter("min_points").value)

        numeric_parameters = (
            self.publish_rate_hz,
            self.window_s,
            self.cloud_timeout_s,
            self.min_x_m,
            self.max_x_m,
            horizontal_angle,
            vertical_angle,
        )
        if not all(math.isfinite(value) for value in numeric_parameters):
            raise ValueError("All numeric parameters must be finite")
        if (
            self.publish_rate_hz <= 0.0
            or self.window_s <= 0.0
            or self.cloud_timeout_s <= 0.0
        ):
            raise ValueError(
                "publish_rate_hz, window_s, and cloud_timeout_s must be positive"
            )
        if self.min_x_m < 0.0 or self.max_x_m <= self.min_x_m:
            raise ValueError("Require 0 <= min_x_m < max_x_m")
        if not 0.0 < horizontal_angle < 90.0:
            raise ValueError("horizontal_half_angle_deg must be between 0 and 90")
        if not 0.0 < vertical_angle < 90.0:
            raise ValueError("vertical_half_angle_deg must be between 0 and 90")
        if self.min_points < 1:
            raise ValueError("min_points must be at least 1")

        self.horizontal_half_angle = math.radians(horizontal_angle)
        self.vertical_half_angle = math.radians(vertical_angle)
        self._cloud_samples = deque()
        self._last_cloud_time = None
        self._last_invalid_log_time = 0.0

        self.publisher = self.create_publisher(Float32, distance_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            cloud_topic,
            self._on_cloud,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz, self._publish_distance
        )
        self.get_logger().info(
            f"Reading forward points from {cloud_topic}; publishing the x-median "
            f"over a {self.window_s:.1f} s window at {self.publish_rate_hz:.1f} Hz "
            f"on {distance_topic}."
        )

    def _decode_forward_x(self, cloud):
        fields = {field.name: field for field in cloud.fields}
        formats = {}
        for name in ("x", "y", "z"):
            field = fields.get(name)
            if field is None:
                raise ValueError(f"PointCloud2 has no '{name}' field")
            format_info = FLOAT_FORMATS.get(field.datatype)
            if format_info is None:
                raise ValueError(f"PointCloud2 field '{name}' is not float32/float64")
            format_char, byte_size = format_info
            if field.offset < 0 or field.offset + byte_size > cloud.point_step:
                raise ValueError(f"PointCloud2 field '{name}' exceeds point_step")
            formats[name] = (format_char, field.offset)

        if cloud.width < 1 or cloud.height < 1 or cloud.point_step < 1:
            raise ValueError("PointCloud2 has an empty or invalid shape")
        if cloud.row_step < cloud.width * cloud.point_step:
            raise ValueError("PointCloud2 row_step is smaller than its row data")
        if len(cloud.data) < cloud.row_step * cloud.height:
            raise ValueError("PointCloud2 data is shorter than its declared shape")

        endian = ">" if cloud.is_bigendian else "<"
        x_format, x_offset = formats["x"]
        y_format, y_offset = formats["y"]
        z_format, z_offset = formats["z"]
        xs = []

        for row in range(cloud.height):
            row_offset = row * cloud.row_step
            for column in range(cloud.width):
                point_offset = row_offset + column * cloud.point_step
                x = struct.unpack_from(
                    endian + x_format, cloud.data, point_offset + x_offset
                )[0]
                if not math.isfinite(x) or not self.min_x_m < x < self.max_x_m:
                    continue

                y = struct.unpack_from(
                    endian + y_format, cloud.data, point_offset + y_offset
                )[0]
                z = struct.unpack_from(
                    endian + z_format, cloud.data, point_offset + z_offset
                )[0]
                if not math.isfinite(y) or not math.isfinite(z):
                    continue

                azimuth = math.atan2(y, x)
                elevation = math.atan2(z, math.hypot(x, y))
                if (
                    abs(azimuth) <= self.horizontal_half_angle
                    and abs(elevation) <= self.vertical_half_angle
                ):
                    xs.append(x)

        return xs

    def _on_cloud(self, cloud):
        now = time.monotonic()
        self._last_cloud_time = now
        try:
            xs = self._decode_forward_x(cloud)
        except (ValueError, struct.error) as exc:
            self._log_invalid(str(exc))
            xs = []
        self._cloud_samples.append((now, xs))

    def _publish_distance(self):
        now = time.monotonic()
        while (
            self._cloud_samples
            and now - self._cloud_samples[0][0] > self.window_s
        ):
            self._cloud_samples.popleft()

        cloud_is_fresh = (
            self._last_cloud_time is not None
            and now - self._last_cloud_time <= self.cloud_timeout_s
        )
        recent_xs = []
        for _, cloud_xs in self._cloud_samples:
            recent_xs.extend(cloud_xs)

        if cloud_is_fresh and len(recent_xs) >= self.min_points:
            distance_m = float(statistics.median(recent_xs))
        else:
            distance_m = None

        message = Float32()
        message.data = distance_m if distance_m is not None else float("nan")
        self.publisher.publish(message)

        if distance_m is None:
            self._log_invalid(
                "waiting for a fresh cloud with enough forward points; publishing NaN"
            )
            return

        self.get_logger().info(
            f"Forward distance: {distance_m:.3f} m "
            f"({len(recent_xs)} points in last {self.window_s:.1f} s)"
        )

    def _log_invalid(self, reason):
        now = time.monotonic()
        if now - self._last_invalid_log_time >= 1.0:
            self.get_logger().warning(reason)
            self._last_invalid_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarFrontDistancePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Could not start lidar distance publisher: {exc}")
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
