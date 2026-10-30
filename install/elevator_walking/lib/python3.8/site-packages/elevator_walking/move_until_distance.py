#!/usr/bin/env python3
"""Move the robot until the published center distance crosses a threshold."""

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

SPORT_API_ID_STOPMOVE = 1003
SPORT_API_ID_MOVE = 1008


class MoveUntilDistance(Node):
    """Publish Unitree ROS 2 velocity requests while distance is safe."""

    def __init__(self):
        super().__init__("move_until_distance")
        self.declare_parameter("distance_topic", "/camera/depth/center_distance")
        self.declare_parameter("threshold_m", 0.5)
        self.declare_parameter("vx", 0.1)
        self.declare_parameter("vy", 0.0)
        self.declare_parameter("vyaw", 0.0)
        self.declare_parameter("command_period_s", 0.1)
        self.declare_parameter("sensor_timeout_s", 0.5)
        self.declare_parameter("max_motion_s", 10.0)

        distance_topic = str(self.get_parameter("distance_topic").value)
        self.threshold_m = float(self.get_parameter("threshold_m").value)
        self.vx = float(self.get_parameter("vx").value)
        self.vy = float(self.get_parameter("vy").value)
        self.vyaw = float(self.get_parameter("vyaw").value)
        command_period_s = float(self.get_parameter("command_period_s").value)
        self.sensor_timeout_s = float(
            self.get_parameter("sensor_timeout_s").value
        )
        self.max_motion_s = float(self.get_parameter("max_motion_s").value)

        numeric_parameters = (
            self.threshold_m,
            self.vx,
            self.vy,
            self.vyaw,
            command_period_s,
            self.sensor_timeout_s,
            self.max_motion_s,
        )
        if not all(math.isfinite(value) for value in numeric_parameters):
            raise ValueError("All numeric parameters must be finite")
        if self.threshold_m <= 0.0:
            raise ValueError("threshold_m must be greater than zero")
        if command_period_s <= 0.0:
            raise ValueError("command_period_s must be greater than zero")
        if self.sensor_timeout_s <= 0.0 or self.max_motion_s <= 0.0:
            raise ValueError("sensor_timeout_s and max_motion_s must be positive")

        self._state = "waiting"
        self._distance_m = None
        self._last_distance_time = None
        self._motion_started_time = None
        self._initialize_sport_publisher()

        self.subscription = self.create_subscription(
            Float32, distance_topic, self._on_distance, 10
        )
        self.timer = self.create_timer(command_period_s, self._control_tick)
        self.get_logger().info(
            f"Waiting for {distance_topic}; will stop below "
            f"{self.threshold_m:.3f} m. Move command: "
            f"({self.vx:.2f}, {self.vy:.2f}, {self.vyaw:.2f})."
        )

    def _initialize_sport_publisher(self):
        try:
            from unitree_api.msg import Request
        except ImportError as exc:
            raise RuntimeError(
                "unitree_api messages are unavailable; source the Unitree "
                "ROS 2 workspace before running this node"
            ) from exc

        self._request_type = Request
        self._sport_request_publisher = self.create_publisher(
            Request, "/api/sport/request", 10
        )

    def _publish_sport_request(self, api_id, parameter=""):
        request = self._request_type()
        request.header.identity.api_id = api_id
        request.parameter = parameter
        request.binary = []
        self._sport_request_publisher.publish(request)

    def _on_distance(self, message):
        now = time.monotonic()
        distance_m = float(message.data)
        self._last_distance_time = now

        if not math.isfinite(distance_m) or distance_m <= 0.0:
            self._distance_m = None
            if self._state == "moving":
                self._stop("invalid distance reading", completed=False)
            return

        self._distance_m = distance_m
        if distance_m < self.threshold_m:
            if self._state == "moving":
                self._stop(
                    f"distance {distance_m:.3f} m is below threshold",
                    completed=True,
                )
            elif self._state == "waiting":
                self._state = "done"
                self.get_logger().info(
                    f"Already below threshold: {distance_m:.3f} m; not moving."
                )

    def _control_tick(self):
        if self._state not in ("waiting", "moving"):
            return

        now = time.monotonic()
        if self._distance_m is None or self._last_distance_time is None:
            if self._state == "moving":
                self._stop("distance reading is invalid", completed=False)
            return

        if now - self._last_distance_time > self.sensor_timeout_s:
            if self._state == "moving":
                self._stop("distance topic timed out", completed=False)
            return

        if self._distance_m < self.threshold_m:
            if self._state == "moving":
                self._stop(
                    f"distance {self._distance_m:.3f} m is below threshold",
                    completed=True,
                )
            else:
                self._state = "done"
                self.get_logger().info(
                    f"Already below threshold: {self._distance_m:.3f} m; "
                    "not moving."
                )
            return

        if self._state == "waiting":
            self._state = "moving"
            self._motion_started_time = now
            self.get_logger().info(
                f"Distance is {self._distance_m:.3f} m; starting movement."
            )

        if now - self._motion_started_time >= self.max_motion_s:
            self._stop(
                f"maximum motion time ({self.max_motion_s:.1f} s) reached",
                completed=False,
            )
            return

        parameter = json.dumps(
            {"x": self.vx, "y": self.vy, "z": self.vyaw},
            separators=(",", ":"),
        )
        try:
            self._publish_sport_request(SPORT_API_ID_MOVE, parameter)
        except Exception as exc:
            self._stop(f"Could not publish Move request: {exc}", completed=False)

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
        """Stop the robot if this node is interrupted while moving."""
        if self._state == "moving":
            self._stop("node shutdown", completed=False)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MoveUntilDistance()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Could not start move-until-distance node: {exc}")
        return 1
    finally:
        if node is not None:
            node.stop_if_moving()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
