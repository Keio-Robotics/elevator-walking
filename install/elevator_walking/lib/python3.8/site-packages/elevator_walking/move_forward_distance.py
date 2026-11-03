#!/usr/bin/env python3
"""Move forward with Unitree ROS API and stop using wheel-q feedback."""

import json
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


WHEEL_INDICES = (12, 13, 14, 15)
DEFAULT_WHEEL_RADIUS_M = 0.0889
SPORT_API_ID_STOPMOVE = 1003
SPORT_API_ID_MOVE = 1008


class MoveForwardDistance(Node):
    """Refresh Move requests and track wheel rotation from LowState."""

    def __init__(self):
        super().__init__("move_forward_distance")

        self.declare_parameter("distance_m", 0.2)
        self.declare_parameter("vx", 0.1)
        self.declare_parameter("wheel_radius_m", DEFAULT_WHEEL_RADIUS_M)
        self.declare_parameter("lowstate_topic", "/lowstate")
        self.declare_parameter("command_period_s", 0.1)
        self.declare_parameter("startup_timeout_s", 5.0)
        self.declare_parameter("sensor_timeout_s", 1.0)
        self.declare_parameter("stall_timeout_s", 2.0)
        self.declare_parameter("max_motion_s", 30.0)

        self.target_distance_m = float(self.get_parameter("distance_m").value)
        self.vx = float(self.get_parameter("vx").value)
        self.wheel_radius_m = float(
            self.get_parameter("wheel_radius_m").value
        )
        self.lowstate_topic = str(
            self.get_parameter("lowstate_topic").value
        ).strip()
        self.command_period_s = float(
            self.get_parameter("command_period_s").value
        )
        self.startup_timeout_s = float(
            self.get_parameter("startup_timeout_s").value
        )
        self.sensor_timeout_s = float(
            self.get_parameter("sensor_timeout_s").value
        )
        self.stall_timeout_s = float(
            self.get_parameter("stall_timeout_s").value
        )
        self.max_motion_s = float(self.get_parameter("max_motion_s").value)

        self._validate_parameters()

        try:
            from unitree_api.msg import Request
            from unitree_go.msg import LowState
        except ImportError as exc:
            raise RuntimeError(
                "Unitree ROS messages are unavailable. Source the Foxy "
                "Unitree workspace providing unitree_api and unitree_go."
            ) from exc

        self._request_type = Request
        self._state = "waiting"
        self._created_at = time.monotonic()
        self._motion_started_at = None
        self._last_state_at = None
        self._last_progress_at = None
        self._last_move_command_at = None
        self._last_log_at = 0.0
        self._previous_q = None
        self._distance_m = 0.0
        self._wheel_speed_mps = 0.0

        self._sport_request_publisher = self.create_publisher(
            Request, "/api/sport/request", 10
        )
        self._lowstate_subscription = self.create_subscription(
            LowState,
            self.lowstate_topic,
            self._on_lowstate,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(0.05, self._control_tick)

        self.get_logger().info(
            f"Waiting for wheel state on {self.lowstate_topic}; target="
            f"{self.target_distance_m:.3f} m. Move command: "
            f"({self.vx:.3f}, 0, 0), repeated every "
            f"{self.command_period_s:.2f} s."
        )

    def _validate_parameters(self):
        values = (
            self.target_distance_m,
            self.vx,
            self.wheel_radius_m,
            self.command_period_s,
            self.startup_timeout_s,
            self.sensor_timeout_s,
            self.stall_timeout_s,
            self.max_motion_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("All numeric parameters must be finite")
        if self.target_distance_m <= 0.0:
            raise ValueError("distance_m must be greater than zero")
        if self.vx <= 0.0:
            raise ValueError("vx must be greater than zero for forward movement")
        if self.wheel_radius_m <= 0.0:
            raise ValueError("wheel_radius_m must be greater than zero")
        if min(
            self.startup_timeout_s,
            self.sensor_timeout_s,
            self.stall_timeout_s,
            self.max_motion_s,
            self.command_period_s,
        ) <= 0.0:
            raise ValueError("timeout parameters must be greater than zero")

    @property
    def finished(self):
        return self._state in ("done", "stopped")

    def _publish_sport_request(self, api_id, parameter=""):
        request = self._request_type()
        request.header.identity.api_id = api_id
        request.parameter = parameter
        request.binary = []
        self._sport_request_publisher.publish(request)

    def _publish_move_request(self):
        parameter = json.dumps(
            {"x": self.vx, "y": 0.0, "z": 0.0},
            separators=(",", ":"),
        )
        self._publish_sport_request(SPORT_API_ID_MOVE, parameter)
        self._last_move_command_at = time.monotonic()

    @staticmethod
    def _read_wheel_state(message):
        motors = message.motor_state
        if len(motors) <= max(WHEEL_INDICES):
            raise ValueError(
                f"LowState has {len(motors)} motors; expected wheel motors "
                f"at indices {WHEEL_INDICES}"
            )

        q = [float(motors[index].q) for index in WHEEL_INDICES]
        dq = [float(motors[index].dq) for index in WHEEL_INDICES]
        if not all(math.isfinite(value) for value in q + dq):
            raise ValueError("wheel q/dq contains a non-finite value")
        return q, dq

    @staticmethod
    def _angle_delta(current, previous):
        return (current - previous + math.pi) % (2.0 * math.pi) - math.pi

    def _on_lowstate(self, message):
        if self.finished:
            return

        now = time.monotonic()
        self._last_state_at = now
        try:
            q, dq = self._read_wheel_state(message)
        except Exception as exc:
            self._stop(f"invalid wheel state: {exc}", completed=False)
            return

        self._wheel_speed_mps = statistics.median(
            abs(value) for value in dq
        ) * self.wheel_radius_m

        if self._state == "waiting":
            self._previous_q = q
            self._motion_started_at = now
            self._last_progress_at = now
            self._state = "moving"
            try:
                self._publish_move_request()
            except Exception as exc:
                self._stop(f"could not publish Move request: {exc}", False)
                return
            self.get_logger().info(
                "LowState received; sent Move request. Tracking wheel q."
            )
            return

        wheel_turns = [
            abs(self._angle_delta(current, previous))
            for current, previous in zip(q, self._previous_q)
        ]
        step_m = statistics.median(wheel_turns) * self.wheel_radius_m
        self._previous_q = q
        self._distance_m += step_m

        if step_m > 0.0001:
            self._last_progress_at = now

        if now - self._last_log_at >= 0.5:
            self.get_logger().info(
                f"Estimated wheel travel: {self._distance_m:.3f}/"
                f"{self.target_distance_m:.3f} m; "
                f"wheel speed={self._wheel_speed_mps:.3f} m/s"
            )
            self._last_log_at = now

        if self._distance_m >= self.target_distance_m:
            self._stop(
                f"wheel estimate reached {self._distance_m:.3f} m",
                completed=True,
            )

    def _control_tick(self):
        if self.finished:
            return

        now = time.monotonic()
        if self._state == "waiting":
            if now - self._created_at > self.startup_timeout_s:
                self._stop(
                    f"no LowState received within {self.startup_timeout_s:.1f} s",
                    completed=False,
                )
            return

        if self._last_state_at is None or (
            now - self._last_state_at > self.sensor_timeout_s
        ):
            self._stop("LowState topic timed out", completed=False)
            return

        if now - self._motion_started_at > self.max_motion_s:
            self._stop(
                f"maximum motion time ({self.max_motion_s:.1f} s) reached",
                completed=False,
            )
            return

        if now - self._last_progress_at > self.stall_timeout_s:
            self._stop(
                f"wheel q stopped changing at {self._distance_m:.3f} m",
                completed=False,
            )
            return

        if (
            self._last_move_command_at is None
            or now - self._last_move_command_at >= self.command_period_s
        ):
            try:
                self._publish_move_request()
            except Exception as exc:
                self._stop(f"could not publish Move request: {exc}", False)

    def _stop(self, reason, completed):
        if self.finished:
            return

        was_moving = self._state == "moving"
        self._state = "done" if completed else "stopped"
        if was_moving:
            try:
                self._publish_sport_request(SPORT_API_ID_STOPMOVE)
            except Exception as exc:
                self.get_logger().error(f"Could not publish StopMove: {exc}")
                self._state = "stopped"
                completed = False

        if completed:
            self.get_logger().info(f"Stopped: {reason}.")
        else:
            self.get_logger().error(f"Stopped for safety: {reason}.")

    def stop_if_moving(self):
        if self._state == "moving":
            self._stop("node shutdown", completed=False)


def main(args=None):
    cli_args = sys.argv[1:] if args is None else args
    if cli_args and "--ros-args" not in cli_args:
        print(
            "Use ROS parameters, for example: --ros-args "
            "-p distance_m:=0.20 -p vx:=0.1. "
            "A network interface positional argument is not used."
        )
        return 2

    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = MoveForwardDistance()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        exit_code = 0 if node._state == "done" else 1
    except KeyboardInterrupt:
        if node is not None:
            node.stop_if_moving()
        exit_code = 130
    except Exception as exc:
        print(f"Could not start move-forward-distance node: {exc}")
        if node is not None:
            node.stop_if_moving()
    finally:
        if node is not None:
            node.stop_if_moving()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
