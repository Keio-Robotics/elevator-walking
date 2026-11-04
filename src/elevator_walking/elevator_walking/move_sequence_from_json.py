#!/usr/bin/env python3
"""Execute turn-then-forward motion steps from a JSON file using ROS topics."""

import json
import math
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


WHEEL_INDICES = (12, 13, 14, 15)
DEFAULT_WHEEL_RADIUS_M = 0.0889
SPORT_API_ID_STOPMOVE = 1003
SPORT_API_ID_MOVE = 1008


class MoveSequenceFromJson(Node):
    """Run each JSON pair as a relative yaw turn followed by forward travel."""

    def __init__(self):
        super().__init__("move_sequence_from_json")
        self.declare_parameter("route_file", "")
        self.declare_parameter("vx", 0.1)
        self.declare_parameter("yaw_rate", 0.3)
        self.declare_parameter("min_yaw_rate", 0.08)
        self.declare_parameter("yaw_slowdown_deg", 20.0)
        self.declare_parameter("angle_tolerance_deg", 3.0)
        self.declare_parameter("wheel_radius_m", DEFAULT_WHEEL_RADIUS_M)
        self.declare_parameter("lowstate_topic", "/lowstate")
        self.declare_parameter("sport_state_topic", "/lf/sportmodestate")
        self.declare_parameter("command_period_s", 0.1)
        self.declare_parameter("startup_timeout_s", 5.0)
        self.declare_parameter("sensor_timeout_s", 2.0)
        self.declare_parameter("stall_timeout_s", 3.0)
        self.declare_parameter("max_phase_s", 60.0)
        self.declare_parameter("settle_time_s", 0.3)

        self.route_file = str(self.get_parameter("route_file").value).strip()
        self.vx = float(self.get_parameter("vx").value)
        self.yaw_rate = float(self.get_parameter("yaw_rate").value)
        self.min_yaw_rate = float(self.get_parameter("min_yaw_rate").value)
        self.yaw_slowdown_deg = float(
            self.get_parameter("yaw_slowdown_deg").value
        )
        self.angle_tolerance_deg = float(
            self.get_parameter("angle_tolerance_deg").value
        )
        self.wheel_radius_m = float(
            self.get_parameter("wheel_radius_m").value
        )
        self.lowstate_topic = str(
            self.get_parameter("lowstate_topic").value
        ).strip()
        self.sport_state_topic = str(
            self.get_parameter("sport_state_topic").value
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
        self.max_phase_s = float(self.get_parameter("max_phase_s").value)
        self.settle_time_s = float(self.get_parameter("settle_time_s").value)

        self._validate_parameters()
        self.phases = self._load_phases(self.route_file)

        try:
            from unitree_api.msg import Request
            from unitree_go.msg import LowState, SportModeState
        except ImportError as exc:
            raise RuntimeError(
                "Unitree ROS messages are unavailable. Source the Foxy "
                "workspace providing unitree_api and unitree_go."
            ) from exc

        self._request_type = Request
        self._sport_request_publisher = self.create_publisher(
            Request, "/api/sport/request", 10
        )
        self._lowstate_subscription = self.create_subscription(
            LowState,
            self.lowstate_topic,
            self._on_lowstate,
            qos_profile_sensor_data,
        )
        self._sport_state_subscription = self.create_subscription(
            SportModeState,
            self.sport_state_topic,
            self._on_sport_state,
            10,
        )

        self._state = "waiting"
        self._phase_index = 0
        self._created_at = time.monotonic()
        self._waiting_since = self._created_at
        self._phase_started_at = None
        self._settle_until = None
        self._last_lowstate_at = None
        self._last_sport_state_at = None
        self._last_progress_at = None
        self._last_move_command_at = None
        self._last_log_at = 0.0
        self._latest_wheel_q = None
        self._latest_wheel_speed_mps = 0.0
        self._latest_yaw = None
        self._previous_wheel_q = None
        self._previous_yaw = None
        self._forward_distance_m = 0.0
        self._rotation_progress_rad = 0.0
        self._rotation_target_rad = 0.0

        self._timer = self.create_timer(0.05, self._control_tick)
        self.get_logger().info(
            f"Loaded {len(self.phases)} motion phase(s) from {self.route_file}. "
            f"Forward Move vx={self.vx:.3f} m/s; yaw rate up to "
            f"{self.yaw_rate:.3f} rad/s. Rotation uses "
            f"{self.sport_state_topic}; forward distance uses "
            f"{self.lowstate_topic}."
        )

    def _validate_parameters(self):
        numeric = (
            self.vx,
            self.yaw_rate,
            self.min_yaw_rate,
            self.yaw_slowdown_deg,
            self.angle_tolerance_deg,
            self.wheel_radius_m,
            self.command_period_s,
            self.startup_timeout_s,
            self.sensor_timeout_s,
            self.stall_timeout_s,
            self.max_phase_s,
            self.settle_time_s,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("All numeric parameters must be finite")
        if not self.route_file:
            raise ValueError(
                "route_file is required; point it to a JSON array of "
                "[forward_distance_m, rotation_degrees] pairs"
            )
        if self.vx <= 0.0:
            raise ValueError("vx must be positive for forward movement")
        if self.min_yaw_rate <= 0.0 or self.yaw_rate < self.min_yaw_rate:
            raise ValueError("yaw_rate must be >= min_yaw_rate > 0")
        if self.angle_tolerance_deg <= 0.0:
            raise ValueError("angle_tolerance_deg must be positive")
        if self.yaw_slowdown_deg <= self.angle_tolerance_deg:
            raise ValueError(
                "yaw_slowdown_deg must be greater than angle_tolerance_deg"
            )
        if self.wheel_radius_m <= 0.0:
            raise ValueError("wheel_radius_m must be positive")
        if min(
            self.command_period_s,
            self.startup_timeout_s,
            self.sensor_timeout_s,
            self.stall_timeout_s,
            self.max_phase_s,
        ) <= 0.0:
            raise ValueError("timeouts and command period must be positive")
        if self.settle_time_s < 0.0:
            raise ValueError("settle_time_s cannot be negative")

    @staticmethod
    def _load_phases(route_file):
        path = Path(route_file).expanduser()
        try:
            with path.open("r", encoding="utf-8") as route_stream:
                route = json.load(route_stream)
        except OSError as exc:
            raise ValueError(f"Could not read route file {path}: {exc}") from exc
        except json.JSONDecodeError as exc:
            raise ValueError(
                f"Invalid JSON in route file {path}: {exc}"
            ) from exc

        if not isinstance(route, list) or not route:
            raise ValueError("route_file must contain a non-empty JSON array")

        phases = []
        for row_index, pair in enumerate(route, start=1):
            if not isinstance(pair, list) or len(pair) != 2:
                raise ValueError(
                    f"Route row {row_index} must be "
                    "[forward_distance_m, rotation_degrees]"
                )
            if any(isinstance(value, bool) for value in pair):
                raise ValueError(f"Route row {row_index} values must be numbers")
            try:
                distance_m, rotation_deg = (float(value) for value in pair)
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"Route row {row_index} values must be numbers"
                ) from exc
            if not math.isfinite(distance_m) or not math.isfinite(rotation_deg):
                raise ValueError(f"Route row {row_index} values must be finite")
            if distance_m < 0.0:
                raise ValueError(
                    f"Route row {row_index} forward distance cannot be negative"
                )

            # Each row is executed as a turn first, then forward travel.
            if abs(rotation_deg) > 0.0:
                phases.append(("rotate", math.radians(rotation_deg), row_index))
            if distance_m > 0.0:
                phases.append(("forward", distance_m, row_index))

        return phases

    @property
    def finished(self):
        return self._state in ("done", "stopped")

    @staticmethod
    def _angle_delta(current, previous):
        return (current - previous + math.pi) % (2.0 * math.pi) - math.pi

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

    def _on_lowstate(self, message):
        now = time.monotonic()
        try:
            q, dq = self._read_wheel_state(message)
        except Exception as exc:
            if self._state == "forwarding":
                self._fail(f"invalid wheel state: {exc}")
            return

        self._last_lowstate_at = now
        self._latest_wheel_q = q
        self._latest_wheel_speed_mps = (
            statistics.median(abs(value) for value in dq) * self.wheel_radius_m
        )

        if self._state != "forwarding" or self._previous_wheel_q is None:
            return

        wheel_turns = [
            abs(self._angle_delta(current, previous))
            for current, previous in zip(q, self._previous_wheel_q)
        ]
        step_m = statistics.median(wheel_turns) * self.wheel_radius_m
        self._previous_wheel_q = q
        self._forward_distance_m += step_m
        if step_m > 0.0001:
            self._last_progress_at = now

    def _on_sport_state(self, message):
        try:
            yaw = float(message.imu_state.rpy[2])
        except (AttributeError, IndexError, TypeError, ValueError):
            return
        if not math.isfinite(yaw):
            return

        now = time.monotonic()
        self._last_sport_state_at = now
        self._latest_yaw = yaw
        if self._state != "rotating" or self._previous_yaw is None:
            return

        delta = self._angle_delta(yaw, self._previous_yaw)
        self._previous_yaw = yaw
        self._rotation_progress_rad += delta
        if abs(delta) >= math.radians(0.3):
            self._last_progress_at = now

    def _publish_sport_request(self, api_id, parameter=""):
        request = self._request_type()
        request.header.identity.api_id = api_id
        request.parameter = parameter
        request.binary = []
        self._sport_request_publisher.publish(request)

    def _publish_move(self, vx, vyaw):
        parameter = json.dumps(
            {"x": vx, "y": 0.0, "z": vyaw},
            separators=(",", ":"),
        )
        self._publish_sport_request(SPORT_API_ID_MOVE, parameter)
        self._last_move_command_at = time.monotonic()

    def _publish_stop(self):
        self._publish_sport_request(SPORT_API_ID_STOPMOVE)

    def _start_current_phase(self, now):
        kind, target, row_index = self.phases[self._phase_index]
        self._phase_started_at = now
        self._last_progress_at = now
        self._last_move_command_at = None
        self._last_log_at = 0.0

        if kind == "rotate":
            if self._latest_yaw is None:
                return
            if abs(target) <= math.radians(self.angle_tolerance_deg):
                self.get_logger().info(
                    f"Row {row_index}: requested turn "
                    f"{math.degrees(target):+.1f}° is within the "
                    "configured angle tolerance; skipping it."
                )
                self._phase_index += 1
                self._state = "settling"
                self._settle_until = now + self.settle_time_s
                return
            self._rotation_target_rad = target
            self._rotation_progress_rad = 0.0
            self._previous_yaw = self._latest_yaw
            self._state = "rotating"
            self.get_logger().info(
                f"Row {row_index}: rotate {math.degrees(target):+.1f}° "
                f"using {self.sport_state_topic}."
            )
            self._send_rotation_command(now)
            return

        if self._latest_wheel_q is None:
            return
        self._forward_distance_m = 0.0
        self._previous_wheel_q = list(self._latest_wheel_q)
        self._state = "forwarding"
        self.get_logger().info(
            f"Row {row_index}: move forward {target:.3f} m using wheel q."
        )
        self._publish_move(self.vx, 0.0)

    def _send_rotation_command(self, now):
        direction = 1.0 if self._rotation_target_rad > 0.0 else -1.0
        directed_progress = self._rotation_progress_rad * direction
        remaining = max(0.0, abs(self._rotation_target_rad) - directed_progress)
        slowdown = math.radians(self.yaw_slowdown_deg)
        if remaining >= slowdown:
            speed = self.yaw_rate
        else:
            speed = max(
                self.min_yaw_rate,
                self.yaw_rate * remaining / slowdown,
            )
        self._publish_move(0.0, direction * speed)

    def _required_sensor_ready(self, phase, now):
        kind, _, _ = phase
        if kind == "rotate":
            return (
                self._latest_yaw is not None
                and self._last_sport_state_at is not None
                and now - self._last_sport_state_at <= self.sensor_timeout_s
            )
        return (
            self._latest_wheel_q is not None
            and self._last_lowstate_at is not None
            and now - self._last_lowstate_at <= self.sensor_timeout_s
        )

    def _control_tick(self):
        if self.finished:
            return
        now = time.monotonic()

        if self._state == "waiting":
            if self._phase_index >= len(self.phases):
                self._state = "done"
                self.get_logger().info("Motion sequence complete.")
                return

            phase = self.phases[self._phase_index]
            if not self._required_sensor_ready(phase, now):
                if now - self._waiting_since > self.startup_timeout_s:
                    kind = phase[0]
                    topic = (
                        self.sport_state_topic
                        if kind == "rotate"
                        else self.lowstate_topic
                    )
                    self._fail(
                        f"no fresh state received on {topic} within "
                        f"{self.startup_timeout_s:.1f} s"
                    )
                return
            try:
                self._start_current_phase(now)
            except Exception as exc:
                self._fail(f"could not start motion phase: {exc}")
            return

        if self._state == "settling":
            if self._settle_until is not None and now >= self._settle_until:
                self._state = "waiting"
                self._waiting_since = now
            return

        if self._phase_started_at is None:
            self._fail("internal error: phase start time is missing")
            return
        if now - self._phase_started_at > self.max_phase_s:
            self._fail(
                f"maximum phase time ({self.max_phase_s:.1f} s) reached"
            )
            return

        if self._state == "rotating":
            if (
                self._last_sport_state_at is None
                or now - self._last_sport_state_at > self.sensor_timeout_s
            ):
                self._fail("SportModeState topic timed out during rotation")
                return
            if (
                self._last_progress_at is not None
                and now - self._last_progress_at > self.stall_timeout_s
            ):
                self._fail(
                    "yaw stopped changing at "
                    f"{math.degrees(self._rotation_progress_rad):+.1f}°"
                )
                return

            direction = 1.0 if self._rotation_target_rad > 0.0 else -1.0
            directed_progress = self._rotation_progress_rad * direction
            target_magnitude = abs(self._rotation_target_rad)
            tolerance = math.radians(self.angle_tolerance_deg)
            if directed_progress >= target_magnitude - tolerance:
                self._finish_phase(
                    f"yaw change {math.degrees(self._rotation_progress_rad):+.1f}°"
                )
                return

            if (
                self._last_move_command_at is None
                or now - self._last_move_command_at >= self.command_period_s
            ):
                try:
                    self._send_rotation_command(now)
                except Exception as exc:
                    self._fail(f"could not publish yaw Move request: {exc}")
                    return

            if now - self._last_log_at >= 0.5:
                self.get_logger().info(
                    f"Yaw change: {math.degrees(self._rotation_progress_rad):+.1f}/"
                    f"{math.degrees(self._rotation_target_rad):+.1f}°"
                )
                self._last_log_at = now
            return

        if self._state == "forwarding":
            if (
                self._last_lowstate_at is None
                or now - self._last_lowstate_at > self.sensor_timeout_s
            ):
                self._fail("LowState topic timed out during forward motion")
                return
            if (
                self._last_progress_at is not None
                and now - self._last_progress_at > self.stall_timeout_s
            ):
                self._fail(
                    "wheel q stopped changing at "
                    f"{self._forward_distance_m:.3f} m"
                )
                return

            _, target_distance_m, _ = self.phases[self._phase_index]
            if self._forward_distance_m >= target_distance_m:
                self._finish_phase(
                    f"wheel estimate reached {self._forward_distance_m:.3f} m"
                )
                return

            if (
                self._last_move_command_at is None
                or now - self._last_move_command_at >= self.command_period_s
            ):
                try:
                    self._publish_move(self.vx, 0.0)
                except Exception as exc:
                    self._fail(f"could not publish forward Move request: {exc}")
                    return

            if now - self._last_log_at >= 0.5:
                self.get_logger().info(
                    f"Estimated wheel travel: {self._forward_distance_m:.3f}/"
                    f"{target_distance_m:.3f} m; "
                    f"wheel speed={self._latest_wheel_speed_mps:.3f} m/s"
                )
                self._last_log_at = now

    def _finish_phase(self, reason):
        try:
            self._publish_stop()
        except Exception as exc:
            self._state = "stopped"
            self.get_logger().error(
                f"Could not publish StopMove after phase: {exc}"
            )
            return

        self.get_logger().info(f"Phase complete: {reason}; sent StopMove.")
        self._phase_index += 1
        self._state = "settling"
        self._settle_until = time.monotonic() + self.settle_time_s

    def _fail(self, reason):
        was_moving = self._state in ("rotating", "forwarding")
        self._state = "stopped"
        if was_moving:
            try:
                self._publish_stop()
            except Exception as exc:
                self.get_logger().error(f"Could not publish safety StopMove: {exc}")
        self.get_logger().error(f"Stopped for safety: {reason}.")

    def stop_if_moving(self):
        if self._state in ("rotating", "forwarding"):
            self._fail("node shutdown")


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = MoveSequenceFromJson()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        exit_code = 0 if node._state == "done" else 1
    except KeyboardInterrupt:
        if node is not None:
            node.stop_if_moving()
        exit_code = 130
    except Exception as exc:
        print(f"Could not start JSON motion sequence: {exc}")
    finally:
        if node is not None:
            node.stop_if_moving()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
