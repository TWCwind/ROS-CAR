"""Platform-independent first lane-following control loop.

All units and frames here are internal conventions, not a SimOne SDK binding.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, isfinite, sin, sqrt
from typing import Sequence

from path_tracking import Point, forward_steering_angle, lookahead_distance, select_lookahead_point


@dataclass(frozen=True)
class VehicleState:
    x_m: float
    y_m: float
    yaw_rad: float
    speed_mps: float


@dataclass(frozen=True)
class ControlCommand:
    front_wheel_rad: float
    throttle: float  # internal normalized 0..1
    brake: float  # internal normalized 0..1


@dataclass(frozen=True)
class ControllerConfig:
    wheelbase_m: float = 2.8
    cruise_speed_mps: float = 8.0
    max_front_wheel_rad: float = 0.45
    comfortable_brake_mps2: float = 3.0
    goal_tolerance_m: float = 0.6
    speed_gain: float = 0.7


def _clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def _project_to_route(position: Point, route: Sequence[Point], start: int) -> tuple[int, Point, float]:
    """Find the closest point on the remaining polyline and distance to goal."""
    best_index, best_point, best_distance_sq = start, route[start], float("inf")
    for index in range(start, len(route) - 1):
        x1, y1 = route[index]
        x2, y2 = route[index + 1]
        dx, dy = x2 - x1, y2 - y1
        length_sq = dx * dx + dy * dy
        fraction = 0.0 if length_sq == 0 else _clamp(
            ((position[0] - x1) * dx + (position[1] - y1) * dy) / length_sq, 0.0, 1.0
        )
        point = (x1 + fraction * dx, y1 + fraction * dy)
        distance_sq = (position[0] - point[0]) ** 2 + (position[1] - point[1]) ** 2
        if distance_sq < best_distance_sq:
            best_index, best_point, best_distance_sq = index, point, distance_sq

    remaining = hypot(route[best_index + 1][0] - best_point[0], route[best_index + 1][1] - best_point[1])
    remaining += sum(
        hypot(route[i + 1][0] - route[i][0], route[i + 1][1] - route[i][1])
        for i in range(best_index + 1, len(route) - 1)
    )
    return best_index, best_point, remaining


class LaneFollower:
    def __init__(self, config: ControllerConfig = ControllerConfig()) -> None:
        if any(not isfinite(v) or v <= 0 for v in (
            config.wheelbase_m, config.cruise_speed_mps, config.max_front_wheel_rad,
            config.comfortable_brake_mps2, config.goal_tolerance_m, config.speed_gain,
        )):
            raise ValueError("controller configuration must contain positive finite values")
        self.config = config
        self._segment_index = 0

    def reset(self) -> None:
        self._segment_index = 0

    def step(self, state: VehicleState, route: Sequence[Point]) -> ControlCommand:
        """Read normalized state and route; return one normalized control command."""
        values = (state.x_m, state.y_m, state.yaw_rad, state.speed_mps)
        if any(not isfinite(v) for v in values) or state.speed_mps < 0:
            raise ValueError("vehicle state must be finite with nonnegative forward speed")
        if len(route) < 2 or any(len(p) != 2 or any(not isfinite(v) for v in p) for p in route):
            return ControlCommand(0.0, 0.0, 1.0)

        self._segment_index = min(self._segment_index, len(route) - 2)
        index, projected, remaining = _project_to_route(
            (state.x_m, state.y_m), route, self._segment_index
        )
        self._segment_index = index
        if remaining <= self.config.goal_tolerance_m:
            return ControlCommand(0.0, 0.0, 1.0)

        cosine, sine = cos(state.yaw_rad), sin(state.yaw_rad)

        def vehicle_frame(point: Point) -> Point:
            dx, dy = point[0] - state.x_m, point[1] - state.y_m
            return cosine * dx + sine * dy, -sine * dx + cosine * dy

        local_route = [vehicle_frame(projected)] + [vehicle_frame(p) for p in route[index + 1 :]]
        radius = lookahead_distance(state.speed_mps)
        target = select_lookahead_point(local_route, radius)
        if target[0] <= 0:
            return ControlCommand(0.0, 0.0, 1.0)
        steer = _clamp(
            forward_steering_angle(target, self.config.wheelbase_m),
            -self.config.max_front_wheel_rad,
            self.config.max_front_wheel_rad,
        )
        stopping_room = max(0.0, remaining - self.config.goal_tolerance_m)
        desired_speed = min(
            self.config.cruise_speed_mps,
            sqrt(2.0 * self.config.comfortable_brake_mps2 * stopping_room),
        )
        speed_error = desired_speed - state.speed_mps
        return ControlCommand(
            steer,
            _clamp(self.config.speed_gain * speed_error, 0.0, 1.0),
            _clamp(-self.config.speed_gain * speed_error, 0.0, 1.0),
        )

