"""Offline bicycle-model closed loop for early algorithm checks only."""

from __future__ import annotations

import json
from dataclasses import dataclass
from math import cos, hypot, pi, sin, tan

from controller import ControlCommand, ControllerConfig, LaneFollower, VehicleState
from path_tracking import Point


@dataclass
class KinematicCar:
    state: VehicleState
    wheelbase_m: float
    dt_s: float = 0.05

    def apply(self, command: ControlCommand) -> VehicleState:
        velocity = max(0.0, self.state.speed_mps + (2.5 * command.throttle - 4.0 * command.brake) * self.dt_s)
        yaw = self.state.yaw_rad + velocity * tan(command.front_wheel_rad) / self.wheelbase_m * self.dt_s
        self.state = VehicleState(
            self.state.x_m + velocity * cos(yaw) * self.dt_s,
            self.state.y_m + velocity * sin(yaw) * self.dt_s,
            yaw,
            velocity,
        )
        return self.state


def distance_to_route(point: Point, route: list[Point]) -> float:
    distances = []
    for start, end in zip(route, route[1:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length_sq = dx * dx + dy * dy
        fraction = 0.0 if length_sq == 0 else max(0.0, min(1.0,
            ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq
        ))
        distances.append(hypot(point[0] - start[0] - fraction * dx, point[1] - start[1] - fraction * dy))
    return min(distances)


def run_scenario(name: str, route: list[Point]) -> dict[str, float | str | bool]:
    config = ControllerConfig()
    controller = LaneFollower(config)
    car = KinematicCar(VehicleState(0.0, 0.25, 0.03, 0.0), config.wheelbase_m)
    max_lateral_error = 0.0
    for tick in range(1200):
        command = controller.step(car.state, route)
        state = car.apply(command)
        max_lateral_error = max(max_lateral_error, distance_to_route((state.x_m, state.y_m), route))
        goal_error = hypot(state.x_m - route[-1][0], state.y_m - route[-1][1])
        if goal_error < 0.8 and state.speed_mps < 0.25:
            return {"scenario": name, "finished": True, "time_s": round((tick + 1) * car.dt_s, 2),
                    "max_lateral_error_m": round(max_lateral_error, 3), "goal_error_m": round(goal_error, 3)}
    return {"scenario": name, "finished": False, "time_s": 1200 * car.dt_s,
            "max_lateral_error_m": round(max_lateral_error, 3), "goal_error_m": round(goal_error, 3)}


def main() -> int:
    straight = [(float(x), 0.0) for x in range(0, 121, 2)]
    curve = [(float(x), 0.0) for x in range(0, 31, 2)]
    curve += [(30.0 + 50.0 * sin((pi / 2) * i / 80), 50.0 - 50.0 * cos((pi / 2) * i / 80))
              for i in range(1, 81)]
    results = [run_scenario("straight", straight), run_scenario("radius_50m_curve", curve)]
    for result in results:
        print(json.dumps(result, ensure_ascii=False))
    return 0 if all(item["finished"] and item["max_lateral_error_m"] < 0.5 for item in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())

