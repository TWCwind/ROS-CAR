"""Small, simulator-independent path tracking reference for forward driving.

Lookahead selection is adapted from the Apache-2.0-licensed
pb_omni_pid_pursuit_controller (Copyright 2025 Lihan Chen), a submodule of
pb2025_sentry_nav. See README.md and LICENSE in this directory.
"""

from __future__ import annotations

from math import atan, hypot, isfinite, sqrt
from typing import Sequence

Point = tuple[float, float]


def lookahead_distance(
    speed_mps: float,
    *,
    fixed_m: float = 2.0,
    scale_with_speed: bool = True,
    lookahead_time_s: float = 1.0,
    min_m: float = 0.5,
    max_m: float = 5.0,
) -> float:
    """Choose a fixed or speed-scaled lookahead distance in metres."""
    values = (speed_mps, fixed_m, lookahead_time_s, min_m, max_m)
    if not all(isfinite(value) for value in values):
        raise ValueError("lookahead parameters must be finite")
    if speed_mps < 0 or fixed_m <= 0 or lookahead_time_s <= 0 or min_m <= 0 or max_m < min_m:
        raise ValueError("invalid lookahead parameters")
    if not scale_with_speed:
        return fixed_m
    return min(max(speed_mps * lookahead_time_s, min_m), max_m)


def select_lookahead_point(
    local_path: Sequence[Point], radius_m: float, *, interpolate: bool = True
) -> Point:
    """Select the first path point outside the lookahead circle.

    The path must be ordered from the vehicle towards its goal and expressed
    in the vehicle frame: +x forward, +y left. With interpolation, the first
    inside-to-outside segment is intersected with the circle.
    """
    if not local_path or not isfinite(radius_m) or radius_m <= 0:
        raise ValueError("a nonempty path and positive finite radius are required")
    if not all(len(p) == 2 and all(isfinite(v) for v in p) for p in local_path):
        raise ValueError("path points must be finite x/y pairs")

    for index, point in enumerate(local_path):
        if hypot(*point) < radius_m:
            continue
        if not interpolate or index == 0:
            return point
        inside = local_path[index - 1]
        dx, dy = point[0] - inside[0], point[1] - inside[1]
        a = dx * dx + dy * dy
        if a == 0:
            return point
        b = 2 * (inside[0] * dx + inside[1] * dy)
        c = inside[0] ** 2 + inside[1] ** 2 - radius_m**2
        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return point
        fraction = (-b + sqrt(discriminant)) / (2 * a)
        if 0 <= fraction <= 1:
            return (inside[0] + fraction * dx, inside[1] + fraction * dy)
        return point
    return local_path[-1]


def forward_steering_angle(target: Point, wheelbase_m: float) -> float:
    """Compute front-wheel steering angle for forward-only bicycle-model pursuit.

    Positive angle turns left. Caller must enforce steering limits and handle
    speed, braking, reverse motion, obstacles, and simulator-specific commands.
    """
    x, y = target
    if not all(isfinite(v) for v in (x, y, wheelbase_m)) or x <= 0 or wheelbase_m <= 0:
        raise ValueError("target must be ahead and wheelbase positive; all values finite")
    distance_sq = x * x + y * y
    return atan(2 * wheelbase_m * y / distance_sq)
