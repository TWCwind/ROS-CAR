#include "controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace icv {
namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double low, double high) {
  return std::min(std::max(value, low), high);
}

double distance(const Point& a, const Point& b) {
  return std::hypot(a.x_m - b.x_m, a.y_m - b.y_m);
}

bool finite(const Point& point) {
  return std::isfinite(point.x_m) && std::isfinite(point.y_m);
}

struct Projection {
  std::size_t segment{};
  Point point{};
  double remaining_m{};
};

Projection projectToRoute(const Point& position,
                          const std::vector<Point>& route,
                          std::size_t start) {
  std::size_t best_index = start;
  Point best_point = route[start];
  double best_distance_sq = std::numeric_limits<double>::infinity();

  for (std::size_t i = start; i + 1 < route.size(); ++i) {
    const Point& a = route[i];
    const Point& b = route[i + 1];
    const double dx = b.x_m - a.x_m;
    const double dy = b.y_m - a.y_m;
    const double length_sq = dx * dx + dy * dy;
    const double fraction = length_sq == 0.0
        ? 0.0
        : clamp(((position.x_m - a.x_m) * dx +
                 (position.y_m - a.y_m) * dy) / length_sq, 0.0, 1.0);
    const Point projected{a.x_m + fraction * dx, a.y_m + fraction * dy};
    const double ex = position.x_m - projected.x_m;
    const double ey = position.y_m - projected.y_m;
    const double distance_sq = ex * ex + ey * ey;
    if (distance_sq < best_distance_sq) {
      best_index = i;
      best_point = projected;
      best_distance_sq = distance_sq;
    }
  }

  double remaining = distance(best_point, route[best_index + 1]);
  for (std::size_t i = best_index + 1; i + 1 < route.size(); ++i) {
    remaining += distance(route[i], route[i + 1]);
  }
  return {best_index, best_point, remaining};
}

Point toVehicleFrame(const Point& point, const VehicleState& state) {
  const double dx = point.x_m - state.x_m;
  const double dy = point.y_m - state.y_m;
  const double cosine = std::cos(state.yaw_rad);
  const double sine = std::sin(state.yaw_rad);
  return {cosine * dx + sine * dy, -sine * dx + cosine * dy};
}

}  // namespace

double lookaheadDistance(double speed_mps, double lookahead_time_s,
                         double min_m, double max_m) {
  if (!std::isfinite(speed_mps) || !std::isfinite(lookahead_time_s) ||
      !std::isfinite(min_m) || !std::isfinite(max_m) || speed_mps < 0.0 ||
      lookahead_time_s <= 0.0 || min_m <= 0.0 || max_m < min_m) {
    throw std::invalid_argument("invalid lookahead parameters");
  }
  return clamp(speed_mps * lookahead_time_s, min_m, max_m);
}

Point selectLookaheadPoint(const std::vector<Point>& local_path,
                           double radius_m, bool interpolate) {
  if (local_path.empty() || !std::isfinite(radius_m) || radius_m <= 0.0 ||
      !std::all_of(local_path.begin(), local_path.end(), finite)) {
    throw std::invalid_argument("lookahead path and radius must be valid");
  }

  for (std::size_t i = 0; i < local_path.size(); ++i) {
    const Point& point = local_path[i];
    if (std::hypot(point.x_m, point.y_m) < radius_m) {
      continue;
    }
    if (!interpolate || i == 0) {
      return point;
    }

    const Point& inside = local_path[i - 1];
    const double dx = point.x_m - inside.x_m;
    const double dy = point.y_m - inside.y_m;
    const double a = dx * dx + dy * dy;
    if (a == 0.0) {
      return point;
    }
    const double b = 2.0 * (inside.x_m * dx + inside.y_m * dy);
    const double c = inside.x_m * inside.x_m + inside.y_m * inside.y_m - radius_m * radius_m;
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) {
      return point;
    }
    const double fraction = (-b + std::sqrt(discriminant)) / (2.0 * a);
    if (fraction >= 0.0 && fraction <= 1.0) {
      return {inside.x_m + fraction * dx, inside.y_m + fraction * dy};
    }
    return point;
  }
  return local_path.back();
}

double forwardSteeringAngle(const Point& target, double wheelbase_m) {
  if (!finite(target) || !std::isfinite(wheelbase_m) ||
      target.x_m <= 0.0 || wheelbase_m <= 0.0) {
    throw std::invalid_argument("target must be ahead and wheelbase positive");
  }
  const double distance_sq = target.x_m * target.x_m + target.y_m * target.y_m;
  return std::atan(2.0 * wheelbase_m * target.y_m / distance_sq);
}

LaneFollower::LaneFollower(ControllerConfig config) : config_(config) {
  const double values[] = {config_.wheelbase_m, config_.cruise_speed_mps,
      config_.max_front_wheel_rad, config_.comfortable_brake_mps2,
      config_.goal_tolerance_m, config_.speed_gain};
  for (double value : values) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("controller config values must be positive and finite");
    }
  }
  if (config_.max_front_wheel_rad >= kPi / 2.0) {
    throw std::invalid_argument("front-wheel limit must be below 90 degrees");
  }
}

void LaneFollower::reset() noexcept { segment_index_ = 0; }

ControlCommand LaneFollower::step(const VehicleState& state,
                                 const std::vector<Point>& world_route) {
  if (!std::isfinite(state.x_m) || !std::isfinite(state.y_m) ||
      !std::isfinite(state.yaw_rad) || !std::isfinite(state.speed_mps) ||
      state.speed_mps < 0.0) {
    throw std::invalid_argument("state must be finite with nonnegative forward speed");
  }
  if (world_route.size() < 2 ||
      !std::all_of(world_route.begin(), world_route.end(), finite)) {
    return {0.0, 0.0, 1.0};
  }

  segment_index_ = std::min(segment_index_, world_route.size() - 2);
  const Projection projection = projectToRoute(
      {state.x_m, state.y_m}, world_route, segment_index_);
  segment_index_ = projection.segment;
  if (projection.remaining_m <= config_.goal_tolerance_m) {
    return {0.0, 0.0, 1.0};
  }

  std::vector<Point> local_route;
  local_route.reserve(world_route.size() - projection.segment);
  local_route.push_back(toVehicleFrame(projection.point, state));
  for (std::size_t i = projection.segment + 1; i < world_route.size(); ++i) {
    local_route.push_back(toVehicleFrame(world_route[i], state));
  }

  const Point target = selectLookaheadPoint(
      local_route, lookaheadDistance(state.speed_mps));
  if (target.x_m <= 0.0) {
    return {0.0, 0.0, 1.0};
  }
  const double steering = clamp(
      forwardSteeringAngle(target, config_.wheelbase_m),
      -config_.max_front_wheel_rad, config_.max_front_wheel_rad);
  const double stopping_room = std::max(
      0.0, projection.remaining_m - config_.goal_tolerance_m);
  const double desired_speed = std::min(
      config_.cruise_speed_mps,
      std::sqrt(2.0 * config_.comfortable_brake_mps2 * stopping_room));
  const double speed_error = desired_speed - state.speed_mps;
  return {steering,
      clamp(config_.speed_gain * speed_error, 0.0, 1.0),
      clamp(-config_.speed_gain * speed_error, 0.0, 1.0)};
}

}  // namespace icv
