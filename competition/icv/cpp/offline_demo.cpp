#include "controller.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct KinematicCar {
  icv::VehicleState state{0.0, 0.25, 0.03, 0.0};
  double wheelbase_m{2.8};
  double dt_s{0.05};

  void apply(const icv::ControlCommand& command) {
    const double acceleration = 2.5 * command.throttle - 4.0 * command.brake;
    const double velocity = std::max(0.0, state.speed_mps + acceleration * dt_s);
    const double yaw = state.yaw_rad + velocity *
        std::tan(command.front_wheel_rad) / wheelbase_m * dt_s;
    state = {state.x_m + velocity * std::cos(yaw) * dt_s,
             state.y_m + velocity * std::sin(yaw) * dt_s,
             yaw, velocity};
  }
};

double distanceToRoute(const icv::Point& point,
                       const std::vector<icv::Point>& route) {
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < route.size(); ++i) {
    const auto& start = route[i];
    const auto& end = route[i + 1];
    const double dx = end.x_m - start.x_m;
    const double dy = end.y_m - start.y_m;
    const double length_sq = dx * dx + dy * dy;
    const double fraction = length_sq == 0.0 ? 0.0 : std::clamp(
        ((point.x_m - start.x_m) * dx + (point.y_m - start.y_m) * dy) /
            length_sq,
        0.0, 1.0);
    const double ex = point.x_m - start.x_m - fraction * dx;
    const double ey = point.y_m - start.y_m - fraction * dy;
    best = std::min(best, std::hypot(ex, ey));
  }
  return best;
}

bool runScenario(const std::string& name,
                 const std::vector<icv::Point>& route) {
  icv::ControllerConfig config;
  icv::LaneFollower controller(config);
  KinematicCar car;
  double max_lateral_error = 0.0;
  double goal_error = std::numeric_limits<double>::infinity();
  bool finished = false;
  int tick = 0;

  for (; tick < 1200; ++tick) {
    const auto command = controller.step(car.state, route);
    car.apply(command);
    max_lateral_error = std::max(max_lateral_error,
        distanceToRoute({car.state.x_m, car.state.y_m}, route));
    goal_error = std::hypot(car.state.x_m - route.back().x_m,
                            car.state.y_m - route.back().y_m);
    if (goal_error < 0.8 && car.state.speed_mps < 0.25) {
      finished = true;
      ++tick;
      break;
    }
  }

  std::cout << std::fixed << std::setprecision(3)
            << "scenario=" << name
            << " finished=" << (finished ? "true" : "false")
            << " time_s=" << std::setprecision(2) << tick * car.dt_s
            << " max_lateral_error_m=" << std::setprecision(3) << max_lateral_error
            << " goal_error_m=" << goal_error << '\n';
  return finished && max_lateral_error < 0.5;
}

std::vector<icv::Point> makeStraightRoute() {
  std::vector<icv::Point> route;
  for (int x = 0; x <= 120; x += 2) {
    route.push_back({static_cast<double>(x), 0.0});
  }
  return route;
}

std::vector<icv::Point> makeRadius50Route() {
  std::vector<icv::Point> route;
  for (int x = 0; x <= 30; x += 2) {
    route.push_back({static_cast<double>(x), 0.0});
  }
  for (int i = 1; i <= 80; ++i) {
    const double angle = (kPi / 2.0) * i / 80.0;
    route.push_back({30.0 + 50.0 * std::sin(angle),
                     50.0 - 50.0 * std::cos(angle)});
  }
  return route;
}

}  // namespace

int main() {
  const bool straight_ok = runScenario("straight", makeStraightRoute());
  const bool curve_ok = runScenario("radius_50m_curve", makeRadius50Route());
  return straight_ok && curve_ok ? 0 : 1;
}
