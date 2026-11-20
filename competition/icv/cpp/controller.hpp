#pragma once

// C++17 port of the repository's Python offline reference. The lookahead
// selection follows the approach adapted from pb_omni_pid_pursuit_controller
// (Copyright 2025 Lihan Chen, Apache-2.0); see ../README.md and ../LICENSE.

#include <cstddef>
#include <vector>

namespace icv {

struct Point {
  double x_m{};
  double y_m{};
};

struct VehicleState {
  double x_m{};
  double y_m{};
  double yaw_rad{};
  double speed_mps{};
};

struct ControlCommand {
  double front_wheel_rad{};
  double throttle{};  // Internal normalized range [0, 1].
  double brake{};     // Internal normalized range [0, 1].
};

struct ControllerConfig {
  double wheelbase_m{2.8};
  double cruise_speed_mps{8.0};
  double max_front_wheel_rad{0.45};
  double comfortable_brake_mps2{3.0};
  double goal_tolerance_m{0.6};
  double speed_gain{0.7};
};

double lookaheadDistance(double speed_mps, double lookahead_time_s = 1.0,
                         double min_m = 0.5, double max_m = 5.0);
Point selectLookaheadPoint(const std::vector<Point>& local_path,
                           double radius_m, bool interpolate = true);
double forwardSteeringAngle(const Point& target, double wheelbase_m);

class LaneFollower {
 public:
  explicit LaneFollower(ControllerConfig config = {});

  void reset() noexcept;
  ControlCommand step(const VehicleState& state,
                      const std::vector<Point>& world_route);

 private:
  ControllerConfig config_;
  std::size_t segment_index_{0};
};

}  // namespace icv
