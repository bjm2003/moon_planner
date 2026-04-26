#pragma once

#include "moon_planner/core/status.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace moon_planner {

struct Point2D {
  double x{0.0};
  double y{0.0};
};

struct Point3D {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Pose3D {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

struct State {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  double pitch{0.0};
  double roll{0.0};
  double v{0.0};
  double omega{0.0};
  double timestamp_s{0.0};
};

struct TrajectoryPoint {
  State state;
  double relative_time_s{0.0};
};

enum class ObstacleType : std::uint8_t {
  kUnknown = 0,
  kPositive = 1,
  kNegative = 2,
  kSlope = 3
};

struct Obstacle {
  ObstacleType type{ObstacleType::kUnknown};
  Point3D center;
  double length{0.0};
  double width{0.0};
  double height{0.0};
  double confidence{1.0};
  double timestamp_s{0.0};
};

struct PlanningRequest {
  State start;
  State goal;
  double goal_tolerance_xy_m{0.2};
  double goal_tolerance_yaw_rad{0.35};
  bool allow_reverse{true};
};

struct PlannerDiagnostics {
  std::uint64_t expanded_nodes{0};
  double planning_time_ms{0.0};
  double path_length_m{0.0};
  double total_cost{0.0};
  double min_obstacle_distance_m{0.0};
  std::string message;
};

struct PlanningResult {
  PlannerStatus status{PlannerStatus::kInvalidInput};
  std::vector<State> states;
  std::vector<TrajectoryPoint> trajectory;
  PlannerDiagnostics diagnostics;
};

}  // namespace moon_planner
