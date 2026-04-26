#pragma once

#include <string>

namespace moon_planner {

enum class PlannerStatus {
  kSuccess,
  kInvalidInput,
  kMapInvalid,
  kStartInCollision,
  kGoalInCollision,
  kTimeout,
  kNoPath,
  kNearObstacleRecovery,
  kEmergencyStop
};

inline const char* ToString(PlannerStatus status) {
  switch (status) {
    case PlannerStatus::kSuccess:
      return "success";
    case PlannerStatus::kInvalidInput:
      return "invalid_input";
    case PlannerStatus::kMapInvalid:
      return "map_invalid";
    case PlannerStatus::kStartInCollision:
      return "start_in_collision";
    case PlannerStatus::kGoalInCollision:
      return "goal_in_collision";
    case PlannerStatus::kTimeout:
      return "timeout";
    case PlannerStatus::kNoPath:
      return "no_path";
    case PlannerStatus::kNearObstacleRecovery:
      return "near_obstacle_recovery";
    case PlannerStatus::kEmergencyStop:
      return "emergency_stop";
  }
  return "unknown";
}

}  // namespace moon_planner
