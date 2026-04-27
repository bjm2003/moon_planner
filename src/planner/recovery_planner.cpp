#include "moon_planner/planner/recovery_planner.hpp"

#include "moon_planner/core/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace moon_planner {

PlanningRequest RecoveryPlanner::BuildReverseRecoveryRequest(const State& current_state,
                                                             double reverse_distance_m) const {
  PlanningRequest request;
  request.start = current_state;
  request.goal = current_state;
  const double distance = std::max(0.0, reverse_distance_m);
  request.goal.x = current_state.x - distance * std::cos(current_state.yaw);
  request.goal.y = current_state.y - distance * std::sin(current_state.yaw);
  request.goal.yaw = NormalizeAngle(current_state.yaw);
  request.allow_reverse = true;
  request.goal_tolerance_xy_m = 0.15;
  request.goal_tolerance_yaw_rad = 0.5;
  return request;
}

PlanningResult RecoveryPlanner::BuildReverseRecovery(const State& current_state,
                                                     double reverse_distance_m,
                                                     double reverse_speed_mps) const {
  if (reverse_distance_m <= 0.0 || reverse_speed_mps <= 0.0) {
    return BuildEmergencyStop(current_state);
  }

  const PlanningRequest request = BuildReverseRecoveryRequest(current_state, reverse_distance_m);
  PlanningResult result;
  result.status = PlannerStatus::kNearObstacleRecovery;
  result.states.push_back(request.start);
  result.states.push_back(request.goal);

  State start = request.start;
  start.v = 0.0;
  result.trajectory.push_back(TrajectoryPoint{start, 0.0});

  State goal = request.goal;
  goal.v = -reverse_speed_mps;
  const double duration_s = reverse_distance_m / reverse_speed_mps;
  result.trajectory.push_back(TrajectoryPoint{goal, duration_s});

  result.diagnostics.path_length_m = reverse_distance_m;
  result.diagnostics.message = "reverse recovery trajectory";
  return result;
}

PlanningResult RecoveryPlanner::BuildEmergencyStop(const State& current_state) const {
  PlanningResult result;
  result.status = PlannerStatus::kEmergencyStop;
  result.states.push_back(current_state);
  result.trajectory.push_back(TrajectoryPoint{current_state, 0.0});
  result.diagnostics.message = "emergency stop trajectory";
  return result;
}

}  // namespace moon_planner
