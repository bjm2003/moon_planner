#include "moon_planner/planner/recovery_planner.hpp"

namespace moon_planner {

PlanningResult RecoveryPlanner::BuildEmergencyStop(const State& current_state) const {
  PlanningResult result;
  result.status = PlannerStatus::kEmergencyStop;
  result.states.push_back(current_state);
  result.trajectory.push_back(TrajectoryPoint{current_state, 0.0});
  result.diagnostics.message = "emergency stop trajectory";
  return result;
}

}  // namespace moon_planner
