#include "moon_planner/planner/recovery_planner.hpp"

#include <cassert>
#include <cmath>

int main() {
  using namespace moon_planner;

  RecoveryPlanner recovery;
  State current;
  current.x = 5.0;
  current.y = 3.0;
  current.yaw = 0.0;

  const PlanningRequest request = recovery.BuildReverseRecoveryRequest(current, 0.5);
  assert(std::abs(request.goal.x - 4.5) < 1e-9);
  assert(std::abs(request.goal.y - 3.0) < 1e-9);
  assert(request.allow_reverse);

  const PlanningResult reverse = recovery.BuildReverseRecovery(current, 0.5, 0.25);
  assert(reverse.status == PlannerStatus::kNearObstacleRecovery);
  assert(reverse.states.size() == 2);
  assert(reverse.trajectory.size() == 2);
  assert(reverse.trajectory.back().state.v < 0.0);
  assert(std::abs(reverse.trajectory.back().relative_time_s - 2.0) < 1e-9);
  assert(std::abs(reverse.diagnostics.path_length_m - 0.5) < 1e-9);

  const PlanningResult stop = recovery.BuildReverseRecovery(current, 0.0, 0.25);
  assert(stop.status == PlannerStatus::kEmergencyStop);
  assert(stop.trajectory.size() == 1);

  return 0;
}
