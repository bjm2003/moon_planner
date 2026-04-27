#include "moon_planner/config/cost_config.hpp"
#include "moon_planner/config/planner_config.hpp"
#include "moon_planner/config/vehicle_config.hpp"
#include "moon_planner/model/motion_constraints.hpp"
#include "moon_planner/planner/lattice_planner.hpp"
#include "moon_planner/planner/recovery_planner.hpp"
#include "moon_planner/primitives/primitive_generator.hpp"

#include <cassert>

int main() {
  using namespace moon_planner;

  PlannerConfig planner_config;
  VehicleConfig vehicle_config;
  CostConfig cost_config;
  PrimitiveLibrary library = PrimitiveGenerator().Generate(
      planner_config, PrimitiveGenerationConfig(), MotionConstraints(vehicle_config));

  OccupancyGrid occupancy(GridIndex(80, 80, planner_config.grid_resolution_m), OccupancyGrid::kFree);
  for (int y = 30; y <= 50; ++y) {
    for (int x = 24; x <= 36; ++x) {
      occupancy.SetCell(x, y, OccupancyGrid::kOccupied);
    }
  }

  PlanningRequest request;
  request.start.x = 3.0;
  request.start.y = 4.0;
  request.start.yaw = 0.0;
  request.goal.x = 6.0;
  request.goal.y = 4.0;
  request.goal.yaw = 0.0;
  request.allow_reverse = planner_config.allow_reverse;
  request.goal_tolerance_xy_m = planner_config.goal_tolerance_xy_m;
  request.goal_tolerance_yaw_rad = planner_config.goal_tolerance_yaw_rad;

  LatticePlanner planner(planner_config, vehicle_config, cost_config, std::move(library));
  const PlanningResult primary = planner.Plan(request, occupancy);
  assert(primary.status != PlannerStatus::kSuccess);

  RecoveryPlanner recovery;
  const PlanningResult recovered = recovery.BuildReverseRecovery(request.start, 0.5, 0.2);
  assert(recovered.status == PlannerStatus::kNearObstacleRecovery);
  assert(recovered.trajectory.size() == 2);
  assert(recovered.trajectory.back().state.x < request.start.x);
  assert(recovered.trajectory.back().state.v < 0.0);
  return 0;
}
