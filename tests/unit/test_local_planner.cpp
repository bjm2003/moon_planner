#include "moon_planner/config/cost_config.hpp"
#include "moon_planner/config/planner_config.hpp"
#include "moon_planner/config/vehicle_config.hpp"
#include "moon_planner/model/motion_constraints.hpp"
#include "moon_planner/planner/local_planner.hpp"
#include "moon_planner/primitives/primitive_generator.hpp"

#include <cassert>
#include <cmath>
#include <utility>

int main() {
  using namespace moon_planner;

  PlannerConfig planner_config;
  planner_config.planning_horizon_m = 3.0;
  VehicleConfig vehicle_config;
  CostConfig cost_config;
  PrimitiveLibrary library = PrimitiveGenerator().Generate(
      planner_config, PrimitiveGenerationConfig(), MotionConstraints(vehicle_config));

  PlanningRequest request;
  request.start.x = 2.0;
  request.start.y = 4.0;
  request.goal.x = 13.0;
  request.goal.y = 4.0;
  request.goal.yaw = 0.0;
  request.allow_reverse = planner_config.allow_reverse;
  request.goal_tolerance_xy_m = planner_config.goal_tolerance_xy_m;
  request.goal_tolerance_yaw_rad = planner_config.goal_tolerance_yaw_rad;

  LocalPlanner planner(planner_config, vehicle_config, cost_config, std::move(library));
  const PlanningRequest local_request = planner.BuildLocalRequest(request);
  assert(std::abs(local_request.goal.x - 5.0) < 1e-9);
  assert(std::abs(local_request.goal.y - 4.0) < 1e-9);
  assert(std::abs(local_request.goal.yaw) < 1e-9);

  OccupancyGrid occupancy(GridIndex(160, 80, planner_config.grid_resolution_m), OccupancyGrid::kFree);
  HistoryLayer history(occupancy.index());
  history.MarkVisited(3.0, 1.0);
  const PlanningResult result = planner.Plan(request, occupancy);
  assert(result.status == PlannerStatus::kSuccess);
  assert(result.states.size() > 2);
  assert(!result.diagnostics.message.empty());

  const PlanningResult history_result = planner.Plan(request, occupancy, nullptr, &history);
  assert(history_result.status == PlannerStatus::kSuccess);
  assert(history_result.states.size() > 2);

  return 0;
}
