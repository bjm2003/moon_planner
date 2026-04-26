#include "moon_planner/config/cost_config.hpp"
#include "moon_planner/config/planner_config.hpp"
#include "moon_planner/config/vehicle_config.hpp"
#include "moon_planner/model/motion_constraints.hpp"
#include "moon_planner/planner/lattice_planner.hpp"
#include "moon_planner/primitives/primitive_generator.hpp"

#include <cassert>

int main() {
  using namespace moon_planner;

  PlannerConfig planner_config;
  VehicleConfig vehicle_config;
  CostConfig cost_config;
  PrimitiveLibrary library = PrimitiveGenerator().Generate(
      planner_config, PrimitiveGenerationConfig(), MotionConstraints(vehicle_config));

  OccupancyGrid occupancy(GridIndex(160, 80, planner_config.grid_resolution_m), OccupancyGrid::kFree);
  PlanningRequest request;
  request.start.x = 2.0;
  request.start.y = 4.0;
  request.start.yaw = 0.0;
  request.goal.x = 13.0;
  request.goal.y = 4.0;
  request.goal.yaw = 0.0;

  LatticePlanner planner(planner_config, vehicle_config, cost_config, std::move(library));
  PlanningResult result = planner.Plan(request, occupancy);
  assert(result.status == PlannerStatus::kSuccess);
  assert(result.states.size() > 2);
  assert(result.diagnostics.expanded_nodes > 0);
  return 0;
}
