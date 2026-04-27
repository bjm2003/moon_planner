#include "moon_planner/config/cost_config.hpp"
#include "moon_planner/config/planner_config.hpp"
#include "moon_planner/config/vehicle_config.hpp"
#include "moon_planner/io/scenario_reader.hpp"
#include "moon_planner/map/map_fusion.hpp"
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

  ScenarioReader reader;
  PlanningScenario scenario = reader.Read("scenarios/history_obstacles.yaml");
  assert(scenario.history.IsValid());
  assert(scenario.history.Weight(75, 15) > 0.0);
  assert(scenario.history.Weight(95, 62) > scenario.history.Weight(75, 15));

  MapFusion fusion;
  CostMap cost_map = fusion.BuildCostMap(scenario.occupancy, &scenario.elevation, 0.5, 1.0,
                                         vehicle_config.max_slope_rad, cost_config.slope, &scenario.history,
                                         cost_config.history);
  assert(cost_map.Cost(75, 15) > 0.0);
  assert(cost_map.Cost(95, 62) > cost_map.Cost(75, 15));

  scenario.request.allow_reverse = planner_config.allow_reverse;
  scenario.request.goal_tolerance_xy_m = planner_config.goal_tolerance_xy_m;
  scenario.request.goal_tolerance_yaw_rad = planner_config.goal_tolerance_yaw_rad;

  LatticePlanner planner(planner_config, vehicle_config, cost_config, std::move(library));
  PlanningResult result = planner.Plan(scenario.request, scenario.occupancy, &scenario.elevation, &scenario.history);
  assert(result.status == PlannerStatus::kSuccess);
  assert(result.states.size() > 2);
  return 0;
}
