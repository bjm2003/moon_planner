#include "moon_planner/config/cost_config.hpp"
#include "moon_planner/config/planner_config.hpp"
#include "moon_planner/config/vehicle_config.hpp"
#include "moon_planner/io/scenario_reader.hpp"
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
  PlanningScenario scenario = reader.Read("scenarios/dense_obstacles.yaml");
  scenario.request.allow_reverse = planner_config.allow_reverse;
  scenario.request.goal_tolerance_xy_m = planner_config.goal_tolerance_xy_m;
  scenario.request.goal_tolerance_yaw_rad = planner_config.goal_tolerance_yaw_rad;

  LatticePlanner planner(planner_config, vehicle_config, cost_config, std::move(library));
  PlanningResult result = planner.Plan(scenario.request, scenario.occupancy);
  assert(result.status == PlannerStatus::kSuccess);
  assert(result.states.size() > 2);
  return 0;
}
