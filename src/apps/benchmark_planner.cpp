#include "moon_planner/config/cost_config.hpp"
#include "moon_planner/config/planner_config.hpp"
#include "moon_planner/config/vehicle_config.hpp"
#include "moon_planner/io/yaml_loader.hpp"
#include "moon_planner/model/motion_constraints.hpp"
#include "moon_planner/planner/lattice_planner.hpp"
#include "moon_planner/primitives/primitive_generator.hpp"

#include <iostream>

int main() {
  using namespace moon_planner;

  PlannerConfig planner_config = LoadPlannerConfig("config/planner_default.yaml");
  VehicleConfig vehicle_config = LoadVehicleConfig("config/vehicle_default.yaml");
  CostConfig cost_config = LoadCostConfig("config/cost_weights_default.yaml");
  PrimitiveGenerationConfig primitive_config = LoadPrimitiveGenerationConfig("config/primitive_default.yaml");
  PrimitiveLibrary library = PrimitiveGenerator().Generate(
      planner_config, primitive_config, MotionConstraints(vehicle_config));
  LatticePlanner planner(planner_config, vehicle_config, cost_config, std::move(library));

  int success_count = 0;
  constexpr int kRuns = 10;
  for (int i = 0; i < kRuns; ++i) {
    OccupancyGrid occupancy(GridIndex(160, 80, planner_config.grid_resolution_m), OccupancyGrid::kFree);
    PlanningRequest request;
    request.start.x = 2.0;
    request.start.y = 4.0;
    request.goal.x = 13.0;
    request.goal.y = 4.0;
    request.allow_reverse = true;
    PlanningResult result = planner.Plan(request, occupancy);
    if (result.status == PlannerStatus::kSuccess) {
      ++success_count;
    }
  }

  std::cout << "runs=" << kRuns << '\n';
  std::cout << "success_count=" << success_count << '\n';
  std::cout << "success_rate=" << static_cast<double>(success_count) / static_cast<double>(kRuns) << '\n';
  return success_count == kRuns ? 0 : 1;
}
