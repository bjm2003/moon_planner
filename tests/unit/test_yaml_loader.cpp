#include "moon_planner/io/yaml_loader.hpp"

#include <cassert>
#include <cmath>

int main() {
  using namespace moon_planner;

  YamlLoader loader;
  assert(loader.Exists("config/planner_default.yaml"));
  assert(loader.Load("config/planner_default.yaml"));
  assert(loader.GetInt("heading_bins", 0) == 16);
  assert(loader.GetBool("allow_reverse", false));
  assert(std::abs(loader.GetDouble("grid_resolution_m", 0.0) - 0.1) < 1e-9);

  const PlannerConfig planner_config = LoadPlannerConfig("config/planner_default.yaml");
  assert(planner_config.IsValid());
  assert(planner_config.heading_bins == 16);
  assert(planner_config.max_expanded_nodes == 200000);

  const VehicleConfig vehicle_config = LoadVehicleConfig("config/vehicle_default.yaml");
  assert(vehicle_config.IsValid());
  assert(std::abs(vehicle_config.length_m - 1.2) < 1e-9);
  assert(std::abs(vehicle_config.max_slope_rad - 0.35) < 1e-9);

  const CostConfig cost_config = LoadCostConfig("config/cost_weights_default.yaml");
  assert(cost_config.IsValid());
  assert(std::abs(cost_config.safety - 4.0) < 1e-9);

  const PrimitiveGenerationConfig primitive_config = LoadPrimitiveGenerationConfig("config/primitive_default.yaml");
  assert(std::abs(primitive_config.duration_s - 1.0) < 1e-9);
  assert(primitive_config.linear_velocities_mps.size() == 4);
  assert(primitive_config.angular_velocities_radps.size() == 5);
  assert(std::abs(primitive_config.linear_velocities_mps.front() + 0.3) < 1e-9);

  return 0;
}
