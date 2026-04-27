#include "moon_planner/config/cost_config.hpp"
#include "moon_planner/config/planner_config.hpp"
#include "moon_planner/config/vehicle_config.hpp"
#include "moon_planner/io/scenario_reader.hpp"
#include "moon_planner/io/trajectory_writer.hpp"
#include "moon_planner/io/yaml_loader.hpp"
#include "moon_planner/model/motion_constraints.hpp"
#include "moon_planner/planner/lattice_planner.hpp"
#include "moon_planner/primitives/primitive_generator.hpp"
#include "moon_planner/runtime/diagnostics.hpp"

#include <exception>
#include <iostream>
#include <string>

namespace {

bool HasYamlExtension(const std::string& path) {
  return path.size() >= 5 &&
         (path.substr(path.size() - 5) == ".yaml" || path.substr(path.size() - 4) == ".yml");
}

}  // namespace

int main(int argc, char** argv) {
  using namespace moon_planner;

  PlannerConfig planner_config = LoadPlannerConfig("config/planner_default.yaml");
  VehicleConfig vehicle_config = LoadVehicleConfig("config/vehicle_default.yaml");
  CostConfig cost_config = LoadCostConfig("config/cost_weights_default.yaml");
  PrimitiveGenerationConfig primitive_config = LoadPrimitiveGenerationConfig("config/primitive_default.yaml");

  MotionConstraints constraints(vehicle_config);
  PrimitiveGenerator generator;
  PrimitiveLibrary library = generator.Generate(planner_config, primitive_config, constraints);

  ScenarioReader scenario_reader;
  PlanningScenario scenario;
  std::string trajectory_output_path;
  try {
    if (argc > 1 && HasYamlExtension(argv[1])) {
      scenario = scenario_reader.Read(argv[1]);
      if (argc > 2) {
        trajectory_output_path = argv[2];
      }
    } else {
      scenario = scenario_reader.DefaultScenario();
      if (argc > 1) {
        trajectory_output_path = argv[1];
      }
    }
  } catch (const std::exception& error) {
    std::cerr << "failed to read scenario: " << error.what() << '\n';
    return 2;
  }

  PlanningRequest request = scenario.request;
  request.allow_reverse = planner_config.allow_reverse;
  request.goal_tolerance_xy_m = planner_config.goal_tolerance_xy_m;
  request.goal_tolerance_yaw_rad = planner_config.goal_tolerance_yaw_rad;

  LatticePlanner planner(planner_config, vehicle_config, cost_config, std::move(library));
  PlanningResult result = planner.Plan(request, scenario.occupancy, &scenario.elevation, &scenario.history);

  std::cout << "scenario=" << scenario.name << '\n';
  std::cout << "status=" << ToString(result.status) << '\n';
  std::cout << FormatDiagnostics(result.diagnostics) << '\n';
  std::cout << "states=" << result.states.size() << ", trajectory_points=" << result.trajectory.size() << '\n';

  if (!trajectory_output_path.empty()) {
    TrajectoryWriter writer;
    if (!writer.WriteCsv(result.trajectory, trajectory_output_path)) {
      std::cerr << "failed to write trajectory: " << trajectory_output_path << '\n';
      return 2;
    }
  }

  return result.status == PlannerStatus::kSuccess ? 0 : 1;
}
