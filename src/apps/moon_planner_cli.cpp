#include "moon_planner/config/cost_config.hpp"
#include "moon_planner/config/planner_config.hpp"
#include "moon_planner/config/vehicle_config.hpp"
#include "moon_planner/io/trajectory_writer.hpp"
#include "moon_planner/model/motion_constraints.hpp"
#include "moon_planner/planner/lattice_planner.hpp"
#include "moon_planner/primitives/primitive_generator.hpp"
#include "moon_planner/runtime/diagnostics.hpp"

#include <iostream>

int main(int argc, char** argv) {
  using namespace moon_planner;

  PlannerConfig planner_config;
  VehicleConfig vehicle_config;
  CostConfig cost_config;
  PrimitiveGenerationConfig primitive_config;

  MotionConstraints constraints(vehicle_config);
  PrimitiveGenerator generator;
  PrimitiveLibrary library = generator.Generate(planner_config, primitive_config, constraints);

  OccupancyGrid occupancy(GridIndex(160, 80, planner_config.grid_resolution_m), OccupancyGrid::kFree);
  for (int y = 70; y < 75; ++y) {
    occupancy.SetCell(70, y, OccupancyGrid::kOccupied);
  }

  PlanningRequest request;
  request.start.x = 2.0;
  request.start.y = 4.0;
  request.start.yaw = 0.0;
  request.goal.x = 13.0;
  request.goal.y = 4.0;
  request.goal.yaw = 0.0;
  request.allow_reverse = planner_config.allow_reverse;
  request.goal_tolerance_xy_m = planner_config.goal_tolerance_xy_m;
  request.goal_tolerance_yaw_rad = planner_config.goal_tolerance_yaw_rad;

  LatticePlanner planner(planner_config, vehicle_config, cost_config, std::move(library));
  PlanningResult result = planner.Plan(request, occupancy);

  std::cout << "status=" << ToString(result.status) << '\n';
  std::cout << FormatDiagnostics(result.diagnostics) << '\n';
  std::cout << "states=" << result.states.size() << ", trajectory_points=" << result.trajectory.size() << '\n';

  if (argc > 1) {
    TrajectoryWriter writer;
    if (!writer.WriteCsv(result.trajectory, argv[1])) {
      std::cerr << "failed to write trajectory: " << argv[1] << '\n';
      return 2;
    }
  }

  return result.status == PlannerStatus::kSuccess ? 0 : 1;
}
