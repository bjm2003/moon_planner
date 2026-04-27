#include "moon_planner/planner/lattice_planner.hpp"

#include "moon_planner/collision/collision_checker.hpp"
#include "moon_planner/collision/footprint.hpp"
#include "moon_planner/map/map_fusion.hpp"
#include "moon_planner/search/hybrid_astar.hpp"
#include "moon_planner/trajectory/trajectory_generator.hpp"

namespace moon_planner {

LatticePlanner::LatticePlanner(PlannerConfig planner_config,
                               VehicleConfig vehicle_config,
                               CostConfig cost_config,
                               PrimitiveLibrary primitive_library)
    : planner_config_(planner_config),
      vehicle_config_(vehicle_config),
      cost_config_(cost_config),
      primitive_library_(std::move(primitive_library)) {}

PlanningResult LatticePlanner::Plan(const PlanningRequest& request, const OccupancyGrid& occupancy) {
  return Plan(request, occupancy, nullptr);
}

PlanningResult LatticePlanner::Plan(const PlanningRequest& request,
                                    const OccupancyGrid& occupancy,
                                    const ElevationGrid* elevation) {
  MapFusion map_fusion;
  CostMap cost_map =
      map_fusion.BuildCostMap(occupancy, elevation, 0.5, 1.0, vehicle_config_.max_slope_rad, cost_config_.slope);
  HybridAStar search(planner_config_, cost_config_, CollisionChecker(Footprint(vehicle_config_)));
  PlanningResult result = search.Plan(request, occupancy, cost_map, primitive_library_);
  TrajectoryGenerator generator;
  result.trajectory = generator.Generate(result.states, vehicle_config_.max_linear_velocity_mps);
  return result;
}

}  // namespace moon_planner
