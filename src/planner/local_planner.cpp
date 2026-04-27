#include "moon_planner/planner/local_planner.hpp"

#include "moon_planner/core/geometry.hpp"

#include <cmath>
#include <string>
#include <utility>

namespace moon_planner {

LocalPlanner::LocalPlanner(PlannerConfig planner_config,
                           VehicleConfig vehicle_config,
                           CostConfig cost_config,
                           PrimitiveLibrary primitive_library)
    : planner_config_(planner_config),
      lattice_planner_(planner_config, vehicle_config, cost_config, std::move(primitive_library)) {}

PlanningResult LocalPlanner::Plan(const PlanningRequest& request, const OccupancyGrid& occupancy) {
  return Plan(request, occupancy, nullptr);
}

PlanningResult LocalPlanner::Plan(const PlanningRequest& request,
                                  const OccupancyGrid& occupancy,
                                  const ElevationGrid* elevation) {
  const PlanningRequest local_request = BuildLocalRequest(request);
  PlanningResult result = lattice_planner_.Plan(local_request, occupancy, elevation);
  const double original_goal_distance = Distance2D(request.start.x, request.start.y, request.goal.x, request.goal.y);
  if (original_goal_distance > planner_config_.planning_horizon_m && result.status == PlannerStatus::kSuccess) {
    result.diagnostics.message = "local horizon target reached; " + result.diagnostics.message;
  }
  return result;
}

PlanningRequest LocalPlanner::BuildLocalRequest(const PlanningRequest& request) const {
  PlanningRequest local_request = request;
  const double dx = request.goal.x - request.start.x;
  const double dy = request.goal.y - request.start.y;
  const double distance = std::hypot(dx, dy);
  if (planner_config_.planning_horizon_m <= 0.0 || distance <= planner_config_.planning_horizon_m ||
      distance <= 1e-9) {
    return local_request;
  }

  const double ratio = planner_config_.planning_horizon_m / distance;
  local_request.goal.x = request.start.x + dx * ratio;
  local_request.goal.y = request.start.y + dy * ratio;
  local_request.goal.z = request.start.z + (request.goal.z - request.start.z) * ratio;
  local_request.goal.yaw = std::atan2(dy, dx);
  return local_request;
}

}  // namespace moon_planner
