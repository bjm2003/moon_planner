#include "moon_planner/config/planner_config.hpp"

namespace moon_planner {

bool PlannerConfig::IsValid() const {
  return grid_resolution_m > 0.0 && heading_bins >= 4 && planning_horizon_m > 0.0 &&
         max_planning_time_ms > 0.0 && max_expanded_nodes > 0 &&
         goal_tolerance_xy_m >= 0.0 && goal_tolerance_yaw_rad >= 0.0;
}

}  // namespace moon_planner
