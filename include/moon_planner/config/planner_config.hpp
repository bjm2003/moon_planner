#pragma once

namespace moon_planner {

struct PlannerConfig {
  double grid_resolution_m{0.1};
  int heading_bins{16};
  double planning_horizon_m{15.0};
  double max_planning_time_ms{900.0};
  int max_expanded_nodes{200000};
  bool allow_reverse{true};
  double goal_tolerance_xy_m{0.2};
  double goal_tolerance_yaw_rad{0.35};

  bool IsValid() const;
};

}  // namespace moon_planner
