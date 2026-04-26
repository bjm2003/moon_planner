#pragma once

#include "moon_planner/core/types.hpp"

#include <vector>

namespace moon_planner {

struct PrimitiveGenerationConfig {
  double duration_s{1.0};
  double integration_step_s{0.1};
  std::vector<double> linear_velocities_mps{-0.3, 0.3, 0.6, 1.0};
  std::vector<double> angular_velocities_radps{-0.6, -0.3, 0.0, 0.3, 0.6};
};

struct MotionPrimitive {
  int id{0};
  int start_heading_bin{0};
  int end_heading_bin{0};
  int end_dx_cells{0};
  int end_dy_cells{0};
  double v_mps{0.0};
  double omega_radps{0.0};
  double duration_s{0.0};
  double length_m{0.0};
  double base_cost{0.0};
  std::vector<State> relative_states;
};

}  // namespace moon_planner
