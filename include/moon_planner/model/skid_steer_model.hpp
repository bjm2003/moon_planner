#pragma once

#include "moon_planner/core/types.hpp"

#include <vector>

namespace moon_planner {

class SkidSteerModel {
 public:
  explicit SkidSteerModel(double slip_ratio = 0.0);

  State Integrate(const State& initial, double v_mps, double omega_radps, double dt_s, double slope_rad = 0.0) const;
  std::vector<State> Rollout(const State& initial,
                             double v_mps,
                             double omega_radps,
                             double duration_s,
                             double step_s,
                             double slope_rad = 0.0) const;

 private:
  double slip_ratio_{0.0};
};

}  // namespace moon_planner
