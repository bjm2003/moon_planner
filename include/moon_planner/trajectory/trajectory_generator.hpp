#pragma once

#include "moon_planner/core/types.hpp"

#include <vector>

namespace moon_planner {

class TrajectoryGenerator {
 public:
  std::vector<TrajectoryPoint> Generate(const std::vector<State>& states, double nominal_speed_mps) const;
};

}  // namespace moon_planner
