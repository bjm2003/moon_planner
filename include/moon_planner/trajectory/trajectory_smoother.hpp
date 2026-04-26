#pragma once

#include "moon_planner/core/types.hpp"

#include <vector>

namespace moon_planner {

class TrajectorySmoother {
 public:
  std::vector<TrajectoryPoint> Smooth(const std::vector<TrajectoryPoint>& trajectory) const;
};

}  // namespace moon_planner
