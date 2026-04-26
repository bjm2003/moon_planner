#pragma once

#include "moon_planner/core/types.hpp"

namespace moon_planner {

class Heuristic {
 public:
  double Estimate(const State& state, const State& goal) const;
};

}  // namespace moon_planner
