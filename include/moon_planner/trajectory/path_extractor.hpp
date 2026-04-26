#pragma once

#include "moon_planner/core/types.hpp"

#include <vector>

namespace moon_planner {

class PathExtractor {
 public:
  std::vector<State> Simplify(const std::vector<State>& path) const;
};

}  // namespace moon_planner
