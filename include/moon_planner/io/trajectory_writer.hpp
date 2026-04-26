#pragma once

#include "moon_planner/core/types.hpp"

#include <string>
#include <vector>

namespace moon_planner {

class TrajectoryWriter {
 public:
  bool WriteCsv(const std::vector<TrajectoryPoint>& trajectory, const std::string& path) const;
};

}  // namespace moon_planner
