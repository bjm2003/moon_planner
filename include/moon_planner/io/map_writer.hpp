#pragma once

#include "moon_planner/map/occupancy_grid.hpp"

#include <string>

namespace moon_planner {

class MapWriter {
 public:
  bool WriteAscii(const OccupancyGrid& grid, const std::string& path) const;
};

}  // namespace moon_planner
