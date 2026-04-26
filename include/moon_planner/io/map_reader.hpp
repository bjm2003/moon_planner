#pragma once

#include "moon_planner/map/occupancy_grid.hpp"

#include <string>

namespace moon_planner {

class MapReader {
 public:
  OccupancyGrid ReadEmptyMap(int width, int height, double resolution_m) const;
};

}  // namespace moon_planner
