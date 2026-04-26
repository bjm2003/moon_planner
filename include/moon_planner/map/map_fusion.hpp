#pragma once

#include "moon_planner/map/cost_map.hpp"
#include "moon_planner/map/elevation_grid.hpp"

namespace moon_planner {

class MapFusion {
 public:
  CostMap BuildCostMap(const OccupancyGrid& occupancy, const ElevationGrid* elevation = nullptr) const;
};

}  // namespace moon_planner
