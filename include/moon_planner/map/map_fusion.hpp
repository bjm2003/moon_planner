#pragma once

#include "moon_planner/map/cost_map.hpp"
#include "moon_planner/map/elevation_grid.hpp"
#include "moon_planner/map/history_layer.hpp"

namespace moon_planner {

class MapFusion {
 public:
  CostMap BuildCostMap(const OccupancyGrid& occupancy,
                       const ElevationGrid* elevation = nullptr,
                       double obstacle_influence_radius_m = 0.5,
                       double obstacle_max_cost = 1.0,
                       double max_slope_rad = 0.35,
                       double slope_max_cost = 1.0,
                       const HistoryLayer* history = nullptr,
                       double history_penalty = 0.5) const;
};

}  // namespace moon_planner
