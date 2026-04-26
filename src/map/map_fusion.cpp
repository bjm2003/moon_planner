#include "moon_planner/map/map_fusion.hpp"

namespace moon_planner {

CostMap MapFusion::BuildCostMap(const OccupancyGrid& occupancy, const ElevationGrid* elevation) const {
  CostMap cost_map(occupancy.index(), 0.0);
  cost_map.ApplyOccupancy(occupancy);
  if (elevation != nullptr && elevation->IsValid()) {
    for (int y = 0; y < occupancy.index().height(); ++y) {
      for (int x = 0; x < occupancy.index().width(); ++x) {
        if (!cost_map.IsLethal(x, y)) {
          cost_map.SetCost(x, y, cost_map.Cost(x, y) + elevation->SlopeMagnitude(x, y));
        }
      }
    }
  }
  return cost_map;
}

}  // namespace moon_planner
