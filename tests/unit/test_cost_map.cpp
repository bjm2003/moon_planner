#include "moon_planner/map/cost_map.hpp"

#include <cassert>
#include <cmath>

int main() {
  using namespace moon_planner;

  const GridIndex index(7, 7, 1.0);
  OccupancyGrid occupancy(index, OccupancyGrid::kFree);
  occupancy.SetCell(3, 3, OccupancyGrid::kOccupied);

  CostMap cost_map(index, 0.0);
  cost_map.ApplyOccupancy(occupancy);
  cost_map.ApplyObstacleDistanceCost(occupancy, 2.0, 10.0);

  assert(cost_map.IsLethal(3, 3));
  assert(cost_map.Cost(4, 3) > 0.0);
  assert(cost_map.Cost(4, 3) > cost_map.Cost(4, 4));
  assert(cost_map.Cost(5, 3) == 0.0);
  assert(cost_map.Cost(0, 0) == 0.0);

  ElevationGrid gentle(index, 0.0);
  for (int y = 0; y < index.height(); ++y) {
    for (int x = 0; x < index.width(); ++x) {
      gentle.SetHeight(x, y, 0.1 * static_cast<double>(x));
    }
  }
  CostMap slope_cost(index, 0.0);
  slope_cost.ApplySlopeCost(gentle, 0.35, 5.0);
  assert(!slope_cost.IsLethal(3, 3));
  assert(slope_cost.Cost(3, 3) > 0.0);

  ElevationGrid steep(index, 0.0);
  for (int y = 0; y < index.height(); ++y) {
    for (int x = 0; x < index.width(); ++x) {
      steep.SetHeight(x, y, 1.0 * static_cast<double>(x));
    }
  }
  CostMap lethal_slope(index, 0.0);
  lethal_slope.ApplySlopeCost(steep, 0.35, 5.0);
  assert(lethal_slope.IsLethal(3, 3));

  return 0;
}
