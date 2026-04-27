#include "moon_planner/map/history_layer.hpp"
#include "moon_planner/map/map_fusion.hpp"

#include <cassert>
#include <cmath>

int main() {
  using namespace moon_planner;

  const GridIndex index(10, 10, 1.0);
  HistoryLayer history(index);
  assert(history.IsValid());

  history.MarkVisited(2.5, 2.5);
  assert(std::abs(history.Weight(2, 2) - 1.0) < 1e-9);

  history.MarkHistoricalObstacle(2.5, 2.5);
  assert(std::abs(history.Weight(2, 2) - 4.0) < 1e-9);

  history.MarkFailedRegion(4.5, 4.5);
  assert(std::abs(history.WeightWorld(4.5, 4.5) - 5.0) < 1e-9);

  history.Decay(0.5);
  assert(std::abs(history.Weight(2, 2) - 2.0) < 1e-9);
  assert(std::abs(history.Weight(4, 4) - 2.5) < 1e-9);

  CostMap cost_map(index, 0.0);
  history.AddPenaltyTo(&cost_map, 0.5);
  assert(std::abs(cost_map.Cost(2, 2) - 1.0) < 1e-9);
  assert(std::abs(cost_map.Cost(4, 4) - 1.25) < 1e-9);

  OccupancyGrid occupancy(index, OccupancyGrid::kFree);
  occupancy.SetCell(4, 4, OccupancyGrid::kOccupied);
  MapFusion fusion;
  CostMap fused = fusion.BuildCostMap(occupancy, nullptr, 0.0, 0.0, 0.35, 1.0, &history, 0.5);
  assert(std::abs(fused.Cost(2, 2) - 1.0) < 1e-9);
  assert(fused.IsLethal(4, 4));

  return 0;
}
