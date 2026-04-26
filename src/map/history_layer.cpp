#include "moon_planner/map/history_layer.hpp"

namespace moon_planner {

HistoryLayer::HistoryLayer(GridIndex index) : index_(index), visited_(index, OccupancyGrid::kFree) {}

void HistoryLayer::MarkVisited(double x_m, double y_m) {
  const auto cell = index_.WorldToCell(x_m, y_m);
  if (cell) {
    visited_.SetCell(cell->x, cell->y, OccupancyGrid::kOccupied);
  }
}

void HistoryLayer::AddPenaltyTo(CostMap* cost_map, double visited_penalty) const {
  if (cost_map == nullptr || !cost_map->IsValid()) {
    return;
  }
  for (int y = 0; y < index_.height(); ++y) {
    for (int x = 0; x < index_.width(); ++x) {
      if (visited_.IsOccupiedCell(x, y)) {
        cost_map->SetCost(x, y, cost_map->Cost(x, y) + visited_penalty);
      }
    }
  }
}

}  // namespace moon_planner
