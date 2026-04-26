#include "moon_planner/map/cost_map.hpp"

namespace moon_planner {

CostMap::CostMap(GridIndex index, double initial_cost) {
  Resize(index, initial_cost);
}

void CostMap::Resize(GridIndex index, double initial_cost) {
  index_ = index;
  costs_.assign(index_.CellCount(), initial_cost);
}

bool CostMap::SetCost(int x, int y, double cost) {
  if (!index_.IsInside(x, y)) {
    return false;
  }
  costs_[index_.FlatIndex(x, y)] = cost;
  return true;
}

double CostMap::Cost(int x, int y) const {
  if (!index_.IsInside(x, y)) {
    return kLethalCost;
  }
  return costs_[index_.FlatIndex(x, y)];
}

double CostMap::CostWorld(double x_m, double y_m) const {
  const auto cell = index_.WorldToCell(x_m, y_m);
  if (!cell) {
    return kLethalCost;
  }
  return Cost(cell->x, cell->y);
}

bool CostMap::IsLethal(int x, int y) const {
  return Cost(x, y) >= kLethalCost;
}

void CostMap::ApplyOccupancy(const OccupancyGrid& occupancy) {
  if (!IsValid() || occupancy.index().CellCount() != index_.CellCount()) {
    return;
  }
  for (int y = 0; y < index_.height(); ++y) {
    for (int x = 0; x < index_.width(); ++x) {
      if (occupancy.IsOccupiedCell(x, y)) {
        SetCost(x, y, kLethalCost);
      }
    }
  }
}

}  // namespace moon_planner
