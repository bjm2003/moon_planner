#include "moon_planner/map/cost_map.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

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

void CostMap::ApplyObstacleDistanceCost(const OccupancyGrid& occupancy,
                                        double influence_radius_m,
                                        double max_cost) {
  if (!IsValid() || occupancy.index().CellCount() != index_.CellCount() ||
      influence_radius_m <= 0.0 || max_cost <= 0.0) {
    return;
  }

  std::vector<GridCell> occupied_cells;
  for (int y = 0; y < index_.height(); ++y) {
    for (int x = 0; x < index_.width(); ++x) {
      if (occupancy.IsOccupiedCell(x, y)) {
        occupied_cells.push_back(GridCell{x, y});
      }
    }
  }
  if (occupied_cells.empty()) {
    return;
  }

  const double radius_cells = influence_radius_m / index_.resolution_m();
  const int search_radius_cells = static_cast<int>(std::ceil(radius_cells));
  for (const GridCell& obstacle : occupied_cells) {
    const Point2D obstacle_center = index_.CellCenter(obstacle.x, obstacle.y);
    for (int dy = -search_radius_cells; dy <= search_radius_cells; ++dy) {
      for (int dx = -search_radius_cells; dx <= search_radius_cells; ++dx) {
        const int x = obstacle.x + dx;
        const int y = obstacle.y + dy;
        if (!index_.IsInside(x, y) || IsLethal(x, y)) {
          continue;
        }
        const Point2D cell_center = index_.CellCenter(x, y);
        const double distance_m = std::hypot(cell_center.x - obstacle_center.x, cell_center.y - obstacle_center.y);
        if (distance_m >= influence_radius_m) {
          continue;
        }
        const double normalized = 1.0 - distance_m / influence_radius_m;
        const double distance_cost = max_cost * normalized;
        SetCost(x, y, std::max(Cost(x, y), distance_cost));
      }
    }
  }
}

void CostMap::ApplySlopeCost(const ElevationGrid& elevation, double max_slope_rad, double max_cost) {
  if (!IsValid() || !elevation.IsValid() || elevation.index().CellCount() != index_.CellCount() ||
      max_slope_rad <= 0.0 || max_cost <= 0.0) {
    return;
  }

  for (int y = 0; y < index_.height(); ++y) {
    for (int x = 0; x < index_.width(); ++x) {
      if (IsLethal(x, y)) {
        continue;
      }
      const double slope = elevation.SlopeMagnitude(x, y);
      if (slope > max_slope_rad) {
        SetCost(x, y, kLethalCost);
        continue;
      }
      const double normalized = std::min(1.0, slope / max_slope_rad);
      SetCost(x, y, Cost(x, y) + max_cost * normalized);
    }
  }
}

}  // namespace moon_planner
