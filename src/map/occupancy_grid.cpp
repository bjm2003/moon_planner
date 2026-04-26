#include "moon_planner/map/occupancy_grid.hpp"

namespace moon_planner {

OccupancyGrid::OccupancyGrid(GridIndex index, std::uint8_t initial_value) {
  Resize(index, initial_value);
}

void OccupancyGrid::Resize(GridIndex index, std::uint8_t initial_value) {
  index_ = index;
  cells_.assign(index_.CellCount(), initial_value);
}

bool OccupancyGrid::SetCell(int x, int y, std::uint8_t value) {
  if (!index_.IsInside(x, y)) {
    return false;
  }
  cells_[index_.FlatIndex(x, y)] = value;
  return true;
}

std::uint8_t OccupancyGrid::GetCell(int x, int y) const {
  if (!index_.IsInside(x, y)) {
    return kOccupied;
  }
  return cells_[index_.FlatIndex(x, y)];
}

bool OccupancyGrid::SetOccupiedWorld(double x_m, double y_m) {
  const auto cell = index_.WorldToCell(x_m, y_m);
  return cell && SetCell(cell->x, cell->y, kOccupied);
}

bool OccupancyGrid::IsOccupiedCell(int x, int y) const {
  return GetCell(x, y) >= kOccupied;
}

bool OccupancyGrid::IsOccupiedWorld(double x_m, double y_m) const {
  const auto cell = index_.WorldToCell(x_m, y_m);
  if (!cell) {
    return true;
  }
  return IsOccupiedCell(cell->x, cell->y);
}

}  // namespace moon_planner
