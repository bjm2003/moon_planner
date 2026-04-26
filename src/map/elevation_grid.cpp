#include "moon_planner/map/elevation_grid.hpp"

#include <cmath>

namespace moon_planner {

ElevationGrid::ElevationGrid(GridIndex index, double initial_height_m)
    : index_(index), heights_m_(index.CellCount(), initial_height_m) {}

bool ElevationGrid::SetHeight(int x, int y, double height_m) {
  if (!index_.IsInside(x, y)) {
    return false;
  }
  heights_m_[index_.FlatIndex(x, y)] = height_m;
  return true;
}

double ElevationGrid::Height(int x, int y) const {
  if (!index_.IsInside(x, y)) {
    return 0.0;
  }
  return heights_m_[index_.FlatIndex(x, y)];
}

double ElevationGrid::SlopeMagnitude(int x, int y) const {
  if (!index_.IsInside(x, y) || x <= 0 || y <= 0 || x >= index_.width() - 1 || y >= index_.height() - 1) {
    return 0.0;
  }
  const double dzdx = (Height(x + 1, y) - Height(x - 1, y)) / (2.0 * index_.resolution_m());
  const double dzdy = (Height(x, y + 1) - Height(x, y - 1)) / (2.0 * index_.resolution_m());
  return std::atan(std::sqrt(dzdx * dzdx + dzdy * dzdy));
}

}  // namespace moon_planner
