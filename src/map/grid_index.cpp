#include "moon_planner/map/grid_index.hpp"

#include <cmath>

namespace moon_planner {

GridIndex::GridIndex(int width, int height, double resolution_m, Point2D origin)
    : width_(width), height_(height), resolution_m_(resolution_m), origin_(origin) {}

std::size_t GridIndex::CellCount() const {
  if (width_ <= 0 || height_ <= 0) {
    return 0;
  }
  return static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
}

bool GridIndex::IsValid() const {
  return width_ > 0 && height_ > 0 && resolution_m_ > 0.0;
}

bool GridIndex::IsInside(int x, int y) const {
  return x >= 0 && y >= 0 && x < width_ && y < height_;
}

bool GridIndex::IsInside(const GridCell& cell) const {
  return IsInside(cell.x, cell.y);
}

std::size_t GridIndex::FlatIndex(int x, int y) const {
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(x);
}

std::optional<GridCell> GridIndex::WorldToCell(double x_m, double y_m) const {
  if (!IsValid()) {
    return std::nullopt;
  }
  const int ix = static_cast<int>(std::floor((x_m - origin_.x) / resolution_m_));
  const int iy = static_cast<int>(std::floor((y_m - origin_.y) / resolution_m_));
  if (!IsInside(ix, iy)) {
    return std::nullopt;
  }
  return GridCell{ix, iy};
}

Point2D GridIndex::CellCenter(int x, int y) const {
  return {origin_.x + (static_cast<double>(x) + 0.5) * resolution_m_,
          origin_.y + (static_cast<double>(y) + 0.5) * resolution_m_};
}

}  // namespace moon_planner
