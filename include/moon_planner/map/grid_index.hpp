#pragma once

#include "moon_planner/core/types.hpp"

#include <cstddef>
#include <optional>

namespace moon_planner {

struct GridCell {
  int x{0};
  int y{0};
};

class GridIndex {
 public:
  GridIndex() = default;
  GridIndex(int width, int height, double resolution_m, Point2D origin = {});

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution_m() const { return resolution_m_; }
  Point2D origin() const { return origin_; }
  std::size_t CellCount() const;

  bool IsValid() const;
  bool IsInside(int x, int y) const;
  bool IsInside(const GridCell& cell) const;
  std::size_t FlatIndex(int x, int y) const;
  std::optional<GridCell> WorldToCell(double x_m, double y_m) const;
  Point2D CellCenter(int x, int y) const;

 private:
  int width_{0};
  int height_{0};
  double resolution_m_{0.1};
  Point2D origin_;
};

}  // namespace moon_planner
