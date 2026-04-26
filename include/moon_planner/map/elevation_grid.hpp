#pragma once

#include "moon_planner/map/grid_index.hpp"

#include <vector>

namespace moon_planner {

class ElevationGrid {
 public:
  ElevationGrid() = default;
  explicit ElevationGrid(GridIndex index, double initial_height_m = 0.0);

  const GridIndex& index() const { return index_; }
  bool IsValid() const { return index_.IsValid() && heights_m_.size() == index_.CellCount(); }
  bool SetHeight(int x, int y, double height_m);
  double Height(int x, int y) const;
  double SlopeMagnitude(int x, int y) const;

 private:
  GridIndex index_;
  std::vector<double> heights_m_;
};

}  // namespace moon_planner
