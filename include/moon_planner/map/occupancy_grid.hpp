#pragma once

#include "moon_planner/map/grid_index.hpp"

#include <cstdint>
#include <vector>

namespace moon_planner {

class OccupancyGrid {
 public:
  static constexpr std::uint8_t kFree = 0;
  static constexpr std::uint8_t kOccupied = 100;
  static constexpr std::uint8_t kUnknown = 255;

  OccupancyGrid() = default;
  explicit OccupancyGrid(GridIndex index, std::uint8_t initial_value = kFree);

  const GridIndex& index() const { return index_; }
  bool IsValid() const { return index_.IsValid() && cells_.size() == index_.CellCount(); }
  void Resize(GridIndex index, std::uint8_t initial_value = kFree);

  bool SetCell(int x, int y, std::uint8_t value);
  std::uint8_t GetCell(int x, int y) const;
  bool SetOccupiedWorld(double x_m, double y_m);
  bool IsOccupiedCell(int x, int y) const;
  bool IsOccupiedWorld(double x_m, double y_m) const;

 private:
  GridIndex index_;
  std::vector<std::uint8_t> cells_;
};

}  // namespace moon_planner
