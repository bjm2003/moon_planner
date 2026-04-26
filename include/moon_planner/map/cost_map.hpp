#pragma once

#include "moon_planner/map/occupancy_grid.hpp"

#include <vector>

namespace moon_planner {

class CostMap {
 public:
  static constexpr double kLethalCost = 1.0e9;

  CostMap() = default;
  explicit CostMap(GridIndex index, double initial_cost = 0.0);

  const GridIndex& index() const { return index_; }
  bool IsValid() const { return index_.IsValid() && costs_.size() == index_.CellCount(); }
  void Resize(GridIndex index, double initial_cost = 0.0);
  bool SetCost(int x, int y, double cost);
  double Cost(int x, int y) const;
  double CostWorld(double x_m, double y_m) const;
  bool IsLethal(int x, int y) const;
  void ApplyOccupancy(const OccupancyGrid& occupancy);

 private:
  GridIndex index_;
  std::vector<double> costs_;
};

}  // namespace moon_planner
