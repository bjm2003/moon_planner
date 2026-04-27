#pragma once

#include "moon_planner/map/cost_map.hpp"

#include <vector>

namespace moon_planner {

class HistoryLayer {
 public:
  explicit HistoryLayer(GridIndex index = {});

  const GridIndex& index() const { return index_; }
  bool IsValid() const;
  void MarkVisited(double x_m, double y_m);
  void MarkHistoricalObstacle(double x_m, double y_m);
  void MarkFailedRegion(double x_m, double y_m);
  void Decay(double factor);
  double Weight(int x, int y) const;
  double WeightWorld(double x_m, double y_m) const;
  void AddPenaltyTo(CostMap* cost_map, double history_penalty) const;

 private:
  bool AddWeightWorld(double x_m, double y_m, double weight);
  bool AddWeightCell(int x, int y, double weight);

  GridIndex index_;
  std::vector<double> weights_;
};

}  // namespace moon_planner
