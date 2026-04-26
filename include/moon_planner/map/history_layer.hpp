#pragma once

#include "moon_planner/map/cost_map.hpp"

namespace moon_planner {

class HistoryLayer {
 public:
  explicit HistoryLayer(GridIndex index = {});

  void MarkVisited(double x_m, double y_m);
  void AddPenaltyTo(CostMap* cost_map, double visited_penalty) const;

 private:
  GridIndex index_;
  OccupancyGrid visited_;
};

}  // namespace moon_planner
