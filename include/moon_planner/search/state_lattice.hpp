#pragma once

#include "moon_planner/config/planner_config.hpp"
#include "moon_planner/core/types.hpp"
#include "moon_planner/map/grid_index.hpp"

namespace moon_planner {

class StateLattice {
 public:
  StateLattice(GridIndex grid_index, PlannerConfig config);

  int HeadingToBin(double yaw_rad) const;
  double BinToHeading(int heading_bin) const;
  State MakeState(int x, int y, int heading_bin) const;

 private:
  GridIndex grid_index_;
  PlannerConfig config_;
};

}  // namespace moon_planner
