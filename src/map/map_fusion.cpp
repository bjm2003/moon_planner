#include "moon_planner/map/map_fusion.hpp"

namespace moon_planner {

CostMap MapFusion::BuildCostMap(const OccupancyGrid& occupancy,
                                 const ElevationGrid* elevation,
                                 double obstacle_influence_radius_m,
                                 double obstacle_max_cost,
                                 double max_slope_rad,
                                 double slope_max_cost,
                                 const HistoryLayer* history,
                                 double history_penalty) const {
  CostMap cost_map(occupancy.index(), 0.0);
  cost_map.ApplyOccupancy(occupancy);
  cost_map.ApplyObstacleDistanceCost(occupancy, obstacle_influence_radius_m, obstacle_max_cost);
  if (elevation != nullptr && elevation->IsValid()) {
    cost_map.ApplySlopeCost(*elevation, max_slope_rad, slope_max_cost);
  }
  if (history != nullptr && history->IsValid()) {
    history->AddPenaltyTo(&cost_map, history_penalty);
  }
  return cost_map;
}

}  // namespace moon_planner
