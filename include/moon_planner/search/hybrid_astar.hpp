#pragma once

#include "moon_planner/collision/collision_checker.hpp"
#include "moon_planner/config/cost_config.hpp"
#include "moon_planner/config/planner_config.hpp"
#include "moon_planner/cost/heuristic.hpp"
#include "moon_planner/cost/primitive_cost.hpp"
#include "moon_planner/map/cost_map.hpp"
#include "moon_planner/primitives/primitive_library.hpp"
#include "moon_planner/search/search_node.hpp"
#include "moon_planner/search/state_lattice.hpp"

#include <unordered_map>
#include <vector>

namespace moon_planner {

class HybridAStar {
 public:
  HybridAStar(PlannerConfig planner_config, CostConfig cost_config, CollisionChecker collision_checker);

  PlanningResult Plan(const PlanningRequest& request,
                      const OccupancyGrid& occupancy,
                      const CostMap& cost_map,
                      const PrimitiveLibrary& primitive_library) const;

 private:
  std::vector<State> ReconstructPath(const std::vector<SearchNode>& nodes, int goal_index, const StateLattice& lattice) const;

  PlannerConfig planner_config_;
  CostConfig cost_config_;
  CollisionChecker collision_checker_;
};

}  // namespace moon_planner
