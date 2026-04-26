#include "moon_planner/search/hybrid_astar.hpp"

#include "moon_planner/core/geometry.hpp"
#include "moon_planner/core/time.hpp"
#include "moon_planner/search/open_list.hpp"
#include "moon_planner/search/state_lattice.hpp"

#include <algorithm>
#include <limits>

namespace moon_planner {

HybridAStar::HybridAStar(PlannerConfig planner_config, CostConfig cost_config, CollisionChecker collision_checker)
    : planner_config_(planner_config), cost_config_(cost_config), collision_checker_(collision_checker) {}

PlanningResult HybridAStar::Plan(const PlanningRequest& request,
                                 const OccupancyGrid& occupancy,
                                 const CostMap& cost_map,
                                 const PrimitiveLibrary& primitive_library) const {
  PlanningResult result;
  Stopwatch stopwatch;
  if (!planner_config_.IsValid() || !occupancy.IsValid() || !cost_map.IsValid() || !primitive_library.IsValid()) {
    result.status = PlannerStatus::kInvalidInput;
    result.diagnostics.message = "invalid planner input";
    return result;
  }

  StateLattice lattice(occupancy.index(), planner_config_);
  const auto start_cell = occupancy.index().WorldToCell(request.start.x, request.start.y);
  const auto goal_cell = occupancy.index().WorldToCell(request.goal.x, request.goal.y);
  if (!start_cell || !goal_cell) {
    result.status = PlannerStatus::kInvalidInput;
    result.diagnostics.message = "start or goal outside map";
    return result;
  }
  if (!collision_checker_.IsStateCollisionFree(request.start, occupancy)) {
    result.status = PlannerStatus::kStartInCollision;
    result.diagnostics.message = "start is in collision";
    return result;
  }
  if (!collision_checker_.IsStateCollisionFree(request.goal, occupancy)) {
    result.status = PlannerStatus::kGoalInCollision;
    result.diagnostics.message = "goal is in collision";
    return result;
  }

  const int start_heading = lattice.HeadingToBin(request.start.yaw);
  const NodeKey start_key{start_cell->x, start_cell->y, start_heading};

  std::vector<SearchNode> nodes;
  nodes.reserve(4096);
  std::unordered_map<NodeKey, int, NodeKeyHasher> node_lookup;
  OpenList open_list;
  Heuristic heuristic;
  PrimitiveCost primitive_cost(cost_config_);

  SearchNode start_node;
  start_node.x = start_key.x;
  start_node.y = start_key.y;
  start_node.heading = start_key.heading;
  start_node.g = 0.0;
  start_node.h = heuristic.Estimate(request.start, request.goal);
  nodes.push_back(start_node);
  node_lookup[start_key] = 0;
  open_list.Push(start_node.f(), 0);

  int best_node = 0;
  double best_h = start_node.h;

  while (!open_list.Empty()) {
    if (stopwatch.ElapsedMilliseconds() > planner_config_.max_planning_time_ms) {
      result.status = PlannerStatus::kTimeout;
      result.diagnostics.message = "planner timeout";
      break;
    }
    if (static_cast<int>(result.diagnostics.expanded_nodes) >= planner_config_.max_expanded_nodes) {
      result.status = PlannerStatus::kTimeout;
      result.diagnostics.message = "expanded node limit reached";
      break;
    }

    const int current_index = open_list.Pop();
    SearchNode& current = nodes[static_cast<std::size_t>(current_index)];
    if (current.closed) {
      continue;
    }
    current.closed = true;
    ++result.diagnostics.expanded_nodes;

    const State current_state = lattice.MakeState(current.x, current.y, current.heading);
    current.h = heuristic.Estimate(current_state, request.goal);
    if (current.h < best_h) {
      best_h = current.h;
      best_node = current_index;
    }
    if (current.h <= request.goal_tolerance_xy_m) {
      best_node = current_index;
      result.status = PlannerStatus::kSuccess;
      result.diagnostics.message = "path found";
      break;
    }

    for (const MotionPrimitive& primitive : primitive_library.Query(current.heading)) {
      if (!request.allow_reverse && primitive.v_mps < 0.0) {
        continue;
      }
      const int nx = current.x + primitive.end_dx_cells;
      const int ny = current.y + primitive.end_dy_cells;
      const int nh = primitive.end_heading_bin;
      if (!occupancy.index().IsInside(nx, ny) || cost_map.IsLethal(nx, ny)) {
        continue;
      }
      if (!collision_checker_.IsPrimitiveCollisionFree(current_state, primitive, occupancy)) {
        continue;
      }

      const State next_state = lattice.MakeState(nx, ny, nh);
      const double edge_cost = primitive_cost.Evaluate(current_state, primitive, cost_map);
      if (edge_cost >= CostMap::kLethalCost) {
        continue;
      }
      const double tentative_g = current.g + edge_cost;
      const NodeKey next_key{nx, ny, nh};
      auto found = node_lookup.find(next_key);
      if (found == node_lookup.end()) {
        SearchNode next_node;
        next_node.x = nx;
        next_node.y = ny;
        next_node.heading = nh;
        next_node.g = tentative_g;
        next_node.h = heuristic.Estimate(next_state, request.goal);
        next_node.parent = current_index;
        next_node.primitive_id = primitive.id;
        const int next_index = static_cast<int>(nodes.size());
        nodes.push_back(next_node);
        node_lookup[next_key] = next_index;
        open_list.Push(next_node.f(), next_index);
      } else {
        SearchNode& existing = nodes[static_cast<std::size_t>(found->second)];
        if (tentative_g < existing.g) {
          existing.g = tentative_g;
          existing.parent = current_index;
          existing.primitive_id = primitive.id;
          open_list.Push(existing.f(), found->second);
        }
      }
    }
  }

  if (result.status != PlannerStatus::kSuccess && result.status != PlannerStatus::kTimeout) {
    result.status = PlannerStatus::kNoPath;
    result.diagnostics.message = "no path";
  }

  result.states = ReconstructPath(nodes, best_node, lattice);
  if (!result.states.empty()) {
    result.diagnostics.total_cost = nodes[static_cast<std::size_t>(best_node)].g;
    for (std::size_t i = 1; i < result.states.size(); ++i) {
      result.diagnostics.path_length_m += Distance2D(result.states[i - 1].x, result.states[i - 1].y,
                                                     result.states[i].x, result.states[i].y);
    }
  }
  result.diagnostics.planning_time_ms = stopwatch.ElapsedMilliseconds();
  if (result.status != PlannerStatus::kSuccess && result.states.size() > 1) {
    result.status = PlannerStatus::kTimeout;
  }
  return result;
}

std::vector<State> HybridAStar::ReconstructPath(const std::vector<SearchNode>& nodes,
                                                int goal_index,
                                                const StateLattice& lattice) const {
  std::vector<State> path;
  if (goal_index < 0 || goal_index >= static_cast<int>(nodes.size())) {
    return path;
  }
  for (int index = goal_index; index >= 0; index = nodes[static_cast<std::size_t>(index)].parent) {
    const SearchNode& node = nodes[static_cast<std::size_t>(index)];
    path.push_back(lattice.MakeState(node.x, node.y, node.heading));
    if (node.parent == index) {
      break;
    }
  }
  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace moon_planner
