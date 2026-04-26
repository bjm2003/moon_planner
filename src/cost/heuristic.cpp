#include "moon_planner/cost/heuristic.hpp"

#include "moon_planner/core/geometry.hpp"

namespace moon_planner {

double Heuristic::Estimate(const State& state, const State& goal) const {
  return Distance2D(state.x, state.y, goal.x, goal.y);
}

}  // namespace moon_planner
