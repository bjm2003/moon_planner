#include "moon_planner/cost/primitive_cost.hpp"

#include <cmath>

namespace moon_planner {

PrimitiveCost::PrimitiveCost(CostConfig config) : config_(config) {}

double PrimitiveCost::Evaluate(const State& origin, const MotionPrimitive& primitive, const CostMap& cost_map) const {
  const double c = std::cos(origin.yaw);
  const double s = std::sin(origin.yaw);
  double map_cost = 0.0;
  for (const State& relative : primitive.relative_states) {
    const double wx = origin.x + c * relative.x - s * relative.y;
    const double wy = origin.y + s * relative.x + c * relative.y;
    map_cost += cost_map.CostWorld(wx, wy);
  }
  const double reverse_penalty = primitive.v_mps < 0.0 ? config_.reverse : 1.0;
  return reverse_penalty * (config_.length * primitive.length_m +
                            config_.turn * std::abs(primitive.omega_radps) * primitive.duration_s +
                            config_.safety * map_cost / static_cast<double>(primitive.relative_states.size()));
}

}  // namespace moon_planner
