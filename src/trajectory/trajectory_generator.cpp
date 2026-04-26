#include "moon_planner/trajectory/trajectory_generator.hpp"

#include "moon_planner/core/geometry.hpp"

#include <algorithm>

namespace moon_planner {

std::vector<TrajectoryPoint> TrajectoryGenerator::Generate(const std::vector<State>& states, double nominal_speed_mps) const {
  std::vector<TrajectoryPoint> trajectory;
  trajectory.reserve(states.size());
  double t = 0.0;
  for (std::size_t i = 0; i < states.size(); ++i) {
    State state = states[i];
    if (i > 0) {
      const double ds = Distance2D(states[i - 1].x, states[i - 1].y, state.x, state.y);
      t += ds / std::max(0.01, nominal_speed_mps);
      state.v = nominal_speed_mps;
    }
    trajectory.push_back(TrajectoryPoint{state, t});
  }
  return trajectory;
}

}  // namespace moon_planner
