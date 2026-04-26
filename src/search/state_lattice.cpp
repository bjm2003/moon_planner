#include "moon_planner/search/state_lattice.hpp"

#include "moon_planner/core/geometry.hpp"

#include <cmath>

namespace moon_planner {

StateLattice::StateLattice(GridIndex grid_index, PlannerConfig config) : grid_index_(grid_index), config_(config) {}

int StateLattice::HeadingToBin(double yaw_rad) const {
  const double normalized = NormalizeAngle(yaw_rad);
  const double shifted = normalized < 0.0 ? normalized + 2.0 * kPi : normalized;
  int bin = static_cast<int>(std::lround(shifted / (2.0 * kPi) * config_.heading_bins));
  bin %= config_.heading_bins;
  return bin;
}

double StateLattice::BinToHeading(int heading_bin) const {
  int bin = heading_bin % config_.heading_bins;
  if (bin < 0) {
    bin += config_.heading_bins;
  }
  return NormalizeAngle(static_cast<double>(bin) * 2.0 * kPi / static_cast<double>(config_.heading_bins));
}

State StateLattice::MakeState(int x, int y, int heading_bin) const {
  const Point2D center = grid_index_.CellCenter(x, y);
  State state;
  state.x = center.x;
  state.y = center.y;
  state.yaw = BinToHeading(heading_bin);
  return state;
}

}  // namespace moon_planner
