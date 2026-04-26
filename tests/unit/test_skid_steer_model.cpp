#include "moon_planner/model/skid_steer_model.hpp"

#include <cassert>
#include <cmath>

int main() {
  moon_planner::SkidSteerModel model;
  moon_planner::State start;
  const auto next = model.Integrate(start, 1.0, 0.0, 1.0);
  assert(std::abs(next.x - 1.0) < 1e-9);
  assert(std::abs(next.y) < 1e-9);
  assert(std::abs(next.yaw) < 1e-9);
  const auto rollout = model.Rollout(start, 1.0, 0.0, 1.0, 0.1);
  assert(rollout.size() == 11);
  return 0;
}
