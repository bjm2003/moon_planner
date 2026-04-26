#include "moon_planner/model/skid_steer_model.hpp"

#include "moon_planner/core/geometry.hpp"

#include <cmath>

namespace moon_planner {

SkidSteerModel::SkidSteerModel(double slip_ratio) : slip_ratio_(slip_ratio) {}

State SkidSteerModel::Integrate(const State& initial, double v_mps, double omega_radps, double dt_s, double slope_rad) const {
  const double effective_v = v_mps * (1.0 - slip_ratio_);
  const double mid_yaw = initial.yaw + 0.5 * omega_radps * dt_s;

  State next = initial;
  next.x += effective_v * std::cos(mid_yaw) * std::cos(slope_rad) * dt_s;
  next.y += effective_v * std::sin(mid_yaw) * std::cos(slope_rad) * dt_s;
  next.z += effective_v * std::sin(slope_rad) * dt_s;
  next.yaw = NormalizeAngle(initial.yaw + omega_radps * dt_s);
  next.pitch = slope_rad;
  next.v = v_mps;
  next.omega = omega_radps;
  next.timestamp_s += dt_s;
  return next;
}

std::vector<State> SkidSteerModel::Rollout(const State& initial,
                                           double v_mps,
                                           double omega_radps,
                                           double duration_s,
                                           double step_s,
                                           double slope_rad) const {
  std::vector<State> states;
  if (duration_s <= 0.0 || step_s <= 0.0) {
    return states;
  }

  State current = initial;
  states.push_back(current);
  double elapsed = 0.0;
  while (elapsed + 1e-9 < duration_s) {
    const double dt = std::min(step_s, duration_s - elapsed);
    current = Integrate(current, v_mps, omega_radps, dt, slope_rad);
    elapsed += dt;
    states.push_back(current);
  }
  return states;
}

}  // namespace moon_planner
