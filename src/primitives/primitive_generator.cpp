#include "moon_planner/primitives/primitive_generator.hpp"

#include "moon_planner/core/geometry.hpp"

#include <cmath>

namespace moon_planner {

PrimitiveLibrary PrimitiveGenerator::Generate(const PlannerConfig& planner_config,
                                              const PrimitiveGenerationConfig& primitive_config,
                                              const MotionConstraints& constraints) const {
  PrimitiveLibrary library(planner_config.heading_bins);
  SkidSteerModel model;
  int id = 0;
  for (int heading = 0; heading < planner_config.heading_bins; ++heading) {
    const double yaw = static_cast<double>(heading) * 2.0 * kPi / static_cast<double>(planner_config.heading_bins);
    for (const double v : primitive_config.linear_velocities_mps) {
      for (const double omega : primitive_config.angular_velocities_radps) {
        if (!constraints.IsControlValid(v, omega)) {
          continue;
        }
        State initial;
        initial.yaw = yaw;
        auto rollout = model.Rollout(initial, v, omega, primitive_config.duration_s, primitive_config.integration_step_s);
        if (rollout.size() < 2) {
          continue;
        }

        const State& end = rollout.back();
        MotionPrimitive primitive;
        primitive.id = id++;
        primitive.start_heading_bin = heading;
        primitive.end_heading_bin = static_cast<int>(
            std::lround(NormalizeAngle(end.yaw) / (2.0 * kPi) * planner_config.heading_bins));
        primitive.end_heading_bin = (primitive.end_heading_bin % planner_config.heading_bins + planner_config.heading_bins) %
                                    planner_config.heading_bins;
        primitive.end_dx_cells = static_cast<int>(std::lround(end.x / planner_config.grid_resolution_m));
        primitive.end_dy_cells = static_cast<int>(std::lround(end.y / planner_config.grid_resolution_m));
        primitive.v_mps = v;
        primitive.omega_radps = omega;
        primitive.duration_s = primitive_config.duration_s;
        primitive.length_m = std::abs(v) * primitive_config.duration_s;
        primitive.base_cost = primitive.length_m + 0.25 * std::abs(omega) * primitive_config.duration_s;
        primitive.relative_states = std::move(rollout);
        library.Add(std::move(primitive));
      }
    }
  }
  return library;
}

}  // namespace moon_planner
