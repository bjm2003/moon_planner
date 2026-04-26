#include "moon_planner/config/planner_config.hpp"
#include "moon_planner/config/vehicle_config.hpp"
#include "moon_planner/io/primitive_io.hpp"
#include "moon_planner/model/motion_constraints.hpp"
#include "moon_planner/primitives/primitive_generator.hpp"

#include <iostream>

int main(int argc, char** argv) {
  using namespace moon_planner;

  PlannerConfig planner_config;
  VehicleConfig vehicle_config;
  PrimitiveGenerationConfig primitive_config;
  PrimitiveGenerator generator;
  PrimitiveLibrary library = generator.Generate(planner_config, primitive_config, MotionConstraints(vehicle_config));

  std::cout << "heading_bins=" << library.heading_bins() << '\n';
  std::cout << "primitive_count=" << library.Size() << '\n';

  if (argc > 1) {
    PrimitiveIO io;
    if (!io.WriteSummary(library, argv[1])) {
      std::cerr << "failed to write primitive summary: " << argv[1] << '\n';
      return 2;
    }
  }
  return library.Size() > 0 ? 0 : 1;
}
