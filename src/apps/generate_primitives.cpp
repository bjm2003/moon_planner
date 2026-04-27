#include "moon_planner/config/planner_config.hpp"
#include "moon_planner/config/vehicle_config.hpp"
#include "moon_planner/io/primitive_io.hpp"
#include "moon_planner/io/yaml_loader.hpp"
#include "moon_planner/model/motion_constraints.hpp"
#include "moon_planner/primitives/primitive_generator.hpp"

#include <iostream>

int main(int argc, char** argv) {
  using namespace moon_planner;

  PlannerConfig planner_config = LoadPlannerConfig("config/planner_default.yaml");
  VehicleConfig vehicle_config = LoadVehicleConfig("config/vehicle_default.yaml");
  PrimitiveGenerationConfig primitive_config = LoadPrimitiveGenerationConfig("config/primitive_default.yaml");
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
