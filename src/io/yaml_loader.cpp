#include "moon_planner/io/yaml_loader.hpp"

#include <fstream>

namespace moon_planner {

bool YamlLoader::Exists(const std::string& path) const {
  std::ifstream input(path);
  return input.good();
}

}  // namespace moon_planner
