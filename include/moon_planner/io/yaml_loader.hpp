#pragma once

#include <string>

namespace moon_planner {

class YamlLoader {
 public:
  bool Exists(const std::string& path) const;
};

}  // namespace moon_planner
