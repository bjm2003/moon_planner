#pragma once

#include "moon_planner/config/cost_config.hpp"
#include "moon_planner/config/planner_config.hpp"
#include "moon_planner/config/vehicle_config.hpp"
#include "moon_planner/primitives/motion_primitive.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace moon_planner {

class YamlLoader {
 public:
  bool Exists(const std::string& path) const;
  bool Load(const std::string& path);

  bool HasKey(const std::string& key) const;
  double GetDouble(const std::string& key, double default_value) const;
  int GetInt(const std::string& key, int default_value) const;
  bool GetBool(const std::string& key, bool default_value) const;
  std::vector<double> GetDoubleVector(const std::string& key, const std::vector<double>& default_value) const;

 private:
  std::unordered_map<std::string, std::string> values_;
};

PlannerConfig LoadPlannerConfig(const std::string& path, PlannerConfig defaults = {});
VehicleConfig LoadVehicleConfig(const std::string& path, VehicleConfig defaults = {});
CostConfig LoadCostConfig(const std::string& path, CostConfig defaults = {});
PrimitiveGenerationConfig LoadPrimitiveGenerationConfig(const std::string& path,
                                                        PrimitiveGenerationConfig defaults = {});

}  // namespace moon_planner
