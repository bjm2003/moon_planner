#include "moon_planner/io/yaml_loader.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

namespace moon_planner {

namespace {

std::string Trim(const std::string& text) {
  auto begin = text.begin();
  while (begin != text.end() && std::isspace(static_cast<unsigned char>(*begin))) {
    ++begin;
  }
  auto end = text.end();
  while (end != begin && std::isspace(static_cast<unsigned char>(*(end - 1)))) {
    --end;
  }
  return std::string(begin, end);
}

std::string StripComment(const std::string& line) {
  const std::size_t comment = line.find('#');
  return comment == std::string::npos ? line : line.substr(0, comment);
}

std::vector<double> ParseDoubleVector(const std::string& value) {
  std::string text = Trim(value);
  if (text.size() < 2 || text.front() != '[' || text.back() != ']') {
    return {};
  }
  text = text.substr(1, text.size() - 2);
  std::vector<double> values;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ',')) {
    item = Trim(item);
    if (!item.empty()) {
      values.push_back(std::stod(item));
    }
  }
  return values;
}

}  // namespace

bool YamlLoader::Exists(const std::string& path) const {
  std::ifstream input(path);
  return input.good();
}

bool YamlLoader::Load(const std::string& path) {
  std::ifstream input(path);
  if (!input.good()) {
    return false;
  }
  values_.clear();
  std::string line;
  while (std::getline(input, line)) {
    line = Trim(StripComment(line));
    if (line.empty()) {
      continue;
    }
    const std::size_t separator = line.find(':');
    if (separator == std::string::npos) {
      continue;
    }
    const std::string key = Trim(line.substr(0, separator));
    const std::string value = Trim(line.substr(separator + 1));
    if (!key.empty() && !value.empty()) {
      values_[key] = value;
    }
  }
  return true;
}

bool YamlLoader::HasKey(const std::string& key) const {
  return values_.find(key) != values_.end();
}

double YamlLoader::GetDouble(const std::string& key, double default_value) const {
  const auto found = values_.find(key);
  return found == values_.end() ? default_value : std::stod(found->second);
}

int YamlLoader::GetInt(const std::string& key, int default_value) const {
  const auto found = values_.find(key);
  return found == values_.end() ? default_value : std::stoi(found->second);
}

bool YamlLoader::GetBool(const std::string& key, bool default_value) const {
  const auto found = values_.find(key);
  if (found == values_.end()) {
    return default_value;
  }
  std::string value = found->second;
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (value == "true" || value == "1" || value == "yes") {
    return true;
  }
  if (value == "false" || value == "0" || value == "no") {
    return false;
  }
  return default_value;
}

std::vector<double> YamlLoader::GetDoubleVector(const std::string& key,
                                                const std::vector<double>& default_value) const {
  const auto found = values_.find(key);
  if (found == values_.end()) {
    return default_value;
  }
  const std::vector<double> values = ParseDoubleVector(found->second);
  return values.empty() ? default_value : values;
}

PlannerConfig LoadPlannerConfig(const std::string& path, PlannerConfig defaults) {
  YamlLoader loader;
  if (!loader.Load(path)) {
    return defaults;
  }
  defaults.grid_resolution_m = loader.GetDouble("grid_resolution_m", defaults.grid_resolution_m);
  defaults.heading_bins = loader.GetInt("heading_bins", defaults.heading_bins);
  defaults.planning_horizon_m = loader.GetDouble("planning_horizon_m", defaults.planning_horizon_m);
  defaults.max_planning_time_ms = loader.GetDouble("max_planning_time_ms", defaults.max_planning_time_ms);
  defaults.max_expanded_nodes = loader.GetInt("max_expanded_nodes", defaults.max_expanded_nodes);
  defaults.allow_reverse = loader.GetBool("allow_reverse", defaults.allow_reverse);
  defaults.goal_tolerance_xy_m = loader.GetDouble("goal_tolerance_xy_m", defaults.goal_tolerance_xy_m);
  defaults.goal_tolerance_yaw_rad = loader.GetDouble("goal_tolerance_yaw_rad", defaults.goal_tolerance_yaw_rad);
  return defaults;
}

VehicleConfig LoadVehicleConfig(const std::string& path, VehicleConfig defaults) {
  YamlLoader loader;
  if (!loader.Load(path)) {
    return defaults;
  }
  defaults.length_m = loader.GetDouble("length_m", defaults.length_m);
  defaults.width_m = loader.GetDouble("width_m", defaults.width_m);
  defaults.height_m = loader.GetDouble("height_m", defaults.height_m);
  defaults.wheel_base_m = loader.GetDouble("wheel_base_m", defaults.wheel_base_m);
  defaults.track_width_m = loader.GetDouble("track_width_m", defaults.track_width_m);
  defaults.ground_clearance_m = loader.GetDouble("ground_clearance_m", defaults.ground_clearance_m);
  defaults.safety_margin_m = loader.GetDouble("safety_margin_m", defaults.safety_margin_m);
  defaults.max_linear_velocity_mps = loader.GetDouble("max_linear_velocity_mps", defaults.max_linear_velocity_mps);
  defaults.max_angular_velocity_radps =
      loader.GetDouble("max_angular_velocity_radps", defaults.max_angular_velocity_radps);
  defaults.max_acceleration_mps2 = loader.GetDouble("max_acceleration_mps2", defaults.max_acceleration_mps2);
  defaults.max_curvature_1pm = loader.GetDouble("max_curvature_1pm", defaults.max_curvature_1pm);
  defaults.max_slope_rad = loader.GetDouble("max_slope_rad", defaults.max_slope_rad);
  return defaults;
}

CostConfig LoadCostConfig(const std::string& path, CostConfig defaults) {
  YamlLoader loader;
  if (!loader.Load(path)) {
    return defaults;
  }
  defaults.length = loader.GetDouble("length", defaults.length);
  defaults.turn = loader.GetDouble("turn", defaults.turn);
  defaults.slope = loader.GetDouble("slope", defaults.slope);
  defaults.roughness = loader.GetDouble("roughness", defaults.roughness);
  defaults.safety = loader.GetDouble("safety", defaults.safety);
  defaults.reverse = loader.GetDouble("reverse", defaults.reverse);
  defaults.history = loader.GetDouble("history", defaults.history);
  return defaults;
}

PrimitiveGenerationConfig LoadPrimitiveGenerationConfig(const std::string& path,
                                                        PrimitiveGenerationConfig defaults) {
  YamlLoader loader;
  if (!loader.Load(path)) {
    return defaults;
  }
  defaults.duration_s = loader.GetDouble("duration_s", defaults.duration_s);
  defaults.integration_step_s = loader.GetDouble("integration_step_s", defaults.integration_step_s);
  defaults.linear_velocities_mps = loader.GetDoubleVector("linear_velocities_mps", defaults.linear_velocities_mps);
  defaults.angular_velocities_radps =
      loader.GetDoubleVector("angular_velocities_radps", defaults.angular_velocities_radps);
  return defaults;
}

}  // namespace moon_planner
