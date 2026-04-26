#include "moon_planner/io/scenario_reader.hpp"

#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace moon_planner {

namespace {

std::string ReadAll(const std::string& path) {
  std::ifstream input(path);
  if (!input.good()) {
    throw std::runtime_error("failed to open scenario: " + path);
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

double ReadDouble(const std::string& text, const std::string& key, double default_value) {
  const std::regex pattern(key + R"(\s*:\s*(-?[0-9]+(?:\.[0-9]+)?))");
  std::smatch match;
  if (std::regex_search(text, match, pattern)) {
    return std::stod(match[1].str());
  }
  return default_value;
}

int ReadInt(const std::string& text, const std::string& key, int default_value) {
  return static_cast<int>(ReadDouble(text, key, static_cast<double>(default_value)));
}

std::string ReadString(const std::string& text, const std::string& key, const std::string& default_value) {
  const std::regex pattern(key + R"(\s*:\s*([A-Za-z0-9_\-]+))");
  std::smatch match;
  if (std::regex_search(text, match, pattern)) {
    return match[1].str();
  }
  return default_value;
}

double ReadInlineDouble(const std::string& text, const std::string& block, const std::string& key, double default_value) {
  const std::regex block_pattern(block + R"(\s*:\s*\{([^}]*)\})");
  std::smatch block_match;
  if (!std::regex_search(text, block_match, block_pattern)) {
    return default_value;
  }
  return ReadDouble(block_match[1].str(), key, default_value);
}

void ApplyObstacleCells(const std::string& text, OccupancyGrid* occupancy) {
  const std::regex obstacle_pattern(R"(\{\s*x0\s*:\s*([0-9]+)\s*,\s*y0\s*:\s*([0-9]+)\s*,\s*x1\s*:\s*([0-9]+)\s*,\s*y1\s*:\s*([0-9]+)\s*\})");
  auto begin = std::sregex_iterator(text.begin(), text.end(), obstacle_pattern);
  auto end = std::sregex_iterator();
  for (auto it = begin; it != end; ++it) {
    const int x0 = std::stoi((*it)[1].str());
    const int y0 = std::stoi((*it)[2].str());
    const int x1 = std::stoi((*it)[3].str());
    const int y1 = std::stoi((*it)[4].str());
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        occupancy->SetCell(x, y, OccupancyGrid::kOccupied);
      }
    }
  }
}

}  // namespace

PlanningScenario ScenarioReader::Read(const std::string& path) const {
  const std::string text = ReadAll(path);

  PlanningScenario scenario;
  scenario.name = ReadString(text, "name", "scenario");
  const int width = ReadInt(text, "width", 160);
  const int height = ReadInt(text, "height", 80);
  const double resolution_m = ReadDouble(text, "resolution_m", 0.1);
  scenario.occupancy = OccupancyGrid(GridIndex(width, height, resolution_m), OccupancyGrid::kFree);
  ApplyObstacleCells(text, &scenario.occupancy);

  scenario.request.start.x = ReadInlineDouble(text, "start", "x", 2.0);
  scenario.request.start.y = ReadInlineDouble(text, "start", "y", 4.0);
  scenario.request.start.yaw = ReadInlineDouble(text, "start", "yaw", 0.0);
  scenario.request.goal.x = ReadInlineDouble(text, "goal", "x", 13.0);
  scenario.request.goal.y = ReadInlineDouble(text, "goal", "y", 4.0);
  scenario.request.goal.yaw = ReadInlineDouble(text, "goal", "yaw", 0.0);
  scenario.request.allow_reverse = true;
  return scenario;
}

PlanningScenario ScenarioReader::DefaultScenario() const {
  PlanningScenario scenario;
  scenario.name = "default";
  scenario.request.start.x = 2.0;
  scenario.request.start.y = 4.0;
  scenario.request.start.yaw = 0.0;
  scenario.request.goal.x = 13.0;
  scenario.request.goal.y = 4.0;
  scenario.request.goal.yaw = 0.0;
  scenario.occupancy = OccupancyGrid(GridIndex(160, 80, 0.1), OccupancyGrid::kFree);
  for (int y = 70; y < 75; ++y) {
    scenario.occupancy.SetCell(70, y, OccupancyGrid::kOccupied);
  }
  return scenario;
}

PlanningRequest ScenarioReader::DefaultRequest() const {
  return DefaultScenario().request;
}

}  // namespace moon_planner
