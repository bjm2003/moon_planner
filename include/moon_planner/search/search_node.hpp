#pragma once

#include <cstddef>

namespace moon_planner {

struct SearchNode {
  int x{0};
  int y{0};
  int heading{0};
  double g{0.0};
  double h{0.0};
  int parent{-1};
  int primitive_id{-1};
  bool closed{false};

  double f() const { return g + h; }
};

struct NodeKey {
  int x{0};
  int y{0};
  int heading{0};

  bool operator==(const NodeKey& other) const {
    return x == other.x && y == other.y && heading == other.heading;
  }
};

struct NodeKeyHasher {
  std::size_t operator()(const NodeKey& key) const {
    std::size_t value = static_cast<std::size_t>(key.x);
    value = value * 73856093u ^ static_cast<std::size_t>(key.y) * 19349663u;
    value = value ^ static_cast<std::size_t>(key.heading) * 83492791u;
    return value;
  }
};

}  // namespace moon_planner
