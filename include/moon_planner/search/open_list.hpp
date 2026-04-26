#pragma once

#include <queue>
#include <vector>

namespace moon_planner {

struct OpenListEntry {
  double f{0.0};
  int node_index{-1};

  bool operator<(const OpenListEntry& other) const {
    return f > other.f;
  }
};

class OpenList {
 public:
  void Push(double f, int node_index);
  bool Empty() const;
  int Pop();
  void Clear();

 private:
  std::priority_queue<OpenListEntry> queue_;
};

}  // namespace moon_planner
