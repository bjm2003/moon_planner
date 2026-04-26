#include "moon_planner/search/open_list.hpp"

namespace moon_planner {

void OpenList::Push(double f, int node_index) {
  queue_.push(OpenListEntry{f, node_index});
}

bool OpenList::Empty() const {
  return queue_.empty();
}

int OpenList::Pop() {
  const int node_index = queue_.top().node_index;
  queue_.pop();
  return node_index;
}

void OpenList::Clear() {
  while (!queue_.empty()) {
    queue_.pop();
  }
}

}  // namespace moon_planner
