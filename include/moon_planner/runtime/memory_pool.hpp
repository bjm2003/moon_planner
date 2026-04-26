#pragma once

#include <cstddef>

namespace moon_planner {

class MemoryPool {
 public:
  explicit MemoryPool(std::size_t capacity_bytes = 0);
  std::size_t capacity_bytes() const { return capacity_bytes_; }

 private:
  std::size_t capacity_bytes_{0};
};

}  // namespace moon_planner
