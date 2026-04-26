#include "moon_planner/primitives/primitive_library.hpp"

#include <cstdlib>

namespace moon_planner {

PrimitiveLibrary::PrimitiveLibrary(int heading_bins) {
  Reset(heading_bins);
}

void PrimitiveLibrary::Reset(int heading_bins) {
  heading_bins_ = heading_bins;
  primitives_by_heading_.clear();
  if (heading_bins_ > 0) {
    primitives_by_heading_.resize(static_cast<std::size_t>(heading_bins_));
  }
}

void PrimitiveLibrary::Add(MotionPrimitive primitive) {
  if (!IsValid()) {
    return;
  }
  const int heading = NormalizeHeadingBin(primitive.start_heading_bin);
  primitive.start_heading_bin = heading;
  primitives_by_heading_[static_cast<std::size_t>(heading)].push_back(std::move(primitive));
}

const std::vector<MotionPrimitive>& PrimitiveLibrary::Query(int heading_bin) const {
  static const std::vector<MotionPrimitive> kEmpty;
  if (!IsValid()) {
    return kEmpty;
  }
  return primitives_by_heading_[static_cast<std::size_t>(NormalizeHeadingBin(heading_bin))];
}

std::size_t PrimitiveLibrary::Size() const {
  std::size_t total = 0;
  for (const auto& group : primitives_by_heading_) {
    total += group.size();
  }
  return total;
}

int PrimitiveLibrary::NormalizeHeadingBin(int heading_bin) const {
  if (heading_bins_ <= 0) {
    return 0;
  }
  int result = heading_bin % heading_bins_;
  if (result < 0) {
    result += heading_bins_;
  }
  return result;
}

}  // namespace moon_planner
