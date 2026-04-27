#include "moon_planner/map/history_layer.hpp"

#include <algorithm>

namespace moon_planner {

namespace {

constexpr double kVisitedWeight = 1.0;
constexpr double kHistoricalObstacleWeight = 3.0;
constexpr double kFailedRegionWeight = 5.0;

}  // namespace

HistoryLayer::HistoryLayer(GridIndex index) : index_(index), weights_(index.CellCount(), 0.0) {}

bool HistoryLayer::IsValid() const {
  return index_.IsValid() && weights_.size() == index_.CellCount();
}

void HistoryLayer::MarkVisited(double x_m, double y_m) {
  AddWeightWorld(x_m, y_m, kVisitedWeight);
}

void HistoryLayer::MarkHistoricalObstacle(double x_m, double y_m) {
  AddWeightWorld(x_m, y_m, kHistoricalObstacleWeight);
}

void HistoryLayer::MarkFailedRegion(double x_m, double y_m) {
  AddWeightWorld(x_m, y_m, kFailedRegionWeight);
}

void HistoryLayer::Decay(double factor) {
  if (!IsValid()) {
    return;
  }
  const double clamped = std::max(0.0, std::min(1.0, factor));
  for (double& weight : weights_) {
    weight *= clamped;
  }
}

double HistoryLayer::Weight(int x, int y) const {
  if (!IsValid() || !index_.IsInside(x, y)) {
    return 0.0;
  }
  return weights_[index_.FlatIndex(x, y)];
}

double HistoryLayer::WeightWorld(double x_m, double y_m) const {
  const auto cell = index_.WorldToCell(x_m, y_m);
  return cell ? Weight(cell->x, cell->y) : 0.0;
}

void HistoryLayer::AddPenaltyTo(CostMap* cost_map, double history_penalty) const {
  if (cost_map == nullptr || !cost_map->IsValid() || !IsValid() || history_penalty <= 0.0 ||
      cost_map->index().CellCount() != index_.CellCount()) {
    return;
  }
  for (int y = 0; y < index_.height(); ++y) {
    for (int x = 0; x < index_.width(); ++x) {
      const double weight = Weight(x, y);
      if (weight > 0.0 && !cost_map->IsLethal(x, y)) {
        cost_map->SetCost(x, y, cost_map->Cost(x, y) + history_penalty * weight);
      }
    }
  }
}

bool HistoryLayer::AddWeightWorld(double x_m, double y_m, double weight) {
  const auto cell = index_.WorldToCell(x_m, y_m);
  return cell && AddWeightCell(cell->x, cell->y, weight);
}

bool HistoryLayer::AddWeightCell(int x, int y, double weight) {
  if (!IsValid() || !index_.IsInside(x, y) || weight <= 0.0) {
    return false;
  }
  weights_[index_.FlatIndex(x, y)] += weight;
  return true;
}

}  // namespace moon_planner
