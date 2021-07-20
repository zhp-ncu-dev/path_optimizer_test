#pragma once

#include "eigen3/Eigen/Core"
#include "src/data_struct/data_struct.h"
namespace PathOptimizationNS {

class Map {
 public:
  Map() = default;
  Map(const std::vector<State>& original_path,
      const std::vector<State>& left_boundary,
      const std::vector<State>& right_boundary);
  ~Map() = default;

  const std::vector<State>& GetOriginalPath() const;

  const std::vector<State>& GetLeftBoundary() const;

  const std::vector<State>& GetRightBoundary() const;

  double GetObstacleDistance(const State& pos) const;

  bool IsInside(const State& pos, double* left_min_dis, int* left_index,
                double* right_min_dis, int* right_index) const;

 private:
  bool ClosedPointIndex(const std::vector<State>& boundary, const State& point,
                        int* index, double* min_dis) const;

 private:
  std::vector<State> original_path_{};
  std::vector<State> left_boundary_{};
  std::vector<State> right_boundary_{};
};
}  // namespace PathOptimizationNS