#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "glog/logging.h"
#include "src/config/planning_flags.h"
#include "src/data_struct/data_struct.h"
#include "src/data_struct/reference_path.h"
#include "src/data_struct/vehicle_state_frenet.h"
#include "src/util/collision_checker.h"
#include "src/util/map.h"

namespace PathOptimizationNS {

class PathOptimizer {
 public:
  PathOptimizer(const State &start_state, const State &end_state,
                const Map &map);
  ~PathOptimizer();

  // Call this to get the optimized path.
  bool solve(const std::vector<State> &reference_points,
             std::vector<State> *final_path);
  bool solveWithoutSmoothing(const std::vector<State> &reference_points,
                             std::vector<State> *final_path);

  // Only for visualization purpose.
  const std::vector<State> &getSmoothedPath() const;
  const std::vector<std::vector<double>> &getSearchResult() const;
  std::vector<std::tuple<State, double, double>> display_abnormal_bounds()
      const;

 private:
  // Core function.
  bool optimizePath(std::vector<State> *final_path);

  // Divide smoothed path into segments.
  bool segmentSmoothedPath();

  const Map grid_map_;
  CollisionChecker collision_checker_;
  ReferencePath reference_path_;
  VehicleState vehicle_state_;
  size_t size_{};

  // For visualization purpose.
  std::vector<State> smoothed_path_;
  std::vector<std::vector<double>> reference_searching_display_;
};
}  // namespace PathOptimizationNS