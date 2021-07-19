#pragma once

#include <ctime>
#include <iostream>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include "src/data_struct/data_struct.h"
#include "src/data_struct/reference_path.h"
#include "src/util/map.h"
#include "src/util/spline.h"

namespace PathOptimizationNS {

// This class uses searching method to improve the quality of the input points
// (if needed), and then uses a smoother to obtain a smoothed reference path.
class ReferencePathSmoother {
 public:
  ReferencePathSmoother() = delete;
  ReferencePathSmoother(const std::vector<State> &input_points,
                        const State &start_state, const Map &grid_map);
  virtual ~ReferencePathSmoother() = default;

  static std::unique_ptr<ReferencePathSmoother> create(
      const std::string &type, const std::vector<State> &input_points,
      const State &start_state, const Map &grid_map);

  bool solve(ReferencePath *reference_path,
             std::vector<State> *smoothed_path_display = nullptr);
  std::vector<std::vector<double>> display() const;

 protected:
  bool segmentRawReference(std::vector<double> *x_list,
                           std::vector<double> *y_list,
                           std::vector<double> *s_list,
                           std::vector<double> *angle_list,
                           std::vector<double> *k_list) const;
  double getClosestPointOnSpline(const tk::spline &x_s, const tk::spline &y_s,
                                 const double max_s) const;
  const State &start_state_;
  const Map &grid_map_;
  // Data to be passed into solvers.
  std::vector<double> x_list_, y_list_, s_list_;

 private:
  virtual bool smooth(PathOptimizationNS::ReferencePath *reference_path,
                      std::vector<State> *smoothed_path_display) = 0;
  void bSpline();
  // search.
  bool modifyInputPoints();
  inline bool checkExistenceInClosedSet(const APoint &point) const;
  inline double getG(const APoint &point, const APoint &parent) const;
  inline double getH(const APoint &p) const;
  const std::vector<State> &input_points_;
  // Sampled points in searching process.
  std::vector<std::vector<APoint>> sampled_points_;
  double target_s_{};
  std::priority_queue<APoint *, std::vector<APoint *>, PointComparator>
      open_set_;
  std::set<const APoint *> closed_set_;
};
}  // namespace PathOptimizationNS