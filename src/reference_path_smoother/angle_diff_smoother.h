#pragma once

#include <cfloat>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <vector>

#include "reference_path_smoother.h"
#include "src/data_struct/data_struct.h"
#include "src/tinyspline_ros/tinysplinecpp.h"

namespace PathOptimizationNS {

using CppAD::AD;
class FgEvalFrenetSmooth {
 public:
  FgEvalFrenetSmooth(const std::vector<double> &seg_x_list,
                     const std::vector<double> &seg_y_list,
                     const std::vector<double> &seg_angle_list,
                     const std::vector<double> &seg_s_list, const int &N,
                     const std::vector<double> &cost_func);
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector &fg, const ADvector &vars);

 private:
  size_t N;
  const std::vector<double> &seg_s_list_;
  const std::vector<double> &seg_x_list_;
  const std::vector<double> &seg_y_list_;
  const std::vector<double> &seg_angle_list_;
  double cost_func_curvature_weight_{};
  double cost_func_curvature_rate_weight_{};
  double cost_func_bound_weight_{};
  double cost_func_s_weight_{};
};

class AngleDiffSmoother final : public ReferencePathSmoother {
 public:
  AngleDiffSmoother() = delete;
  AngleDiffSmoother(const std::vector<State> &input_points,
                    const State &start_state, const Map &grid_map);
  ~AngleDiffSmoother() override = default;

 private:
  bool smooth(PathOptimizationNS::ReferencePath *reference_path,
              std::vector<State> *smoothed_path_display) override;
};

}  // namespace PathOptimizationNS