#pragma once

#include <cfloat>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <vector>

#include "reference_path_smoother.h"

namespace PathOptimizationNS {

using CppAD::AD;
class FgEvalReferenceSmoothing {
 public:
  FgEvalReferenceSmoothing(const std::vector<double> &seg_x_list,
                           const std::vector<double> &seg_y_list,
                           const std::vector<double> &seg_s_list,
                           const std::vector<double> &seg_angle_list)
      : seg_s_list_(seg_s_list),
        seg_x_list_(seg_x_list),
        seg_y_list_(seg_y_list),
        seg_angle_list_(seg_angle_list) {}
  virtual ~FgEvalReferenceSmoothing() = default;
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  typedef AD<double> ad;
  virtual void operator()(ADvector &fg, const ADvector &vars);

 protected:
  const std::vector<double> &seg_s_list_;
  const std::vector<double> &seg_x_list_;
  const std::vector<double> &seg_y_list_;
  const std::vector<double> &seg_angle_list_;
};

class TensionSmoother : public ReferencePathSmoother {
 public:
  TensionSmoother() = delete;
  TensionSmoother(const std::vector<State> &input_points,
                  const State &start_state, const Map &grid_map);
  ~TensionSmoother() override = default;

 private:
  bool smooth(PathOptimizationNS::ReferencePath *reference_path,
              std::vector<State> *smoothed_path_display) override;
  virtual bool ipoptSmooth(
      const std::vector<double> &x_list, const std::vector<double> &y_list,
      const std::vector<double> &angle_list, const std::vector<double> &k_list,
      const std::vector<double> &s_list, std::vector<double> *result_x_list,
      std::vector<double> *result_y_list, std::vector<double> *result_s_list);
  virtual bool osqpSmooth(
      const std::vector<double> &x_list, const std::vector<double> &y_list,
      const std::vector<double> &angle_list, const std::vector<double> &k_list,
      const std::vector<double> &s_list, std::vector<double> *result_x_list,
      std::vector<double> *result_y_list, std::vector<double> *result_s_list);
  virtual void setHessianMatrix(size_t size,
                                Eigen::SparseMatrix<double> *matrix_h) const;
  virtual void setConstraintMatrix(
      const std::vector<double> &x_list, const std::vector<double> &y_list,
      const std::vector<double> &angle_list, const std::vector<double> &k_list,
      const std::vector<double> &s_list,
      Eigen::SparseMatrix<double> *matrix_constraints,
      Eigen::VectorXd *lower_bound, Eigen::VectorXd *upper_bound) const;
};

}  // namespace PathOptimizationNS