#pragma once

#include "solver.h"

namespace PathOptimizationNS {

class SolverKpAsInputConstrained : public OsqpSolver {
 public:
  SolverKpAsInputConstrained() = delete;

  SolverKpAsInputConstrained(const ReferencePath &reference_path,
                             const VehicleState &vehicle_state,
                             const size_t &horizon);

  ~SolverKpAsInputConstrained() override = default;

 private:
  void setHessianMatrix(Eigen::SparseMatrix<double> *matrix_h) const override;

  void setConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                           Eigen::VectorXd *lower_bound,
                           Eigen::VectorXd *upper_bound) const override;
  void getOptimizedPath(const Eigen::VectorXd &optimization_result,
                        std::vector<State> *optimized_path) const override;

  const int keep_control_steps_{};
  const size_t control_horizon_{};
  const size_t state_size_{};
  const size_t control_size_{};
  const size_t slack_size_{};
};
}