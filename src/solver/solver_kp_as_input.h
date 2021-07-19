#pragma once

#include "solver.h"

namespace PathOptimizationNS {

class SolverKpAsInput : public OsqpSolver {
 public:
  SolverKpAsInput() = delete;

  SolverKpAsInput(const ReferencePath &reference_path,
                  const VehicleState &vehicle_state, const size_t &horizon);

  ~SolverKpAsInput() override = default;

  //  bool solve(std::vector<State> *optimized_path) override ;

 private:
  void setHessianMatrix(Eigen::SparseMatrix<double> *matrix_h) const override;

  void setConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                           Eigen::VectorXd *lower_bound,
                           Eigen::VectorXd *upper_bound) const override;
  void getOptimizedPath(
      const Eigen::VectorXd &optimization_result,
      std::vector<PathOptimizationNS::State> *optimized_path) const override;
  const int keep_control_steps_{};
  const size_t control_horizon_{};
  const size_t state_size_{};
  const size_t control_size_{};
  const size_t slack_size_{};
};
}  // namespace PathOptimizationNS