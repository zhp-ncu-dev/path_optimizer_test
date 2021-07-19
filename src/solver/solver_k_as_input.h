#pragma once

#include <cassert>

#include "solver.h"

namespace PathOptimizationNS {

class SolverKAsInput : public OsqpSolver {
 public:
  SolverKAsInput() = delete;

  SolverKAsInput(const ReferencePath &reference_path,
                 const VehicleState &vehicle_state, const size_t &horizon);

  ~SolverKAsInput() override = default;

 private:
  // Set Matrices for osqp solver.
  void setHessianMatrix(Eigen::SparseMatrix<double> *matrix_h) const override;

  void setDynamicMatrix(size_t i, Eigen::Matrix<double, 2, 2> *matrix_a,
                        Eigen::Matrix<double, 2, 1> *matrix_b) const;

  void setConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                           Eigen::VectorXd *lower_bound,
                           Eigen::VectorXd *upper_bound) const override;
  void getOptimizedPath(const Eigen::VectorXd &optimization_result,
                        std::vector<State> *optimized_path) const override;
};
} 