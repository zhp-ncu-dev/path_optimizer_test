#pragma once

#include <eigen3/Eigen/Dense>
#include <memory>

#include "OsqpEigen/OsqpEigen.h"
#include "glog/logging.h"
#include "src/data_struct/data_struct.h"
#include "src/data_struct/reference_path.h"
#include "src/data_struct/vehicle_state_frenet.h"

namespace PathOptimizationNS {

class OsqpSolver {
 public:
  OsqpSolver() = delete;

  OsqpSolver(const ReferencePath &reference_path,
             const VehicleState &vehicle_state, const size_t &horizon);

  virtual ~OsqpSolver() = default;

  static std::unique_ptr<OsqpSolver> create(std::string &type,
                                            const ReferencePath &reference_path,
                                            const VehicleState &vehicle_state,
                                            const size_t &horizon);

  virtual bool solve(std::vector<State> *optimized_path);

 private:
  // Set Matrices for osqp solver.
  virtual void setHessianMatrix(
      Eigen::SparseMatrix<double> *matrix_h) const = 0;

  virtual void setConstraintMatrix(
      Eigen::SparseMatrix<double> *matrix_constraints,
      Eigen::VectorXd *lower_bound, Eigen::VectorXd *upper_bound) const = 0;
  virtual void getOptimizedPath(const Eigen::VectorXd &optimization_result,
                                std::vector<State> *optimized_path) const = 0;

 protected:
  const size_t horizon_{};
  const ReferencePath &reference_path_;
  const VehicleState &vehicle_state_;
  OsqpEigen::Solver solver_;
  double reference_interval_;
  int num_of_variables_, num_of_constraints_;
};

}  // namespace PathOptimizationNS