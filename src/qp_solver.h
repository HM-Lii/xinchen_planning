#ifndef QP_SOLVER_H
#define QP_SOLVER_H

#include <string>

#include <Eigen/Core>
#include <Eigen/SparseCore>

struct QuadraticProgram {
  Eigen::SparseMatrix<double> hessian;
  Eigen::VectorXd gradient;
  Eigen::SparseMatrix<double> constraint_matrix;
  Eigen::VectorXd lower_bound;
  Eigen::VectorXd upper_bound;
};

struct QpSolverResult {
  // success guarantees a finite objective and a fully finite primal vector.
  bool success = false;
  std::string status;
  double objective = 0.0;
  Eigen::VectorXd primal;
};

struct QpSolverOptions {
  int maximum_iterations = 6000;
  double absolute_tolerance = 1e-6;
  double relative_tolerance = 1e-6;
  // Zero preserves OSQP's effectively-unbounded default. Auxiliary callers
  // may set a positive wall-clock limit for bounded computation.
  double time_limit_seconds = 0.0;
};

class QpSolver {
public:
  QpSolverResult Solve(const QuadraticProgram &problem,
                       const Eigen::VectorXd *warm_start = nullptr,
                       const QpSolverOptions &options =
                           QpSolverOptions()) const;
};

#endif // QP_SOLVER_H
