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
  bool success = false;
  std::string status;
  double objective = 0.0;
  Eigen::VectorXd primal;
};

class QpSolver {
public:
  QpSolverResult Solve(const QuadraticProgram &problem,
                       const Eigen::VectorXd *warm_start = nullptr) const;
};

#endif // QP_SOLVER_H
