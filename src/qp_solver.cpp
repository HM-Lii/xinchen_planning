#include "qp_solver.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/SparseCore>
#include <osqp.h>

namespace {

using SparseMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor, Eigen::Index>;

class OsqpSolverDeleter {
public:
  void operator()(OSQPSolver *solver) const {
    if (solver != nullptr) {
      osqp_cleanup(solver);
    }
  }
};

struct CscStorage {
  OSQPInt rows = 0;
  OSQPInt columns = 0;
  std::vector<OSQPFloat> values;
  std::vector<OSQPInt> row_indices;
  std::vector<OSQPInt> column_pointers;

  OSQPCscMatrix Matrix() {
    OSQPCscMatrix result;
    csc_set_data(&result, rows, columns, static_cast<OSQPInt>(values.size()),
                 values.data(), row_indices.data(), column_pointers.data());
    return result;
  }
};

std::string StatusName(OSQPInt status) {
  switch (status) {
  case OSQP_SOLVED:
    return "solved";
  case OSQP_SOLVED_INACCURATE:
    return "solved inaccurate";
  case OSQP_PRIMAL_INFEASIBLE:
    return "primal infeasible";
  case OSQP_PRIMAL_INFEASIBLE_INACCURATE:
    return "primal infeasible inaccurate";
  case OSQP_DUAL_INFEASIBLE:
    return "dual infeasible";
  case OSQP_DUAL_INFEASIBLE_INACCURATE:
    return "dual infeasible inaccurate";
  case OSQP_MAX_ITER_REACHED:
    return "maximum iterations reached";
  case OSQP_TIME_LIMIT_REACHED:
    return "time limit reached";
  case OSQP_NON_CVX:
    return "non-convex problem";
  case OSQP_SIGINT:
    return "interrupted";
  case OSQP_UNSOLVED:
    return "unsolved";
  default:
    return "unknown";
  }
}

void ValidateFiniteMatrix(const Eigen::SparseMatrix<double> &matrix,
                          const char *name) {
  for (int column = 0; column < matrix.outerSize(); ++column) {
    for (Eigen::SparseMatrix<double>::InnerIterator item(matrix, column); item;
         ++item) {
      if (!std::isfinite(item.value())) {
        throw std::invalid_argument(std::string(name) +
                                    " contains a non-finite value");
      }
    }
  }
}

void ValidateProblem(const QuadraticProgram &problem) {
  const Eigen::Index variables = problem.hessian.rows();
  const Eigen::Index constraints = problem.constraint_matrix.rows();
  if (variables <= 0 || problem.hessian.cols() != variables ||
      problem.gradient.size() != variables ||
      problem.constraint_matrix.cols() != variables ||
      problem.lower_bound.size() != constraints ||
      problem.upper_bound.size() != constraints) {
    throw std::invalid_argument("inconsistent QP matrix dimensions");
  }
  ValidateFiniteMatrix(problem.hessian, "QP Hessian");
  ValidateFiniteMatrix(problem.constraint_matrix, "QP constraint matrix");
  for (Eigen::Index variable = 0; variable < variables; ++variable) {
    if (!std::isfinite(problem.gradient[variable])) {
      throw std::invalid_argument("QP gradient contains a non-finite value");
    }
  }
  for (Eigen::Index row = 0; row < constraints; ++row) {
    if (std::isnan(problem.lower_bound[row]) ||
        std::isnan(problem.upper_bound[row]) ||
        problem.lower_bound[row] > problem.upper_bound[row]) {
      throw std::invalid_argument("invalid QP constraint bounds");
    }
  }
}

CscStorage ConvertMatrix(const Eigen::SparseMatrix<double> &source,
                         bool upper_triangular_only) {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(source.nonZeros()));
  for (int column = 0; column < source.outerSize(); ++column) {
    for (Eigen::SparseMatrix<double>::InnerIterator item(source, column); item;
         ++item) {
      if (!upper_triangular_only || item.row() <= item.col()) {
        triplets.emplace_back(item.row(), item.col(), item.value());
      }
    }
  }

  SparseMatrix compressed(source.rows(), source.cols());
  compressed.setFromTriplets(triplets.begin(), triplets.end());
  compressed.makeCompressed();

  CscStorage result;
  result.rows = static_cast<OSQPInt>(compressed.rows());
  result.columns = static_cast<OSQPInt>(compressed.cols());
  result.values.reserve(static_cast<std::size_t>(compressed.nonZeros()));
  result.row_indices.reserve(static_cast<std::size_t>(compressed.nonZeros()));
  for (Eigen::Index index = 0; index < compressed.nonZeros(); ++index) {
    result.values.push_back(
        static_cast<OSQPFloat>(compressed.valuePtr()[index]));
    result.row_indices.push_back(
        static_cast<OSQPInt>(compressed.innerIndexPtr()[index]));
  }
  result.column_pointers.reserve(
      static_cast<std::size_t>(compressed.cols() + 1));
  for (Eigen::Index column = 0; column <= compressed.cols(); ++column) {
    result.column_pointers.push_back(
        static_cast<OSQPInt>(compressed.outerIndexPtr()[column]));
  }
  return result;
}

std::vector<OSQPFloat> ConvertVector(const Eigen::VectorXd &source,
                                     bool allow_infinity) {
  std::vector<OSQPFloat> result(static_cast<std::size_t>(source.size()));
  for (Eigen::Index index = 0; index < source.size(); ++index) {
    double value = source[index];
    if (allow_infinity) {
      value = std::max(-static_cast<double>(OSQP_INFTY),
                       std::min(value, static_cast<double>(OSQP_INFTY)));
    }
    result[static_cast<std::size_t>(index)] = static_cast<OSQPFloat>(value);
  }
  return result;
}

} // namespace

QpSolverResult QpSolver::Solve(const QuadraticProgram &problem,
                               const Eigen::VectorXd *warm_start) const {
  ValidateProblem(problem);

  CscStorage hessian_storage = ConvertMatrix(problem.hessian, true);
  CscStorage constraint_storage =
      ConvertMatrix(problem.constraint_matrix, false);
  OSQPCscMatrix hessian = hessian_storage.Matrix();
  OSQPCscMatrix constraints = constraint_storage.Matrix();
  std::vector<OSQPFloat> gradient = ConvertVector(problem.gradient, false);
  std::vector<OSQPFloat> lower_bound = ConvertVector(problem.lower_bound, true);
  std::vector<OSQPFloat> upper_bound = ConvertVector(problem.upper_bound, true);

  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.warm_starting = 1;
  settings.max_iter = 6000;
  settings.eps_abs = 1e-5;
  settings.eps_rel = 1e-5;
  settings.polishing = 1;

  OSQPSolver *raw_solver = nullptr;
  const OSQPInt setup_error =
      osqp_setup(&raw_solver, &hessian, gradient.data(), &constraints,
                 lower_bound.data(), upper_bound.data(),
                 static_cast<OSQPInt>(problem.constraint_matrix.rows()),
                 static_cast<OSQPInt>(problem.hessian.rows()), &settings);
  std::unique_ptr<OSQPSolver, OsqpSolverDeleter> solver(raw_solver);

  QpSolverResult result;
  if (setup_error != OSQP_NO_ERROR || solver == nullptr) {
    result.status =
        std::string("OSQP setup failed: ") + osqp_error_message(setup_error);
    return result;
  }

  std::vector<OSQPFloat> warm_start_values;
  if (warm_start != nullptr && warm_start->size() == problem.hessian.rows()) {
    warm_start_values = ConvertVector(*warm_start, false);
    osqp_warm_start(solver.get(), warm_start_values.data(), nullptr);
  }

  const OSQPInt solve_error = osqp_solve(solver.get());
  if (solve_error != OSQP_NO_ERROR) {
    result.status = std::string("OSQP execution failed: ") +
                    osqp_error_message(solve_error);
    return result;
  }
  if (solver->info == nullptr) {
    result.status = "OSQP returned no solver information";
    return result;
  }

  result.status = StatusName(solver->info->status_val);
  result.success = solver->info->status_val == OSQP_SOLVED ||
                   solver->info->status_val == OSQP_SOLVED_INACCURATE;
  result.objective = solver->info->obj_val;
  if (result.success && solver->solution != nullptr &&
      solver->solution->x != nullptr) {
    result.primal.resize(problem.hessian.rows());
    for (Eigen::Index index = 0; index < result.primal.size(); ++index) {
      result.primal[index] = solver->solution->x[index];
    }
  } else if (result.success) {
    result.success = false;
    result.status = "OSQP returned no primal solution";
  }
  return result;
}
