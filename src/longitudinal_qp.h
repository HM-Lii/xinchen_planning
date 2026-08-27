#ifndef LONGITUDINAL_QP_H
#define LONGITUDINAL_QP_H

#include <Eigen/Core>

#include "longitudinal_types.h"
#include "qp_solver.h"

class LongitudinalQp {
public:
  explicit LongitudinalQp(
      const LongitudinalQpConfig &config = LongitudinalQpConfig());

  LongitudinalQpResult Solve(const LongitudinalQpInput &input);
  LongitudinalQpResult Evaluate(const LongitudinalQpInput &input,
                                const QpWarmStartState &warm_start) const;
  LongitudinalQpResult Evaluate(
      const LongitudinalQpInput &input,
      const QpWarmStartState &warm_start,
      const QpSolverOptions &solver_options) const;
  void CommitWarmStart(const LongitudinalQpResult &result);
  void ResetWarmStart();
  QpWarmStartState warm_start() const;

private:
  LongitudinalQpConfig config_;
  QpSolver solver_;
  Eigen::VectorXd warm_start_;
};

#endif // LONGITUDINAL_QP_H
