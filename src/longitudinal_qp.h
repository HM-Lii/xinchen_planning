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

private:
  LongitudinalQpConfig config_;
  QpSolver solver_;
  Eigen::VectorXd warm_start_;
};

#endif // LONGITUDINAL_QP_H
