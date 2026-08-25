#ifndef SPEED_REFERENCE_H
#define SPEED_REFERENCE_H

#include "longitudinal_types.h"
#include "qp_solver.h"

class SpeedReferenceGenerator {
public:
  explicit SpeedReferenceGenerator(
      const SpeedReferenceConfig &config = SpeedReferenceConfig());

  SpeedReferenceResult
  Generate(const std::vector<PredictedObstacle> &obstacles) const;

private:
  SpeedReferenceConfig config_;
  QpSolver solver_;
};

#endif // SPEED_REFERENCE_H
