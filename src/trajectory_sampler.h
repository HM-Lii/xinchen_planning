#ifndef TRAJECTORY_SAMPLER_H
#define TRAJECTORY_SAMPLER_H

#include <cstddef>
#include <vector>

#include "longitudinal_types.h"

LongitudinalState EvaluateTrajectory(const LongitudinalTrajectory &trajectory,
                                     double time_seconds);

std::vector<LongitudinalState>
SampleTrajectory(const LongitudinalTrajectory &trajectory,
                 double sample_time_step_seconds, std::size_t sample_count);

#endif // TRAJECTORY_SAMPLER_H
