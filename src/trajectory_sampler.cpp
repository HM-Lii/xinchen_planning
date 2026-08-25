#include "trajectory_sampler.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {

void ValidateTrajectory(const LongitudinalTrajectory &trajectory) {
  if (trajectory.time_step_seconds <= 0.0 ||
      !std::isfinite(trajectory.time_step_seconds) ||
      trajectory.states.size() < 2) {
    throw std::invalid_argument("invalid longitudinal trajectory");
  }
  for (const LongitudinalState &state : trajectory.states) {
    if (!std::isfinite(state.s) || !std::isfinite(state.v) ||
        !std::isfinite(state.a) || !std::isfinite(state.j)) {
      throw std::invalid_argument("longitudinal trajectory is non-finite");
    }
  }
}

} // namespace

LongitudinalState EvaluateTrajectory(const LongitudinalTrajectory &trajectory,
                                     double time_seconds) {
  ValidateTrajectory(trajectory);
  const double dt = trajectory.time_step_seconds;
  const double duration =
      static_cast<double>(trajectory.states.size() - 1) * dt;
  if (!std::isfinite(time_seconds) || time_seconds < -1e-12 ||
      time_seconds > duration + 1e-9) {
    throw std::out_of_range("trajectory sample time is outside the horizon");
  }
  if (time_seconds >= duration - 1e-12) {
    return trajectory.states.back();
  }

  const double nonnegative_time = std::max(0.0, time_seconds);
  const std::size_t interval =
      std::min(static_cast<std::size_t>(std::floor(nonnegative_time / dt)),
               trajectory.states.size() - 2);
  const double interval_start = static_cast<double>(interval) * dt;
  const double tau = nonnegative_time - interval_start;
  const LongitudinalState &start = trajectory.states[interval];

  LongitudinalState result;
  result.s = start.s + start.v * tau + 0.5 * start.a * tau * tau +
             start.j * tau * tau * tau / 6.0;
  result.v = start.v + start.a * tau + 0.5 * start.j * tau * tau;
  result.a = start.a + start.j * tau;
  result.j = start.j;
  return result;
}

std::vector<LongitudinalState>
SampleTrajectory(const LongitudinalTrajectory &trajectory,
                 double sample_time_step_seconds, std::size_t sample_count) {
  ValidateTrajectory(trajectory);
  if (!std::isfinite(sample_time_step_seconds) ||
      sample_time_step_seconds <= 0.0) {
    throw std::invalid_argument("invalid trajectory sample time step");
  }

  std::vector<LongitudinalState> samples;
  samples.reserve(sample_count);
  for (std::size_t index = 1; index <= sample_count; ++index) {
    samples.push_back(EvaluateTrajectory(
        trajectory, static_cast<double>(index) * sample_time_step_seconds));
  }
  return samples;
}
