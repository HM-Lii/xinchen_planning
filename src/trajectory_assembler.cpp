#include "trajectory_assembler.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "trajectory_sampler.h"

namespace {

constexpr double kCanonicalizationTolerance = 1e-3;

bool Finite(double value) {
  return std::isfinite(value);
}

double Clamp(double value, double minimum, double maximum) {
  return std::max(minimum, std::min(value, maximum));
}

} // namespace

std::vector<LongitudinalState>
SampleCanonicalTrajectory(const LongitudinalQpResult &qp,
                          const TrajectoryCanonicalizationConfig &config) {
  std::vector<LongitudinalState> sampled = SampleTrajectory(
      qp.trajectory, config.sample_time_step_s, config.sample_count);
  double previous_progress_m = 0.0;
  for (LongitudinalState &state : sampled) {
    if (!Finite(state.s) || !Finite(state.v) || !Finite(state.a) ||
        !Finite(state.j) ||
        state.s + kCanonicalizationTolerance < previous_progress_m ||
        state.v < -kCanonicalizationTolerance ||
        state.a <
            config.minimum_acceleration_mps2 - kCanonicalizationTolerance ||
        state.a >
            config.maximum_acceleration_mps2 + kCanonicalizationTolerance ||
        std::fabs(state.j) >
            config.maximum_jerk_mps3 + kCanonicalizationTolerance ||
        state.s > config.maximum_progress_m + kCanonicalizationTolerance) {
      throw std::runtime_error(config.invalid_trajectory_message);
    }
    state.s = std::max(previous_progress_m, std::max(0.0, state.s));
    state.v = std::max(0.0, state.v);
    state.a = Clamp(state.a, config.minimum_acceleration_mps2,
                    config.maximum_acceleration_mps2);
    state.j =
        Clamp(state.j, -config.maximum_jerk_mps3, config.maximum_jerk_mps3);
    previous_progress_m = state.s;
  }
  return sampled;
}

FullTrajectory InitializeFullTrajectory(const PlanningSnapshot &snapshot,
                                        double time_step_s,
                                        double planning_horizon_s,
                                        std::size_t generated_point_count) {
  FullTrajectory trajectory;
  trajectory.time_step_s = time_step_s;
  trajectory.retained_prefix_points = snapshot.retained_prefix_points;
  trajectory.planning_frontier_delay_s =
      snapshot.frontier.time_from_telemetry_s;
  trajectory.planning_horizon_s = planning_horizon_s;
  trajectory.exact_retained_state = snapshot.historical_plan_aligned;
  trajectory.kinematic_seed_x = snapshot.kinematic_seed_x;
  trajectory.kinematic_seed_y = snapshot.kinematic_seed_y;
  trajectory.points.reserve(snapshot.retained_prefix_points +
                            generated_point_count);
  return trajectory;
}

void ValidateRetainedTrajectoryPrefix(const PlanningSnapshot &snapshot) {
  if (snapshot.retained_prefix_points !=
          snapshot.retained_longitudinal_states.size() ||
      snapshot.retained_prefix_points !=
          snapshot.retained_lateral_states.size() ||
      snapshot.retained_prefix_points !=
          snapshot.input.previous_path_x.size() ||
      snapshot.input.previous_path_x.size() !=
          snapshot.input.previous_path_y.size()) {
    throw std::invalid_argument("retained trajectory state is incomplete");
  }
}

void AppendRetainedTrajectoryPrefix(const PlanningSnapshot &snapshot,
                                    double time_step_s,
                                    double expected_frame_offset_x,
                                    double expected_frame_offset_y,
                                    bool normalize_expected_frame,
                                    FullTrajectory *trajectory) {
  if (trajectory == nullptr) {
    throw std::invalid_argument("trajectory output is null");
  }
  ValidateRetainedTrajectoryPrefix(snapshot);
  for (std::size_t index = 0; index < snapshot.retained_prefix_points;
       ++index) {
    TrajectoryPoint point;
    point.time_from_telemetry_s = static_cast<double>(index + 1) * time_step_s;
    point.x = snapshot.input.previous_path_x[index];
    point.y = snapshot.input.previous_path_y[index];
    point.longitudinal = snapshot.retained_longitudinal_states[index];
    point.lateral = snapshot.retained_lateral_states[index];
    if (normalize_expected_frame && point.lateral.valid) {
      point.lateral.expected_x += expected_frame_offset_x;
      point.lateral.expected_y += expected_frame_offset_y;
    }
    point.retained_prefix = true;
    trajectory->points.push_back(point);
  }
}

TrajectoryOutputSlice SliceTrajectoryOutput(const FullTrajectory &trajectory,
                                            std::size_t output_points,
                                            const char *error_message) {
  if (trajectory.points.size() < output_points) {
    throw std::logic_error(error_message);
  }
  TrajectoryOutputSlice result;
  result.output.next_x.reserve(output_points);
  result.output.next_y.reserve(output_points);
  result.longitudinal.reserve(output_points);
  result.lateral.reserve(output_points);
  for (std::size_t index = 0; index < output_points; ++index) {
    const TrajectoryPoint &point = trajectory.points[index];
    result.output.next_x.push_back(point.x);
    result.output.next_y.push_back(point.y);
    result.longitudinal.push_back(point.longitudinal);
    result.lateral.push_back(point.lateral);
  }
  return result;
}
