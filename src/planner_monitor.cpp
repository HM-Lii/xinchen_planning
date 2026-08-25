#include "planner_monitor.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#endif

namespace {

constexpr double kMilesPerHourToMetersPerSecond = 0.44704;
constexpr double kMetersPerSecondToMilesPerHour =
    1.0 / kMilesPerHourToMetersPerSecond;
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

enum ViolationBit {
  kQpSpeedViolation = 1U << 0,
  kQpAccelerationViolation = 1U << 1,
  kQpJerkViolation = 1U << 2,
  kQpCollisionViolation = 1U << 3,
  kCartesianSpeedViolation = 1U << 4,
  kCartesianAccelerationViolation = 1U << 5,
  kCartesianJerkViolation = 1U << 6,
  kLaneDeviationViolation = 1U << 7
};

bool IsFinite(double value) { return std::isfinite(value); }

bool HardCollisionActive(const PredictedObstacle &obstacle,
                         std::size_t node, std::size_t nodes) {
  return obstacle.hard_collision_active.empty() ||
         (obstacle.hard_collision_active.size() == nodes &&
          obstacle.hard_collision_active[node] != 0U);
}

void UpdateMinimum(IndexedMetric *metric, double value, std::size_t index) {
  if (!IsFinite(value)) {
    return;
  }
  if (!metric->valid || value < metric->value) {
    metric->valid = true;
    metric->value = value;
    metric->index = index;
  }
}

void UpdateMaximum(IndexedMetric *metric, double value, std::size_t index) {
  if (!IsFinite(value)) {
    return;
  }
  if (!metric->valid || value > metric->value) {
    metric->valid = true;
    metric->value = value;
    metric->index = index;
  }
}

bool Below(const IndexedMetric &metric, double limit) {
  return metric.valid && metric.value < limit;
}

bool Above(const IndexedMetric &metric, double limit) {
  return metric.valid && metric.value > limit;
}

double MetricValueOrNan(const IndexedMetric &metric) {
  return metric.valid ? metric.value : std::numeric_limits<double>::quiet_NaN();
}

long long MetricIndexOrNegativeOne(const IndexedMetric &metric) {
  return metric.valid ? static_cast<long long>(metric.index) : -1LL;
}

unsigned int ViolationMask(const PlannerCycleDiagnostics &diagnostics) {
  unsigned int mask = 0;
  if (diagnostics.qp_speed_violation) {
    mask |= kQpSpeedViolation;
  }
  if (diagnostics.qp_acceleration_violation) {
    mask |= kQpAccelerationViolation;
  }
  if (diagnostics.qp_jerk_violation) {
    mask |= kQpJerkViolation;
  }
  if (diagnostics.qp_collision_violation) {
    mask |= kQpCollisionViolation;
  }
  if (diagnostics.cartesian_speed_violation) {
    mask |= kCartesianSpeedViolation;
  }
  if (diagnostics.cartesian_acceleration_violation) {
    mask |= kCartesianAccelerationViolation;
  }
  if (diagnostics.cartesian_jerk_violation) {
    mask |= kCartesianJerkViolation;
  }
  if (diagnostics.lane_deviation_violation) {
    mask |= kLaneDeviationViolation;
  }
  return mask;
}

std::string ViolationNames(unsigned int mask) {
  std::ostringstream names;
  bool first = true;
  const auto append = [&names, &first](const char *name) {
    if (!first) {
      names << '|';
    }
    names << name;
    first = false;
  };
  if ((mask & kQpSpeedViolation) != 0U) {
    append("qp_speed");
  }
  if ((mask & kQpAccelerationViolation) != 0U) {
    append("qp_acc");
  }
  if ((mask & kQpJerkViolation) != 0U) {
    append("qp_jerk");
  }
  if ((mask & kQpCollisionViolation) != 0U) {
    append("qp_collision");
  }
  if ((mask & kCartesianSpeedViolation) != 0U) {
    append("xy_speed");
  }
  if ((mask & kCartesianAccelerationViolation) != 0U) {
    append("xy_acc");
  }
  if ((mask & kCartesianJerkViolation) != 0U) {
    append("xy_jerk");
  }
  if ((mask & kLaneDeviationViolation) != 0U) {
    append("lane_deviation");
  }
  return names.str();
}

std::string MetricText(const IndexedMetric &metric) {
  if (!metric.valid) {
    return "n/a";
  }
  std::ostringstream text;
  text << std::fixed << std::setprecision(3) << metric.value << '@'
       << metric.index;
  return text.str();
}

std::string CsvSafe(std::string text) {
  for (char &character : text) {
    if (character == ',' || character == '\n' || character == '\r') {
      character = ';';
    }
  }
  return text;
}

bool DirectoryExists(const std::string &path) {
  struct stat information;
  return stat(path.c_str(), &information) == 0 &&
         (information.st_mode & S_IFDIR) != 0;
}

bool CreateOneDirectory(const std::string &path) {
#ifdef _WIN32
  const int result = _mkdir(path.c_str());
#else
  const int result = mkdir(path.c_str(), 0755);
#endif
  return result == 0 || errno == EEXIST;
}

bool EnsureDirectory(const std::string &path) {
  if (path.empty()) {
    return false;
  }
  if (DirectoryExists(path)) {
    return true;
  }
  const std::size_t separator = path.find_last_of("/\\");
  if (separator != std::string::npos && separator > 0) {
    const std::string parent = path.substr(0, separator);
    if (!parent.empty() && !DirectoryExists(parent) &&
        !EnsureDirectory(parent)) {
      return false;
    }
  }
  return CreateOneDirectory(path) && DirectoryExists(path);
}

std::string JoinPath(const std::string &directory,
                     const std::string &filename) {
  if (directory.empty()) {
    return filename;
  }
  const char last = directory[directory.size() - 1];
  if (last == '/' || last == '\\') {
    return directory + filename;
  }
  return directory + "/" + filename;
}

void ValidateMonitorConfig(const PlannerMonitorConfig &config) {
  if (config.maximum_cartesian_acceleration_mps2 <= 0.0 ||
      config.maximum_cartesian_jerk_mps3 <= 0.0 ||
      config.speed_tolerance_mps < 0.0 ||
      config.acceleration_tolerance_mps2 < 0.0 ||
      config.jerk_tolerance_mps3 < 0.0 ||
      config.safety_tolerance_meters < 0.0 ||
      (config.enabled && config.write_csv && config.log_directory.empty())) {
    throw std::invalid_argument("invalid planner monitor configuration");
  }
}

void ValidateMonitorLimits(const PlannerMonitorLimits &limits) {
  if (!IsFinite(limits.output_time_step_seconds) ||
      limits.output_time_step_seconds <= 0.0 ||
      !IsFinite(limits.maximum_speed_mps) || limits.maximum_speed_mps <= 0.0 ||
      !IsFinite(limits.minimum_acceleration_mps2) ||
      !IsFinite(limits.maximum_acceleration_mps2) ||
      limits.minimum_acceleration_mps2 >= limits.maximum_acceleration_mps2 ||
      !IsFinite(limits.maximum_jerk_mps3) || limits.maximum_jerk_mps3 <= 0.0 ||
      !IsFinite(limits.maximum_cartesian_acceleration_mps2) ||
      limits.maximum_cartesian_acceleration_mps2 <= 0.0 ||
      !IsFinite(limits.maximum_cartesian_jerk_mps3) ||
      limits.maximum_cartesian_jerk_mps3 <= 0.0 ||
      limits.speed_tolerance_mps < 0.0 ||
      limits.acceleration_tolerance_mps2 < 0.0 ||
      limits.jerk_tolerance_mps3 < 0.0 ||
      limits.safety_tolerance_meters < 0.0 ||
      !IsFinite(limits.maximum_lateral_deviation_meters) ||
      limits.maximum_lateral_deviation_meters <= 0.0 ||
      limits.time_headway_seconds < 0.0 ||
      limits.fixed_headway_gap_meters < 0.0 ||
      limits.collision_gap_meters < 0.0) {
    throw std::invalid_argument("invalid planner monitor limits");
  }
}

} // namespace

bool PlannerCycleDiagnostics::HasViolation() const {
  return qp_speed_violation || qp_acceleration_violation || qp_jerk_violation ||
         qp_collision_violation || cartesian_speed_violation ||
         cartesian_acceleration_violation || cartesian_jerk_violation ||
         lane_deviation_violation;
}

std::vector<CartesianKinematicSample> ComputeCartesianKinematics(
    const PlannerInput &input, const PlannerOutput &output,
    double time_step_seconds, std::size_t previous_path_size) {
  if (!IsFinite(time_step_seconds) || time_step_seconds <= 0.0 ||
      output.next_x.size() != output.next_y.size() ||
      previous_path_size > output.next_x.size() || !IsFinite(input.ego.x) ||
      !IsFinite(input.ego.y) || !IsFinite(input.ego.yaw_deg) ||
      !IsFinite(input.ego.speed_mph)) {
    throw std::invalid_argument("invalid Cartesian monitor input");
  }

  const double ego_speed_mps =
      std::max(0.0, input.ego.speed_mph * kMilesPerHourToMetersPerSecond);
  const double yaw_radians = input.ego.yaw_deg * kDegreesToRadians;
  double previous_x = input.ego.x;
  double previous_y = input.ego.y;
  double previous_velocity_x = ego_speed_mps * std::cos(yaw_radians);
  double previous_velocity_y = ego_speed_mps * std::sin(yaw_radians);
  double previous_speed = ego_speed_mps;
  double previous_acceleration_x = 0.0;
  double previous_acceleration_y = 0.0;
  double previous_tangential_acceleration = 0.0;
  bool previous_acceleration_valid = false;

  std::vector<CartesianKinematicSample> samples;
  samples.reserve(output.next_x.size());
  for (std::size_t index = 0; index < output.next_x.size(); ++index) {
    const double x = output.next_x[index];
    const double y = output.next_y[index];
    if (!IsFinite(x) || !IsFinite(y)) {
      throw std::invalid_argument("Cartesian path contains a non-finite point");
    }

    CartesianKinematicSample sample;
    sample.output_index = index;
    sample.is_previous_path = index < previous_path_size;
    sample.x = x;
    sample.y = y;
    sample.velocity_x_mps = (x - previous_x) / time_step_seconds;
    sample.velocity_y_mps = (y - previous_y) / time_step_seconds;
    sample.speed_mps = std::hypot(sample.velocity_x_mps, sample.velocity_y_mps);
    sample.acceleration_x_mps2 =
        (sample.velocity_x_mps - previous_velocity_x) / time_step_seconds;
    sample.acceleration_y_mps2 =
        (sample.velocity_y_mps - previous_velocity_y) / time_step_seconds;
    sample.acceleration_mps2 =
        std::hypot(sample.acceleration_x_mps2, sample.acceleration_y_mps2);
    sample.tangential_acceleration_mps2 =
        (sample.speed_mps - previous_speed) / time_step_seconds;

    if (previous_acceleration_valid) {
      sample.jerk_valid = true;
      sample.jerk_x_mps3 =
          (sample.acceleration_x_mps2 - previous_acceleration_x) /
          time_step_seconds;
      sample.jerk_y_mps3 =
          (sample.acceleration_y_mps2 - previous_acceleration_y) /
          time_step_seconds;
      sample.jerk_mps3 = std::hypot(sample.jerk_x_mps3, sample.jerk_y_mps3);
      sample.tangential_jerk_mps3 = (sample.tangential_acceleration_mps2 -
                                     previous_tangential_acceleration) /
                                    time_step_seconds;
    }

    samples.push_back(sample);
    previous_x = x;
    previous_y = y;
    previous_velocity_x = sample.velocity_x_mps;
    previous_velocity_y = sample.velocity_y_mps;
    previous_speed = sample.speed_mps;
    previous_acceleration_x = sample.acceleration_x_mps2;
    previous_acceleration_y = sample.acceleration_y_mps2;
    previous_tangential_acceleration = sample.tangential_acceleration_mps2;
    // The telemetry velocity is instantaneous, while path velocities are
    // forward differences over one sample. The first derived acceleration is
    // therefore not centered consistently enough to form a jerk estimate.
    previous_acceleration_valid = index >= 1;
  }
  return samples;
}

PlannerCycleDiagnostics BuildPlannerDiagnostics(
    std::uint64_t cycle, const PlannerInput &input, const PlannerOutput &output,
    std::size_t previous_path_size, const LongitudinalState &initial_state,
    double plan_start_s, double plan_start_d, double lane_center_d,
    const std::vector<PredictedObstacle> &obstacles,
    const std::vector<double> &reference_speed_mps,
    const LongitudinalQpResult &qp_result,
    const std::vector<LongitudinalState> &output_longitudinal_states,
    const PlannerMonitorLimits &limits,
    const LateralStitchDiagnostics &lateral,
    const std::vector<LateralPathState> &output_lateral_states,
    bool historical_plan_aligned) {
  ValidateMonitorLimits(limits);
  if (output.next_x.size() != output_longitudinal_states.size()) {
    throw std::invalid_argument(
        "monitor output path and longitudinal state counts differ");
  }
  if (!output_lateral_states.empty() &&
      output.next_x.size() != output_lateral_states.size()) {
    throw std::invalid_argument(
        "monitor output path and lateral state counts differ");
  }

  PlannerCycleDiagnostics diagnostics;
  diagnostics.cycle = cycle;
  diagnostics.previous_path_size = previous_path_size;
  diagnostics.new_point_count = output.next_x.size() - previous_path_size;
  diagnostics.ego_speed_mps =
      std::max(0.0, input.ego.speed_mph * kMilesPerHourToMetersPerSecond);
  diagnostics.initial_speed_mps = initial_state.v;
  diagnostics.initial_acceleration_mps2 = initial_state.a;
  diagnostics.initial_jerk_mps3 = initial_state.j;
  diagnostics.plan_start_s = plan_start_s;
  diagnostics.plan_start_d = plan_start_d;
  diagnostics.lane_center_d = lane_center_d;
  diagnostics.historical_plan_aligned = historical_plan_aligned;
  diagnostics.lateral = lateral;
  diagnostics.relevant_obstacle_count = obstacles.size();
  diagnostics.qp_status = qp_result.status;
  diagnostics.qp_objective = qp_result.objective;
  diagnostics.qp_maximum_headway_slack_meters =
      qp_result.maximum_headway_slack_meters;
  diagnostics.emergency = qp_result.trajectory.emergency;
  diagnostics.output_longitudinal_states = output_longitudinal_states;
  diagnostics.output_lateral_states = output_lateral_states;

  for (std::size_t index = 0; index < obstacles.size(); ++index) {
    UpdateMinimum(&diagnostics.nearest_obstacle_distance,
                  obstacles[index].relative_s, index);
  }

  if (!reference_speed_mps.empty()) {
    diagnostics.reference_first_mps = reference_speed_mps.front();
    diagnostics.reference_last_mps = reference_speed_mps.back();
    diagnostics.reference_minimum_mps = *std::min_element(
        reference_speed_mps.begin(), reference_speed_mps.end());
  }

  const std::vector<LongitudinalState> &qp_states = qp_result.trajectory.states;
  for (const PredictedObstacle &obstacle : obstacles) {
    if ((!obstacle.hard_collision_active.empty() &&
         obstacle.hard_collision_active.size() != qp_states.size()) ||
        (!obstacle.intrusion_speed_limit_mps.empty() &&
         obstacle.intrusion_speed_limit_mps.size() != qp_states.size())) {
      throw std::invalid_argument(
          "monitor obstacle prediction and QP node counts differ");
    }
  }
  diagnostics.qp_samples.reserve(qp_states.size());
  for (std::size_t node = 0; node < qp_states.size(); ++node) {
    const LongitudinalState &state = qp_states[node];
    UpdateMinimum(&diagnostics.qp_minimum_speed, state.v, node);
    UpdateMaximum(&diagnostics.qp_maximum_speed, state.v, node);
    UpdateMinimum(&diagnostics.qp_minimum_acceleration, state.a, node);
    UpdateMaximum(&diagnostics.qp_maximum_acceleration, state.a, node);
    UpdateMaximum(&diagnostics.qp_maximum_absolute_jerk, std::fabs(state.j),
                  node);

    QpNodeMonitorSample sample;
    sample.node_index = node;
    sample.time_seconds =
        static_cast<double>(node) * qp_result.trajectory.time_step_seconds;
    sample.state = state;
    if (node < reference_speed_mps.size()) {
      sample.reference_speed_mps = reference_speed_mps[node];
    }

    for (const PredictedObstacle &obstacle : obstacles) {
      if (!obstacle.intrusion_speed_limit_mps.empty()) {
        const double speed_limit =
            obstacle.intrusion_speed_limit_mps[node];
        if (speed_limit < limits.maximum_speed_mps - 1e-9 &&
            (!sample.intrusion_speed_limit_valid ||
             speed_limit < sample.minimum_intrusion_speed_limit_mps)) {
          sample.intrusion_speed_limit_valid = true;
          sample.minimum_intrusion_speed_limit_mps = speed_limit;
          sample.intrusion_limiting_obstacle_id = obstacle.id;
        }
      }
      if (!HardCollisionActive(obstacle, node, qp_states.size())) {
        continue;
      }
      const double obstacle_s =
          obstacle.relative_s + obstacle.speed_mps * sample.time_seconds;
      const double required_ego_rear_s =
          state.s + limits.time_headway_seconds * state.v;
      const double headway_margin = obstacle_s -
                                    limits.fixed_headway_gap_meters -
                                    required_ego_rear_s;
      if (!sample.headway_margin_valid ||
          headway_margin < sample.minimum_headway_margin_meters) {
        sample.headway_margin_valid = true;
        sample.minimum_headway_margin_meters = headway_margin;
        sample.headway_limiting_obstacle_id = obstacle.id;
      }
      const double collision_margin =
          obstacle_s - limits.collision_gap_meters - state.s;
      if (!sample.collision_margin_valid ||
          collision_margin < sample.minimum_collision_margin_meters) {
        sample.collision_margin_valid = true;
        sample.minimum_collision_margin_meters = collision_margin;
        sample.collision_limiting_obstacle_id = obstacle.id;
      }
    }
    if (sample.intrusion_speed_limit_valid &&
        (!diagnostics.minimum_intrusion_speed_limit.valid ||
         sample.minimum_intrusion_speed_limit_mps <
             diagnostics.minimum_intrusion_speed_limit.value)) {
      diagnostics.minimum_intrusion_speed_limit.valid = true;
      diagnostics.minimum_intrusion_speed_limit.value =
          sample.minimum_intrusion_speed_limit_mps;
      diagnostics.minimum_intrusion_speed_limit.index = node;
      diagnostics.intrusion_limiting_obstacle_id =
          sample.intrusion_limiting_obstacle_id;
    }
    if (sample.headway_margin_valid &&
        (!diagnostics.qp_minimum_headway_margin.valid ||
         sample.minimum_headway_margin_meters <
             diagnostics.qp_minimum_headway_margin.value)) {
      diagnostics.qp_minimum_headway_margin.valid = true;
      diagnostics.qp_minimum_headway_margin.value =
          sample.minimum_headway_margin_meters;
      diagnostics.qp_minimum_headway_margin.index = node;
      diagnostics.qp_headway_limiting_obstacle_id =
          sample.headway_limiting_obstacle_id;
    }
    if (sample.collision_margin_valid &&
        (!diagnostics.qp_minimum_collision_margin.valid ||
         sample.minimum_collision_margin_meters <
             diagnostics.qp_minimum_collision_margin.value)) {
      diagnostics.qp_minimum_collision_margin.valid = true;
      diagnostics.qp_minimum_collision_margin.value =
          sample.minimum_collision_margin_meters;
      diagnostics.qp_minimum_collision_margin.index = node;
      diagnostics.qp_collision_limiting_obstacle_id =
          sample.collision_limiting_obstacle_id;
    }
    diagnostics.qp_samples.push_back(sample);
  }

  // Keep raw received points in the detail log so simulator quantization stays
  // observable.  For constraint classification, restore aligned historical
  // points to their exact generated coordinates and evaluate only newly added
  // points.  Thus quantization is monitored but never becomes a planning state.
  diagnostics.cartesian_samples = ComputeCartesianKinematics(
      input, output, limits.output_time_step_seconds, previous_path_size);
  PlannerOutput evaluation_output = output;
  const bool exact_history_available =
      historical_plan_aligned && lateral.state_aligned &&
      output_lateral_states.size() == output.next_x.size();
  double history_anchor_offset_x = 0.0;
  double history_anchor_offset_y = 0.0;
  if (exact_history_available && previous_path_size != 0) {
    const LateralPathState &anchor =
        output_lateral_states[previous_path_size - 1];
    if (anchor.valid) {
      history_anchor_offset_x =
          output.next_x[previous_path_size - 1] - anchor.expected_x;
      history_anchor_offset_y =
          output.next_y[previous_path_size - 1] - anchor.expected_y;
    }
  }
  if (exact_history_available) {
    for (std::size_t index = 0; index < previous_path_size; ++index) {
      const LateralPathState &state = output_lateral_states[index];
      if (!state.valid) {
        continue;
      }
      UpdateMaximum(&diagnostics.historical_path_position_residual,
                    std::hypot(output.next_x[index] - state.expected_x,
                               output.next_y[index] - state.expected_y),
                    index);
      // Preserve the exact historical shape but translate it into the same
      // endpoint frame used by the residual-corrected new path. This removes
      // simulator coordinate quantization without creating an artificial
      // position step at the old/new path boundary.
      evaluation_output.next_x[index] =
          state.expected_x + history_anchor_offset_x;
      evaluation_output.next_y[index] =
          state.expected_y + history_anchor_offset_y;
    }
  }
  const std::vector<CartesianKinematicSample> evaluation_samples =
      ComputeCartesianKinematics(input, evaluation_output,
                                 limits.output_time_step_seconds,
                                 previous_path_size);
  diagnostics.cartesian_metrics_new_path_only =
      historical_plan_aligned && previous_path_size != 0;
  for (const CartesianKinematicSample &sample : evaluation_samples) {
    const std::size_t index = sample.output_index;
    if (diagnostics.cartesian_metrics_new_path_only &&
        index < previous_path_size) {
      continue;
    }
    UpdateMaximum(&diagnostics.cartesian_maximum_speed, sample.speed_mps,
                  index);
    UpdateMinimum(&diagnostics.cartesian_minimum_tangential_acceleration,
                  sample.tangential_acceleration_mps2, index);
    UpdateMaximum(&diagnostics.cartesian_maximum_tangential_acceleration,
                  sample.tangential_acceleration_mps2, index);
    UpdateMaximum(&diagnostics.cartesian_maximum_acceleration,
                  sample.acceleration_mps2, index);
    if (sample.jerk_valid) {
      UpdateMaximum(&diagnostics.cartesian_maximum_absolute_tangential_jerk,
                    std::fabs(sample.tangential_jerk_mps3), index);
      UpdateMaximum(&diagnostics.cartesian_maximum_jerk, sample.jerk_mps3,
                    index);
    }
  }
  if (previous_path_size < evaluation_samples.size()) {
    diagnostics.new_path_junction_speed.valid = true;
    diagnostics.new_path_junction_speed.index = previous_path_size;
    diagnostics.new_path_junction_speed.value =
        evaluation_samples[previous_path_size].speed_mps;
  }

  diagnostics.qp_speed_violation =
      Below(diagnostics.qp_minimum_speed, -limits.speed_tolerance_mps) ||
      Above(diagnostics.qp_maximum_speed,
            limits.maximum_speed_mps + limits.speed_tolerance_mps);
  diagnostics.qp_acceleration_violation =
      Below(diagnostics.qp_minimum_acceleration,
            limits.minimum_acceleration_mps2 -
                limits.acceleration_tolerance_mps2) ||
      Above(diagnostics.qp_maximum_acceleration,
            limits.maximum_acceleration_mps2 +
                limits.acceleration_tolerance_mps2);
  diagnostics.qp_jerk_violation =
      Above(diagnostics.qp_maximum_absolute_jerk,
            limits.maximum_jerk_mps3 + limits.jerk_tolerance_mps3);
  diagnostics.qp_collision_violation = Below(
      diagnostics.qp_minimum_collision_margin,
      -limits.safety_tolerance_meters);

  diagnostics.cartesian_speed_violation =
      Above(diagnostics.cartesian_maximum_speed,
            limits.maximum_speed_mps + limits.speed_tolerance_mps);
  diagnostics.cartesian_acceleration_violation =
      Below(diagnostics.cartesian_minimum_tangential_acceleration,
            limits.minimum_acceleration_mps2 -
                limits.acceleration_tolerance_mps2) ||
      Above(diagnostics.cartesian_maximum_tangential_acceleration,
            limits.maximum_acceleration_mps2 +
                limits.acceleration_tolerance_mps2) ||
      Above(diagnostics.cartesian_maximum_acceleration,
            limits.maximum_cartesian_acceleration_mps2 +
                limits.acceleration_tolerance_mps2);
  diagnostics.cartesian_jerk_violation =
      Above(diagnostics.cartesian_maximum_absolute_tangential_jerk,
            limits.maximum_jerk_mps3 + limits.jerk_tolerance_mps3) ||
      Above(diagnostics.cartesian_maximum_jerk,
            limits.maximum_cartesian_jerk_mps3 + limits.jerk_tolerance_mps3);
  diagnostics.lane_deviation_violation =
      std::fabs(plan_start_d - lane_center_d) >
      limits.maximum_lateral_deviation_meters;
  return diagnostics;
}

PlannerRuntimeMonitor::PlannerRuntimeMonitor(const PlannerMonitorConfig &config)
    : config_(config) {
  ValidateMonitorConfig(config_);
  session_id_ = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

void PlannerRuntimeMonitor::EnsureCsvStreams() {
  if (csv_initialization_attempted_) {
    return;
  }
  csv_initialization_attempted_ = true;
  if (!config_.enabled || !config_.write_csv) {
    return;
  }
  if (!EnsureDirectory(config_.log_directory)) {
    std::cerr << "[MONITOR][WARNING] cannot create log directory: "
              << config_.log_directory << std::endl;
    return;
  }

  const std::string cycle_path =
      JoinPath(config_.log_directory, "planner_cycle.csv");
  const std::string point_path =
      JoinPath(config_.log_directory, "planner_points.csv");
  const std::string qp_path = JoinPath(config_.log_directory, "planner_qp.csv");
  cycle_csv_.open(cycle_path.c_str(), std::ios::out | std::ios::trunc);
  point_csv_.open(point_path.c_str(), std::ios::out | std::ios::trunc);
  qp_csv_.open(qp_path.c_str(), std::ios::out | std::ios::trunc);
  if (!cycle_csv_ || !point_csv_ || !qp_csv_) {
    cycle_csv_.close();
    point_csv_.close();
    qp_csv_.close();
    std::cerr << "[MONITOR][WARNING] cannot open CSV files in: "
              << config_.log_directory << std::endl;
    return;
  }
  cycle_csv_ << std::setprecision(12);
  point_csv_ << std::setprecision(12);
  qp_csv_ << std::setprecision(12);
  cycle_csv_
      << "session_id,cycle,previous_size,new_count,ego_speed_mps,"
           "initial_speed_mps,initial_acceleration_mps2,initial_jerk_mps3,"
           "plan_start_s,"
           "plan_start_d,lane_center_d,lateral_snap_m,history_state_aligned,"
           "lateral_transition_id,lateral_active,lateral_state_reset,"
           "lateral_state_aligned,lateral_rolling_replanned,"
           "lateral_rolling_replan_count,lateral_rolling_origin_m,"
           "lateral_transition_length_m,"
           "lateral_progress_m,lateral_remaining_m,lateral_residual_m,"
           "lateral_frontier_d,obstacle_count,"
           "nearest_obstacle_m,reference_first_mps,reference_min_mps,"
           "reference_last_mps,qp_status,qp_objective,"
           "qp_headway_slack_max_m,emergency,qp_v_min,"
           "qp_v_min_index,qp_v_max,qp_v_max_index,qp_a_min,qp_a_min_index,"
           "qp_a_max,qp_a_max_index,qp_abs_j_max,qp_abs_j_max_index,"
           "intrusion_speed_limit_min,intrusion_speed_limit_index,"
           "intrusion_limiting_obstacle_id,"
           "qp_headway_margin_min,qp_headway_index,"
           "qp_headway_limiting_obstacle_id,qp_collision_margin_min,"
           "qp_collision_index,qp_collision_limiting_obstacle_id,"
           "xy_metrics_new_path_only,history_xy_residual_max_m,"
           "history_xy_residual_max_index,"
           "xy_v_max,xy_v_max_index,xy_tan_a_min,xy_tan_a_min_index,"
           "xy_tan_a_max,xy_tan_a_max_index,xy_a_max,xy_a_max_index,"
           "xy_abs_tan_j_max,xy_abs_tan_j_max_index,xy_j_max,xy_j_max_index,"
           "junction_speed_mps,junction_index,violation_mask\n";
  point_csv_
      << "session_id,cycle,index,is_previous,x,y,vx_mps,vy_mps,speed_mps,"
           "ax_mps2,ay_mps2,acceleration_mps2,tangential_acceleration_mps2,"
           "jerk_valid,jx_mps3,jy_mps3,jerk_mps3,tangential_jerk_mps3,"
           "planned_s,planned_v_mps,planned_a_mps2,planned_j_mps3,"
           "lateral_state_valid,lateral_transition_id,lateral_progress_m,"
           "road_parameter_s,planned_d\n";
  qp_csv_ << "session_id,cycle,node,time_s,s,v_mps,a_mps2,j_mps3,"
             "reference_v_mps,intrusion_speed_limit_valid,"
             "min_intrusion_speed_limit_mps,intrusion_limiting_obstacle_id,"
             "headway_margin_valid,min_headway_margin_m,"
             "headway_limiting_obstacle_id,collision_margin_valid,"
             "min_collision_margin_m,collision_limiting_obstacle_id,"
             "emergency\n";
  csv_available_ = true;
}

void PlannerRuntimeMonitor::WriteCycleCsv(
    const PlannerCycleDiagnostics &diagnostics) {
  cycle_csv_
      << session_id_ << ',' << diagnostics.cycle << ','
      << diagnostics.previous_path_size << ',' << diagnostics.new_point_count
      << ',' << diagnostics.ego_speed_mps << ','
      << diagnostics.initial_speed_mps << ','
      << diagnostics.initial_acceleration_mps2 << ','
      << diagnostics.initial_jerk_mps3 << ','
      << diagnostics.plan_start_s << ',' << diagnostics.plan_start_d << ','
      << diagnostics.lane_center_d << ','
      << diagnostics.lane_center_d - diagnostics.plan_start_d << ','
      << static_cast<int>(diagnostics.historical_plan_aligned) << ','
      << diagnostics.lateral.transition_id << ','
      << static_cast<int>(diagnostics.lateral.transition_active) << ','
      << static_cast<int>(diagnostics.lateral.state_reset) << ','
      << static_cast<int>(diagnostics.lateral.state_aligned) << ','
      << static_cast<int>(diagnostics.lateral.rolling_replanned) << ','
      << diagnostics.lateral.rolling_replan_count << ','
      << diagnostics.lateral.rolling_origin_m << ','
      << diagnostics.lateral.transition_length_m << ','
      << diagnostics.lateral.progress_m << ','
      << diagnostics.lateral.remaining_m << ','
      << diagnostics.lateral.position_residual_m << ','
      << diagnostics.lateral.frontier_d << ','
      << diagnostics.relevant_obstacle_count << ','
      << MetricValueOrNan(diagnostics.nearest_obstacle_distance) << ','
      << diagnostics.reference_first_mps << ','
      << diagnostics.reference_minimum_mps << ','
      << diagnostics.reference_last_mps << ',' << CsvSafe(diagnostics.qp_status)
      << ',' << diagnostics.qp_objective << ','
      << diagnostics.qp_maximum_headway_slack_meters << ','
      << static_cast<int>(diagnostics.emergency) << ','
      << MetricValueOrNan(diagnostics.qp_minimum_speed) << ','
      << MetricIndexOrNegativeOne(diagnostics.qp_minimum_speed) << ','
      << MetricValueOrNan(diagnostics.qp_maximum_speed) << ','
      << MetricIndexOrNegativeOne(diagnostics.qp_maximum_speed) << ','
      << MetricValueOrNan(diagnostics.qp_minimum_acceleration) << ','
      << MetricIndexOrNegativeOne(diagnostics.qp_minimum_acceleration) << ','
      << MetricValueOrNan(diagnostics.qp_maximum_acceleration) << ','
      << MetricIndexOrNegativeOne(diagnostics.qp_maximum_acceleration) << ','
      << MetricValueOrNan(diagnostics.qp_maximum_absolute_jerk) << ','
      << MetricIndexOrNegativeOne(diagnostics.qp_maximum_absolute_jerk) << ','
      << MetricValueOrNan(diagnostics.minimum_intrusion_speed_limit) << ','
      << MetricIndexOrNegativeOne(diagnostics.minimum_intrusion_speed_limit)
      << ',' << diagnostics.intrusion_limiting_obstacle_id << ','
      << MetricValueOrNan(diagnostics.qp_minimum_headway_margin) << ','
      << MetricIndexOrNegativeOne(diagnostics.qp_minimum_headway_margin) << ','
      << diagnostics.qp_headway_limiting_obstacle_id << ','
      << MetricValueOrNan(diagnostics.qp_minimum_collision_margin) << ','
      << MetricIndexOrNegativeOne(diagnostics.qp_minimum_collision_margin)
      << ',' << diagnostics.qp_collision_limiting_obstacle_id << ','
      << static_cast<int>(diagnostics.cartesian_metrics_new_path_only) << ','
      << MetricValueOrNan(diagnostics.historical_path_position_residual) << ','
      << MetricIndexOrNegativeOne(
             diagnostics.historical_path_position_residual)
      << ','
      << MetricValueOrNan(diagnostics.cartesian_maximum_speed) << ','
      << MetricIndexOrNegativeOne(diagnostics.cartesian_maximum_speed) << ','
      << MetricValueOrNan(diagnostics.cartesian_minimum_tangential_acceleration)
      << ','
      << MetricIndexOrNegativeOne(
             diagnostics.cartesian_minimum_tangential_acceleration)
      << ','
      << MetricValueOrNan(diagnostics.cartesian_maximum_tangential_acceleration)
      << ','
      << MetricIndexOrNegativeOne(
             diagnostics.cartesian_maximum_tangential_acceleration)
      << ',' << MetricValueOrNan(diagnostics.cartesian_maximum_acceleration)
      << ','
      << MetricIndexOrNegativeOne(diagnostics.cartesian_maximum_acceleration)
      << ','
      << MetricValueOrNan(
             diagnostics.cartesian_maximum_absolute_tangential_jerk)
      << ','
      << MetricIndexOrNegativeOne(
             diagnostics.cartesian_maximum_absolute_tangential_jerk)
      << ',' << MetricValueOrNan(diagnostics.cartesian_maximum_jerk) << ','
      << MetricIndexOrNegativeOne(diagnostics.cartesian_maximum_jerk) << ','
      << MetricValueOrNan(diagnostics.new_path_junction_speed) << ','
      << MetricIndexOrNegativeOne(diagnostics.new_path_junction_speed) << ','
      << ViolationMask(diagnostics) << '\n';
}

void PlannerRuntimeMonitor::WriteDetailCsv(
    const PlannerCycleDiagnostics &diagnostics) {
  for (const CartesianKinematicSample &sample : diagnostics.cartesian_samples) {
    const bool state_valid =
        sample.output_index < diagnostics.output_longitudinal_states.size();
    const LongitudinalState state =
        state_valid
            ? diagnostics.output_longitudinal_states[sample.output_index]
            : LongitudinalState();
    const bool lateral_state_valid =
        sample.output_index < diagnostics.output_lateral_states.size() &&
        diagnostics.output_lateral_states[sample.output_index].valid;
    const LateralPathState lateral_state =
        lateral_state_valid
            ? diagnostics.output_lateral_states[sample.output_index]
            : LateralPathState();
    point_csv_
        << session_id_ << ',' << diagnostics.cycle << ',' << sample.output_index
        << ',' << static_cast<int>(sample.is_previous_path) << ',' << sample.x
        << ',' << sample.y << ',' << sample.velocity_x_mps << ','
        << sample.velocity_y_mps << ',' << sample.speed_mps << ','
        << sample.acceleration_x_mps2 << ',' << sample.acceleration_y_mps2
        << ',' << sample.acceleration_mps2 << ','
        << sample.tangential_acceleration_mps2 << ','
        << static_cast<int>(sample.jerk_valid) << ',' << sample.jerk_x_mps3
        << ',' << sample.jerk_y_mps3 << ',' << sample.jerk_mps3 << ','
        << sample.tangential_jerk_mps3 << ','
        << (state_valid ? state.s : std::numeric_limits<double>::quiet_NaN())
        << ','
        << (state_valid ? state.v : std::numeric_limits<double>::quiet_NaN())
        << ','
        << (state_valid ? state.a : std::numeric_limits<double>::quiet_NaN())
        << ','
        << (state_valid ? state.j : std::numeric_limits<double>::quiet_NaN())
        << ',' << static_cast<int>(lateral_state_valid) << ','
        << (lateral_state_valid
                ? static_cast<double>(lateral_state.transition_id)
                : std::numeric_limits<double>::quiet_NaN())
        << ','
        << (lateral_state_valid
                ? lateral_state.correction_progress_m
                : std::numeric_limits<double>::quiet_NaN())
        << ','
        << (lateral_state_valid
                ? lateral_state.road_parameter_s
                : std::numeric_limits<double>::quiet_NaN())
        << ','
        << (lateral_state_valid ? lateral_state.planned_d
                                : std::numeric_limits<double>::quiet_NaN())
        << '\n';
  }
  for (const QpNodeMonitorSample &sample : diagnostics.qp_samples) {
    qp_csv_ << session_id_ << ',' << diagnostics.cycle << ','
            << sample.node_index << ',' << sample.time_seconds << ','
            << sample.state.s << ',' << sample.state.v << ',' << sample.state.a
            << ',' << sample.state.j << ',' << sample.reference_speed_mps << ','
            << static_cast<int>(sample.intrusion_speed_limit_valid) << ','
            << (sample.intrusion_speed_limit_valid
                    ? sample.minimum_intrusion_speed_limit_mps
                    : std::numeric_limits<double>::quiet_NaN())
            << ','
            << (sample.intrusion_speed_limit_valid
                    ? sample.intrusion_limiting_obstacle_id
                    : std::numeric_limits<double>::quiet_NaN())
            << ','
            << static_cast<int>(sample.headway_margin_valid) << ','
            << (sample.headway_margin_valid
                    ? sample.minimum_headway_margin_meters
                    : std::numeric_limits<double>::quiet_NaN())
            << ','
            << (sample.headway_margin_valid
                    ? sample.headway_limiting_obstacle_id
                    : std::numeric_limits<double>::quiet_NaN())
            << ',' << static_cast<int>(sample.collision_margin_valid) << ','
            << (sample.collision_margin_valid
                    ? sample.minimum_collision_margin_meters
                    : std::numeric_limits<double>::quiet_NaN())
            << ','
            << (sample.collision_margin_valid
                    ? sample.collision_limiting_obstacle_id
                    : std::numeric_limits<double>::quiet_NaN())
            << ',' << static_cast<int>(diagnostics.emergency) << '\n';
  }
}

void PlannerRuntimeMonitor::PrintSummary(
    const PlannerCycleDiagnostics &diagnostics) const {
  std::ostringstream line;
  line << std::fixed << std::setprecision(3)
       << "[MONITOR][CYCLE] session=" << session_id_
       << " cycle=" << diagnostics.cycle
       << " prev/new=" << diagnostics.previous_path_size << '/'
       << diagnostics.new_point_count << " ego=" << diagnostics.ego_speed_mps
       << "m/s(" << diagnostics.ego_speed_mps * kMetersPerSecondToMilesPerHour
       << "mph) init(v/a)=" << diagnostics.initial_speed_mps << '/'
       << diagnostics.initial_acceleration_mps2
       << " ref(first/min/last)=" << diagnostics.reference_first_mps << '/'
       << diagnostics.reference_minimum_mps << '/'
       << diagnostics.reference_last_mps << " init_j="
       << diagnostics.initial_jerk_mps3 << " headway_slack_max="
       << diagnostics.qp_maximum_headway_slack_meters
       << " intrusion_v_limit="
       << MetricText(diagnostics.minimum_intrusion_speed_limit)
       << " qp(vmax/amin/amax/jmax)="
       << MetricText(diagnostics.qp_maximum_speed) << '/'
       << MetricText(diagnostics.qp_minimum_acceleration) << '/'
       << MetricText(diagnostics.qp_maximum_acceleration) << '/'
       << MetricText(diagnostics.qp_maximum_absolute_jerk)
       << " qp_margin(headway/collision)="
       << MetricText(diagnostics.qp_minimum_headway_margin) << '/'
       << MetricText(diagnostics.qp_minimum_collision_margin)
       << " xy(v/a/j)=" << MetricText(diagnostics.cartesian_maximum_speed)
       << '/' << MetricText(diagnostics.cartesian_maximum_acceleration) << '/'
       << MetricText(diagnostics.cartesian_maximum_jerk)
       << " history_xy_res="
       << MetricText(diagnostics.historical_path_position_residual)
       << " junction_v=" << MetricText(diagnostics.new_path_junction_speed)
       << " lateral(id/p/rem/res/reset/roll/count)="
       << diagnostics.lateral.transition_id << '/'
       << diagnostics.lateral.progress_m << '/'
       << diagnostics.lateral.remaining_m << '/'
       << diagnostics.lateral.position_residual_m << '/'
       << static_cast<int>(diagnostics.lateral.state_reset) << '/'
       << static_cast<int>(diagnostics.lateral.rolling_replanned) << '/'
       << diagnostics.lateral.rolling_replan_count
       << " obstacles=" << diagnostics.relevant_obstacle_count
       << " emergency=" << static_cast<int>(diagnostics.emergency)
       << " status=" << diagnostics.qp_status;
  std::cerr << line.str() << std::endl;
}

void PlannerRuntimeMonitor::PrintViolation(
    const PlannerCycleDiagnostics &diagnostics) const {
  const unsigned int mask = ViolationMask(diagnostics);
  std::ostringstream line;
  line << std::fixed << std::setprecision(3)
       << "[MONITOR][VIOLATION] session=" << session_id_
       << " cycle=" << diagnostics.cycle << " flags=" << ViolationNames(mask)
       << " prev/new=" << diagnostics.previous_path_size << '/'
       << diagnostics.new_point_count
       << " lane_d(start/center)=" << diagnostics.plan_start_d << '/'
       << diagnostics.lane_center_d
       << " qp(vmax/amin/amax/jmax/headway/collision)="
       << MetricText(diagnostics.qp_maximum_speed) << '/'
       << MetricText(diagnostics.qp_minimum_acceleration) << '/'
       << MetricText(diagnostics.qp_maximum_acceleration) << '/'
       << MetricText(diagnostics.qp_maximum_absolute_jerk) << '/'
       << MetricText(diagnostics.qp_minimum_headway_margin) << '/'
       << MetricText(diagnostics.qp_minimum_collision_margin)
       << " xy(v/tan_a/a/tan_j/j)="
       << MetricText(diagnostics.cartesian_maximum_speed) << '/'
       << MetricText(diagnostics.cartesian_maximum_tangential_acceleration)
       << '/' << MetricText(diagnostics.cartesian_maximum_acceleration) << '/'
       << MetricText(diagnostics.cartesian_maximum_absolute_tangential_jerk)
       << '/' << MetricText(diagnostics.cartesian_maximum_jerk)
       << " history_xy_res="
       << MetricText(diagnostics.historical_path_position_residual)
       << " junction_v=" << MetricText(diagnostics.new_path_junction_speed)
       << " lateral(id/p/rem/res/reset/roll/count)="
       << diagnostics.lateral.transition_id << '/'
       << diagnostics.lateral.progress_m << '/'
       << diagnostics.lateral.remaining_m << '/'
       << diagnostics.lateral.position_residual_m << '/'
       << static_cast<int>(diagnostics.lateral.state_reset) << '/'
       << static_cast<int>(diagnostics.lateral.rolling_replanned) << '/'
       << diagnostics.lateral.rolling_replan_count
       << " emergency=" << static_cast<int>(diagnostics.emergency);
  std::cerr << line.str() << std::endl;
}

void PlannerRuntimeMonitor::Record(const PlannerCycleDiagnostics &diagnostics) {
  if (!config_.enabled) {
    return;
  }

  const unsigned int mask = ViolationMask(diagnostics);
  const bool new_violation_type = (mask & ~previous_violation_mask_) != 0U;
  const bool report_interval_elapsed =
      config_.violation_report_interval_cycles != 0 &&
      diagnostics.cycle >= last_violation_report_cycle_ +
                               config_.violation_report_interval_cycles;
  const bool report_violation =
      mask != 0U && (last_violation_report_cycle_ == 0 || new_violation_type ||
                     report_interval_elapsed);
  const bool periodic_summary =
      diagnostics.cycle == 1 ||
      (config_.console_summary_interval_cycles != 0 &&
       diagnostics.cycle % config_.console_summary_interval_cycles == 0);
  const bool periodic_detail =
      diagnostics.cycle == 1 ||
      (config_.detail_csv_interval_cycles != 0 &&
       diagnostics.cycle % config_.detail_csv_interval_cycles == 0);

  if (periodic_summary) {
    PrintSummary(diagnostics);
  }
  if (report_violation) {
    PrintViolation(diagnostics);
    last_violation_report_cycle_ = diagnostics.cycle;
  }

  EnsureCsvStreams();
  if (csv_available_) {
    WriteCycleCsv(diagnostics);
    if (periodic_detail || report_violation) {
      WriteDetailCsv(diagnostics);
    }
    if (periodic_summary || report_violation) {
      cycle_csv_.flush();
      point_csv_.flush();
      qp_csv_.flush();
    }
  }
  previous_violation_mask_ = mask;
}
