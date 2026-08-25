#include "planner.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "map.h"
#include "path_stitcher.h"
#include "traffic_predictor.h"
#include "trajectory_sampler.h"

namespace {

constexpr double kMilesPerHourToMetersPerSecond = 0.44704;
constexpr double kHistoryAlignmentToleranceMeters = 0.02;
constexpr double kEgoContinuationToleranceMeters = 0.2;

bool IsFiniteVector(const std::vector<double> &values) {
  for (double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

double TargetSpeedMps(const PlannerConfig &config) {
  return config.target_speed_mph * kMilesPerHourToMetersPerSecond;
}

LongitudinalQpConfig MakeLongitudinalConfig(const PlannerConfig &config) {
  LongitudinalQpConfig result;
  result.horizon_steps = config.qp_horizon_steps;
  result.time_step_seconds = config.qp_time_step_seconds;
  result.maximum_speed_mps = TargetSpeedMps(config);
  result.minimum_acceleration_mps2 = config.min_acceleration_mps2;
  result.maximum_acceleration_mps2 = config.max_acceleration_mps2;
  result.maximum_jerk_mps3 = config.max_jerk_mps3;
  result.initial_jerk_continuity_weight = config.initial_jerk_continuity_weight;
  result.time_headway_seconds = config.time_headway_seconds;
  result.standstill_gap_meters = config.standstill_gap_meters;
  result.ego_length_meters = config.ego_length_meters;
  result.obstacle_length_meters = config.obstacle_length_meters;
  result.prediction_margin_meters = config.prediction_margin_meters;
  result.headway_slack_weight = config.headway_slack_weight;
  return result;
}

SpeedReferenceConfig MakeReferenceConfig(const PlannerConfig &config) {
  SpeedReferenceConfig result;
  result.horizon_steps = config.qp_horizon_steps;
  result.time_step_seconds = config.qp_time_step_seconds;
  result.maximum_speed_mps = TargetSpeedMps(config);
  result.minimum_acceleration_mps2 = config.reference_min_acceleration_mps2;
  result.maximum_jerk_mps3 = config.reference_max_jerk_mps3;
  result.time_headway_seconds = config.time_headway_seconds;
  result.gap_closing_time_seconds = config.gap_closing_time_seconds;
  result.standstill_gap_meters = config.standstill_gap_meters;
  result.ego_length_meters = config.ego_length_meters;
  result.obstacle_length_meters = config.obstacle_length_meters;
  result.prediction_margin_meters = config.prediction_margin_meters;
  return result;
}

TrafficPredictionConfig MakeTrafficConfig(const PlannerConfig &config) {
  TrafficPredictionConfig result;
  result.simulator_time_step_seconds = config.time_step_seconds;
  result.horizon_steps = config.qp_horizon_steps;
  result.maximum_speed_mps = TargetSpeedMps(config);
  result.lane_width_meters = config.lane_width_meters;
  result.lane_boundary_margin_meters = config.lane_boundary_margin_meters;
  result.ego_width_meters = config.ego_width_meters;
  result.obstacle_width_meters = config.obstacle_width_meters;
  result.lookahead_distance_meters = config.traffic_lookahead_meters;
  return result;
}

PlannerMonitorLimits MakeMonitorLimits(const PlannerConfig &config) {
  PlannerMonitorLimits result;
  result.output_time_step_seconds = config.time_step_seconds;
  result.maximum_speed_mps = TargetSpeedMps(config);
  result.minimum_acceleration_mps2 = config.min_acceleration_mps2;
  result.maximum_acceleration_mps2 = config.max_acceleration_mps2;
  result.maximum_jerk_mps3 = config.max_jerk_mps3;
  result.maximum_cartesian_acceleration_mps2 =
      config.monitor.maximum_cartesian_acceleration_mps2;
  result.maximum_cartesian_jerk_mps3 =
      config.monitor.maximum_cartesian_jerk_mps3;
  result.speed_tolerance_mps = config.monitor.speed_tolerance_mps;
  result.acceleration_tolerance_mps2 =
      config.monitor.acceleration_tolerance_mps2;
  result.jerk_tolerance_mps3 = config.monitor.jerk_tolerance_mps3;
  result.safety_tolerance_meters = config.monitor.safety_tolerance_meters;
  result.maximum_lateral_deviation_meters =
      0.5 * config.lane_width_meters - config.lane_boundary_margin_meters;
  result.time_headway_seconds = config.time_headway_seconds;
  result.fixed_headway_gap_meters =
      config.standstill_gap_meters + config.prediction_margin_meters +
      0.5 * (config.ego_length_meters + config.obstacle_length_meters);
  result.collision_gap_meters =
      0.5 * (config.ego_length_meters + config.obstacle_length_meters);
  return result;
}

void ValidatePlannerConfig(const PlannerConfig &config) {
  if (config.output_points == 0 || config.time_step_seconds <= 0.0 ||
      config.target_speed_mph <= 0.0 || config.qp_horizon_steps == 0 ||
      config.qp_time_step_seconds <= 0.0 ||
      config.qp_time_step_seconds < config.time_step_seconds ||
      config.min_acceleration_mps2 >= 0.0 ||
      config.max_acceleration_mps2 <= 0.0 || config.max_jerk_mps3 <= 0.0 ||
      config.initial_jerk_continuity_weight < 0.0 ||
      config.reference_min_acceleration_mps2 >= 0.0 ||
      config.reference_max_jerk_mps3 <= 0.0 ||
      config.reference_min_acceleration_mps2 < config.min_acceleration_mps2 ||
      config.reference_max_jerk_mps3 > config.max_jerk_mps3 ||
      config.time_headway_seconds < 0.0 || config.standstill_gap_meters < 0.0 ||
      !std::isfinite(config.gap_closing_time_seconds) ||
      config.gap_closing_time_seconds <= 0.0 ||
      config.ego_length_meters <= 0.0 || config.obstacle_length_meters <= 0.0 ||
      config.ego_width_meters <= 0.0 || config.obstacle_width_meters <= 0.0 ||
      config.prediction_margin_meters < 0.0 ||
      config.headway_slack_weight <= 0.0 ||
      config.traffic_lookahead_meters <= 0.0 ||
      config.lane_boundary_margin_meters < 0.0 ||
      config.lane_width_meters <= 0.0 || config.lane_count <= 0 ||
      config.lane_boundary_margin_meters >= 0.5 * config.lane_width_meters ||
      static_cast<double>(config.output_points) * config.time_step_seconds >
          static_cast<double>(config.qp_horizon_steps) *
                  config.qp_time_step_seconds +
              1e-9) {
    throw std::invalid_argument("invalid planner configuration");
  }
}

LongitudinalState TelemetryInitialState(const PlannerInput &input) {
  LongitudinalState state;
  state.v = std::max(0.0, input.ego.speed_mph * kMilesPerHourToMetersPerSecond);
  return state;
}

LongitudinalState ColdStartInitialState(const PlannerInput &input,
                                        const PlannerConfig &config) {
  LongitudinalState state = TelemetryInitialState(input);
  const std::size_t size = input.previous_path_x.size();
  if (size < 2) {
    return state;
  }

  // This branch is used only when no matching locally planned state queue
  // exists (for example, process startup with an external previous_path).
  // The QP starts at the historical endpoint, so recover that endpoint's speed
  // even when it legitimately differs from the current-time telemetry speed.
  // Normal replanning inherits exact saved states and never differentiates
  // quantized points.
  const double last_speed =
      std::hypot(
          input.previous_path_x[size - 1] - input.previous_path_x[size - 2],
          input.previous_path_y[size - 1] - input.previous_path_y[size - 2]) /
      config.time_step_seconds;
  if (std::isfinite(last_speed)) {
    state.v = std::max(0.0, last_speed);
  }
  if (size < 3) {
    return state;
  }
  const double previous_speed =
      std::hypot(
          input.previous_path_x[size - 2] - input.previous_path_x[size - 3],
          input.previous_path_y[size - 2] - input.previous_path_y[size - 3]) /
      config.time_step_seconds;
  const double acceleration =
      (last_speed - previous_speed) / config.time_step_seconds;
  const double maximum_abs_acceleration =
      std::max(std::fabs(config.min_acceleration_mps2),
               std::fabs(config.max_acceleration_mps2));
  if (std::isfinite(acceleration) &&
      std::fabs(acceleration) <= maximum_abs_acceleration + 0.5) {
    state.a = std::max(config.min_acceleration_mps2,
                       std::min(acceleration, config.max_acceleration_mps2));
  }
  return state;
}

bool HistoricalPlanAligned(const PlannerInput &input,
                           const std::vector<double> &last_output_x,
                           const std::vector<double> &last_output_y,
                           std::size_t output_points) {
  const std::size_t previous_size = input.previous_path_x.size();
  if (last_output_x.size() != output_points ||
      last_output_y.size() != output_points || previous_size > output_points) {
    return false;
  }
  if (previous_size == 0) {
    return std::hypot(input.ego.x - last_output_x.back(),
                      input.ego.y - last_output_y.back()) <=
           kEgoContinuationToleranceMeters;
  }
  const std::size_t consumed = output_points - previous_size;
  for (std::size_t index = 0; index < previous_size; ++index) {
    if (std::hypot(
            input.previous_path_x[index] - last_output_x[consumed + index],
            input.previous_path_y[index] - last_output_y[consumed + index]) >
        kHistoryAlignmentToleranceMeters) {
      return false;
    }
  }
  return true;
}

std::vector<double> ReferenceSpeeds(const SpeedReferenceResult &reference,
                                    std::size_t expected_size,
                                    double fallback_speed) {
  std::vector<double> speeds;
  if (reference.success &&
      reference.trajectory.states.size() == expected_size) {
    speeds.reserve(expected_size);
    for (const LongitudinalState &state : reference.trajectory.states) {
      speeds.push_back(state.v);
    }
  } else if (reference.raw_speed_limits_mps.size() == expected_size) {
    speeds = reference.raw_speed_limits_mps;
  } else {
    speeds.assign(expected_size, fallback_speed);
  }
  return speeds;
}

} // namespace

PathPlanner::PathPlanner(const PlannerConfig &config)
    : config_(config), longitudinal_qp_(MakeLongitudinalConfig(config)),
      speed_reference_generator_(MakeReferenceConfig(config)),
      runtime_monitor_(config.monitor) {
  ValidatePlannerConfig(config_);
}

PlannerOutput PathPlanner::Plan(const PlannerInput &input, const MapData &map) {
  std::string map_error;
  if (!ValidateMap(map, &map_error)) {
    throw std::invalid_argument("invalid map: " + map_error);
  }
  if (input.previous_path_x.size() != input.previous_path_y.size()) {
    throw std::invalid_argument(
        "previous path x/y arrays have different lengths");
  }
  if (input.previous_path_x.size() > config_.output_points) {
    throw std::invalid_argument("previous path is longer than planner output");
  }
  if (!IsFiniteVector(input.previous_path_x) ||
      !IsFiniteVector(input.previous_path_y) || !std::isfinite(input.ego.x) ||
      !std::isfinite(input.ego.y) || !std::isfinite(input.ego.s) ||
      !std::isfinite(input.ego.d) || !std::isfinite(input.ego.yaw_deg) ||
      !std::isfinite(input.ego.speed_mph) || !std::isfinite(input.end_path_s) ||
      !std::isfinite(input.end_path_d)) {
    throw std::invalid_argument("planner input contains a non-finite value");
  }

  const std::size_t previous_size = input.previous_path_x.size();
  const bool has_previous_path = previous_size != 0;
  const double plan_start_s =
      has_previous_path ? input.end_path_s : input.ego.s;
  const double current_d = has_previous_path ? input.end_path_d : input.ego.d;
  if (target_lane_ < 0) {
    target_lane_ =
        static_cast<int>(std::floor(input.ego.d / config_.lane_width_meters));
    target_lane_ = std::max(0, std::min(target_lane_, config_.lane_count - 1));
  }
  const double lane_center_d =
      (static_cast<double>(target_lane_) + 0.5) * config_.lane_width_meters;

  std::vector<LongitudinalState> retained_states;
  const bool historical_plan_aligned = HistoricalPlanAligned(
      input, last_output_x_, last_output_y_, config_.output_points);
  LongitudinalState initial_state = historical_plan_aligned
                                        ? TelemetryInitialState(input)
                                        : ColdStartInitialState(input, config_);
  if (historical_plan_aligned &&
      last_output_states_.size() == config_.output_points) {
    const std::size_t consumed = last_output_states_.size() - previous_size;
    retained_states.assign(last_output_states_.begin() + consumed,
                           last_output_states_.end());
    if (!retained_states.empty()) {
      initial_state.v = retained_states.back().v;
      initial_state.a = retained_states.back().a;
      initial_state.j = retained_states.back().j;
    } else {
      initial_state.v = last_output_states_.back().v;
      initial_state.a = last_output_states_.back().a;
      initial_state.j = last_output_states_.back().j;
    }
  }
  if (retained_states.size() != previous_size) {
    retained_states.assign(previous_size, initial_state);
  }

  const std::vector<PredictedObstacle> obstacles =
      PredictRelevantTraffic(input, plan_start_s, current_d, lane_center_d, map,
                             MakeTrafficConfig(config_));
  const SpeedReferenceResult reference = speed_reference_generator_.Generate(
      obstacles, std::max(0.0, initial_state.v));
  const std::size_t reference_size = config_.qp_horizon_steps + 1;

  LongitudinalQpInput qp_input;
  qp_input.initial_speed_mps = std::max(0.0, initial_state.v);
  qp_input.initial_acceleration_mps2 = initial_state.a;
  qp_input.initial_jerk_valid = historical_plan_aligned;
  qp_input.initial_jerk_mps3 = initial_state.j;
  qp_input.reference_speed_mps =
      ReferenceSpeeds(reference, reference_size, TargetSpeedMps(config_));
  qp_input.obstacles = obstacles;

  std::vector<double> solved_reference_speed_mps = qp_input.reference_speed_mps;
  LongitudinalQpResult solved = longitudinal_qp_.Solve(qp_input);
  if (!solved.success) {
    // A hard body-collision boundary may already be impossible at the
    // immutable end of the previous path. Generate the strongest feasible stop
    // allowed by the same acceleration and jerk constraints.
    LongitudinalQpInput emergency_input = qp_input;
    emergency_input.reference_speed_mps.assign(reference_size, 0.0);
    emergency_input.obstacles.clear();
    emergency_input.emergency_stop = true;
    solved = longitudinal_qp_.Solve(emergency_input);
    solved.trajectory.emergency = solved.success;
    solved_reference_speed_mps = emergency_input.reference_speed_mps;
  }
  if (!solved.success) {
    throw std::runtime_error("longitudinal QP failed: " + solved.status);
  }
  last_plan_emergency_ = solved.trajectory.emergency;

  const std::size_t missing_points = config_.output_points - previous_size;
  const std::vector<LongitudinalState> new_states = SampleTrajectory(
      solved.trajectory, config_.time_step_seconds, missing_points);

  PlannerOutput output;
  output.next_x = input.previous_path_x;
  output.next_y = input.previous_path_y;
  output.next_x.reserve(config_.output_points);
  output.next_y.reserve(config_.output_points);
  const StitchedRoadPathResult stitched_path = path_stitcher_.Sample(
      input, plan_start_s, lane_center_d, new_states,
      std::max(TargetSpeedMps(config_), initial_state.v), map);
  for (const std::pair<double, double> &xy : stitched_path.new_points) {
    output.next_x.push_back(xy.first);
    output.next_y.push_back(xy.second);
  }

  if (output.next_x.size() != config_.output_points ||
      output.next_y.size() != config_.output_points) {
    throw std::logic_error("planner produced the wrong number of points");
  }

  retained_states.insert(retained_states.end(), new_states.begin(),
                         new_states.end());
  if (retained_states.size() != config_.output_points) {
    throw std::logic_error("planner state queue has the wrong size");
  }
  last_output_states_ = retained_states;
  last_output_x_ = output.next_x;
  last_output_y_ = output.next_y;
  if (!new_states.empty()) {
    reference_speed_mps_ = new_states.back().v;
  } else {
    reference_speed_mps_ = initial_state.v;
  }

  ++plan_cycle_;
  last_diagnostics_ = BuildPlannerDiagnostics(
      plan_cycle_, input, output, previous_size, initial_state, plan_start_s,
      current_d, lane_center_d, obstacles, solved_reference_speed_mps, solved,
      last_output_states_, MakeMonitorLimits(config_),
      stitched_path.diagnostics, stitched_path.output_states,
      historical_plan_aligned);
  runtime_monitor_.Record(last_diagnostics_);
  return output;
}
