#include "planner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "map.h"
#include "traffic_predictor.h"
#include "trajectory_assembler.h"

namespace {

constexpr double kMilesPerHourToMetersPerSecond = 0.44704;
constexpr double kHistoryAlignmentToleranceMeters = 0.02;
constexpr double kEgoContinuationToleranceMeters = 0.2;
constexpr std::size_t kProductionMaximumRetainedPathPoints = 15;

double ElapsedMilliseconds(
    const std::chrono::steady_clock::time_point &start) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
      .count();
}

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

double PlanningHorizonS(const PlannerConfig &config) {
  return static_cast<double>(config.qp_horizon_steps) *
         config.qp_time_step_seconds;
}

std::size_t FullTrajectorySampleCount(const PlannerConfig &config) {
  return static_cast<std::size_t>(
      std::llround(PlanningHorizonS(config) / config.time_step_seconds));
}

LongitudinalQpConfig MakeLongitudinalConfig(const PlannerConfig &config) {
  LongitudinalQpConfig result;
  result.horizon_steps = config.qp_horizon_steps;
  result.time_step_seconds = config.qp_time_step_seconds;
  if (config.time_step_seconds > 0.0 &&
      config.qp_time_step_seconds >= config.time_step_seconds) {
    result.intermediate_speed_substeps = static_cast<std::size_t>(
        std::llround(config.qp_time_step_seconds /
                     config.time_step_seconds));
  }
  result.maximum_speed_mps = TargetSpeedMps(config);
  result.minimum_acceleration_mps2 = config.min_acceleration_mps2;
  result.maximum_acceleration_mps2 = config.max_acceleration_mps2;
  result.maximum_jerk_mps3 = config.max_jerk_mps3;
  result.initial_jerk_continuity_weight =
      config.initial_jerk_continuity_weight;
  result.time_headway_seconds = config.time_headway_seconds;
  result.standstill_gap_meters = config.standstill_gap_meters;
  result.ego_length_meters = config.ego_length_meters;
  result.obstacle_length_meters = config.obstacle_length_meters;
  result.prediction_margin_meters = config.prediction_margin_meters;
  result.physical_collision_margin_meters =
      config.physical_collision_margin_meters;
  result.headway_slack_weight = config.headway_slack_weight;
  result.collision_violation_weight = config.collision_violation_weight;
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
  result.qp_time_step_seconds = config.qp_time_step_seconds;
  result.horizon_steps = config.qp_horizon_steps;
  result.maximum_speed_mps = TargetSpeedMps(config);
  result.lane_width_meters = config.lane_width_meters;
  result.lane_boundary_margin_meters = config.lane_boundary_margin_meters;
  result.ego_width_meters = config.ego_width_meters;
  result.obstacle_width_meters = config.obstacle_width_meters;
  result.lookahead_distance_meters = config.traffic_lookahead_meters;
  return result;
}

TrajectoryValidatorConfig MakeValidatorConfig(const PlannerConfig &config) {
  TrajectoryValidatorConfig result;
  result.time_step_s = config.time_step_seconds;
  result.maximum_speed_mps = TargetSpeedMps(config);
  result.minimum_acceleration_mps2 = config.min_acceleration_mps2;
  result.maximum_acceleration_mps2 = config.max_acceleration_mps2;
  result.maximum_jerk_mps3 = config.max_jerk_mps3;
  result.maximum_cartesian_acceleration_mps2 =
      config.monitor.maximum_cartesian_acceleration_mps2;
  result.maximum_cartesian_jerk_mps3 =
      config.validator_maximum_cartesian_jerk_mps3;
  result.speed_tolerance_mps = config.monitor.speed_tolerance_mps;
  result.acceleration_tolerance_mps2 =
      config.monitor.acceleration_tolerance_mps2;
  result.jerk_tolerance_mps3 = config.monitor.jerk_tolerance_mps3;
  result.safety_tolerance_m = config.monitor.safety_tolerance_meters;
  result.ego_length_m = config.ego_length_meters;
  result.ego_width_m = config.ego_width_meters;
  result.obstacle_length_m = config.obstacle_length_meters;
  result.obstacle_width_m = config.obstacle_width_meters;
  result.physical_collision_margin_m =
      config.physical_collision_margin_meters;
  result.lane_boundary_margin_m = config.lane_boundary_margin_meters;
  result.lane_width_m = config.lane_width_meters;
  result.lane_count = config.lane_count;
  return result;
}

ActiveBehaviorPlannerConfig
MakeActiveBehaviorConfig(const PlannerConfig &config) {
  ActiveBehaviorPlannerConfig result = config.active_behavior;
  result.longitudinal_qp = MakeLongitudinalConfig(config);
  result.minimum_discretionary_acceleration_mps2 = std::max(
      result.minimum_discretionary_acceleration_mps2,
      config.min_acceleration_mps2);
  result.traffic_tracker.simulator_time_step_s = config.time_step_seconds;
  result.traffic_tracker.obstacle_length_m = config.obstacle_length_meters;
  result.traffic_tracker.obstacle_width_m = config.obstacle_width_meters;
  result.traffic_prediction.planning_horizon_s = PlanningHorizonS(config);
  result.traffic_prediction.lane_width_m = config.lane_width_meters;
  result.traffic_prediction.lane_count = config.lane_count;
  result.traffic_prediction.maximum_source_tracks =
      result.traffic_tracker.maximum_tracks;

  result.behavior_planner.simulator_time_step_s = config.time_step_seconds;
  result.behavior_planner.planning_horizon_s = PlanningHorizonS(config);
  result.behavior_planner.maximum_retained_prefix_s =
      static_cast<double>(config.retained_path.maximum_points) *
      config.time_step_seconds;
  result.behavior_planner.post_maneuver_observation_s =
      result.traffic_prediction.post_maneuver_observation_s;
  result.behavior_planner.maximum_longitudinal_acceleration_mps2 =
      config.max_acceleration_mps2;
  result.behavior_planner.minimum_longitudinal_acceleration_mps2 =
      config.min_acceleration_mps2;
  result.behavior_planner.target_speed_mps = TargetSpeedMps(config);
  result.behavior_planner.standstill_clearance_m =
      config.standstill_gap_meters;
  result.behavior_planner.physical_collision_margin_m =
      config.physical_collision_margin_meters;
  result.behavior_planner.ego_length_m = config.ego_length_meters;
  result.behavior_planner.ego_width_m = config.ego_width_meters;
  result.behavior_planner.lane_width_m = config.lane_width_meters;
  result.behavior_planner.lane_count = config.lane_count;

  result.spatial_path.planning_horizon_s = PlanningHorizonS(config);
  result.spatial_path.post_maneuver_observation_s =
      result.traffic_prediction.post_maneuver_observation_s;
  result.spatial_path.speed_upper_bound_mps = TargetSpeedMps(config);
  result.spatial_path.maximum_abs_longitudinal_acceleration_mps2 =
      std::max(std::fabs(config.min_acceleration_mps2),
               std::fabs(config.max_acceleration_mps2));
  // P2.4, not P2.3's all-path speed-upper-bound estimate, owns the hard
  // curvature/speed coupling. Keep the early Cartesian bound aligned with the
  // final validator so ordinary road curvature can be slowed by the node-wise
  // ST budget instead of rejecting the entire lane-change geometry.
  result.spatial_path.maximum_lateral_acceleration_mps2 = std::max(
      result.spatial_path.maximum_lateral_acceleration_mps2,
      config.monitor.maximum_cartesian_acceleration_mps2);
  result.spatial_path.maximum_lateral_jerk_mps3 = std::max(
      result.spatial_path.maximum_lateral_jerk_mps3,
      4.0 * config.validator_maximum_cartesian_jerk_mps3);
  result.spatial_path.ego_length_m = config.ego_length_meters;
  result.spatial_path.ego_width_m = config.ego_width_meters;
  result.spatial_path.lane_boundary_margin_m =
      config.lane_boundary_margin_meters;
  result.spatial_path.lane_width_m = config.lane_width_meters;
  result.spatial_path.lane_count = config.lane_count;
  // Keep an additional target-lane tail beyond the fresh QP horizon.  During
  // Settling, lane-cruise handoff can remain temporarily unavailable while
  // inherited lateral history clears its Cartesian jerk gate.  Without this
  // reserve the receding QP reaches the immutable path end on the first
  // target-only cycle and creates a false braking constraint.
  result.spatial_path.committed_continuation_horizon_s =
      PlanningHorizonS(config) +
      result.spatial_path.post_maneuver_observation_s;
  result.spatial_path.nominal_transition_duration_s = std::max(
      result.spatial_path.nominal_transition_duration_s, 5.0);
  result.spatial_path.use_frontier_speed_for_transition_length = true;
  // A committed path must cover its transition, a complete fresh horizon and
  // the settling reserve without regenerating or silently extrapolating
  // geometry beyond P2.3 evidence.
  const double committed_extent_m =
      TargetSpeedMps(config) *
          (result.spatial_path.committed_continuation_horizon_s +
           result.spatial_path.nominal_transition_duration_s) +
      config.ego_length_meters;
  result.spatial_path.maximum_path_extent_m = std::max(
      result.spatial_path.maximum_path_extent_m, committed_extent_m);
  const std::size_t required_geometry_samples =
      static_cast<std::size_t>(std::ceil(
          result.spatial_path.maximum_path_extent_m /
          result.spatial_path.geometry_sample_step_m)) +
      3;
  result.spatial_path.maximum_geometry_samples = std::max(
      result.spatial_path.maximum_geometry_samples,
      required_geometry_samples);

  result.st_corridor.qp_horizon_steps = config.qp_horizon_steps;
  result.st_corridor.qp_time_step_s = config.qp_time_step_seconds;
  result.st_corridor.post_maneuver_observation_s =
      result.traffic_prediction.post_maneuver_observation_s;
  result.st_corridor.maximum_speed_mps = TargetSpeedMps(config);
  result.st_corridor.minimum_longitudinal_acceleration_mps2 =
      config.min_acceleration_mps2;
  result.st_corridor.maximum_longitudinal_acceleration_mps2 =
      config.max_acceleration_mps2;
  result.st_corridor.lateral_acceleration_budget_mps2 = std::min(
      result.st_corridor.lateral_acceleration_budget_mps2,
      result.spatial_path.maximum_lateral_acceleration_mps2);
  result.st_corridor.standstill_clearance_m =
      config.standstill_gap_meters;
  result.st_corridor.physical_collision_margin_m =
      config.physical_collision_margin_meters;
  result.st_corridor.ego_length_m = config.ego_length_meters;
  result.st_corridor.ego_width_m = config.ego_width_meters;
  result.st_corridor.lane_count = config.lane_count;
  result.st_corridor.maximum_candidates =
      result.behavior_planner.maximum_candidates;

  result.trajectory_validator = MakeValidatorConfig(config);
  result.output_time_step_s = config.time_step_seconds;
  result.output_points = config.output_points;
  result.full_trajectory_sample_count =
      FullTrajectorySampleCount(config);
  result.target_speed_mps = TargetSpeedMps(config);
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
      config.validator_maximum_cartesian_jerk_mps3;
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
      config.physical_collision_margin_meters +
      0.5 * (config.ego_length_meters + config.obstacle_length_meters);
  result.collision_gap_meters =
      config.physical_collision_margin_meters +
      0.5 * (config.ego_length_meters + config.obstacle_length_meters);
  return result;
}

void ValidatePlannerConfig(const PlannerConfig &config) {
  const double full_sample_count =
      PlanningHorizonS(config) / config.time_step_seconds;
  const double qp_sample_ratio =
      config.qp_time_step_seconds / config.time_step_seconds;
  const bool supported_mode =
      config.operating_mode == PlannerOperatingMode::kLaneCruiseOnly ||
      config.operating_mode == PlannerOperatingMode::kBehaviorActive;
  if (!supported_mode || config.output_points == 0 ||
      config.time_step_seconds <= 0.0 ||
      config.target_speed_mph <= 0.0 || config.qp_horizon_steps == 0 ||
      config.qp_time_step_seconds <= 0.0 ||
      config.qp_time_step_seconds < config.time_step_seconds ||
      std::fabs(qp_sample_ratio - std::round(qp_sample_ratio)) > 1e-9 ||
      config.min_acceleration_mps2 >= 0.0 ||
      config.max_acceleration_mps2 <= 0.0 || config.max_jerk_mps3 <= 0.0 ||
      !std::isfinite(config.validator_maximum_cartesian_jerk_mps3) ||
      config.validator_maximum_cartesian_jerk_mps3 <= 0.0 ||
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
      config.physical_collision_margin_meters < 0.0 ||
      config.headway_slack_weight <= 0.0 ||
      config.collision_violation_weight <= 0.0 ||
      config.traffic_lookahead_meters <= 0.0 ||
      config.lane_boundary_margin_meters < 0.0 ||
      config.lane_width_meters <= 0.0 || config.lane_count <= 0 ||
      config.lane_boundary_margin_meters >= 0.5 * config.lane_width_meters ||
      !config.retained_path.enabled ||
      config.retained_path.maximum_points == 0 ||
      config.retained_path.maximum_points >
          kProductionMaximumRetainedPathPoints ||
      config.retained_path.maximum_points >= config.output_points ||
      config.retained_path.minimum_stitch_points == 0 ||
      config.retained_path.minimum_stitch_points >
          config.retained_path.maximum_points ||
      static_cast<double>(config.output_points) * config.time_step_seconds >
          PlanningHorizonS(config) + 1e-9 ||
      std::fabs(full_sample_count - std::round(full_sample_count)) > 1e-9) {
    throw std::invalid_argument("invalid planner configuration");
  }
}

void ValidatePlannerInput(const PlannerInput &input,
                          std::size_t maximum_previous_points) {
  if (input.previous_path_x.size() != input.previous_path_y.size()) {
    throw std::invalid_argument(
        "previous path x/y arrays have different lengths");
  }
  if (input.previous_path_x.size() > maximum_previous_points) {
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
  for (const DetectedVehicle &vehicle : input.traffic) {
    if (!std::isfinite(vehicle.id) || !std::isfinite(vehicle.x) ||
        !std::isfinite(vehicle.y) || !std::isfinite(vehicle.vx_mps) ||
        !std::isfinite(vehicle.vy_mps) || !std::isfinite(vehicle.s) ||
        !std::isfinite(vehicle.d)) {
      throw std::invalid_argument("traffic contains a non-finite value");
    }
  }
}

LongitudinalState TelemetryInitialState(const PlannerInput &input) {
  LongitudinalState state;
  state.v = std::max(0.0, input.ego.speed_mph *
                              kMilesPerHourToMetersPerSecond);
  return state;
}

LongitudinalState ColdStartInitialState(const PlannerInput &input,
                                        const PlannerConfig &config) {
  LongitudinalState state = TelemetryInitialState(input);
  const std::size_t size = input.previous_path_x.size();
  if (size < 2) {
    return state;
  }
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
    // Chord length over the last simulator interval yields that interval's
    // average speed, while the QP initial state is defined at the interval
    // endpoint. Advance by half an interval under the estimated acceleration
    // so the first newly sampled segment preserves discrete acceleration and
    // does not create an artificial stitching jerk.
    state.v = std::max(
        0.0, last_speed +
                 0.5 * state.a * config.time_step_seconds);
  }
  return state;
}

bool HistoricalPlanAligned(const PlannerInput &input,
                           const PlannerState &state,
                           std::size_t output_points,
                           PlanningStateResetReason *reset_reason) {
  const std::size_t previous_size = input.previous_path_x.size();
  if (state.output_x.size() != output_points ||
      state.output_y.size() != output_points ||
      state.output_longitudinal.size() != output_points ||
      state.output_lateral.size() != output_points ||
      state.path_stitcher.last_output_states.size() != output_points ||
      previous_size > output_points) {
    *reset_reason = state.output_x.empty()
                        ? PlanningStateResetReason::kNoCommittedHistory
                        : PlanningStateResetReason::kHistoryLengthMismatch;
    return false;
  }
  if (previous_size == 0) {
    const bool aligned =
        std::hypot(input.ego.x - state.output_x.back(),
                   input.ego.y - state.output_y.back()) <=
        kEgoContinuationToleranceMeters;
    *reset_reason = aligned ? PlanningStateResetReason::kNone
                            : PlanningStateResetReason::kHistoryPositionMismatch;
    return aligned;
  }
  const std::size_t consumed = output_points - previous_size;
  for (std::size_t index = 0; index < previous_size; ++index) {
    if (std::hypot(input.previous_path_x[index] -
                       state.output_x[consumed + index],
                   input.previous_path_y[index] -
                       state.output_y[consumed + index]) >
        kHistoryAlignmentToleranceMeters) {
      *reset_reason = PlanningStateResetReason::kHistoryPositionMismatch;
      return false;
    }
  }
  *reset_reason = PlanningStateResetReason::kNone;
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

double ForwardParameterSpan(double from_s, double to_s,
                            double track_length) {
  const double from = NormalizeS(from_s, track_length);
  const double to = NormalizeS(to_s, track_length);
  double span = to - from;
  if (span < 0.0) {
    span += track_length;
  }
  return span;
}

std::vector<LongitudinalState>
EstimateRetainedLongitudinalStates(const PlannerInput &bounded_input,
                                   const PlannerConfig &config) {
  std::vector<LongitudinalState> result;
  result.reserve(bounded_input.previous_path_x.size());
  double previous_x = bounded_input.ego.x;
  double previous_y = bounded_input.ego.y;
  double previous_speed =
      std::max(0.0, bounded_input.ego.speed_mph *
                        kMilesPerHourToMetersPerSecond);
  double previous_acceleration = 0.0;
  double accumulated_distance = 0.0;
  for (std::size_t index = 0; index < bounded_input.previous_path_x.size();
       ++index) {
    const double distance =
        std::hypot(bounded_input.previous_path_x[index] - previous_x,
                   bounded_input.previous_path_y[index] - previous_y);
    const double speed = distance / config.time_step_seconds;
    const double acceleration =
        std::max(config.min_acceleration_mps2,
                 std::min((speed - previous_speed) / config.time_step_seconds,
                          config.max_acceleration_mps2));
    const double jerk =
        std::max(-config.max_jerk_mps3,
                 std::min((acceleration - previous_acceleration) /
                              config.time_step_seconds,
                          config.max_jerk_mps3));
    accumulated_distance += distance;
    LongitudinalState state;
    state.s = accumulated_distance;
    state.v = speed;
    state.a = acceleration;
    state.j = jerk;
    result.push_back(state);
    previous_x = bounded_input.previous_path_x[index];
    previous_y = bounded_input.previous_path_y[index];
    previous_speed = speed;
    previous_acceleration = acceleration;
  }
  return result;
}

FallbackLevel FallbackForPolicy(LongitudinalSafetyPolicy policy) {
  switch (policy) {
  case LongitudinalSafetyPolicy::kNormalOperational:
    return FallbackLevel::kNormal;
  case LongitudinalSafetyPolicy::kDegradedBraking:
    return FallbackLevel::kDegradedBraking;
  case LongitudinalSafetyPolicy::kMaximumBraking:
    return FallbackLevel::kMaximumBraking;
  }
  return FallbackLevel::kInfrastructureFailure;
}

struct BuiltTrajectory {
  FullTrajectory full;
  PlannerOutput output;
  std::vector<LongitudinalState> output_longitudinal;
  std::vector<LateralPathState> output_lateral;
  PathStitcherState next_path_state;
  LateralStitchDiagnostics lateral_diagnostics;
};

BuiltTrajectory BuildFullTrajectory(
    const PlanningSnapshot &snapshot, const PlannerConfig &config,
    const MapData &map, const FixedSpatialPath &fixed_path,
    const LongitudinalQpResult &qp, const PathStitcher &path_stitcher) {
  TrajectoryCanonicalizationConfig assembly_config;
  assembly_config.sample_time_step_s = config.time_step_seconds;
  assembly_config.sample_count = FullTrajectorySampleCount(config);
  assembly_config.minimum_acceleration_mps2 = config.min_acceleration_mps2;
  assembly_config.maximum_acceleration_mps2 = config.max_acceleration_mps2;
  assembly_config.maximum_jerk_mps3 = config.max_jerk_mps3;
  assembly_config.invalid_trajectory_message =
      "QP trajectory exceeds canonicalization tolerance";
  const std::vector<LongitudinalState> sampled =
      SampleCanonicalTrajectory(qp, assembly_config);
  const StitchedRoadPathResult stitched =
      path_stitcher.Sample(fixed_path, sampled, map);
  if (stitched.new_points.size() != sampled.size() ||
      stitched.output_states.size() !=
          snapshot.retained_prefix_points + sampled.size()) {
    throw std::logic_error("full spatial path has inconsistent state counts");
  }

  BuiltTrajectory result;
  result.full =
      InitializeFullTrajectory(snapshot, config.time_step_seconds,
                               PlanningHorizonS(config), sampled.size());
  ValidateRetainedTrajectoryPrefix(snapshot);
  double inherited_frame_offset_x = 0.0;
  double inherited_frame_offset_y = 0.0;
  if (snapshot.historical_plan_aligned) {
    if (snapshot.retained_prefix_points > 0) {
      const LateralPathState &anchor =
          snapshot.retained_lateral_states.back();
      inherited_frame_offset_x =
          snapshot.input.previous_path_x.back() - anchor.expected_x;
      inherited_frame_offset_y =
          snapshot.input.previous_path_y.back() - anchor.expected_y;
    } else if (snapshot.frontier.lateral.valid) {
      inherited_frame_offset_x =
          snapshot.input.ego.x - snapshot.frontier.lateral.expected_x;
      inherited_frame_offset_y =
          snapshot.input.ego.y - snapshot.frontier.lateral.expected_y;
    }
    for (double &seed_x : result.full.kinematic_seed_x) {
      seed_x += inherited_frame_offset_x;
    }
    for (double &seed_y : result.full.kinematic_seed_y) {
      seed_y += inherited_frame_offset_y;
    }
  }
  // Persist the same rigid stitching frame used by the hard validator.
  AppendRetainedTrajectoryPrefix(
      snapshot, config.time_step_seconds, inherited_frame_offset_x,
      inherited_frame_offset_y, snapshot.historical_plan_aligned, &result.full);
  if (snapshot.retained_prefix_points > 0) {
    // The current fixed path's residual correction is defined from this
    // frontier, even when the frontier came from an older retained segment.
    result.full.points.back().lateral.residual_correction_progress_m =
        fixed_path.residual_start_progress_m;
  }
  for (std::size_t index = 0; index < sampled.size(); ++index) {
    TrajectoryPoint point;
    point.time_from_telemetry_s =
        snapshot.frontier.time_from_telemetry_s +
        static_cast<double>(index + 1) * config.time_step_seconds;
    point.x = stitched.new_points[index].first;
    point.y = stitched.new_points[index].second;
    point.longitudinal = sampled[index];
    point.lateral =
        stitched.output_states[snapshot.retained_prefix_points + index];
    result.full.points.push_back(point);
  }
  TrajectoryOutputSlice output =
      SliceTrajectoryOutput(result.full, config.output_points,
                            "full trajectory cannot satisfy output contract");
  result.output = std::move(output.output);
  result.output_longitudinal = std::move(output.longitudinal);
  result.output_lateral = std::move(output.lateral);
  result.next_path_state = stitched.next_state;
  result.next_path_state.last_output_states = result.output_lateral;
  result.lateral_diagnostics = stitched.diagnostics;
  return result;
}

bool IsProvenPhysicalInfeasibility(const CandidateEvaluation &evaluation) {
  if (evaluation.failure == CandidateFailureReason::kQpInfeasible) {
    return evaluation.failure_detail.find("primal infeasible") !=
               std::string::npos ||
           evaluation.failure_detail.find("primal infeasible inaccurate") !=
               std::string::npos;
  }
  return evaluation.has_plan &&
         evaluation.validation.HasOnlyCollisionViolations();
}

const char *CandidateFailureReasonName(CandidateFailureReason reason) {
  switch (reason) {
  case CandidateFailureReason::kNone:
    return "None";
  case CandidateFailureReason::kQpInfeasible:
    return "QpInfeasible";
  case CandidateFailureReason::kHardValidation:
    return "HardValidation";
  case CandidateFailureReason::kInfrastructure:
    return "Infrastructure";
  }
  return "Unknown";
}

std::vector<CollisionEventDiagnostics> BuildCollisionEventDiagnostics(
    const ValidationResult &validation,
    const std::vector<PredictedObstacle> &qp_obstacles) {
  std::vector<CollisionEventDiagnostics> result;
  result.reserve(validation.collision_evidence.size());
  for (const CollisionEvidence &evidence : validation.collision_evidence) {
    CollisionEventDiagnostics event;
    event.evidence = evidence;
    for (const PredictedObstacle &obstacle : qp_obstacles) {
      if (obstacle.id == evidence.object_id) {
        event.qp_relevant = true;
        event.qp_initial_relative_s_m = obstacle.relative_s;
        event.qp_predicted_speed_mps = obstacle.speed_mps;
        event.qp_obstacle_d_m = obstacle.d;
        break;
      }
    }
    result.push_back(event);
  }
  return result;
}

ControlCandidateDiagnostics BuildControlCandidateDiagnostics(
    const CandidateEvaluation &evaluation, bool selected) {
  ControlCandidateDiagnostics result;
  result.candidate_id = evaluation.candidate_id;
  result.safety_policy = evaluation.safety_policy;
  result.minimum_risk_candidate = false;
  result.selected_for_dispatch = selected;
  result.has_plan = evaluation.has_plan;
  result.validation_valid = evaluation.valid;
  result.failure_reason = CandidateFailureReasonName(evaluation.failure);
  result.failure_detail = evaluation.failure_detail;
  result.evaluate_time_ms = evaluation.evaluate_time_ms;
  result.validate_time_ms = evaluation.validate_time_ms;
  if (!evaluation.has_plan) {
    result.fallback_level =
        evaluation.safety_policy == LongitudinalSafetyPolicy::kNormalOperational
            ? FallbackLevel::kNormal
            : (evaluation.safety_policy ==
                       LongitudinalSafetyPolicy::kDegradedBraking
                   ? FallbackLevel::kDegradedBraking
                   : FallbackLevel::kMaximumBraking);
    return result;
  }

  const CandidatePlan &plan = evaluation.plan;
  result.candidate_id = plan.candidate_id;
  result.fallback_level = plan.fallback_level;
  result.qp_success = plan.qp.success;
  result.qp_hard_safe = plan.qp.hard_safe;
  result.qp_status = plan.qp.status;
  result.qp_minimum_physical_margin_m =
      plan.qp.minimum_physical_margin_meters;
  result.qp_minimum_operational_margin_m =
      plan.qp.minimum_operational_margin_meters;
  result.qp_maximum_headway_slack_m =
      plan.qp.maximum_headway_slack_meters;
  result.qp_maximum_collision_violation_m =
      plan.qp.maximum_collision_violation_meters;
  result.validation_minimum_collision_margin_m =
      plan.validation.minimum_collision_margin_m;
  result.validation_minimum_road_margin_m =
      plan.validation.minimum_road_margin_m;
  result.shielded_same_lane_rear_vehicle_ids =
      plan.validation.shielded_same_lane_rear_vehicle_ids;
  result.collision_events =
      BuildCollisionEventDiagnostics(plan.validation, plan.obstacles);
  return result;
}

ControlCandidateDiagnostics BuildControlCandidateDiagnostics(
    const MinimumRiskDispatch &dispatch, bool selected) {
  ControlCandidateDiagnostics result;
  result.candidate_id = dispatch.candidate_id;
  result.safety_policy = LongitudinalSafetyPolicy::kMaximumBraking;
  result.fallback_level = FallbackLevel::kMinimumRisk;
  result.minimum_risk_candidate = true;
  result.selected_for_dispatch = selected;
  result.has_plan = !dispatch.full_trajectory.points.empty();
  result.validation_valid = dispatch.validation.valid;
  result.failure_reason = dispatch.validation.valid
                              ? "None"
                              : "MinimumRiskCollisionEvidence";
  result.failure_detail = dispatch.risk.selected_reason;
  result.qp_success = dispatch.qp.success;
  result.qp_hard_safe = dispatch.qp.hard_safe;
  result.qp_status = dispatch.qp.status;
  result.qp_minimum_physical_margin_m =
      dispatch.qp.minimum_physical_margin_meters;
  result.qp_minimum_operational_margin_m =
      dispatch.qp.minimum_operational_margin_meters;
  result.qp_maximum_headway_slack_m =
      dispatch.qp.maximum_headway_slack_meters;
  result.qp_maximum_collision_violation_m =
      dispatch.qp.maximum_collision_violation_meters;
  result.validation_minimum_collision_margin_m =
      dispatch.validation.minimum_collision_margin_m;
  result.validation_minimum_road_margin_m =
      dispatch.validation.minimum_road_margin_m;
  result.shielded_same_lane_rear_vehicle_ids =
      dispatch.validation.shielded_same_lane_rear_vehicle_ids;
  result.evaluate_time_ms = dispatch.exclusive_evaluate_time_ms;
  result.validate_time_ms = dispatch.exclusive_validate_time_ms;
  result.collision_events = BuildCollisionEventDiagnostics(
      dispatch.validation, dispatch.obstacles);
  return result;
}

void PopulateControlCandidateDiagnostics(
    const PlanningCycleDecision &decision,
    PlannerCycleDiagnostics *diagnostics) {
  diagnostics->control_candidates.clear();
  diagnostics->control_candidates.reserve(
      decision.ordinary_evaluations.size() +
      (decision.minimum_risk.candidate_id != 0 ? 1U : 0U));
  for (const CandidateEvaluation &evaluation :
       decision.ordinary_evaluations) {
    const bool selected =
        decision.disposition == PlanDisposition::kValidatedCandidate &&
        decision.has_validated_candidate &&
        decision.validated_candidate.candidate_id == evaluation.candidate_id;
    diagnostics->control_candidates.push_back(
        BuildControlCandidateDiagnostics(evaluation, selected));
  }
  if (decision.minimum_risk.candidate_id != 0) {
    diagnostics->control_candidates.push_back(
        BuildControlCandidateDiagnostics(
            decision.minimum_risk,
            decision.disposition == PlanDisposition::kMinimumRiskDispatch &&
                decision.has_minimum_risk));
  }
}

template <typename Reason>
std::string JoinReasonNames(const std::vector<Reason> &reasons,
                            const char *(*name)(Reason)) {
  std::ostringstream text;
  for (std::size_t index = 0; index < reasons.size(); ++index) {
    if (index != 0) {
      text << '|';
    }
    text << name(reasons[index]);
  }
  return text.str();
}

const FinalBehaviorCandidate *FindFinalBehaviorCandidate(
    const ActiveBehaviorCycleResult &result, std::uint64_t candidate_id) {
  for (const FinalBehaviorCandidate &candidate : result.final_candidates) {
    if (candidate.candidate_id == candidate_id) {
      return &candidate;
    }
  }
  return nullptr;
}

void PopulateBehaviorCandidateDiagnostics(
    const ActiveBehaviorCycleResult &result,
    const PlanningCycleDecision &decision,
    PlannerCycleDiagnostics *diagnostics) {
  diagnostics->behavior_candidates.clear();
  diagnostics->behavior_candidates.reserve(result.behavior.candidates.size());
  for (const BehaviorCandidate &behavior : result.behavior.candidates) {
    BehaviorCandidateDiagnostics candidate;
    candidate.candidate_id = behavior.candidate_id;
    candidate.behavior = BehaviorTypeName(behavior.behavior);
    candidate.source_lane = behavior.source_lane;
    candidate.target_lane = behavior.target_lane;
    candidate.passing_order = PassingOrderName(behavior.order);
    candidate.behavior_status = BehaviorCandidateStatusName(behavior.status);
    candidate.gap_has_front_vehicle = behavior.gap.has_front_vehicle;
    candidate.gap_front_vehicle_id = behavior.gap.front_vehicle_id;
    candidate.gap_has_rear_vehicle = behavior.gap.has_rear_vehicle;
    candidate.gap_rear_vehicle_id = behavior.gap.rear_vehicle_id;
    candidate.stable_observations = behavior.stable_observations;
    candidate.stable_duration_s = behavior.stable_duration_s;
    candidate.estimated_progress_m = behavior.estimated_progress_m;
    candidate.estimated_progress_gain_m =
        behavior.estimated_progress_gain_m;
    candidate.estimated_speed_gain_mps =
        behavior.estimated_speed_gain_mps;
    candidate.uncertainty_cost = behavior.uncertainty_cost;
    candidate.admission_evaluated = behavior.coarse_admission.evaluated;
    candidate.admission_passed = behavior.coarse_admission.passed;
    candidate.admission_rejection_reasons = JoinReasonNames(
        behavior.coarse_admission.rejection_reasons,
        CoarseAdmissionRejectionReasonName);
    candidate.has_front_margin = behavior.coarse_admission.has_front_margin;
    candidate.minimum_front_margin_m =
        behavior.coarse_admission.minimum_front_margin_m;
    candidate.has_rear_margin = behavior.coarse_admission.has_rear_margin;
    candidate.minimum_rear_margin_m =
        behavior.coarse_admission.minimum_rear_margin_m;
    candidate.has_rear_ttc = behavior.coarse_admission.has_rear_ttc;
    candidate.minimum_rear_ttc_s =
        behavior.coarse_admission.minimum_rear_ttc_s;
    candidate.reachable_center_overlap_m =
        behavior.coarse_admission.reachable_center_overlap_m;
    candidate.has_source_front_margin =
        behavior.coarse_admission.has_source_front_margin;
    candidate.minimum_source_front_margin_m =
        behavior.coarse_admission.minimum_source_front_margin_m;
    candidate.source_front_limiting_vehicle_id =
        behavior.coarse_admission.source_front_limiting_vehicle_id;
    candidate.source_front_limiting_time_s =
        behavior.coarse_admission.source_front_limiting_time_s;
    candidate.source_front_limiting_ego_road_s_m =
        behavior.coarse_admission.source_front_limiting_ego_road_s_m;
    candidate.source_front_limiting_occupied_min_road_s_m =
        behavior.coarse_admission
            .source_front_limiting_occupied_min_road_s_m;
    candidate.source_front_limiting_occupied_max_road_s_m =
        behavior.coarse_admission
            .source_front_limiting_occupied_max_road_s_m;
    candidate.source_front_limiting_min_rate_mps =
        behavior.coarse_admission.source_front_limiting_min_rate_mps;
    candidate.source_front_limiting_max_rate_mps =
        behavior.coarse_admission.source_front_limiting_max_rate_mps;
    candidate.source_front_limiting_uncertainty_m =
        behavior.coarse_admission.source_front_limiting_uncertainty_m;
    candidate.source_front_limiting_hypothesis =
        TrafficPredictionHypothesisName(
            behavior.coarse_admission
                .source_front_limiting_hypothesis);
    candidate.source_front_risk_observed =
        behavior.coarse_admission.source_front_risk_observed;
    candidate.first_source_front_risk_time_s =
        behavior.coarse_admission.first_source_front_risk_time_s;
    candidate.first_source_front_risk_vehicle_id =
        behavior.coarse_admission.first_source_front_risk_vehicle_id;
    candidate.first_source_front_risk_margin_m =
        behavior.coarse_admission.first_source_front_risk_margin_m;
    candidate.first_source_front_risk_hypothesis =
        TrafficPredictionHypothesisName(
            behavior.coarse_admission
                .first_source_front_risk_hypothesis);
    candidate.target_kinematic_failure_observed =
        behavior.coarse_admission
            .target_kinematic_failure_observed;
    candidate.first_target_kinematic_failure_time_s =
        behavior.coarse_admission
            .first_target_kinematic_failure_time_s;
    candidate.first_target_propagated_state_count =
        behavior.coarse_admission
            .first_target_propagated_state_count;
    candidate.first_target_feasible_state_count =
        behavior.coarse_admission
            .first_target_feasible_state_count;
    candidate.first_target_best_front_margin_m =
        behavior.coarse_admission
            .first_target_best_front_margin_m;
    candidate.first_target_best_rear_margin_m =
        behavior.coarse_admission
            .first_target_best_rear_margin_m;
    candidate.first_target_best_rear_ttc_margin_m =
        behavior.coarse_admission
            .first_target_best_rear_ttc_margin_m;
    candidate.gap_current_observation_valid =
        behavior.traffic_gap.current_observation_valid;
    candidate.gap_topology_consistent =
        behavior.traffic_gap.topology_consistent;
    candidate.topology_failure_observed =
        behavior.traffic_gap.topology_failure_observed;
    candidate.first_topology_failure_kind =
        GapTopologyFailureKindName(
            behavior.traffic_gap.first_topology_failure_kind);
    candidate.first_topology_failure_time_s =
        behavior.traffic_gap.first_topology_failure_time_s;
    candidate.first_topology_expected_front_found =
        behavior.traffic_gap.first_topology_expected_front_found;
    candidate.first_topology_expected_rear_found =
        behavior.traffic_gap.first_topology_expected_rear_found;
    candidate.first_topology_actual_front_present =
        behavior.traffic_gap.first_topology_actual_front_present;
    candidate.first_topology_actual_front_vehicle_id =
        behavior.traffic_gap.first_topology_actual_front_vehicle_id;
    candidate.first_topology_actual_rear_present =
        behavior.traffic_gap.first_topology_actual_rear_present;
    candidate.first_topology_actual_rear_vehicle_id =
        behavior.traffic_gap.first_topology_actual_rear_vehicle_id;
    candidate.expected_boundaries_reversed_observed =
        behavior.traffic_gap.expected_boundaries_reversed_observed;
    candidate.first_expected_boundaries_reversed_time_s =
        behavior.traffic_gap
            .first_expected_boundaries_reversed_time_s;
    candidate.merge_corridor_intrusion_observed =
        behavior.traffic_gap.merge_corridor_intrusion_observed;
    candidate.first_merge_corridor_intrusion_time_s =
        behavior.traffic_gap.first_merge_corridor_intrusion_time_s;
    candidate.first_merge_corridor_intrusion_vehicle_id =
        behavior.traffic_gap
            .first_merge_corridor_intrusion_vehicle_id;
    candidate.first_merge_corridor_intrusion_hypothesis =
        TrafficPredictionHypothesisName(
            behavior.traffic_gap
                .first_merge_corridor_intrusion_hypothesis);
    candidate.merge_corridor_blocked_observed =
        behavior.coarse_admission
            .merge_corridor_blocked_observed;
    candidate.first_merge_corridor_blocked_time_s =
        behavior.coarse_admission
            .first_merge_corridor_blocked_time_s;
    candidate.first_merge_corridor_candidate_state_count =
        behavior.coarse_admission
            .first_merge_corridor_candidate_state_count;
    candidate.first_merge_corridor_feasible_state_count =
        behavior.coarse_admission
            .first_merge_corridor_feasible_state_count;
    candidate.first_merge_corridor_blocking_vehicle_id =
        behavior.coarse_admission
            .first_merge_corridor_blocking_vehicle_id;
    candidate.first_merge_corridor_blocking_hypothesis =
        TrafficPredictionHypothesisName(
            behavior.coarse_admission
                .first_merge_corridor_blocking_hypothesis);
    candidate.first_merge_corridor_blocking_margin_m =
        behavior.coarse_admission
            .first_merge_corridor_blocking_margin_m;
    candidate.prediction_evidence_complete =
        behavior.traffic_gap.prediction_evidence_complete;
    candidate.minimum_gap_window_m =
        behavior.traffic_gap.minimum_ego_center_window_m;

    const SpatialPathCandidate *spatial = FindSpatialPathCandidate(
        result.spatial_paths, behavior.candidate_id);
    if (spatial != nullptr) {
      candidate.spatial_evaluated = spatial->precheck.evaluated;
      candidate.spatial_status = SpatialPathCandidateStatusName(spatial->status);
      candidate.spatial_precheck_passed = spatial->precheck.passed;
      candidate.spatial_rejection_reasons = JoinReasonNames(
          spatial->precheck.rejection_reasons,
          SpatialPathPrecheckReasonName);
      candidate.spatial_transition_length_m = spatial->transition_length_m;
      candidate.spatial_path_extent_m = spatial->geometry.path_extent_m;
      candidate.spatial_completion_progress_m =
          spatial->occupancy.lane_change_completion_path_progress_m;
      candidate.spatial_minimum_road_margin_m =
          spatial->precheck.minimum_road_margin_m;
      candidate.spatial_speed_upper_bound_mps =
          spatial->precheck.speed_upper_bound_mps;
      candidate.spatial_maximum_lateral_acceleration_mps2 =
          spatial->precheck.maximum_estimated_lateral_acceleration_mps2;
      candidate.spatial_maximum_lateral_jerk_mps3 =
          spatial->precheck.maximum_estimated_lateral_jerk_mps3;
    }

    const STCandidateEvaluation *st = FindSTCandidate(
        result.st_candidates, behavior.candidate_id);
    if (st != nullptr) {
      candidate.st_evaluated = true;
      candidate.st_status = STCandidateStatusName(st->status);
      candidate.st_prediction_evidence_complete =
          st->corridor.prediction_evidence_complete;
      candidate.st_corridor_empty = st->corridor.empty;
      candidate.st_first_evidence_failure_node =
          st->corridor.first_evidence_failure_node;
      candidate.st_first_empty_node = st->corridor.first_empty_node;
      candidate.st_minimum_width_m = st->corridor.minimum_width_m;
      candidate.st_minimum_width_node = st->corridor.minimum_width_node;
      candidate.st_minimum_width_lower_source_present =
          st->corridor.minimum_width_lower_source.present;
      candidate.st_minimum_width_lower_source_vehicle_id =
          st->corridor.minimum_width_lower_source.vehicle_id;
      candidate.st_minimum_width_upper_source_present =
          st->corridor.minimum_width_upper_source.present;
      candidate.st_minimum_width_upper_source_vehicle_id =
          st->corridor.minimum_width_upper_source.vehicle_id;
      candidate.st_first_empty_lower_source_present =
          st->corridor.first_empty_lower_source.present;
      candidate.st_first_empty_lower_source_vehicle_id =
          st->corridor.first_empty_lower_source.vehicle_id;
      candidate.st_first_empty_upper_source_present =
          st->corridor.first_empty_upper_source.present;
      candidate.st_first_empty_upper_source_vehicle_id =
          st->corridor.first_empty_upper_source.vehicle_id;
      candidate.st_curvature_initial_speed_feasible =
          st->speed_budget.initial_speed_feasible;
      candidate.st_minimum_speed_limit_mps =
          st->speed_budget.minimum_speed_limit_mps;
      candidate.st_qp_attempted = st->qp_attempted;
      candidate.st_qp_success = st->qp_success;
      candidate.st_qp_bounds_satisfied = st->qp_bounds_satisfied;
      candidate.st_qp_status = st->qp_status;
      candidate.st_qp_objective = st->qp_objective;
      candidate.st_terminal_progress_m = st->terminal_progress_m;
      candidate.st_terminal_speed_mps = st->terminal_speed_mps;
      candidate.st_source_lane_departed_in_time =
          st->source_lane_departed_in_time;
      candidate.st_source_lane_departure_time_s =
          st->source_lane_departure_time_s;
      candidate.st_source_lane_departure_deadline_s =
          st->latest_allowed_source_lane_departure_time_s;
      candidate.st_lane_change_completed_in_time =
          st->lane_change_completed_in_time;
      candidate.st_lane_change_completion_time_s =
          st->lane_change_completion_time_s;
      candidate.st_lane_change_completion_deadline_s =
          st->latest_allowed_completion_time_s;
      candidate.st_minimum_lower_margin_m = st->minimum_lower_margin_m;
      candidate.st_minimum_upper_margin_m = st->minimum_upper_margin_m;
      candidate.st_maximum_speed_excess_mps = st->maximum_speed_excess_mps;
    }

    const FinalBehaviorCandidate *final = FindFinalBehaviorCandidate(
        result, behavior.candidate_id);
    if (final != nullptr) {
      candidate.final_evaluated = true;
      candidate.final_status = FinalBehaviorCandidateStatusName(final->status);
      candidate.best_valid_candidate =
          result.has_best_valid_candidate &&
          result.best_candidate_id == final->candidate_id;
      candidate.selected_for_dispatch =
          decision.disposition == PlanDisposition::kValidatedCandidate &&
          decision.has_validated_candidate &&
          decision.validated_candidate.behavior_lane_change &&
          decision.validated_candidate.candidate_id == final->candidate_id;
      candidate.tightening_attempted = final->tightening_attempted;
      candidate.tightening_succeeded = final->tightening_succeeded;
      candidate.final_validation_valid = final->validation.valid;
      candidate.final_rejection_detail = final->rejection_detail;
      candidate.final_minimum_physical_margin_m =
          final->minimum_physical_margin_m;
      candidate.final_minimum_operational_margin_m =
          final->minimum_operational_margin_m;
      candidate.final_minimum_road_margin_m = final->minimum_road_margin_m;
      candidate.final_maximum_acceleration_mps2 =
          final->maximum_acceleration_mps2;
      candidate.final_maximum_jerk_mps3 = final->maximum_jerk_mps3;
      candidate.final_minimum_longitudinal_acceleration_mps2 =
          final->minimum_longitudinal_acceleration_mps2;
      candidate.final_cost = final->cost;
    }
    diagnostics->behavior_candidates.push_back(std::move(candidate));
  }
}

void PopulateActiveBehaviorResultDiagnostics(
    const ActiveBehaviorDiagnostics &behavior,
    const PlanningCycleDecision &decision, std::uint64_t cycle,
    PlannerCycleDiagnostics *diagnostics) {
  diagnostics->behavior_full_validation_rejection_count =
      behavior.full_validation_rejection_count;
  diagnostics->behavior_tightening_attempt_count =
      behavior.tightening_attempt_count;
  diagnostics->behavior_tightening_success_count =
      behavior.tightening_success_count;
  if (!behavior.has_latest_result || behavior.latest_result.cycle != cycle) {
    return;
  }

  const ActiveBehaviorCycleResult &result = behavior.latest_result;
  diagnostics->behavior_transaction_committed =
      behavior.latest_result_committed;
  diagnostics->behavior_result_cycle = result.cycle;
  diagnostics->behavior_tracker_reset_reason =
      TrafficTrackerResetReasonName(result.tracking.reset_reason);
  diagnostics->behavior_observed_track_count =
      result.tracking.observed_track_count;
  diagnostics->behavior_valid_track_count = result.tracking.valid_track_count;
  diagnostics->behavior_admissible_track_count =
      result.tracking.safety_admissible_track_count;
  diagnostics->behavior_stale_track_count = result.tracking.stale_track_count;
  diagnostics->behavior_reacquired_track_count =
      result.tracking.reacquired_track_count;
  diagnostics->behavior_reused_id_count = result.tracking.reused_id_count;
  diagnostics->behavior_prediction_trajectory_count =
      result.prediction.trajectories.size();
  diagnostics->behavior_admissible_prediction_count =
      result.prediction.safety_admissible_trajectory_count;
  diagnostics->behavior_generated_candidate_count =
      result.behavior.generated_candidate_count;
  diagnostics->behavior_stable_gap_count = result.behavior.stable_gap_count;
  diagnostics->behavior_coarse_admitted_count =
      result.behavior.coarse_admitted_candidate_count;
  diagnostics->behavior_spatial_evaluated_count =
      result.spatial_paths.evaluated_candidate_count;
  diagnostics->behavior_spatial_generated_count =
      result.spatial_paths.generated_candidate_count;
  diagnostics->behavior_spatial_passed_count =
      result.spatial_paths.precheck_passed_candidate_count;
  diagnostics->behavior_st_evaluated_count =
      result.st_candidates.evaluated_candidate_count;
  diagnostics->behavior_st_corridor_ready_count =
      result.st_candidates.corridor_ready_candidate_count;
  diagnostics->behavior_st_corridor_empty_count =
      result.st_candidates.corridor_empty_candidate_count;
  diagnostics->behavior_st_prediction_rejected_count =
      result.st_candidates.prediction_rejected_candidate_count;
  diagnostics->behavior_st_curvature_rejected_count =
      result.st_candidates.curvature_rejected_candidate_count;
  diagnostics->behavior_st_qp_attempted_count =
      result.st_candidates.qp_attempted_candidate_count;
  diagnostics->behavior_st_qp_solved_count =
      result.st_candidates.qp_solved_candidate_count;
  diagnostics->behavior_st_qp_failed_count =
      result.st_candidates.qp_failed_candidate_count;
  diagnostics->behavior_st_horizon_rejected_count =
      result.st_candidates.horizon_rejected_candidate_count;
  diagnostics->behavior_st_deadline_skipped_count =
      result.st_candidates.deadline_skipped_candidate_count;
  diagnostics->behavior_final_candidate_count = result.final_candidates.size();
  diagnostics->behavior_has_best_valid_candidate =
      result.has_best_valid_candidate;
  diagnostics->behavior_best_candidate_id = result.best_candidate_id;
  diagnostics->behavior_best_target_lane = result.best_target_lane;
  for (const FinalBehaviorCandidate &candidate : result.final_candidates) {
    if (candidate.status == FinalBehaviorCandidateStatus::kValid) {
      ++diagnostics->behavior_final_valid_count;
    }
    if (candidate.status ==
        FinalBehaviorCandidateStatus::kValidationRejected) {
      ++diagnostics->behavior_final_validation_rejected_count;
    }
  }
  PopulateBehaviorCandidateDiagnostics(result, decision, diagnostics);
}

} // namespace

PathPlanner::PathPlanner(const PlannerConfig &config)
    : config_(config), longitudinal_qp_(MakeLongitudinalConfig(config)),
      speed_reference_generator_(MakeReferenceConfig(config)),
      trajectory_validator_(MakeValidatorConfig(config)),
      runtime_monitor_(config.monitor) {
  ValidatePlannerConfig(config_);
  active_behavior_planner_.reset(
      new ActiveBehaviorPlanner(MakeActiveBehaviorConfig(config_)));
  SetOperatingMode(config_.operating_mode);
}

PathPlanner::~PathPlanner() = default;

void PathPlanner::SetOperatingMode(PlannerOperatingMode mode) {
  if (mode != PlannerOperatingMode::kLaneCruiseOnly &&
      mode != PlannerOperatingMode::kBehaviorActive) {
    throw std::invalid_argument("unknown planner operating mode");
  }
  if (mode == PlannerOperatingMode::kBehaviorActive) {
    if (effective_mode_ != PlannerOperatingMode::kBehaviorActive) {
      active_behavior_planner_->Reset();
    }
    requested_mode_ = mode;
    effective_mode_ = mode;
    return;
  }
  requested_mode_ = mode;
  if (mode == PlannerOperatingMode::kLaneCruiseOnly &&
      effective_mode_ == PlannerOperatingMode::kBehaviorActive &&
      active_behavior_planner_->ManeuverInProgress()) {
    // Stop admitting new maneuvers immediately, but keep the active executor
    // responsible for a committed path until target-lane settling.
    return;
  }
  effective_mode_ = mode;
  if (mode != PlannerOperatingMode::kBehaviorActive) {
    active_behavior_planner_->Reset();
  }
}

ActiveBehaviorDiagnostics PathPlanner::behavior_diagnostics() const {
  return active_behavior_planner_
             ? active_behavior_planner_->diagnostics()
             : ActiveBehaviorDiagnostics();
}

PlanningSnapshot PathPlanner::BuildSnapshot(const PlannerInput &input,
                                            const MapData &map) const {
  ValidatePlannerInput(input, config_.output_points);
  PlanningSnapshot snapshot;
  snapshot.input = input;
  snapshot.cycle = plan_cycle_;
  snapshot.original_previous_path_points = input.previous_path_x.size();
  snapshot.retained_prefix_points =
      std::min(snapshot.original_previous_path_points,
               config_.retained_path.maximum_points);
  snapshot.input.previous_path_x.resize(snapshot.retained_prefix_points);
  snapshot.input.previous_path_y.resize(snapshot.retained_prefix_points);
  snapshot.frontier.time_from_telemetry_s =
      static_cast<double>(snapshot.retained_prefix_points) *
      config_.time_step_seconds;

  snapshot.historical_plan_aligned = HistoricalPlanAligned(
      input, state_, config_.output_points, &snapshot.reset_reason);
  snapshot.telemetry_time_s = state_.telemetry_time_s;
  if (snapshot.historical_plan_aligned) {
    const std::size_t consumed_points =
        config_.output_points - snapshot.original_previous_path_points;
    snapshot.telemetry_time_s +=
        static_cast<double>(consumed_points) * config_.time_step_seconds;
  }
  snapshot.target_lane = state_.target_lane;
  if (!snapshot.historical_plan_aligned || snapshot.target_lane < 0) {
    snapshot.target_lane =
        static_cast<int>(std::floor(input.ego.d / config_.lane_width_meters));
    snapshot.target_lane =
        std::max(0, std::min(snapshot.target_lane, config_.lane_count - 1));
  }

  if (snapshot.historical_plan_aligned) {
    const std::size_t consumed =
        config_.output_points - snapshot.original_previous_path_points;
    const std::size_t seed_begin = consumed > 3 ? consumed - 3 : 0;
    for (std::size_t index = seed_begin; index < consumed; ++index) {
      snapshot.kinematic_seed_x.push_back(
          state_.output_lateral[index].expected_x);
      snapshot.kinematic_seed_y.push_back(
          state_.output_lateral[index].expected_y);
    }
    if (snapshot.retained_prefix_points > 0) {
      snapshot.retained_longitudinal_states.assign(
          state_.output_longitudinal.begin() + consumed,
          state_.output_longitudinal.begin() + consumed +
              snapshot.retained_prefix_points);
      snapshot.retained_lateral_states.assign(
          state_.output_lateral.begin() + consumed,
          state_.output_lateral.begin() + consumed +
              snapshot.retained_prefix_points);
      snapshot.frontier.longitudinal =
          snapshot.retained_longitudinal_states.back();
      snapshot.frontier.lateral = snapshot.retained_lateral_states.back();
      snapshot.frontier.road_s_unwrapped_m =
          snapshot.frontier.lateral.road_parameter_s;
      snapshot.frontier.d_m = snapshot.frontier.lateral.planned_d;
    } else {
      snapshot.frontier.longitudinal = state_.output_longitudinal.back();
      snapshot.frontier.lateral = state_.output_lateral.back();
      snapshot.frontier.road_s_unwrapped_m =
          snapshot.frontier.lateral.road_parameter_s;
      snapshot.frontier.d_m = snapshot.frontier.lateral.planned_d;
    }
    snapshot.frontier.state_source = PlanningStateSource::kExactInherited;
  } else {
    snapshot.retained_longitudinal_states =
        EstimateRetainedLongitudinalStates(snapshot.input, config_);
    snapshot.retained_lateral_states.resize(snapshot.retained_prefix_points);
    const double parameter_span =
        ForwardParameterSpan(input.ego.s, input.end_path_s, map.track_length);
    for (std::size_t index = 0; index < snapshot.retained_prefix_points;
         ++index) {
      const double fraction =
          static_cast<double>(index + 1) /
          static_cast<double>(
              std::max<std::size_t>(1, snapshot.original_previous_path_points));
      const RoadProjection projection = ProjectCartesianToRoad(
          snapshot.input.previous_path_x[index],
          snapshot.input.previous_path_y[index],
          input.ego.s + fraction * parameter_span, map);
      LateralPathState &lateral = snapshot.retained_lateral_states[index];
      lateral.valid = true;
      lateral.road_parameter_s = projection.road_s_unwrapped_m;
      lateral.planned_d = projection.d_m;
      lateral.expected_x = snapshot.input.previous_path_x[index];
      lateral.expected_y = snapshot.input.previous_path_y[index];
    }
    snapshot.frontier.longitudinal =
        ColdStartInitialState(snapshot.input, config_);
    if (!snapshot.retained_longitudinal_states.empty()) {
      snapshot.retained_longitudinal_states.back().v =
          snapshot.frontier.longitudinal.v;
      snapshot.retained_longitudinal_states.back().a =
          snapshot.frontier.longitudinal.a;
      snapshot.retained_longitudinal_states.back().j =
          snapshot.frontier.longitudinal.j;
    }
    if (snapshot.retained_prefix_points > 0) {
      snapshot.frontier.lateral = snapshot.retained_lateral_states.back();
      snapshot.frontier.road_s_unwrapped_m =
          snapshot.frontier.lateral.road_parameter_s;
      snapshot.frontier.d_m = snapshot.frontier.lateral.planned_d;
      snapshot.frontier.state_source =
          PlanningStateSource::kColdStartProjected;
    } else {
      snapshot.frontier.road_s_unwrapped_m = input.ego.s;
      snapshot.frontier.d_m = input.ego.d;
      snapshot.frontier.lateral.valid = true;
      snapshot.frontier.lateral.road_parameter_s = input.ego.s;
      snapshot.frontier.lateral.planned_d = input.ego.d;
      snapshot.frontier.lateral.expected_x = input.ego.x;
      snapshot.frontier.lateral.expected_y = input.ego.y;
      snapshot.frontier.state_source = PlanningStateSource::kTelemetry;
    }
  }
  snapshot.input.end_path_s = snapshot.frontier.road_s_unwrapped_m;
  snapshot.input.end_path_d = snapshot.frontier.d_m;
  return snapshot;
}

CandidateEvaluation PathPlanner::EvaluateLaneCruise(
    const PlanningSnapshot &snapshot, const MapData &map,
    const FixedSpatialPath &fixed_path,
    const std::vector<PredictedObstacle> &obstacles,
    const std::vector<double> &normal_reference_speed_mps,
    LongitudinalSafetyPolicy policy, std::uint64_t candidate_id) const {
  CandidateEvaluation evaluation;
  evaluation.candidate_id = candidate_id;
  evaluation.safety_policy = policy;
  const std::chrono::steady_clock::time_point evaluate_start =
      std::chrono::steady_clock::now();
  try {
    LongitudinalQpInput qp_input;
    qp_input.initial_speed_mps =
        std::max(0.0, snapshot.frontier.longitudinal.v);
    qp_input.initial_acceleration_mps2 =
        snapshot.frontier.longitudinal.a;
    qp_input.initial_jerk_valid =
        snapshot.frontier.state_source ==
        PlanningStateSource::kExactInherited;
    qp_input.initial_jerk_mps3 = snapshot.frontier.longitudinal.j;
    qp_input.reference_speed_mps = normal_reference_speed_mps;
    qp_input.obstacles = obstacles;
    qp_input.safety_policy = policy;
    if (policy == LongitudinalSafetyPolicy::kMaximumBraking) {
      qp_input.reference_speed_mps.assign(
          config_.qp_horizon_steps + 1, 0.0);
    }
    LongitudinalQpResult qp =
        longitudinal_qp_.Evaluate(qp_input, state_.qp_warm_start);
    qp.trajectory.emergency =
        policy == LongitudinalSafetyPolicy::kMaximumBraking;
    if (!qp.success) {
      evaluation.failure = CandidateFailureReason::kQpInfeasible;
      evaluation.failure_detail = qp.status;
      evaluation.evaluate_time_ms = ElapsedMilliseconds(evaluate_start);
      return evaluation;
    }
    if (!qp.hard_safe) {
      evaluation.failure = CandidateFailureReason::kInfrastructure;
      evaluation.failure_detail =
          "ordinary QP returned a result without hard-safety evidence";
      evaluation.evaluate_time_ms = ElapsedMilliseconds(evaluate_start);
      return evaluation;
    }

    const BuiltTrajectory built = BuildFullTrajectory(
        snapshot, config_, map, fixed_path, qp, path_stitcher_);
    CandidatePlan plan;
    plan.candidate_id = candidate_id;
    plan.fallback_level = FallbackForPolicy(policy);
    plan.full_trajectory = built.full;
    plan.output = built.output;
    plan.qp = qp;
    plan.obstacles = obstacles;
    plan.reference_speed_mps = qp_input.reference_speed_mps;
    plan.lateral_diagnostics = built.lateral_diagnostics;
    plan.next_state = state_;
    plan.next_state.target_lane = snapshot.target_lane;
    plan.next_state.output_longitudinal = built.output_longitudinal;
    plan.next_state.output_lateral = built.output_lateral;
    plan.next_state.output_x = built.output.next_x;
    plan.next_state.output_y = built.output.next_y;
    plan.next_state.path_stitcher = built.next_path_state;
    plan.next_state.qp_warm_start = qp.proposed_warm_start;
    plan.next_state.reference_speed_mps =
        built.output_longitudinal.empty()
            ? snapshot.frontier.longitudinal.v
            : built.output_longitudinal.back().v;
    plan.next_state.telemetry_time_s = snapshot.telemetry_time_s;
    plan.next_state.committed_generation =
        state_.committed_generation + 1;
    plan.cost.progress_loss_m =
        std::max(0.0, normal_reference_speed_mps.back() *
                          PlanningHorizonS(config_) -
                          qp.trajectory.states.back().s);

    TrajectoryValidationContext validation_context;
    validation_context.input = &snapshot.input;
    validation_context.map = &map;
    validation_context.trajectory = &plan.full_trajectory;
    validation_context.prediction_coverage_s =
        snapshot.frontier.time_from_telemetry_s +
        PlanningHorizonS(config_);
    const std::chrono::steady_clock::time_point validate_start =
        std::chrono::steady_clock::now();
    plan.validation = trajectory_validator_.Validate(validation_context);
    const std::chrono::steady_clock::time_point validate_end =
        std::chrono::steady_clock::now();
    plan.validate_time_ms =
        std::chrono::duration<double, std::milli>(validate_end -
                                                  validate_start)
            .count();
    plan.evaluate_time_ms =
        std::chrono::duration<double, std::milli>(validate_end -
                                                  evaluate_start)
            .count();
    evaluation.has_plan = true;
    evaluation.plan = plan;
    evaluation.validation = plan.validation;
    evaluation.valid = plan.validation.valid;
    evaluation.failure = evaluation.valid
                             ? CandidateFailureReason::kNone
                             : CandidateFailureReason::kHardValidation;
    evaluation.evaluate_time_ms = plan.evaluate_time_ms;
    evaluation.validate_time_ms = plan.validate_time_ms;
    if (!evaluation.valid) {
      const TrajectoryViolation &first = plan.validation.violations.front();
      evaluation.failure_detail =
          std::string(ViolationTypeName(first.type)) + " at t=" +
          std::to_string(first.time_from_telemetry_s) + " magnitude=" +
          std::to_string(first.magnitude) + " value(v/a/j)=" +
          std::to_string(plan.full_trajectory.points[first.point_index]
                             .longitudinal.v) +
          "/" +
          std::to_string(plan.full_trajectory.points[first.point_index]
                             .longitudinal.a) +
          "/" +
          std::to_string(plan.full_trajectory.points[first.point_index]
                             .longitudinal.j) +
          " qp_initial=" +
          std::to_string(plan.qp.trajectory.states.front().v) +
          " snapshot_initial=" +
          std::to_string(snapshot.frontier.longitudinal.v) +
          "/" +
          std::to_string(snapshot.frontier.longitudinal.a) +
          "/" +
          std::to_string(snapshot.frontier.longitudinal.j) +
          " lateral_transition_length=" +
          std::to_string(plan.lateral_diagnostics.transition_length_m) +
          " lateral_progress=" +
          std::to_string(plan.lateral_diagnostics.progress_m) +
          " (" + first.detail + ")";
    }
    return evaluation;
  } catch (const std::exception &error) {
    evaluation.failure = CandidateFailureReason::kInfrastructure;
    evaluation.failure_detail = error.what();
    evaluation.evaluate_time_ms = ElapsedMilliseconds(evaluate_start);
    return evaluation;
  }
}

CandidatePlan PathPlanner::BuildActiveCandidatePlan(
    const PlanningSnapshot &snapshot,
    const FinalBehaviorCandidate &candidate,
    bool first_commit, bool continuation) const {
  if (candidate.status != FinalBehaviorCandidateStatus::kValid ||
      !candidate.qp.success || !candidate.qp.hard_safe ||
      !candidate.validation.valid ||
      candidate.output.next_x.size() != config_.output_points ||
      candidate.output.next_y.size() != config_.output_points ||
      candidate.output_longitudinal.size() != config_.output_points ||
      candidate.output_lateral.size() != config_.output_points) {
    throw std::invalid_argument("invalid active behavior control candidate");
  }

  CandidatePlan plan;
  plan.candidate_id = candidate.candidate_id;
  plan.fallback_level = candidate.emergency_continuation
                            ? FallbackLevel::kMaximumBraking
                            : FallbackLevel::kNormal;
  plan.full_trajectory = candidate.full_trajectory;
  plan.output = candidate.output;
  plan.qp = candidate.qp;
  plan.validation = candidate.validation;
  plan.reference_speed_mps = candidate.reference_speed_mps;
  plan.next_state = state_;
  plan.next_state.target_lane = candidate.target_lane;
  plan.next_state.output_longitudinal = candidate.output_longitudinal;
  plan.next_state.output_lateral = candidate.output_lateral;
  plan.next_state.output_x = candidate.output.next_x;
  plan.next_state.output_y = candidate.output.next_y;
  // Active geometry owns its committed identity. If lane-cruise later becomes
  // effective, PathStitcher deliberately cold-starts a C2 return/continuation
  // from these exact output states instead of pretending its old polynomial
  // generated the active path.
  plan.next_state.path_stitcher = PathStitcherState();
  plan.next_state.path_stitcher.last_output_states =
      candidate.output_lateral;
  plan.next_state.path_stitcher.next_transition_id =
      state_.path_stitcher.next_transition_id;
  // ActiveBehavior owns a separate longitudinal QP and commits its warm start
  // through ActiveBehaviorPlanner::Commit. Preserve the independently evolved
  // LaneCruise warm start so a behavior ST solution cannot seed a different
  // constraint layout on the following cycle.
  plan.next_state.qp_warm_start = state_.qp_warm_start;
  plan.next_state.reference_speed_mps =
      candidate.output_longitudinal.empty()
          ? snapshot.frontier.longitudinal.v
          : candidate.output_longitudinal.back().v;
  plan.next_state.telemetry_time_s = snapshot.telemetry_time_s;
  plan.next_state.committed_generation =
      state_.committed_generation + 1;
  plan.cost.progress_loss_m = std::max(
      0.0, TargetSpeedMps(config_) * PlanningHorizonS(config_) -
               candidate.terminal_progress_m);

  plan.lateral_diagnostics.transition_id =
      candidate.output_lateral.empty()
          ? candidate.candidate_id
          : candidate.output_lateral.back().transition_id;
  plan.lateral_diagnostics.transition_active = true;
  plan.lateral_diagnostics.state_aligned =
      snapshot.historical_plan_aligned;
  plan.lateral_diagnostics.progress_m =
      candidate.output_lateral.empty()
          ? 0.0
          : candidate.output_lateral.back().correction_progress_m;
  plan.lateral_diagnostics.transition_length_m =
      candidate.spatial_path.geometry
          .transition_completion_path_progress_m;
  plan.lateral_diagnostics.remaining_m = std::max(
      0.0, plan.lateral_diagnostics.transition_length_m -
               plan.lateral_diagnostics.progress_m);
  plan.lateral_diagnostics.frontier_d = snapshot.frontier.d_m;
  plan.behavior_lane_change = true;
  plan.first_behavior_commit = first_commit;
  plan.behavior_continuation = continuation;
  plan.behavior_source_lane = candidate.source_lane;
  plan.behavior_target_lane = candidate.target_lane;
  plan.behavior_gap = candidate.gap;
  return plan;
}

MinimumRiskDispatch PathPlanner::BuildActiveMinimumRiskDispatch(
    const PlanningSnapshot &snapshot,
    const FinalBehaviorCandidate &candidate) const {
  if (!candidate.emergency_continuation || !candidate.qp.success ||
      !candidate.validation.HasOnlyCollisionViolations() ||
      candidate.output.next_x.size() != config_.output_points ||
      candidate.output.next_y.size() != config_.output_points ||
      candidate.output_longitudinal.size() != config_.output_points ||
      candidate.output_lateral.size() != config_.output_points) {
    throw std::invalid_argument(
        "invalid active behavior minimum-risk candidate");
  }

  MinimumRiskDispatch dispatch;
  dispatch.candidate_id = candidate.candidate_id;
  dispatch.full_trajectory = candidate.full_trajectory;
  dispatch.output = candidate.output;
  dispatch.qp = candidate.qp;
  dispatch.validation = candidate.validation;
  dispatch.reference_speed_mps = candidate.reference_speed_mps;
  dispatch.next_emergency_state.target_lane = candidate.target_lane;
  dispatch.next_emergency_state.output_longitudinal =
      candidate.output_longitudinal;
  dispatch.next_emergency_state.output_lateral = candidate.output_lateral;
  dispatch.next_emergency_state.output_x = candidate.output.next_x;
  dispatch.next_emergency_state.output_y = candidate.output.next_y;
  dispatch.next_emergency_state.path_stitcher = PathStitcherState();
  dispatch.next_emergency_state.path_stitcher.last_output_states =
      candidate.output_lateral;
  dispatch.next_emergency_state.path_stitcher.next_transition_id =
      state_.path_stitcher.next_transition_id;
  dispatch.next_emergency_state.reference_speed_mps =
      candidate.output_longitudinal.empty()
          ? snapshot.frontier.longitudinal.v
          : candidate.output_longitudinal.back().v;
  dispatch.next_emergency_state.telemetry_time_s = snapshot.telemetry_time_s;
  dispatch.lateral_diagnostics.transition_id =
      candidate.output_lateral.empty()
          ? candidate.candidate_id
          : candidate.output_lateral.back().transition_id;
  dispatch.lateral_diagnostics.transition_active = true;
  dispatch.lateral_diagnostics.state_aligned =
      snapshot.historical_plan_aligned;
  dispatch.lateral_diagnostics.frontier_d = snapshot.frontier.d_m;

  std::set<double> collision_objects;
  double first_collision_object_id = 0.0;
  dispatch.risk.predicted_first_collision_time_s =
      std::numeric_limits<double>::infinity();
  for (const TrajectoryViolation &violation :
       dispatch.validation.violations) {
    if (violation.type != ViolationType::kCollision) {
      continue;
    }
    collision_objects.insert(violation.object_id);
    if (violation.time_from_telemetry_s <
        dispatch.risk.predicted_first_collision_time_s) {
      dispatch.risk.predicted_first_collision_time_s =
          violation.time_from_telemetry_s;
      first_collision_object_id = violation.object_id;
    }
    dispatch.risk.maximum_overlap_m =
        std::max(dispatch.risk.maximum_overlap_m, violation.magnitude);
  }
  dispatch.risk.predicted_collision_object_count = collision_objects.size();
  if (!std::isfinite(dispatch.risk.predicted_first_collision_time_s)) {
    dispatch.risk.predicted_first_collision_time_s = 0.0;
  }
  for (const DetectedVehicle &vehicle : snapshot.input.traffic) {
    if (vehicle.id == first_collision_object_id) {
      dispatch.risk.predicted_relative_collision_speed_mps =
          std::fabs(snapshot.frontier.longitudinal.v -
                    std::hypot(vehicle.vx_mps, vehicle.vy_mps));
      break;
    }
  }
  dispatch.risk.selected_reason =
      "maximum braking on immutable committed geometry minimizes risk "
      "when refreshed ST evidence has no ordinary continuation";
  return dispatch;
}

MinimumRiskDispatch PathPlanner::EvaluateMinimumRisk(
    const PlanningSnapshot &snapshot, const MapData &map,
    const FixedSpatialPath &fixed_path,
    const std::vector<PredictedObstacle> &obstacles,
    std::uint64_t candidate_id) const {
  const std::chrono::steady_clock::time_point evaluate_start =
      std::chrono::steady_clock::now();
  LongitudinalQpInput qp_input;
  qp_input.initial_speed_mps =
      std::max(0.0, snapshot.frontier.longitudinal.v);
  qp_input.initial_acceleration_mps2 = snapshot.frontier.longitudinal.a;
  qp_input.initial_jerk_valid =
      snapshot.frontier.state_source == PlanningStateSource::kExactInherited;
  qp_input.initial_jerk_mps3 = snapshot.frontier.longitudinal.j;
  qp_input.reference_speed_mps.assign(config_.qp_horizon_steps + 1, 0.0);
  qp_input.obstacles = obstacles;
  qp_input.safety_policy = LongitudinalSafetyPolicy::kMaximumBraking;
  qp_input.allow_physical_collision_violation = true;
  LongitudinalQpResult qp =
      longitudinal_qp_.Evaluate(qp_input, QpWarmStartState());
  qp.trajectory.emergency = true;
  if (!qp.success) {
    throw std::runtime_error("minimum-risk QP failed: " + qp.status);
  }

  const BuiltTrajectory built = BuildFullTrajectory(
      snapshot, config_, map, fixed_path, qp, path_stitcher_);
  MinimumRiskDispatch dispatch;
  dispatch.candidate_id = candidate_id;
  dispatch.full_trajectory = built.full;
  dispatch.output = built.output;
  dispatch.qp = qp;
  dispatch.obstacles = obstacles;
  dispatch.reference_speed_mps = qp_input.reference_speed_mps;
  dispatch.lateral_diagnostics = built.lateral_diagnostics;
  dispatch.next_emergency_state.target_lane = snapshot.target_lane;
  dispatch.next_emergency_state.output_longitudinal =
      built.output_longitudinal;
  dispatch.next_emergency_state.output_lateral = built.output_lateral;
  dispatch.next_emergency_state.output_x = built.output.next_x;
  dispatch.next_emergency_state.output_y = built.output.next_y;
  dispatch.next_emergency_state.path_stitcher = built.next_path_state;
  dispatch.next_emergency_state.emergency_qp_warm_start =
      qp.proposed_warm_start;
  dispatch.next_emergency_state.reference_speed_mps =
      built.output_longitudinal.empty()
          ? snapshot.frontier.longitudinal.v
          : built.output_longitudinal.back().v;
  dispatch.next_emergency_state.telemetry_time_s = snapshot.telemetry_time_s;

  TrajectoryValidationContext validation_context;
  validation_context.input = &snapshot.input;
  validation_context.map = &map;
  validation_context.trajectory = &dispatch.full_trajectory;
  validation_context.prediction_coverage_s =
      snapshot.frontier.time_from_telemetry_s + PlanningHorizonS(config_);
  const std::chrono::steady_clock::time_point validate_start =
      std::chrono::steady_clock::now();
  dispatch.validation = trajectory_validator_.Validate(validation_context);
  const std::chrono::steady_clock::time_point validate_end =
      std::chrono::steady_clock::now();
  dispatch.validate_time_ms =
      std::chrono::duration<double, std::milli>(validate_end - validate_start)
          .count();
  dispatch.evaluate_time_ms =
      std::chrono::duration<double, std::milli>(validate_end - evaluate_start)
          .count();
  dispatch.exclusive_evaluate_time_ms = dispatch.evaluate_time_ms;
  dispatch.exclusive_validate_time_ms = dispatch.validate_time_ms;

  std::set<double> collision_objects;
  double first_collision_object_id = 0.0;
  dispatch.risk.predicted_first_collision_time_s =
      std::numeric_limits<double>::infinity();
  for (const TrajectoryViolation &violation :
       dispatch.validation.violations) {
    if (violation.type != ViolationType::kCollision) {
      continue;
    }
    collision_objects.insert(violation.object_id);
    if (violation.time_from_telemetry_s <
        dispatch.risk.predicted_first_collision_time_s) {
      dispatch.risk.predicted_first_collision_time_s =
          violation.time_from_telemetry_s;
      first_collision_object_id = violation.object_id;
    }
    dispatch.risk.maximum_overlap_m =
        std::max(dispatch.risk.maximum_overlap_m, violation.magnitude);
  }
  dispatch.risk.predicted_collision_object_count =
      collision_objects.size();
  if (!std::isfinite(dispatch.risk.predicted_first_collision_time_s)) {
    dispatch.risk.predicted_first_collision_time_s = 0.0;
  }
  if (!collision_objects.empty()) {
    for (const DetectedVehicle &vehicle : snapshot.input.traffic) {
      if (vehicle.id == first_collision_object_id) {
        dispatch.risk.predicted_relative_collision_speed_mps =
            std::fabs(snapshot.frontier.longitudinal.v -
                      std::hypot(vehicle.vx_mps, vehicle.vy_mps));
        break;
      }
    }
  }
  dispatch.risk.selected_reason =
      "maximum braking minimizes explicit collision-separation violation "
      "without relaxing road or dynamics bounds";
  return dispatch;
}

void PathPlanner::CommitCandidate(const CandidatePlan &winner) {
  if (!winner.validation.valid || !winner.qp.success ||
      !winner.qp.hard_safe ||
      winner.output.next_x.size() != config_.output_points ||
      winner.output.next_y.size() != config_.output_points) {
    throw std::logic_error("attempted to commit an invalid candidate");
  }
  state_ = winner.next_state;
  if (!winner.behavior_lane_change) {
    longitudinal_qp_.CommitWarmStart(winner.qp);
  }
}

void PathPlanner::DispatchEmergency(const MinimumRiskDispatch &dispatch) {
  if (!dispatch.qp.success ||
      !dispatch.validation.HasOnlyCollisionViolations() ||
      dispatch.output.next_x.size() != config_.output_points ||
      dispatch.output.next_y.size() != config_.output_points) {
    throw std::logic_error("attempted to dispatch invalid minimum-risk output");
  }
  state_.output_longitudinal =
      dispatch.next_emergency_state.output_longitudinal;
  state_.target_lane = dispatch.next_emergency_state.target_lane;
  state_.output_lateral = dispatch.next_emergency_state.output_lateral;
  state_.output_x = dispatch.next_emergency_state.output_x;
  state_.output_y = dispatch.next_emergency_state.output_y;
  state_.path_stitcher = dispatch.next_emergency_state.path_stitcher;
  state_.reference_speed_mps =
      dispatch.next_emergency_state.reference_speed_mps;
  state_.telemetry_time_s = dispatch.next_emergency_state.telemetry_time_s;
  ++state_.committed_generation;
}

PathPlanner::LaneCruiseCycleContext PathPlanner::EvaluateLaneCruiseCandidates(
    PlanningSnapshot *snapshot, const MapData &map,
    PlanningCycleDecision *decision) const {
  if (snapshot == nullptr || decision == nullptr) {
    throw std::invalid_argument("lane-cruise cycle output is null");
  }
  LaneCruiseCycleContext context;
  const double lane_center_d =
      (static_cast<double>(snapshot->target_lane) + 0.5) *
      config_.lane_width_meters;
  context.fixed_path = path_stitcher_.Prepare(
      snapshot->input, snapshot->frontier.road_s_unwrapped_m,
      snapshot->frontier.d_m, lane_center_d,
      std::max(TargetSpeedMps(config_), snapshot->frontier.longitudinal.v), map,
      state_.path_stitcher, snapshot->retained_lateral_states,
      snapshot->historical_plan_aligned);
  context.obstacles = PredictRelevantTraffic(
      snapshot->input, snapshot->frontier.road_s_unwrapped_m,
      snapshot->frontier.d_m, lane_center_d, map, MakeTrafficConfig(config_),
      &context.fixed_path);
  const SpeedReferenceResult reference = speed_reference_generator_.Generate(
      context.obstacles, std::max(0.0, snapshot->frontier.longitudinal.v));
  const std::vector<double> reference_speed_mps = ReferenceSpeeds(
      reference, config_.qp_horizon_steps + 1, TargetSpeedMps(config_));

  const LongitudinalSafetyPolicy policies[3] = {
      LongitudinalSafetyPolicy::kNormalOperational,
      LongitudinalSafetyPolicy::kDegradedBraking,
      LongitudinalSafetyPolicy::kMaximumBraking};
  for (std::size_t index = 0; index < 3; ++index) {
    const CandidateEvaluation evaluation = EvaluateLaneCruise(
        *snapshot, map, context.fixed_path, context.obstacles,
        reference_speed_mps, policies[index], plan_cycle_ * 10 + index + 1);
    context.evaluate_time_ms += evaluation.evaluate_time_ms;
    context.validate_time_ms += evaluation.validate_time_ms;
    decision->ordinary_evaluations.push_back(evaluation);
    if (evaluation.valid) {
      decision->disposition = PlanDisposition::kValidatedCandidate;
      decision->has_validated_candidate = true;
      decision->validated_candidate = evaluation.plan;
      decision->validated_candidate.evaluate_time_ms = context.evaluate_time_ms;
      decision->validated_candidate.validate_time_ms = context.validate_time_ms;
      break;
    }
  }

  snapshot->control_lane_cruise_backup_valid =
      decision->has_validated_candidate &&
      decision->validated_candidate.qp.hard_safe &&
      decision->validated_candidate.validation.valid;
  snapshot->control_lane_cruise_handoff_valid =
      snapshot->control_lane_cruise_backup_valid &&
      decision->validated_candidate.fallback_level == FallbackLevel::kNormal;
  return context;
}

void PathPlanner::EvaluateActiveBehaviorCandidate(
    const PlanningSnapshot &snapshot, const MapData &map,
    PlanningCycleDecision *decision, ActiveBehaviorTransaction *transaction) {
  if (decision == nullptr || transaction == nullptr) {
    throw std::invalid_argument("active behavior cycle output is null");
  }
  const bool should_evaluate =
      effective_mode_ == PlannerOperatingMode::kBehaviorActive &&
      (snapshot.control_lane_cruise_backup_valid ||
       active_behavior_planner_->ManeuverInProgress());
  if (!should_evaluate) {
    return;
  }

  active_behavior_evaluation_attempted_ = true;
  try {
    const ActiveBehaviorCycleResult active = active_behavior_planner_->Evaluate(
        snapshot, map,
        requested_mode_ == PlannerOperatingMode::kBehaviorActive &&
            snapshot.reset_reason == PlanningStateResetReason::kNone);
    active_behavior_evaluation_succeeded_ = true;
    transaction->open = true;
    const bool active_emergency =
        active.has_control_candidate &&
        active.control_candidate.emergency_continuation;
    if (active.has_control_candidate &&
        (!active_emergency || !snapshot.control_lane_cruise_backup_valid)) {
      CandidatePlan active_plan =
          BuildActiveCandidatePlan(snapshot, active.control_candidate,
                                   active.first_commit, active.continuation);
      CandidateEvaluation active_evaluation;
      active_evaluation.candidate_id = active_plan.candidate_id;
      active_evaluation.safety_policy =
          LongitudinalSafetyPolicy::kNormalOperational;
      active_evaluation.valid = true;
      active_evaluation.has_plan = true;
      active_evaluation.plan = active_plan;
      active_evaluation.validation = active_plan.validation;
      decision->ordinary_evaluations.push_back(active_evaluation);
      decision->validated_candidate = std::move(active_plan);
      decision->disposition = PlanDisposition::kValidatedCandidate;
      decision->has_validated_candidate = true;
      transaction->candidate_dispatched = true;
    } else if (!snapshot.control_lane_cruise_backup_valid &&
               active.has_minimum_risk_candidate) {
      decision->minimum_risk = BuildActiveMinimumRiskDispatch(
          snapshot, active.minimum_risk_candidate);
      decision->disposition = PlanDisposition::kMinimumRiskDispatch;
      decision->has_minimum_risk = true;
      transaction->candidate_dispatched = true;
    } else if (!snapshot.control_lane_cruise_backup_valid) {
      active_behavior_error_stage_ = "NoControlCandidate";
      active_behavior_error_detail_ =
          std::string("phase=") +
          BehaviorManeuverPhaseName(active.phase_before) +
          ", target_only=" + (active.target_only ? "true" : "false") +
          ", target_stable=" + (active.target_stable ? "true" : "false") +
          ", final_candidates=" +
          std::to_string(active.final_candidates.size());
      if (!active.final_candidates.empty()) {
        active_behavior_error_detail_ +=
            ", final_status=" +
            std::string(FinalBehaviorCandidateStatusName(
                active.final_candidates.front().status)) +
            ", detail=" + active.final_candidates.front().rejection_detail;
      }
    }
  } catch (const std::exception &error) {
    active_behavior_error_stage_ = active_behavior_evaluation_succeeded_
                                       ? "ControlCandidateAdaptation"
                                       : "Evaluate";
    active_behavior_error_detail_ = error.what();
    active_behavior_planner_->Discard(plan_cycle_);
    transaction->open = false;
  }
}

void PathPlanner::CommitSelectedDecision(
    PlanningCycleDecision *decision, ActiveBehaviorTransaction *transaction) {
  if (decision == nullptr || transaction == nullptr) {
    throw std::invalid_argument("control decision output is null");
  }
  if (decision->has_validated_candidate) {
    CommitCandidate(decision->validated_candidate);
    if (transaction->open) {
      try {
        active_behavior_planner_->Commit(plan_cycle_,
                                         transaction->candidate_dispatched);
      } catch (const std::exception &error) {
        // The control output is already committed; isolate behavior
        // bookkeeping failures from the selected trajectory.
        active_behavior_error_stage_ = "Commit";
        active_behavior_error_detail_ = error.what();
        active_behavior_planner_->Discard(plan_cycle_);
        requested_mode_ = PlannerOperatingMode::kLaneCruiseOnly;
        effective_mode_ = PlannerOperatingMode::kLaneCruiseOnly;
      }
      transaction->open = false;
    }
    if (requested_mode_ == PlannerOperatingMode::kLaneCruiseOnly &&
        effective_mode_ == PlannerOperatingMode::kBehaviorActive &&
        !active_behavior_planner_->ManeuverInProgress()) {
      effective_mode_ = PlannerOperatingMode::kLaneCruiseOnly;
    }
    return;
  }

  if (decision->disposition == PlanDisposition::kMinimumRiskDispatch &&
      decision->has_minimum_risk) {
    DispatchEmergency(decision->minimum_risk);
    if (transaction->open) {
      try {
        active_behavior_planner_->Commit(plan_cycle_,
                                         transaction->candidate_dispatched);
      } catch (const std::exception &error) {
        active_behavior_error_stage_ = "MinimumRiskCommit";
        active_behavior_error_detail_ = error.what();
        active_behavior_planner_->Discard(plan_cycle_);
        requested_mode_ = PlannerOperatingMode::kLaneCruiseOnly;
        effective_mode_ = PlannerOperatingMode::kLaneCruiseOnly;
      }
      transaction->open = false;
    }
    return;
  }

  if (transaction->open) {
    active_behavior_planner_->Discard(plan_cycle_);
    transaction->open = false;
  }
}

void PathPlanner::ResolveLaneCruiseFailure(
    const PlanningSnapshot &snapshot, const MapData &map,
    const LaneCruiseCycleContext &lane_cruise,
    PlanningCycleDecision *decision) {
  if (decision == nullptr || decision->has_validated_candidate ||
      decision->has_minimum_risk) {
    return;
  }
  if (decision->ordinary_evaluations.empty()) {
    throw std::logic_error("lane-cruise evaluation produced no candidates");
  }

  const CandidateEvaluation &maximum = decision->ordinary_evaluations.back();
  if (!IsProvenPhysicalInfeasibility(maximum)) {
    decision->disposition = PlanDisposition::kInfrastructureFailure;
    decision->has_infrastructure_failure = true;
    decision->infrastructure_failure.reason =
        "ordinary candidates failed without proof that collision is "
        "physically unavoidable";
    for (const CandidateEvaluation &failed : decision->ordinary_evaluations) {
      decision->infrastructure_failure.reason +=
          "; candidate failure: " + failed.failure_detail;
    }
    return;
  }

  MinimumRiskDispatch minimum_risk =
      EvaluateMinimumRisk(snapshot, map, lane_cruise.fixed_path,
                          lane_cruise.obstacles, plan_cycle_ * 10 + 4);
  minimum_risk.evaluate_time_ms += lane_cruise.evaluate_time_ms;
  minimum_risk.validate_time_ms += lane_cruise.validate_time_ms;
  // Preserve the evaluated MRM candidate for diagnostics even if the later
  // consistency check classifies the cycle as infrastructure failure.
  decision->minimum_risk = minimum_risk;
  if (!minimum_risk.validation.HasOnlyCollisionViolations()) {
    decision->disposition = PlanDisposition::kInfrastructureFailure;
    decision->has_infrastructure_failure = true;
    decision->infrastructure_failure.reason =
        minimum_risk.validation.valid
            ? "minimum-risk solver disagrees with physical "
              "infeasibility evidence; qp collision violation=" +
                  std::to_string(
                      minimum_risk.qp.maximum_collision_violation_meters) +
                  ", Cartesian minimum separation=" +
                  std::to_string(
                      minimum_risk.validation.minimum_collision_margin_m)
            : std::string("minimum-risk output has non-collision "
                          "violation: ") +
                  ViolationTypeName(
                      minimum_risk.validation.FirstViolationType());
    if (!minimum_risk.validation.valid) {
      for (const TrajectoryViolation &violation :
           minimum_risk.validation.violations) {
        decision->infrastructure_failure.reason +=
            std::string("; ") + ViolationTypeName(violation.type) +
            " at t=" + std::to_string(violation.time_from_telemetry_s) +
            " magnitude=" + std::to_string(violation.magnitude) + " (" +
            violation.detail + ")";
        if (violation.point_index <
            minimum_risk.full_trajectory.points.size()) {
          const LongitudinalState &state =
              minimum_risk.full_trajectory.points[violation.point_index]
                  .longitudinal;
          decision->infrastructure_failure.reason +=
              " value(v/a/j)=" + std::to_string(state.v) + "/" +
              std::to_string(state.a) + "/" + std::to_string(state.j);
        }
      }
    }
    return;
  }

  decision->disposition = PlanDisposition::kMinimumRiskDispatch;
  decision->has_minimum_risk = true;
  DispatchEmergency(decision->minimum_risk);
}

PlanningCycleDecision PathPlanner::PlanCycle(const PlannerInput &input,
                                             const MapData &map) {
  ++plan_cycle_;
  active_behavior_evaluation_attempted_ = false;
  active_behavior_evaluation_succeeded_ = false;
  active_behavior_error_stage_.clear();
  active_behavior_error_detail_.clear();
  PlanningCycleDecision decision;
  PlanningSnapshot snapshot;
  snapshot.cycle = plan_cycle_;
  ActiveBehaviorTransaction active_transaction;
  try {
    std::string map_error;
    if (!ValidateMap(map, &map_error)) {
      throw std::invalid_argument("invalid map: " + map_error);
    }
    snapshot = BuildSnapshot(input, map);
    if (snapshot.reset_reason != PlanningStateResetReason::kNone) {
      active_behavior_planner_->HandleControlHistoryReset();
    }
    const LaneCruiseCycleContext lane_cruise =
        EvaluateLaneCruiseCandidates(&snapshot, map, &decision);
    EvaluateActiveBehaviorCandidate(snapshot, map, &decision,
                                    &active_transaction);

    CommitSelectedDecision(&decision, &active_transaction);

    ResolveLaneCruiseFailure(snapshot, map, lane_cruise, &decision);
  } catch (const std::exception &error) {
    if (active_transaction.open) {
      active_behavior_planner_->Discard(plan_cycle_);
    }
    if (active_behavior_evaluation_attempted_ &&
        active_behavior_error_detail_.empty()) {
      active_behavior_error_stage_ = "ControlIntegration";
      active_behavior_error_detail_ = error.what();
    }
    decision.disposition = PlanDisposition::kInfrastructureFailure;
    decision.has_validated_candidate = false;
    decision.has_minimum_risk = false;
    decision.has_infrastructure_failure = true;
    decision.infrastructure_failure.reason = error.what();
  }

  last_decision_ = decision;
  RecordDecisionDiagnostics(snapshot, decision);
  return decision;
}

PlannerOutput PathPlanner::Plan(const PlannerInput &input,
                                const MapData &map) {
  const PlanningCycleDecision decision = PlanCycle(input, map);
  if (decision.disposition == PlanDisposition::kValidatedCandidate &&
      decision.has_validated_candidate) {
    return decision.validated_candidate.output;
  }
  if (decision.disposition == PlanDisposition::kMinimumRiskDispatch &&
      decision.has_minimum_risk) {
    return decision.minimum_risk.output;
  }
  throw std::runtime_error(
      "planning infrastructure failure: " +
      (decision.has_infrastructure_failure
           ? decision.infrastructure_failure.reason
           : std::string("missing failure details")));
}

void PathPlanner::RecordDecisionDiagnostics(
    const PlanningSnapshot &snapshot,
    const PlanningCycleDecision &decision) {
  try {
    const auto populate_behavior_diagnostics =
        [this, &decision, &snapshot](PlannerCycleDiagnostics *diagnostics) {
          const ActiveBehaviorDiagnostics behavior =
              active_behavior_planner_->diagnostics();
          diagnostics->operating_mode = effective_mode_;
          diagnostics->requested_operating_mode = requested_mode_;
          diagnostics->behavior_phase =
              BehaviorManeuverPhaseName(behavior.phase);
          diagnostics->behavior_commit_cycle = behavior.commit_cycle;
          diagnostics->behavior_stable_proposal_cycles =
              behavior.stable_proposal_cycles;
          diagnostics->behavior_target_stable_cycles =
              behavior.target_stable_cycles;
          diagnostics->behavior_oscillation_count =
              behavior.oscillation_count;
          diagnostics->behavior_completed_maneuver_count =
              behavior.completed_maneuver_count;
          diagnostics->behavior_cancelled_proposal_count =
              behavior.cancelled_proposal_count;
          diagnostics->behavior_evaluation_attempted =
              active_behavior_evaluation_attempted_;
          diagnostics->behavior_evaluation_succeeded =
              active_behavior_evaluation_succeeded_;
          diagnostics->behavior_error_stage = active_behavior_error_stage_;
          diagnostics->behavior_error_detail = active_behavior_error_detail_;
          PopulateActiveBehaviorResultDiagnostics(
              behavior, decision, snapshot.cycle, diagnostics);
          if (decision.disposition ==
                  PlanDisposition::kValidatedCandidate &&
              decision.has_validated_candidate) {
            const CandidatePlan &selected =
                decision.validated_candidate;
            diagnostics->behavior_lane_change_selected =
                selected.behavior_lane_change;
            diagnostics->behavior_first_commit =
                selected.first_behavior_commit;
            diagnostics->behavior_continuation =
                selected.behavior_continuation;
            diagnostics->behavior_source_lane =
                selected.behavior_source_lane;
            diagnostics->behavior_target_lane =
                selected.behavior_target_lane;
          }
        };
    if (decision.disposition == PlanDisposition::kInfrastructureFailure) {
      last_diagnostics_ = PlannerCycleDiagnostics();
      last_diagnostics_.cycle = snapshot.cycle;
      last_diagnostics_.plan_disposition = decision.disposition;
      last_diagnostics_.fallback_level =
          FallbackLevel::kInfrastructureFailure;
      last_diagnostics_.original_previous_path_size =
          snapshot.original_previous_path_points;
      last_diagnostics_.retained_prefix_points =
          snapshot.retained_prefix_points;
      last_diagnostics_.previous_path_size =
          snapshot.retained_prefix_points;
      last_diagnostics_.planning_frontier_delay_s =
          snapshot.frontier.time_from_telemetry_s;
      last_diagnostics_.state_source = snapshot.frontier.state_source;
      last_diagnostics_.state_reset_reason = snapshot.reset_reason;
      last_diagnostics_.historical_plan_aligned =
          snapshot.historical_plan_aligned;
      last_diagnostics_.traffic_vehicle_count =
          snapshot.input.traffic.size();
      last_diagnostics_.infrastructure_failure = true;
      last_diagnostics_.infrastructure_failure_reason =
          decision.has_infrastructure_failure
              ? decision.infrastructure_failure.reason
              : "missing infrastructure failure details";
      PopulateControlCandidateDiagnostics(decision, &last_diagnostics_);
      populate_behavior_diagnostics(&last_diagnostics_);
      runtime_monitor_.Record(last_diagnostics_);
      return;
    }

    const PlannerOutput *output = nullptr;
    const LongitudinalQpResult *qp = nullptr;
    const std::vector<PredictedObstacle> *obstacles = nullptr;
    const std::vector<double> *reference = nullptr;
    const LateralStitchDiagnostics *lateral = nullptr;
    const ValidationResult *validation = nullptr;
    std::uint64_t candidate_id = 0;
    FallbackLevel fallback_level = FallbackLevel::kInfrastructureFailure;
    std::size_t full_trajectory_points = 0;
    double evaluate_time_ms = 0.0;
    double validate_time_ms = 0.0;
    if (decision.disposition == PlanDisposition::kValidatedCandidate) {
      output = &decision.validated_candidate.output;
      qp = &decision.validated_candidate.qp;
      obstacles = &decision.validated_candidate.obstacles;
      reference = &decision.validated_candidate.reference_speed_mps;
      lateral = &decision.validated_candidate.lateral_diagnostics;
      validation = &decision.validated_candidate.validation;
      candidate_id = decision.validated_candidate.candidate_id;
      fallback_level = decision.validated_candidate.fallback_level;
      full_trajectory_points =
          decision.validated_candidate.full_trajectory.points.size();
      evaluate_time_ms = decision.validated_candidate.evaluate_time_ms;
      validate_time_ms = decision.validated_candidate.validate_time_ms;
    } else {
      output = &decision.minimum_risk.output;
      qp = &decision.minimum_risk.qp;
      obstacles = &decision.minimum_risk.obstacles;
      reference = &decision.minimum_risk.reference_speed_mps;
      lateral = &decision.minimum_risk.lateral_diagnostics;
      validation = &decision.minimum_risk.validation;
      candidate_id = decision.minimum_risk.candidate_id;
      fallback_level = FallbackLevel::kMinimumRisk;
      full_trajectory_points =
          decision.minimum_risk.full_trajectory.points.size();
      evaluate_time_ms = decision.minimum_risk.evaluate_time_ms;
      validate_time_ms = decision.minimum_risk.validate_time_ms;
    }
    const double lane_center_d =
        (static_cast<double>(snapshot.target_lane) + 0.5) *
        config_.lane_width_meters;
    last_diagnostics_ = BuildPlannerDiagnostics(
        snapshot.cycle, snapshot.input, *output,
        snapshot.retained_prefix_points, snapshot.frontier.longitudinal,
        snapshot.frontier.road_s_unwrapped_m, snapshot.frontier.d_m,
        lane_center_d, *obstacles, *reference, *qp,
        state_.output_longitudinal, MakeMonitorLimits(config_), *lateral,
        state_.output_lateral, snapshot.historical_plan_aligned);
    if (active_behavior_planner_->ManeuverInProgress()) {
      // Lane-center deviation is a LaneCruise invariant. During an accepted
      // lane change the full road-boundary validator owns lateral safety.
      last_diagnostics_.lane_deviation_violation = false;
    }
    last_diagnostics_.candidate_id = candidate_id;
    last_diagnostics_.plan_disposition = decision.disposition;
    last_diagnostics_.fallback_level = fallback_level;
    last_diagnostics_.original_previous_path_size =
        snapshot.original_previous_path_points;
    last_diagnostics_.retained_prefix_points =
        snapshot.retained_prefix_points;
    last_diagnostics_.planning_frontier_delay_s =
        snapshot.frontier.time_from_telemetry_s;
    last_diagnostics_.state_source = snapshot.frontier.state_source;
    last_diagnostics_.state_reset_reason = snapshot.reset_reason;
    last_diagnostics_.validation_valid = validation->valid;
    last_diagnostics_.first_violation_type =
        ViolationTypeName(validation->FirstViolationType());
    last_diagnostics_.first_violation_time_s =
        validation->FirstViolationTimeS();
    last_diagnostics_.traffic_vehicle_count =
        snapshot.input.traffic.size();
    last_diagnostics_.shielded_same_lane_rear_vehicle_ids =
        validation->shielded_same_lane_rear_vehicle_ids;
    last_diagnostics_.validation_minimum_collision_margin_m =
        validation->minimum_collision_margin_m;
    const std::vector<CollisionEventDiagnostics> selected_collisions =
        BuildCollisionEventDiagnostics(*validation, *obstacles);
    if (!selected_collisions.empty()) {
      last_diagnostics_.has_first_collision_evidence = true;
      last_diagnostics_.first_collision = selected_collisions.front();
    }
    last_diagnostics_.minimum_physical_margin_m =
        qp->minimum_physical_margin_meters;
    last_diagnostics_.minimum_operational_margin_m =
        qp->minimum_operational_margin_meters;
    last_diagnostics_.maximum_headway_slack_m =
        qp->maximum_headway_slack_meters;
    last_diagnostics_.full_trajectory_points = full_trajectory_points;
    last_diagnostics_.evaluate_time_ms = evaluate_time_ms;
    last_diagnostics_.validate_time_ms = validate_time_ms;
    if (decision.disposition == PlanDisposition::kMinimumRiskDispatch) {
      last_diagnostics_.collision_unavoidable = true;
      last_diagnostics_.minimum_risk_dispatch = true;
      last_diagnostics_.minimum_risk_reason =
          decision.minimum_risk.risk.selected_reason;
      last_diagnostics_.predicted_collision_object_count =
          decision.minimum_risk.risk.predicted_collision_object_count;
      last_diagnostics_.predicted_first_collision_time_s =
          decision.minimum_risk.risk.predicted_first_collision_time_s;
      last_diagnostics_.predicted_relative_collision_speed_mps =
          decision.minimum_risk.risk
              .predicted_relative_collision_speed_mps;
    }
    PopulateControlCandidateDiagnostics(decision, &last_diagnostics_);
    populate_behavior_diagnostics(&last_diagnostics_);
    runtime_monitor_.Record(last_diagnostics_);
  } catch (const std::exception &) {
    // Diagnostics and logging are deliberately outside the commit boundary.
    // A logging failure cannot change the selected control output or state.
  }
}
