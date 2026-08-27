#include "traffic_prediction.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {

constexpr double kGridTolerance = 1e-10;
constexpr double kMinimumFrontConservativeAccelerationMps2 = -2.0;
constexpr double kMaximumRearConservativeAccelerationMps2 = 1.0;

double Clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

void ValidateConfig(const FullLaneTrafficPredictionConfig &config) {
  if (!std::isfinite(config.time_step_s) || config.time_step_s <= 0.0 ||
      !std::isfinite(config.planning_horizon_s) ||
      config.planning_horizon_s <= 0.0 ||
      !std::isfinite(config.post_maneuver_observation_s) ||
      config.post_maneuver_observation_s < 0.0 ||
      !std::isfinite(config.front_conservative_deceleration_mps2) ||
      config.front_conservative_deceleration_mps2 >= 0.0 ||
      config.front_conservative_deceleration_mps2 <
          kMinimumFrontConservativeAccelerationMps2 ||
      !std::isfinite(config.rear_conservative_acceleration_mps2) ||
      config.rear_conservative_acceleration_mps2 <= 0.0 ||
      config.rear_conservative_acceleration_mps2 >
          kMaximumRearConservativeAccelerationMps2 ||
      !std::isfinite(config.longitudinal_acceleration_duration_s) ||
      config.longitudinal_acceleration_duration_s <= 0.0 ||
      !std::isfinite(config.maximum_predicted_speed_mps) ||
      config.maximum_predicted_speed_mps <= 0.0 ||
      !std::isfinite(config.lateral_motion_threshold_mps) ||
      config.lateral_motion_threshold_mps < 0.0 ||
      !std::isfinite(config.lateral_continuation_duration_s) ||
      config.lateral_continuation_duration_s <= 0.0 ||
      !std::isfinite(config.maximum_abs_lateral_prediction_rate_mps) ||
      config.maximum_abs_lateral_prediction_rate_mps <= 0.0 ||
      !std::isfinite(config.longitudinal_uncertainty_growth_mps) ||
      config.longitudinal_uncertainty_growth_mps < 0.0 ||
      !std::isfinite(config.lane_width_m) || config.lane_width_m <= 0.0 ||
      config.lane_count <= 0 || config.lane_count > 63 ||
      config.maximum_source_tracks == 0 ||
      config.maximum_prediction_nodes < 2) {
    throw std::invalid_argument(
        "invalid full-lane traffic prediction configuration");
  }
}

struct PredictedKinematics {
  double road_s_unwrapped_m = 0.0;
  double d_m = 0.0;
  double road_s_rate_mps = 0.0;
  double d_rate_mps = 0.0;
  double acceleration_mps2 = 0.0;
};

PredictedKinematics KinematicsAt(
    const TrackedVehicleState &track,
    TrafficPredictionHypothesis hypothesis, double time_s,
    const FullLaneTrafficPredictionConfig &config) {
  PredictedKinematics result;
  const double initial_speed =
      Clamp(track.road_s_rate_mps, 0.0,
            config.maximum_predicted_speed_mps);
  result.road_s_unwrapped_m =
      track.road_s_unwrapped_m + initial_speed * time_s;
  result.d_m = track.d_m;
  result.road_s_rate_mps = initial_speed;

  if (hypothesis ==
      TrafficPredictionHypothesis::kFrontConservativeBraking) {
    const double acceleration =
        config.front_conservative_deceleration_mps2;
    const double stop_time = initial_speed / -acceleration;
    const double acceleration_duration =
        std::min(stop_time,
                 config.longitudinal_acceleration_duration_s);
    const double accelerating_time =
        std::min(time_s, acceleration_duration);
    const double cruising_time = time_s - accelerating_time;
    const double speed_after_acceleration =
        std::max(0.0,
                 initial_speed + acceleration * accelerating_time);
    result.road_s_unwrapped_m =
        track.road_s_unwrapped_m + initial_speed * accelerating_time +
        0.5 * acceleration * accelerating_time * accelerating_time +
        speed_after_acceleration * cruising_time;
    result.road_s_rate_mps = speed_after_acceleration;
    result.acceleration_mps2 =
        time_s < acceleration_duration ? acceleration : 0.0;
  } else if (hypothesis ==
             TrafficPredictionHypothesis::kRearConservativeAcceleration) {
    const double acceleration =
        config.rear_conservative_acceleration_mps2;
    const double time_to_speed_cap =
        std::max(0.0, (config.maximum_predicted_speed_mps - initial_speed) /
                          acceleration);
    const double acceleration_duration =
        std::min(time_to_speed_cap,
                 config.longitudinal_acceleration_duration_s);
    const double accelerating_time =
        std::min(time_s, acceleration_duration);
    const double cruising_time = time_s - accelerating_time;
    const double capped_speed =
        std::min(config.maximum_predicted_speed_mps,
                 initial_speed + acceleration * accelerating_time);
    result.road_s_unwrapped_m =
        track.road_s_unwrapped_m + initial_speed * accelerating_time +
        0.5 * acceleration * accelerating_time * accelerating_time +
        capped_speed * cruising_time;
    result.road_s_rate_mps = capped_speed;
    result.acceleration_mps2 =
        time_s < acceleration_duration ? acceleration : 0.0;
  } else if (hypothesis ==
             TrafficPredictionHypothesis::kLateralContinuation) {
    const double lateral_rate =
        Clamp(track.d_rate_mps,
              -config.maximum_abs_lateral_prediction_rate_mps,
              config.maximum_abs_lateral_prediction_rate_mps);
    const double lateral_time =
        std::min(time_s, config.lateral_continuation_duration_s);
    result.d_m = track.d_m + lateral_rate * lateral_time;
    result.d_rate_mps =
        time_s < config.lateral_continuation_duration_s ? lateral_rate : 0.0;
  }
  return result;
}

std::uint64_t LaneMask(double occupied_d_min_m,
                       double occupied_d_max_m,
                       const FullLaneTrafficPredictionConfig &config) {
  std::uint64_t mask = 0;
  for (int lane = 0; lane < config.lane_count; ++lane) {
    const double lane_min = static_cast<double>(lane) * config.lane_width_m;
    const double lane_max = lane_min + config.lane_width_m;
    if (occupied_d_max_m >= lane_min - kGridTolerance &&
        occupied_d_min_m <= lane_max + kGridTolerance) {
      mask |= (std::uint64_t(1) << static_cast<unsigned int>(lane));
    }
  }
  return mask;
}

TrafficPredictionTrajectory MakeTrajectory(
    const TrackedVehicleState &track,
    TrafficPredictionHypothesis hypothesis, std::size_t intervals,
    const FullLaneTrafficPredictionConfig &config) {
  TrafficPredictionTrajectory trajectory;
  trajectory.vehicle_id = track.id;
  trajectory.hypothesis = hypothesis;
  trajectory.source_track_valid = track.valid;
  trajectory.safety_admissible = track.safety_admissible;
  trajectory.source_relative_road_s_m = track.relative_road_s_m;
  trajectory.vehicle_length_m = track.length_m;
  trajectory.vehicle_width_m = track.width_m;
  trajectory.occupancies.reserve(intervals + 1);
  for (std::size_t index = 0; index <= intervals; ++index) {
    const double time_s = static_cast<double>(index) * config.time_step_s;
    const PredictedKinematics kinematics =
        KinematicsAt(track, hypothesis, time_s, config);
    PredictedTrafficOccupancy occupancy;
    occupancy.prediction_time_s = time_s;
    occupancy.road_s_unwrapped_m = kinematics.road_s_unwrapped_m;
    occupancy.d_m = kinematics.d_m;
    occupancy.road_s_rate_mps = kinematics.road_s_rate_mps;
    occupancy.d_rate_mps = kinematics.d_rate_mps;
    occupancy.longitudinal_acceleration_mps2 =
        kinematics.acceleration_mps2;
    occupancy.longitudinal_uncertainty_m =
        track.longitudinal_uncertainty_m +
        config.longitudinal_uncertainty_growth_mps * time_s;
    occupancy.occupied_road_s_min_m =
        occupancy.road_s_unwrapped_m - 0.5 * track.length_m -
        occupancy.longitudinal_uncertainty_m;
    occupancy.occupied_road_s_max_m =
        occupancy.road_s_unwrapped_m + 0.5 * track.length_m +
        occupancy.longitudinal_uncertainty_m;
    occupancy.occupied_d_min_m =
        occupancy.d_m - 0.5 * track.width_m;
    occupancy.occupied_d_max_m =
        occupancy.d_m + 0.5 * track.width_m;
    occupancy.occupied_lane_mask =
        LaneMask(occupancy.occupied_d_min_m,
                 occupancy.occupied_d_max_m, config);
    trajectory.occupancies.push_back(occupancy);
  }
  return trajectory;
}

} // namespace

const char *TrafficPredictionHypothesisName(
    TrafficPredictionHypothesis hypothesis) {
  switch (hypothesis) {
  case TrafficPredictionHypothesis::kNominalConstantVelocity:
    return "NominalConstantVelocity";
  case TrafficPredictionHypothesis::kFrontConservativeBraking:
    return "FrontConservativeBraking";
  case TrafficPredictionHypothesis::kRearConservativeAcceleration:
    return "RearConservativeAcceleration";
  case TrafficPredictionHypothesis::kLateralContinuation:
    return "LateralContinuation";
  }
  return "Unknown";
}

bool TrafficOccupiesLane(const PredictedTrafficOccupancy &occupancy,
                         int lane) {
  return lane >= 0 && lane < 63 &&
         (occupancy.occupied_lane_mask &
          (std::uint64_t(1) << static_cast<unsigned int>(lane))) != 0;
}

FullLaneTrafficPredictor::FullLaneTrafficPredictor(
    const FullLaneTrafficPredictionConfig &config)
    : config_(config) {
  ValidateConfig(config_);
}

FullLaneTrafficPredictionSnapshot FullLaneTrafficPredictor::Predict(
    const TrafficTrackingSnapshot &tracking,
    double retained_prefix_duration_s) const {
  if (tracking.cycle == 0 ||
      !std::isfinite(retained_prefix_duration_s) ||
      retained_prefix_duration_s < 0.0) {
    throw std::invalid_argument("invalid full-lane prediction input");
  }
  if (tracking.tracks.size() > config_.maximum_source_tracks) {
    throw std::invalid_argument(
        "full-lane prediction exceeds configured track capacity");
  }
  const double required_coverage_s =
      retained_prefix_duration_s + config_.planning_horizon_s +
      config_.post_maneuver_observation_s;
  const std::size_t intervals = static_cast<std::size_t>(
      std::ceil(required_coverage_s / config_.time_step_s -
                kGridTolerance));
  if (intervals + 1 > config_.maximum_prediction_nodes) {
    throw std::invalid_argument(
        "full-lane prediction exceeds configured node capacity");
  }

  FullLaneTrafficPredictionSnapshot result;
  result.cycle = tracking.cycle;
  result.time_step_s = config_.time_step_s;
  result.retained_prefix_duration_s = retained_prefix_duration_s;
  result.planning_horizon_s = config_.planning_horizon_s;
  result.post_maneuver_observation_s =
      config_.post_maneuver_observation_s;
  result.longitudinal_acceleration_duration_s =
      config_.longitudinal_acceleration_duration_s;
  result.required_coverage_s = required_coverage_s;
  result.grid_coverage_s =
      static_cast<double>(intervals) * config_.time_step_s;
  result.source_track_count = tracking.tracks.size();
  result.safety_admissible_track_count =
      tracking.safety_admissible_track_count;
  result.trajectories.reserve(tracking.tracks.size() * 3);

  for (const TrackedVehicleState &track : tracking.tracks) {
    result.trajectories.push_back(MakeTrajectory(
        track, TrafficPredictionHypothesis::kNominalConstantVelocity,
        intervals, config_));
    if (track.relative_road_s_m >= 0.0) {
      result.trajectories.push_back(MakeTrajectory(
          track, TrafficPredictionHypothesis::kFrontConservativeBraking,
          intervals, config_));
    } else {
      result.trajectories.push_back(MakeTrajectory(
          track, TrafficPredictionHypothesis::kRearConservativeAcceleration,
          intervals, config_));
    }
    if (std::fabs(track.d_rate_mps) + kGridTolerance >=
        config_.lateral_motion_threshold_mps) {
      result.trajectories.push_back(MakeTrajectory(
          track, TrafficPredictionHypothesis::kLateralContinuation,
          intervals, config_));
    }
  }
  for (const TrafficPredictionTrajectory &trajectory :
       result.trajectories) {
    if (trajectory.safety_admissible) {
      ++result.safety_admissible_trajectory_count;
    }
  }
  return result;
}
