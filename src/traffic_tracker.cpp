#include "traffic_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

#include "map.h"
#include "planning_snapshot.h"

namespace {

constexpr double kMinimumFrenetBasisDeterminant = 1e-8;
constexpr double kIntegerIdTolerance = 1e-9;
constexpr double kTimeTolerance = 1e-12;

double Clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

double ShortestLoopDelta(double from_wrapped_s, double to_wrapped_s,
                         double track_length) {
  double delta = NormalizeS(to_wrapped_s, track_length) -
                 NormalizeS(from_wrapped_s, track_length);
  if (delta > 0.5 * track_length) {
    delta -= track_length;
  } else if (delta < -0.5 * track_length) {
    delta += track_length;
  }
  return delta;
}

int VehicleId(double value) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument("traffic vehicle ID is non-finite");
  }
  const double rounded = std::round(value);
  if (std::fabs(value - rounded) > kIntegerIdTolerance ||
      rounded < static_cast<double>(std::numeric_limits<int>::min()) ||
      rounded > static_cast<double>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument(
        "traffic vehicle ID must be an exactly represented integer");
  }
  return static_cast<int>(rounded);
}

struct VelocityProjection {
  double road_s_rate_mps = 0.0;
  double d_rate_mps = 0.0;
  bool within_configured_bounds = true;
};

VelocityProjection ProjectVelocity(const DetectedVehicle &vehicle,
                                   const MapData &map,
                                   const TrafficTrackerConfig &config) {
  const RoadGeometrySample road =
      EvaluateRoadGeometryOnValidatedMap(vehicle.s, vehicle.d, map);
  const RoadGeometrySample center =
      EvaluateRoadGeometryOnValidatedMap(vehicle.s, 0.0, map);
  const RoadGeometrySample unit_offset =
      EvaluateRoadGeometryOnValidatedMap(vehicle.s, 1.0, map);
  const double normal_x = unit_offset.x - center.x;
  const double normal_y = unit_offset.y - center.y;
  const double determinant =
      road.first_derivative_x * normal_y -
      road.first_derivative_y * normal_x;
  const double tangent_metric =
      std::hypot(road.first_derivative_x, road.first_derivative_y);
  if (!std::isfinite(determinant) ||
      std::fabs(determinant) <= kMinimumFrenetBasisDeterminant ||
      !std::isfinite(tangent_metric) || tangent_metric <= 0.0) {
    throw std::runtime_error("traffic Frenet velocity basis is degenerate");
  }

  const double parameter_rate =
      (vehicle.vx_mps * normal_y - vehicle.vy_mps * normal_x) /
      determinant;
  // RoadSUnwrapped is the map parameter, not offset-path arc length. Keep its
  // derivative in the same coordinate so prediction may integrate it without
  // silently treating Cartesian speed magnitude as RoadS progress.
  const double raw_road_s_rate = parameter_rate;
  const double raw_d_rate =
      (road.first_derivative_x * vehicle.vy_mps -
       road.first_derivative_y * vehicle.vx_mps) /
      determinant;
  if (!std::isfinite(raw_road_s_rate) || !std::isfinite(raw_d_rate)) {
    throw std::runtime_error("traffic velocity projection is non-finite");
  }

  VelocityProjection result;
  result.within_configured_bounds =
      std::fabs(raw_road_s_rate) <=
          config.maximum_abs_road_s_rate_mps + 1e-9 &&
      std::fabs(raw_d_rate) <= config.maximum_abs_d_rate_mps + 1e-9;
  result.road_s_rate_mps =
      Clamp(raw_road_s_rate, -config.maximum_abs_road_s_rate_mps,
            config.maximum_abs_road_s_rate_mps);
  result.d_rate_mps =
      Clamp(raw_d_rate, -config.maximum_abs_d_rate_mps,
            config.maximum_abs_d_rate_mps);
  return result;
}

bool RequiresControlHistoryReset(PlanningStateResetReason reason) {
  return reason == PlanningStateResetReason::kNoCommittedHistory ||
         reason == PlanningStateResetReason::kHistoryLengthMismatch ||
         reason == PlanningStateResetReason::kHistoryPositionMismatch;
}

TrackedVehicleState NewTrack(const DetectedVehicle &vehicle, int id,
                             const VelocityProjection &velocity,
                             double ego_wrapped_s,
                             double ego_unwrapped_s,
                             std::uint64_t cycle, const MapData &map,
                             const TrafficTrackerConfig &config) {
  TrackedVehicleState track;
  track.id = id;
  track.road_s_wrapped_m = NormalizeS(vehicle.s, map.track_length);
  track.road_s_unwrapped_m =
      ego_unwrapped_s +
      ShortestLoopDelta(ego_wrapped_s, track.road_s_wrapped_m,
                        map.track_length);
  track.relative_road_s_m = track.road_s_unwrapped_m - ego_unwrapped_s;
  track.d_m = vehicle.d;
  track.road_s_rate_mps = velocity.road_s_rate_mps;
  track.d_rate_mps = velocity.d_rate_mps;
  track.longitudinal_acceleration_mps2 = 0.0;
  track.length_m = config.obstacle_length_m;
  track.width_m = config.obstacle_width_m;
  track.track_age_s = 0.0;
  track.time_since_update_s = 0.0;
  track.longitudinal_uncertainty_m =
      config.initial_longitudinal_uncertainty_m;
  track.last_update_cycle = cycle;
  track.observation_count = 1;
  track.observed_this_cycle = true;
  track.motion_within_configured_bounds =
      velocity.within_configured_bounds;
  track.valid = velocity.within_configured_bounds;
  track.safety_admissible =
      track.valid && config.minimum_safety_track_age_s <= kTimeTolerance;
  track.last_observed_road_s_wrapped_m = track.road_s_wrapped_m;
  track.last_observed_road_s_unwrapped_m = track.road_s_unwrapped_m;
  track.last_observed_road_s_rate_mps = track.road_s_rate_mps;
  return track;
}

void ValidateConfig(const TrafficTrackerConfig &config) {
  if (!std::isfinite(config.simulator_time_step_s) ||
      config.simulator_time_step_s <= 0.0 ||
      !std::isfinite(config.stale_after_s) ||
      config.stale_after_s < config.simulator_time_step_s ||
      !std::isfinite(config.drop_after_s) ||
      config.drop_after_s <= config.stale_after_s ||
      !std::isfinite(config.minimum_safety_track_age_s) ||
      config.minimum_safety_track_age_s < 0.0 ||
      config.minimum_safety_track_age_s > config.stale_after_s ||
      !std::isfinite(config.maximum_abs_road_s_rate_mps) ||
      config.maximum_abs_road_s_rate_mps <= 0.0 ||
      !std::isfinite(config.maximum_abs_d_rate_mps) ||
      config.maximum_abs_d_rate_mps <= 0.0 ||
      !std::isfinite(
          config.maximum_abs_longitudinal_acceleration_mps2) ||
      config.maximum_abs_longitudinal_acceleration_mps2 <= 0.0 ||
      !std::isfinite(config.id_reuse_longitudinal_tolerance_m) ||
      config.id_reuse_longitudinal_tolerance_m <= 0.0 ||
      !std::isfinite(config.id_reuse_lateral_tolerance_m) ||
      config.id_reuse_lateral_tolerance_m <= 0.0 ||
      !std::isfinite(config.obstacle_length_m) ||
      config.obstacle_length_m <= 0.0 ||
      !std::isfinite(config.obstacle_width_m) ||
      config.obstacle_width_m <= 0.0 ||
      !std::isfinite(config.initial_longitudinal_uncertainty_m) ||
      config.initial_longitudinal_uncertainty_m < 0.0 ||
      !std::isfinite(
          config.stale_longitudinal_uncertainty_growth_mps) ||
      config.stale_longitudinal_uncertainty_growth_mps < 0.0 ||
      config.maximum_tracks == 0) {
    throw std::invalid_argument("invalid traffic tracker configuration");
  }
}

} // namespace

const char *TrafficTrackerResetReasonName(TrafficTrackerResetReason reason) {
  switch (reason) {
  case TrafficTrackerResetReason::kNone:
    return "None";
  case TrafficTrackerResetReason::kInitialization:
    return "Initialization";
  case TrafficTrackerResetReason::kControlHistoryReset:
    return "ControlHistoryReset";
  case TrafficTrackerResetReason::kTimeReversed:
    return "TimeReversed";
  }
  return "Unknown";
}

TrafficTracker::TrafficTracker(const TrafficTrackerConfig &config)
    : config_(config) {
  ValidateConfig(config_);
}

TrafficTrackingUpdate
TrafficTracker::Evaluate(const PlanningSnapshot &snapshot,
                         const MapData &map,
                         const TrafficTrackerState &state) const {
  if (snapshot.cycle == 0) {
    throw std::invalid_argument("traffic tracking cycle must be non-zero");
  }
  if (!std::isfinite(snapshot.telemetry_time_s) ||
      snapshot.telemetry_time_s < 0.0 ||
      (state.initialized && (!std::isfinite(state.telemetry_time_s) ||
                             state.telemetry_time_s < 0.0))) {
    throw std::invalid_argument("traffic tracking time is invalid");
  }
  if (snapshot.input.traffic.size() > config_.maximum_tracks) {
    throw std::invalid_argument(
        "traffic observation exceeds configured track capacity");
  }
  if (state.tracks.size() > config_.maximum_tracks) {
    throw std::invalid_argument(
        "committed traffic state exceeds configured track capacity");
  }
  std::string map_error;
  if (!ValidateMap(map, &map_error)) {
    throw std::invalid_argument("invalid traffic tracking map: " + map_error);
  }

  TrafficTrackerResetReason reset_reason = TrafficTrackerResetReason::kNone;
  if (!state.initialized) {
    reset_reason = TrafficTrackerResetReason::kInitialization;
  } else if (snapshot.cycle <= state.cycle ||
             snapshot.telemetry_time_s + kTimeTolerance <
                 state.telemetry_time_s) {
    reset_reason = TrafficTrackerResetReason::kTimeReversed;
  } else if (RequiresControlHistoryReset(snapshot.reset_reason)) {
    reset_reason = TrafficTrackerResetReason::kControlHistoryReset;
  }

  TrafficTrackingUpdate result;
  TrafficTrackerState &next = result.next_state;
  const bool reset = reset_reason != TrafficTrackerResetReason::kNone;
  if (!reset) {
    next = state;
  }
  next.initialized = true;
  next.cycle = snapshot.cycle;
  next.telemetry_time_s = snapshot.telemetry_time_s;

  const double ego_wrapped_s =
      NormalizeS(snapshot.input.ego.s, map.track_length);
  if (reset) {
    next.tracks.clear();
    next.ego_road_s_wrapped_m = ego_wrapped_s;
    next.ego_road_s_unwrapped_m = snapshot.input.ego.s;
  } else {
    next.ego_road_s_unwrapped_m =
        state.ego_road_s_unwrapped_m +
        ShortestLoopDelta(state.ego_road_s_wrapped_m, ego_wrapped_s,
                          map.track_length);
    next.ego_road_s_wrapped_m = ego_wrapped_s;
  }

  const double elapsed_since_state_s =
      reset ? 0.0
            : std::max(0.0, snapshot.telemetry_time_s - state.telemetry_time_s);
  for (auto iterator = next.tracks.begin(); iterator != next.tracks.end();) {
    TrackedVehicleState &track = iterator->second;
    const double initial_rate = track.road_s_rate_mps;
    track.road_s_unwrapped_m +=
        initial_rate * elapsed_since_state_s +
        0.5 * track.longitudinal_acceleration_mps2 *
            elapsed_since_state_s * elapsed_since_state_s;
    track.road_s_rate_mps = Clamp(
        initial_rate + track.longitudinal_acceleration_mps2 *
                           elapsed_since_state_s,
        -config_.maximum_abs_road_s_rate_mps,
        config_.maximum_abs_road_s_rate_mps);
    track.d_m += track.d_rate_mps * elapsed_since_state_s;
    track.road_s_wrapped_m =
        NormalizeS(track.road_s_unwrapped_m, map.track_length);
    track.relative_road_s_m =
        track.road_s_unwrapped_m - next.ego_road_s_unwrapped_m;
    track.track_age_s += elapsed_since_state_s;
    track.time_since_update_s += elapsed_since_state_s;
    track.longitudinal_uncertainty_m =
        config_.initial_longitudinal_uncertainty_m +
        config_.stale_longitudinal_uncertainty_growth_mps *
            track.time_since_update_s;
    track.observed_this_cycle = false;
    track.valid = track.motion_within_configured_bounds &&
                  track.time_since_update_s <=
                      config_.stale_after_s + kTimeTolerance;
    track.safety_admissible =
        track.valid &&
        track.track_age_s + kTimeTolerance >=
            config_.minimum_safety_track_age_s;
    if (track.time_since_update_s >
        config_.drop_after_s + kTimeTolerance) {
      iterator = next.tracks.erase(iterator);
    } else {
      ++iterator;
    }
  }

  std::set<int> observed_ids;
  for (const DetectedVehicle &vehicle : snapshot.input.traffic) {
    const int id = VehicleId(vehicle.id);
    if (!observed_ids.insert(id).second) {
      throw std::invalid_argument("duplicate traffic vehicle ID in one frame");
    }
    const VelocityProjection velocity =
        ProjectVelocity(vehicle, map, config_);
    auto existing = next.tracks.find(id);
    if (existing == next.tracks.end()) {
      if (next.tracks.size() >= config_.maximum_tracks) {
        throw std::invalid_argument(
            "retained and observed traffic exceed track capacity");
      }
      next.tracks[id] = NewTrack(
          vehicle, id, velocity, next.ego_road_s_wrapped_m,
          next.ego_road_s_unwrapped_m, snapshot.cycle, map, config_);
      continue;
    }

    TrackedVehicleState &track = existing->second;
    const bool reacquired =
        track.time_since_update_s > config_.stale_after_s + kTimeTolerance;
    const double observed_wrapped_s =
        NormalizeS(vehicle.s, map.track_length);
    const double observed_unwrapped_s =
        track.last_observed_road_s_unwrapped_m +
        ShortestLoopDelta(track.last_observed_road_s_wrapped_m,
                          observed_wrapped_s, map.track_length);
    const double observation_elapsed_s = track.time_since_update_s;
    const double longitudinal_tolerance =
        config_.id_reuse_longitudinal_tolerance_m +
        0.5 * config_.maximum_abs_longitudinal_acceleration_mps2 *
            observation_elapsed_s * observation_elapsed_s;
    const bool reused =
        !reacquired &&
        (std::fabs(observed_unwrapped_s - track.road_s_unwrapped_m) >
             longitudinal_tolerance ||
         std::fabs(vehicle.d - track.d_m) >
             config_.id_reuse_lateral_tolerance_m);
    if (reacquired || reused) {
      if (reacquired) {
        ++result.snapshot.reacquired_track_count;
      } else {
        ++result.snapshot.reused_id_count;
      }
      track = NewTrack(vehicle, id, velocity,
                       next.ego_road_s_wrapped_m,
                       next.ego_road_s_unwrapped_m, snapshot.cycle, map,
                       config_);
      continue;
    }

    const double raw_acceleration =
        observation_elapsed_s > 0.0
            ? (velocity.road_s_rate_mps -
               track.last_observed_road_s_rate_mps) /
                  observation_elapsed_s
            : 0.0;
    track.road_s_wrapped_m = observed_wrapped_s;
    track.road_s_unwrapped_m = observed_unwrapped_s;
    track.relative_road_s_m =
        observed_unwrapped_s - next.ego_road_s_unwrapped_m;
    track.d_m = vehicle.d;
    track.road_s_rate_mps = velocity.road_s_rate_mps;
    track.d_rate_mps = velocity.d_rate_mps;
    track.longitudinal_acceleration_mps2 = Clamp(
        raw_acceleration,
        -config_.maximum_abs_longitudinal_acceleration_mps2,
        config_.maximum_abs_longitudinal_acceleration_mps2);
    track.time_since_update_s = 0.0;
    track.longitudinal_uncertainty_m =
        config_.initial_longitudinal_uncertainty_m;
    track.last_update_cycle = snapshot.cycle;
    ++track.observation_count;
    track.observed_this_cycle = true;
    track.motion_within_configured_bounds =
        velocity.within_configured_bounds;
    track.valid = velocity.within_configured_bounds;
    track.safety_admissible =
        track.valid &&
        track.track_age_s + kTimeTolerance >=
            config_.minimum_safety_track_age_s;
    track.last_observed_road_s_wrapped_m = observed_wrapped_s;
    track.last_observed_road_s_unwrapped_m = observed_unwrapped_s;
    track.last_observed_road_s_rate_mps = velocity.road_s_rate_mps;
  }

  result.snapshot.cycle = snapshot.cycle;
  result.snapshot.reset_reason = reset_reason;
  result.snapshot.ego_road_s_unwrapped_m =
      next.ego_road_s_unwrapped_m;
  result.snapshot.tracks.reserve(next.tracks.size());
  for (const auto &entry : next.tracks) {
    const TrackedVehicleState &track = entry.second;
    result.snapshot.tracks.push_back(track);
    if (track.observed_this_cycle) {
      ++result.snapshot.observed_track_count;
    }
    if (track.valid) {
      ++result.snapshot.valid_track_count;
    }
    if (track.safety_admissible) {
      ++result.snapshot.safety_admissible_track_count;
    }
    if (track.time_since_update_s >
        config_.stale_after_s + kTimeTolerance) {
      ++result.snapshot.stale_track_count;
    }
  }
  return result;
}
