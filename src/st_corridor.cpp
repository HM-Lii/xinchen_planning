#include "st_corridor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>

#include "planning_snapshot.h"

namespace {

const double kTolerance = 1e-9;

bool Finite(double value) { return std::isfinite(value); }

double Clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

std::uint64_t LaneBit(int lane) {
  return std::uint64_t(1) << static_cast<unsigned>(lane);
}

std::uint64_t HypothesisBit(TrafficPredictionHypothesis hypothesis) {
  return std::uint64_t(1) << static_cast<unsigned>(hypothesis);
}

bool IsTimeLimitStatus(const std::string &status) {
  return status.find("time limit") != std::string::npos;
}

void ValidateConfig(const STCorridorPlannerConfig &config) {
  const bool finite =
      Finite(config.qp_time_step_s) &&
      Finite(config.post_maneuver_observation_s) &&
      Finite(config.maximum_speed_mps) &&
      Finite(config.minimum_longitudinal_acceleration_mps2) &&
      Finite(config.maximum_longitudinal_acceleration_mps2) &&
      Finite(config.lateral_acceleration_budget_mps2) &&
      Finite(config.curvature_epsilon_per_m) &&
      Finite(config.standstill_clearance_m) &&
      Finite(config.physical_collision_margin_m) &&
      Finite(config.projection_tightening_m) &&
      Finite(config.ego_length_m) && Finite(config.ego_width_m) &&
      Finite(config.qp_bound_tolerance_m) &&
      Finite(config.qp_speed_tolerance_mps);
  if (!finite || config.qp_horizon_steps == 0 ||
      config.qp_time_step_s <= 0.0 ||
      config.post_maneuver_observation_s < 0.0 ||
      config.post_maneuver_observation_s >=
          static_cast<double>(config.qp_horizon_steps) *
              config.qp_time_step_s ||
      config.maximum_speed_mps <= 0.0 ||
      config.minimum_longitudinal_acceleration_mps2 >= 0.0 ||
      config.maximum_longitudinal_acceleration_mps2 <= 0.0 ||
      config.lateral_acceleration_budget_mps2 <= 0.0 ||
      config.curvature_epsilon_per_m <= 0.0 ||
      config.standstill_clearance_m < 0.0 ||
      config.physical_collision_margin_m < 0.0 ||
      config.projection_tightening_m < 0.0 ||
      config.ego_length_m <= 0.0 || config.ego_width_m <= 0.0 ||
      config.lane_count <= 0 || config.lane_count > 63 ||
      config.maximum_candidates == 0 ||
      config.maximum_corridor_nodes < config.qp_horizon_steps + 1 ||
      config.qp_bound_tolerance_m < 0.0 ||
      config.qp_speed_tolerance_mps < 0.0) {
    throw std::invalid_argument("invalid ST corridor configuration");
  }
}

const BehaviorCandidate *FindBehaviorCandidate(
    const BehaviorPlanningSnapshot &behavior, std::uint64_t candidate_id) {
  for (const BehaviorCandidate &candidate : behavior.candidates) {
    if (candidate.candidate_id == candidate_id) {
      return &candidate;
    }
  }
  return nullptr;
}

bool SampleTrajectory(
    const TrafficPredictionTrajectory &trajectory, double time_s,
    std::vector<const PredictedTrafficOccupancy *> *samples) {
  samples->clear();
  if (trajectory.occupancies.empty() || !Finite(time_s) ||
      time_s + kTolerance <
          trajectory.occupancies.front().prediction_time_s ||
      time_s > trajectory.occupancies.back().prediction_time_s +
                   kTolerance) {
    return false;
  }
  const std::vector<PredictedTrafficOccupancy>::const_iterator upper =
      std::lower_bound(
          trajectory.occupancies.begin(), trajectory.occupancies.end(),
          time_s,
          [](const PredictedTrafficOccupancy &occupancy, double time) {
            return occupancy.prediction_time_s < time;
          });
  if (upper == trajectory.occupancies.end()) {
    samples->push_back(&trajectory.occupancies.back());
    return true;
  }
  if (upper == trajectory.occupancies.begin() ||
      std::fabs(upper->prediction_time_s - time_s) <= kTolerance) {
    samples->push_back(&*upper);
    return true;
  }
  // A union of both bracketing samples is conservative without inventing an
  // interpolation model for an already conservative occupancy envelope.
  samples->push_back(&*(upper - 1));
  samples->push_back(&*upper);
  return true;
}

struct VehicleEnvelope {
  int vehicle_id = 0;
  bool source_track_valid = true;
  bool safety_admissible = true;
  double occupied_road_s_min_m = 0.0;
  double occupied_road_s_max_m = 0.0;
  double minimum_road_s_rate_mps = 0.0;
  double maximum_road_s_rate_mps = 0.0;
  std::uint64_t occupied_lane_mask = 0;
  std::uint64_t hypothesis_mask = 0;
};

struct EnvelopeAccumulator {
  bool initialized = false;
  VehicleEnvelope envelope;
};

void ExtendEnvelope(const TrafficPredictionTrajectory &trajectory,
                    const PredictedTrafficOccupancy &occupancy,
                    EnvelopeAccumulator *accumulator) {
  if (!accumulator->initialized) {
    accumulator->initialized = true;
    accumulator->envelope.vehicle_id = trajectory.vehicle_id;
    accumulator->envelope.source_track_valid =
        trajectory.source_track_valid;
    accumulator->envelope.safety_admissible =
        trajectory.source_track_valid && trajectory.safety_admissible;
    accumulator->envelope.occupied_road_s_min_m =
        occupancy.occupied_road_s_min_m;
    accumulator->envelope.occupied_road_s_max_m =
        occupancy.occupied_road_s_max_m;
    accumulator->envelope.minimum_road_s_rate_mps =
        occupancy.road_s_rate_mps;
    accumulator->envelope.maximum_road_s_rate_mps =
        occupancy.road_s_rate_mps;
    accumulator->envelope.occupied_lane_mask =
        occupancy.occupied_lane_mask;
    accumulator->envelope.hypothesis_mask =
        HypothesisBit(trajectory.hypothesis);
    return;
  }
  VehicleEnvelope &envelope = accumulator->envelope;
  envelope.source_track_valid =
      envelope.source_track_valid && trajectory.source_track_valid;
  envelope.safety_admissible =
      envelope.safety_admissible && trajectory.source_track_valid &&
      trajectory.safety_admissible;
  envelope.occupied_road_s_min_m =
      std::min(envelope.occupied_road_s_min_m,
               occupancy.occupied_road_s_min_m);
  envelope.occupied_road_s_max_m =
      std::max(envelope.occupied_road_s_max_m,
               occupancy.occupied_road_s_max_m);
  envelope.minimum_road_s_rate_mps =
      std::min(envelope.minimum_road_s_rate_mps,
               occupancy.road_s_rate_mps);
  envelope.maximum_road_s_rate_mps =
      std::max(envelope.maximum_road_s_rate_mps,
               occupancy.road_s_rate_mps);
  envelope.occupied_lane_mask |= occupancy.occupied_lane_mask;
  envelope.hypothesis_mask |= HypothesisBit(trajectory.hypothesis);
}

std::vector<VehicleEnvelope> EnvelopesAtTime(
    const FullLaneTrafficPredictionSnapshot &prediction, double time_s,
    std::uint64_t lane_mask, bool *evidence_complete) {
  std::map<int, EnvelopeAccumulator> accumulators;
  *evidence_complete = true;
  for (const TrafficPredictionTrajectory &trajectory :
       prediction.trajectories) {
    std::vector<const PredictedTrafficOccupancy *> samples;
    if (!SampleTrajectory(trajectory, time_s, &samples)) {
      *evidence_complete = false;
      continue;
    }
    for (const PredictedTrafficOccupancy *occupancy : samples) {
      if ((occupancy->occupied_lane_mask & lane_mask) != 0) {
        ExtendEnvelope(trajectory, *occupancy,
                       &accumulators[trajectory.vehicle_id]);
      }
    }
  }
  std::vector<VehicleEnvelope> result;
  result.reserve(accumulators.size());
  for (const std::pair<const int, EnvelopeAccumulator> &entry :
       accumulators) {
    if (entry.second.initialized) {
      result.push_back(entry.second.envelope);
    }
  }
  return result;
}

double EnvelopeCenter(const VehicleEnvelope &envelope) {
  return 0.5 * (envelope.occupied_road_s_min_m +
                envelope.occupied_road_s_max_m);
}

double DistanceWithAcceleration(double speed_mps,
                                double acceleration_mps2,
                                double duration_s) {
  speed_mps = std::max(0.0, speed_mps);
  if (duration_s <= 0.0) {
    return 0.0;
  }
  if (acceleration_mps2 < 0.0 &&
      speed_mps + acceleration_mps2 * duration_s < 0.0) {
    const double stop_time_s = -speed_mps / acceleration_mps2;
    return speed_mps * stop_time_s +
           0.5 * acceleration_mps2 * stop_time_s * stop_time_s;
  }
  return std::max(0.0, speed_mps * duration_s +
                           0.5 * acceleration_mps2 * duration_s *
                               duration_s);
}

double DistanceWithAccelerationAndCap(double speed_mps,
                                      double acceleration_mps2,
                                      double duration_s,
                                      double speed_cap_mps) {
  speed_mps = std::max(0.0, speed_mps);
  speed_cap_mps = std::max(speed_mps, speed_cap_mps);
  if (duration_s <= 0.0) {
    return 0.0;
  }
  if (acceleration_mps2 <= 0.0 || speed_mps >= speed_cap_mps) {
    return speed_mps * duration_s;
  }
  const double cap_time_s =
      (speed_cap_mps - speed_mps) / acceleration_mps2;
  if (cap_time_s >= duration_s) {
    return DistanceWithAcceleration(speed_mps, acceleration_mps2,
                                    duration_s);
  }
  return speed_mps * cap_time_s +
         0.5 * acceleration_mps2 * cap_time_s * cap_time_s +
         speed_cap_mps * (duration_s - cap_time_s);
}

double ProjectRoadSToPathProgress(
    const SpatialPathGeometryTable &geometry, double road_s_m) {
  if (geometry.samples.size() < 2) {
    throw std::invalid_argument(
        "ST projection requires at least two geometry samples");
  }
  const std::vector<SpatialPathGeometrySample> &samples = geometry.samples;
  const std::vector<SpatialPathGeometrySample>::const_iterator upper =
      std::lower_bound(
          samples.begin(), samples.end(), road_s_m,
          [](const SpatialPathGeometrySample &sample, double road_s) {
            return sample.road_s_unwrapped_m < road_s;
          });
  std::size_t right = 1;
  if (upper == samples.begin()) {
    right = 1;
  } else if (upper == samples.end()) {
    right = samples.size() - 1;
  } else {
    right = static_cast<std::size_t>(upper - samples.begin());
  }
  const SpatialPathGeometrySample &left_sample = samples[right - 1];
  const SpatialPathGeometrySample &right_sample = samples[right];
  const double road_span = right_sample.road_s_unwrapped_m -
                           left_sample.road_s_unwrapped_m;
  if (!Finite(road_span) || road_span <= kTolerance) {
    throw std::invalid_argument("spatial path RoadS mapping is not monotonic");
  }
  const double ratio =
      (road_s_m - left_sample.road_s_unwrapped_m) / road_span;
  return left_sample.path_progress_m +
         ratio * (right_sample.path_progress_m -
                  left_sample.path_progress_m);
}

double MaximumAbsCurvature(
    const SpatialPathGeometryTable &geometry, double minimum_progress_m,
    double maximum_progress_m) {
  minimum_progress_m =
      Clamp(minimum_progress_m, 0.0, geometry.path_extent_m);
  maximum_progress_m =
      Clamp(maximum_progress_m, 0.0, geometry.path_extent_m);
  if (minimum_progress_m > maximum_progress_m) {
    std::swap(minimum_progress_m, maximum_progress_m);
  }
  double result = std::max(
      std::fabs(SampleSpatialPathAtProgress(geometry,
                                           minimum_progress_m)
                    .curvature_per_m),
      std::fabs(SampleSpatialPathAtProgress(geometry,
                                           maximum_progress_m)
                    .curvature_per_m));
  for (const SpatialPathGeometrySample &sample : geometry.samples) {
    if (sample.path_progress_m + kTolerance >= minimum_progress_m &&
        sample.path_progress_m <= maximum_progress_m + kTolerance) {
      result = std::max(result, std::fabs(sample.curvature_per_m));
    }
  }
  return result;
}

std::uint64_t PossibleEgoLaneMask(
    const SpatialPathCandidate &path, double reachable_minimum_progress_m,
    double reachable_maximum_progress_m) {
  std::uint64_t mask = 0;
  if (!path.occupancy.source_lane_departed ||
      reachable_minimum_progress_m <=
          path.occupancy.source_lane_departure_path_progress_m +
              kTolerance) {
    mask |= LaneBit(path.source_lane);
  }
  if (path.occupancy.target_lane_coverage_started &&
      reachable_maximum_progress_m + kTolerance >=
          path.occupancy.target_lane_coverage_start_path_progress_m) {
    mask |= LaneBit(path.target_lane);
  }
  return mask;
}

void SetSource(const VehicleEnvelope &envelope,
               STConstraintSource *source) {
  source->present = true;
  source->vehicle_id = envelope.vehicle_id;
  source->hypothesis_mask = envelope.hypothesis_mask;
}

bool HasVehicleInLane(const std::vector<VehicleEnvelope> &envelopes,
                      int vehicle_id, int lane) {
  for (const VehicleEnvelope &envelope : envelopes) {
    if (envelope.vehicle_id == vehicle_id &&
        (envelope.occupied_lane_mask & LaneBit(lane)) != 0) {
      return true;
    }
  }
  return false;
}

bool HasSafetyAdmissibleVehicleInLane(
    const std::vector<VehicleEnvelope> &envelopes,
    int vehicle_id, int lane) {
  for (const VehicleEnvelope &envelope : envelopes) {
    if (envelope.vehicle_id == vehicle_id &&
        envelope.safety_admissible &&
        (envelope.occupied_lane_mask & LaneBit(lane)) != 0) {
      return true;
    }
  }
  return false;
}

STConstraintSide SideForVehicle(
    const BehaviorCandidate &behavior,
    const std::map<int, STConstraintSide> &fixed_sides,
    int vehicle_id, bool preserve_front_boundary,
    bool preserve_rear_boundary) {
  if (preserve_front_boundary && behavior.gap.has_front_vehicle &&
      behavior.gap.front_vehicle_id == vehicle_id) {
    return STConstraintSide::kUpper;
  }
  if (preserve_rear_boundary && behavior.gap.has_rear_vehicle &&
      behavior.gap.rear_vehicle_id == vehicle_id) {
    return STConstraintSide::kLower;
  }
  const std::map<int, STConstraintSide>::const_iterator found =
      fixed_sides.find(vehicle_id);
  return found == fixed_sides.end() ? STConstraintSide::kNone
                                    : found->second;
}

bool CandidateInputValid(const BehaviorCandidate &behavior,
                         const SpatialPathCandidate &path,
                         int lane_count) {
  if (behavior.status !=
          BehaviorCandidateStatus::kCoarseAdmissionPassed ||
      !behavior.coarse_admission.evaluated ||
      !behavior.coarse_admission.passed ||
      behavior.behavior == BehaviorType::kKeepLane ||
      behavior.source_lane != path.source_lane ||
      behavior.target_lane != path.target_lane ||
      behavior.gap.target_lane != behavior.target_lane ||
      path.status != SpatialPathCandidateStatus::kPrecheckPassed ||
      !path.precheck.evaluated || !path.precheck.passed ||
      path.source_lane < 0 || path.source_lane >= lane_count ||
      path.target_lane < 0 || path.target_lane >= lane_count ||
      std::abs(path.target_lane - path.source_lane) != 1 ||
      path.geometry.samples.size() < 2 ||
      path.geometry.sample_count != path.geometry.samples.size() ||
      !Finite(path.geometry.path_extent_m) ||
      path.geometry.path_extent_m <= 0.0 ||
      !path.occupancy.target_lane_coverage_started ||
      !path.occupancy.source_lane_departed ||
      !path.occupancy.lane_change_completed) {
    return false;
  }
  const bool direction_valid =
      (behavior.target_lane < behavior.source_lane &&
       behavior.behavior == BehaviorType::kChangeLeft) ||
      (behavior.target_lane > behavior.source_lane &&
       behavior.behavior == BehaviorType::kChangeRight);
  const double target_coverage_progress_m =
      path.occupancy.target_lane_coverage_start_path_progress_m;
  const double source_departure_progress_m =
      path.occupancy.source_lane_departure_path_progress_m;
  const double completion_progress_m =
      path.occupancy.lane_change_completion_path_progress_m;
  if (!direction_valid || !Finite(target_coverage_progress_m) ||
      !Finite(source_departure_progress_m) ||
      !Finite(completion_progress_m) || target_coverage_progress_m < 0.0 ||
      target_coverage_progress_m > source_departure_progress_m ||
      source_departure_progress_m > completion_progress_m ||
      completion_progress_m > path.geometry.path_extent_m + kTolerance) {
    return false;
  }
  if (behavior.order == PassingOrder::kMergeAheadOfRear &&
      !behavior.gap.has_rear_vehicle) {
    return false;
  }
  if (behavior.order == PassingOrder::kWaitBehindRear &&
      !behavior.gap.has_front_vehicle) {
    return false;
  }
  return true;
}

void BuildOneCandidate(
    const PlanningSnapshot &planning, const BehaviorCandidate &behavior,
    const SpatialPathCandidate &path,
    const FullLaneTrafficPredictionSnapshot &prediction,
    const STCorridorPlannerConfig &config,
    const STManeuverSchedule &schedule,
    STCandidateEvaluation *candidate) {
  candidate->candidate_id = behavior.candidate_id;
  candidate->source_lane = behavior.source_lane;
  candidate->target_lane = behavior.target_lane;
  candidate->order = behavior.order;
  candidate->corridor.time_step_s = config.qp_time_step_s;
  candidate->corridor.order = behavior.order;
  candidate->corridor.node_count = config.qp_horizon_steps + 1;
  candidate->speed_budget.node_count = config.qp_horizon_steps + 1;
  candidate->source_lane_departure_progress_m =
      path.occupancy.source_lane_departure_path_progress_m;
  candidate->lane_change_completion_progress_m =
      path.occupancy.lane_change_completion_path_progress_m;
  const double default_deadline_s =
      static_cast<double>(config.qp_horizon_steps) *
          config.qp_time_step_s -
      config.post_maneuver_observation_s;
  if ((schedule.has_source_lane_departure_deadline &&
       (!Finite(schedule.source_lane_departure_deadline_s) ||
        schedule.source_lane_departure_deadline_s < 0.0 ||
        schedule.source_lane_departure_deadline_s >
            default_deadline_s + kTolerance)) ||
      (schedule.has_completion_deadline &&
       (!Finite(schedule.completion_deadline_s) ||
        schedule.completion_deadline_s < 0.0 ||
        schedule.completion_deadline_s >
            default_deadline_s + kTolerance))) {
    candidate->status = STCandidateStatus::kInvalidInput;
    return;
  }
  candidate->latest_allowed_source_lane_departure_time_s =
      schedule.source_lane_released
          ? 0.0
          : (schedule.has_source_lane_departure_deadline
                 ? schedule.source_lane_departure_deadline_s
                 : default_deadline_s);
  candidate->latest_allowed_completion_time_s =
      schedule.has_completion_deadline
          ? schedule.completion_deadline_s
          : default_deadline_s;
  if (!schedule.source_lane_released &&
      candidate->latest_allowed_source_lane_departure_time_s >
          candidate->latest_allowed_completion_time_s + kTolerance) {
    candidate->status = STCandidateStatus::kInvalidInput;
    return;
  }

  if (!CandidateInputValid(behavior, path, config.lane_count)) {
    candidate->status = STCandidateStatus::kInvalidInput;
    return;
  }
  if (prediction.grid_coverage_s + kTolerance <
      prediction.retained_prefix_duration_s +
          static_cast<double>(config.qp_horizon_steps) *
              config.qp_time_step_s) {
    candidate->status =
        STCandidateStatus::kPredictionEvidenceIncomplete;
    candidate->corridor.prediction_evidence_complete = false;
    return;
  }

  const std::uint64_t all_lane_mask =
      (std::uint64_t(1) << static_cast<unsigned>(config.lane_count)) - 1;
  bool frontier_evidence_complete = true;
  const std::vector<VehicleEnvelope> frontier_envelopes =
      EnvelopesAtTime(prediction, prediction.retained_prefix_duration_s,
                      all_lane_mask, &frontier_evidence_complete);
  std::map<int, STConstraintSide> fixed_sides;
  for (const VehicleEnvelope &envelope : frontier_envelopes) {
    fixed_sides[envelope.vehicle_id] =
        EnvelopeCenter(envelope) < path.start_road_s_unwrapped_m -
                                       kTolerance
            ? STConstraintSide::kLower
            : STConstraintSide::kUpper;
  }
  const bool preserve_front_boundary =
      !schedule.committed_continuation ||
      !behavior.gap.has_front_vehicle ||
      HasSafetyAdmissibleVehicleInLane(
          frontier_envelopes, behavior.gap.front_vehicle_id,
          path.target_lane);
  const bool preserve_rear_boundary =
      !schedule.committed_continuation ||
      !behavior.gap.has_rear_vehicle ||
      HasSafetyAdmissibleVehicleInLane(
          frontier_envelopes, behavior.gap.rear_vehicle_id,
          path.target_lane);

  const double initial_speed_mps =
      std::max(0.0, planning.frontier.longitudinal.v);
  const double reach_speed_cap_mps =
      std::max(initial_speed_mps, config.maximum_speed_mps);
  const double ego_longitudinal_half_extent_m =
      0.5 * std::hypot(config.ego_length_m, config.ego_width_m);
  const double clearance_m = ego_longitudinal_half_extent_m +
                             config.standstill_clearance_m +
                             config.physical_collision_margin_m +
                             config.projection_tightening_m;
  candidate->corridor.prediction_evidence_complete =
      frontier_evidence_complete;
  candidate->corridor.first_evidence_failure_node =
      config.qp_horizon_steps + 1;
  candidate->corridor.first_empty_node = config.qp_horizon_steps + 1;
  candidate->corridor.minimum_width_m =
      std::numeric_limits<double>::infinity();
  candidate->speed_budget.minimum_speed_limit_mps =
      std::numeric_limits<double>::infinity();
  candidate->corridor.nodes.reserve(config.qp_horizon_steps + 1);
  candidate->speed_budget.maximum_speed_mps.reserve(
      config.qp_horizon_steps + 1);

  const std::size_t source_lane_release_node =
      schedule.source_lane_released
          ? 0
          : std::min(
                config.qp_horizon_steps,
                static_cast<std::size_t>(std::ceil(
                    std::max(
                        0.0,
                        candidate
                                ->latest_allowed_source_lane_departure_time_s -
                            kTolerance) /
                    config.qp_time_step_s)));
  const std::size_t completion_deadline_node =
      std::min(
          config.qp_horizon_steps,
          static_cast<std::size_t>(std::ceil(
              std::max(0.0,
                       candidate->latest_allowed_completion_time_s -
                           kTolerance) /
              config.qp_time_step_s)));
  // A carried deadline advances at controller cadence while this QP grid is
  // coarser.  Publish and validate the effective grid deadline so a 4.38 s
  // remaining budget, for example, is enforced consistently at node 4.4 s.
  candidate->latest_allowed_source_lane_departure_time_s =
      static_cast<double>(source_lane_release_node) *
      config.qp_time_step_s;
  candidate->latest_allowed_completion_time_s =
      static_cast<double>(completion_deadline_node) *
      config.qp_time_step_s;
  bool maneuver_progress_kinematically_reachable = true;

  for (std::size_t node_index = 0;
       node_index <= config.qp_horizon_steps; ++node_index) {
    const double time_s =
        static_cast<double>(node_index) * config.qp_time_step_s;
    const double reachable_minimum_progress_m =
        DistanceWithAcceleration(
            initial_speed_mps,
            config.minimum_longitudinal_acceleration_mps2, time_s);
    const double reachable_maximum_progress_m =
        DistanceWithAccelerationAndCap(
            initial_speed_mps,
            config.maximum_longitudinal_acceleration_mps2, time_s,
            reach_speed_cap_mps);
    const double clamped_reachable_minimum_progress_m =
        Clamp(reachable_minimum_progress_m, 0.0,
              path.geometry.path_extent_m);
    const double clamped_reachable_maximum_progress_m =
        Clamp(reachable_maximum_progress_m, 0.0,
              path.geometry.path_extent_m);

    STCorridorNode node;
    node.time_from_frontier_s = time_s;
    node.prediction_time_s =
        prediction.retained_prefix_duration_s + time_s;
    node.reachable_minimum_progress_m =
        clamped_reachable_minimum_progress_m;
    node.reachable_maximum_progress_m =
        clamped_reachable_maximum_progress_m;
    const bool source_lane_released =
        schedule.source_lane_released ||
        node_index >= source_lane_release_node;
    const bool completion_required =
        node_index >= completion_deadline_node;
    double mandatory_minimum_progress_m = 0.0;
    if (source_lane_released) {
      mandatory_minimum_progress_m =
          candidate->source_lane_departure_progress_m;
    }
    if (completion_required) {
      mandatory_minimum_progress_m = std::max(
          mandatory_minimum_progress_m,
          candidate->lane_change_completion_progress_m);
    }
    mandatory_minimum_progress_m =
        Clamp(mandatory_minimum_progress_m, 0.0,
              path.geometry.path_extent_m);
    if (mandatory_minimum_progress_m >
        clamped_reachable_maximum_progress_m + kTolerance) {
      maneuver_progress_kinematically_reachable = false;
    }
    node.ego_lane_mask =
        source_lane_released
            ? LaneBit(path.target_lane)
            : PossibleEgoLaneMask(
                  path,
                  std::max(clamped_reachable_minimum_progress_m,
                           mandatory_minimum_progress_m),
                  clamped_reachable_maximum_progress_m);
    node.lower_path_progress_m = mandatory_minimum_progress_m;
    node.upper_path_progress_m = path.geometry.path_extent_m;

    bool node_evidence_complete = node.ego_lane_mask != 0;
    std::vector<VehicleEnvelope> envelopes;
    if (node_evidence_complete) {
      envelopes = EnvelopesAtTime(prediction, node.prediction_time_s,
                                  node.ego_lane_mask,
                                  &node_evidence_complete);
    }
    const bool target_lane_possible =
        (node.ego_lane_mask & LaneBit(path.target_lane)) != 0;
    const bool front_boundary_missing_from_target =
        target_lane_possible && behavior.gap.has_front_vehicle &&
        !HasVehicleInLane(envelopes, behavior.gap.front_vehicle_id,
                          path.target_lane);
    const bool rear_boundary_missing_from_target =
        target_lane_possible && behavior.gap.has_rear_vehicle &&
        !HasVehicleInLane(envelopes, behavior.gap.rear_vehicle_id,
                          path.target_lane);
    if (front_boundary_missing_from_target ||
        rear_boundary_missing_from_target) {
      if (!schedule.committed_continuation) {
        node_evidence_complete = false;
      } else {
        bool all_lane_evidence_complete = true;
        const std::vector<VehicleEnvelope> all_lane_envelopes =
            EnvelopesAtTime(prediction, node.prediction_time_s,
                            all_lane_mask, &all_lane_evidence_complete);
        node_evidence_complete =
            node_evidence_complete && all_lane_evidence_complete;
        const auto has_valid_vehicle =
            [&all_lane_envelopes](int vehicle_id) {
              for (const VehicleEnvelope &envelope :
                   all_lane_envelopes) {
                if (envelope.vehicle_id == vehicle_id &&
                    envelope.source_track_valid) {
                  return true;
                }
              }
              return false;
            };
        if ((front_boundary_missing_from_target &&
             !has_valid_vehicle(behavior.gap.front_vehicle_id)) ||
            (rear_boundary_missing_from_target &&
             !has_valid_vehicle(behavior.gap.rear_vehicle_id))) {
          node_evidence_complete = false;
        }
      }
    }

    for (const VehicleEnvelope &envelope : envelopes) {
      const STConstraintSide side =
          SideForVehicle(behavior, fixed_sides, envelope.vehicle_id,
                         preserve_front_boundary,
                         preserve_rear_boundary);
      ++node.relevant_vehicle_count;
      if (!envelope.safety_admissible &&
          !(schedule.committed_continuation &&
            envelope.source_track_valid)) {
        node_evidence_complete = false;
      }
      if (side == STConstraintSide::kUpper) {
        const double upper = ProjectRoadSToPathProgress(
            path.geometry,
            envelope.occupied_road_s_min_m - clearance_m);
        if (upper < node.upper_path_progress_m - kTolerance ||
            (std::fabs(upper - node.upper_path_progress_m) <= kTolerance &&
             (!node.upper_source.present ||
              envelope.vehicle_id < node.upper_source.vehicle_id))) {
          node.upper_path_progress_m = upper;
          SetSource(envelope, &node.upper_source);
        }
      } else if (side == STConstraintSide::kLower) {
        const double lower = ProjectRoadSToPathProgress(
            path.geometry,
            envelope.occupied_road_s_max_m + clearance_m);
        if (lower > node.lower_path_progress_m + kTolerance ||
            (std::fabs(lower - node.lower_path_progress_m) <= kTolerance &&
             (!node.lower_source.present ||
              envelope.vehicle_id < node.lower_source.vehicle_id))) {
          node.lower_path_progress_m = lower;
          SetSource(envelope, &node.lower_source);
        }
      } else {
        // A trajectory without a frontier-side classification is unsafe to
        // place into a convex corridor; do not silently choose a side later.
        node_evidence_complete = false;
      }
    }

    node.lower_path_progress_m =
        std::max(0.0, node.lower_path_progress_m);
    node.upper_path_progress_m =
        std::min(path.geometry.path_extent_m,
                 node.upper_path_progress_m);
    node.available_width_m = node.upper_path_progress_m -
                             node.lower_path_progress_m;
    if ((node.ego_lane_mask & (node.ego_lane_mask - 1)) != 0) {
      ++candidate->corridor.dual_lane_node_count;
    }
    if (node.lower_source.present) {
      ++candidate->corridor.lower_constrained_node_count;
    }
    if (node.upper_source.present) {
      ++candidate->corridor.upper_constrained_node_count;
    }
    if (node.available_width_m < candidate->corridor.minimum_width_m) {
      candidate->corridor.minimum_width_m = node.available_width_m;
      candidate->corridor.minimum_width_node = node_index;
      candidate->corridor.minimum_width_lower_source = node.lower_source;
      candidate->corridor.minimum_width_upper_source = node.upper_source;
    }
    if (!node_evidence_complete) {
      candidate->corridor.prediction_evidence_complete = false;
      if (candidate->corridor.first_evidence_failure_node >
          config.qp_horizon_steps) {
        candidate->corridor.first_evidence_failure_node = node_index;
      }
    }
    const bool initial_progress_excluded =
        node_index == 0 &&
        (node.lower_path_progress_m > kTolerance ||
         node.upper_path_progress_m < -kTolerance);
    if (node.available_width_m < 0.0 ||
        initial_progress_excluded) {
      candidate->corridor.empty = true;
      if (candidate->corridor.first_empty_node >
          config.qp_horizon_steps) {
        candidate->corridor.first_empty_node = node_index;
        candidate->corridor.first_empty_lower_source = node.lower_source;
        candidate->corridor.first_empty_upper_source = node.upper_source;
      }
    }
    candidate->corridor.nodes.push_back(node);

    const double maximum_abs_curvature_per_m = MaximumAbsCurvature(
        path.geometry, clamped_reachable_minimum_progress_m,
        clamped_reachable_maximum_progress_m);
    const double curvature_speed_limit_mps = std::sqrt(
        config.lateral_acceleration_budget_mps2 /
        std::max(maximum_abs_curvature_per_m,
                 config.curvature_epsilon_per_m));
    // Match the control QP's recoverable-speed convention for an inherited
    // regulatory overspeed. Curvature is never grandfathered: violating its
    // physical budget at the frontier rejects the candidate before QP.
    const double regulatory_speed_limit_mps =
        std::max(config.maximum_speed_mps, initial_speed_mps);
    const double speed_limit_mps = std::min(
        regulatory_speed_limit_mps, curvature_speed_limit_mps);
    candidate->speed_budget.maximum_speed_mps.push_back(speed_limit_mps);
    if (speed_limit_mps <
            candidate->speed_budget.minimum_speed_limit_mps ||
        (speed_limit_mps ==
             candidate->speed_budget.minimum_speed_limit_mps &&
         node_index < candidate->speed_budget.limiting_node)) {
      candidate->speed_budget.minimum_speed_limit_mps = speed_limit_mps;
      candidate->speed_budget.limiting_node = node_index;
      candidate->speed_budget.limiting_abs_curvature_per_m =
          maximum_abs_curvature_per_m;
    }
  }

  candidate->speed_budget.initial_speed_feasible =
      !candidate->speed_budget.maximum_speed_mps.empty() &&
      initial_speed_mps <=
          candidate->speed_budget.maximum_speed_mps.front() +
              kTolerance;
  if (!candidate->corridor.prediction_evidence_complete) {
    candidate->status =
        STCandidateStatus::kPredictionEvidenceIncomplete;
  } else if (candidate->corridor.empty) {
    candidate->status = STCandidateStatus::kCorridorEmpty;
  } else if (!maneuver_progress_kinematically_reachable) {
    candidate->status = STCandidateStatus::kHorizonInsufficient;
  } else if (!candidate->speed_budget.initial_speed_feasible) {
    candidate->status =
        STCandidateStatus::kCurvatureSpeedInfeasible;
  } else {
    candidate->status = STCandidateStatus::kCorridorReady;
  }
}

} // namespace

const char *STConstraintSideName(STConstraintSide side) {
  switch (side) {
  case STConstraintSide::kNone:
    return "None";
  case STConstraintSide::kLower:
    return "Lower";
  case STConstraintSide::kUpper:
    return "Upper";
  }
  return "Unknown";
}

const char *STCandidateStatusName(STCandidateStatus status) {
  switch (status) {
  case STCandidateStatus::kInvalidInput:
    return "InvalidInput";
  case STCandidateStatus::kPredictionEvidenceIncomplete:
    return "PredictionEvidenceIncomplete";
  case STCandidateStatus::kCorridorEmpty:
    return "CorridorEmpty";
  case STCandidateStatus::kCurvatureSpeedInfeasible:
    return "CurvatureSpeedInfeasible";
  case STCandidateStatus::kCorridorReady:
    return "CorridorReady";
  case STCandidateStatus::kQpInfeasible:
    return "QpInfeasible";
  case STCandidateStatus::kQpTimeout:
    return "QpTimeout";
  case STCandidateStatus::kQpResultInvalid:
    return "QpResultInvalid";
  case STCandidateStatus::kHorizonInsufficient:
    return "HorizonInsufficient";
  case STCandidateStatus::kQpSolved:
    return "QpSolved";
  case STCandidateStatus::kDeadlineSkipped:
    return "DeadlineSkipped";
  }
  return "Unknown";
}

const STCandidateEvaluation *FindSTCandidate(
    const STCandidateBatchSnapshot &snapshot,
    std::uint64_t candidate_id) {
  for (const STCandidateEvaluation &candidate : snapshot.candidates) {
    if (candidate.candidate_id == candidate_id) {
      return &candidate;
    }
  }
  return nullptr;
}

STCorridorPlanner::STCorridorPlanner(
    const STCorridorPlannerConfig &config)
    : config_(config) {
  ValidateConfig(config_);
}

STCandidateBatchSnapshot STCorridorPlanner::Build(
    const PlanningSnapshot &planning,
    const BehaviorPlanningSnapshot &behavior,
    const SpatialPathBatchSnapshot &spatial_paths,
    const FullLaneTrafficPredictionSnapshot &prediction,
    const STManeuverSchedule &schedule) const {
  STCandidateBatchSnapshot result;
  result.cycle = planning.cycle;
  if (behavior.cycle != planning.cycle ||
      spatial_paths.cycle != planning.cycle ||
      prediction.cycle != planning.cycle ||
      !Finite(prediction.retained_prefix_duration_s) ||
      prediction.retained_prefix_duration_s < 0.0) {
    throw std::invalid_argument("inconsistent P2.4 snapshot cycle");
  }

  result.candidates.reserve(std::min(
      config_.maximum_candidates, spatial_paths.candidates.size()));
  for (const SpatialPathCandidate &path : spatial_paths.candidates) {
    if (path.status != SpatialPathCandidateStatus::kPrecheckPassed) {
      continue;
    }
    if (result.candidates.size() >= config_.maximum_candidates) {
      throw std::invalid_argument(
          "P2.4 candidate count exceeds configured capacity");
    }
    STCandidateEvaluation candidate;
    const BehaviorCandidate *behavior_candidate =
        FindBehaviorCandidate(behavior, path.candidate_id);
    if (behavior_candidate == nullptr) {
      candidate.candidate_id = path.candidate_id;
      candidate.source_lane = path.source_lane;
      candidate.target_lane = path.target_lane;
      candidate.status = STCandidateStatus::kInvalidInput;
    } else {
      BuildOneCandidate(planning, *behavior_candidate, path, prediction,
                        config_, schedule, &candidate);
    }
    result.candidates.push_back(std::move(candidate));
  }
  RefreshCounts(&result);
  return result;
}

LongitudinalQpInput STCorridorPlanner::MakeQpInput(
    const PlanningSnapshot &planning,
    const STCandidateEvaluation &candidate,
    double target_speed_mps) const {
  if (candidate.status != STCandidateStatus::kCorridorReady ||
      candidate.corridor.nodes.size() != config_.qp_horizon_steps + 1 ||
      candidate.speed_budget.maximum_speed_mps.size() !=
          config_.qp_horizon_steps + 1 ||
      !Finite(target_speed_mps) || target_speed_mps <= 0.0) {
    throw std::invalid_argument("candidate is not ready for P2.4 QP");
  }
  LongitudinalQpInput input;
  input.initial_speed_mps =
      std::max(0.0, planning.frontier.longitudinal.v);
  input.initial_acceleration_mps2 = planning.frontier.longitudinal.a;
  input.initial_jerk_valid =
      planning.frontier.state_source ==
      PlanningStateSource::kExactInherited;
  input.initial_jerk_mps3 = planning.frontier.longitudinal.j;
  input.safety_policy = LongitudinalSafetyPolicy::kNormalOperational;
  input.reference_speed_mps.reserve(config_.qp_horizon_steps + 1);
  input.minimum_progress_m.reserve(config_.qp_horizon_steps + 1);
  input.maximum_progress_m.reserve(config_.qp_horizon_steps + 1);
  input.maximum_speed_mps = candidate.speed_budget.maximum_speed_mps;
  for (std::size_t index = 0;
       index <= config_.qp_horizon_steps; ++index) {
    input.reference_speed_mps.push_back(
        std::min(target_speed_mps, input.maximum_speed_mps[index]));
    input.minimum_progress_m.push_back(
        candidate.corridor.nodes[index].lower_path_progress_m);
    input.maximum_progress_m.push_back(
        candidate.corridor.nodes[index].upper_path_progress_m);
  }
  return input;
}

void STCorridorPlanner::AttachQpResult(
    const LongitudinalQpResult &result,
    STCandidateEvaluation *candidate) const {
  if (candidate == nullptr ||
      candidate->status != STCandidateStatus::kCorridorReady) {
    throw std::invalid_argument("invalid P2.4 QP result target");
  }
  candidate->qp_attempted = true;
  candidate->qp_success = result.success;
  candidate->qp_status = result.status;
  candidate->qp_objective = result.objective;
  candidate->qp_result = result;
  if (!result.success) {
    candidate->status = IsTimeLimitStatus(result.status)
                            ? STCandidateStatus::kQpTimeout
                            : STCandidateStatus::kQpInfeasible;
    return;
  }
  if (result.trajectory.states.size() !=
          candidate->corridor.nodes.size() ||
      candidate->speed_budget.maximum_speed_mps.size() !=
          result.trajectory.states.size()) {
    candidate->status = STCandidateStatus::kQpResultInvalid;
    return;
  }

  candidate->minimum_lower_margin_m =
      std::numeric_limits<double>::infinity();
  candidate->minimum_upper_margin_m =
      std::numeric_limits<double>::infinity();
  candidate->maximum_speed_excess_mps = 0.0;
  candidate->qp_bounds_satisfied = true;
  for (std::size_t index = 0;
       index < result.trajectory.states.size(); ++index) {
    const LongitudinalState &state = result.trajectory.states[index];
    const STCorridorNode &node = candidate->corridor.nodes[index];
    const double lower_margin =
        state.s - node.lower_path_progress_m;
    const double upper_margin =
        node.upper_path_progress_m - state.s;
    const double speed_excess =
        state.v - candidate->speed_budget.maximum_speed_mps[index];
    candidate->minimum_lower_margin_m =
        std::min(candidate->minimum_lower_margin_m, lower_margin);
    candidate->minimum_upper_margin_m =
        std::min(candidate->minimum_upper_margin_m, upper_margin);
    candidate->maximum_speed_excess_mps =
        std::max(candidate->maximum_speed_excess_mps, speed_excess);
    if (!Finite(state.s) || !Finite(state.v) ||
        lower_margin < -config_.qp_bound_tolerance_m ||
        upper_margin < -config_.qp_bound_tolerance_m ||
        speed_excess > config_.qp_speed_tolerance_mps) {
      candidate->qp_bounds_satisfied = false;
    }
  }
  candidate->terminal_progress_m = result.trajectory.states.back().s;
  candidate->terminal_speed_mps = result.trajectory.states.back().v;
  for (std::size_t index = 0;
       index < result.trajectory.states.size(); ++index) {
    if (!candidate->source_lane_departure_observed &&
        result.trajectory.states[index].s +
                config_.qp_bound_tolerance_m >=
            candidate->source_lane_departure_progress_m) {
      candidate->source_lane_departure_observed = true;
      candidate->source_lane_departed_in_time =
          static_cast<double>(index) * config_.qp_time_step_s <=
          candidate->latest_allowed_source_lane_departure_time_s +
              kTolerance;
      candidate->source_lane_departure_time_s =
          static_cast<double>(index) * config_.qp_time_step_s;
    }
    if (result.trajectory.states[index].s +
            config_.qp_bound_tolerance_m >=
        candidate->lane_change_completion_progress_m) {
      candidate->lane_change_completion_observed = true;
      candidate->lane_change_completed_in_time =
          static_cast<double>(index) * config_.qp_time_step_s <=
          candidate->latest_allowed_completion_time_s + kTolerance;
      candidate->lane_change_completion_time_s =
          static_cast<double>(index) * config_.qp_time_step_s;
      break;
    }
  }
  if (!candidate->qp_bounds_satisfied) {
    candidate->status = STCandidateStatus::kQpResultInvalid;
  } else if (!candidate->source_lane_departed_in_time ||
             !candidate->lane_change_completed_in_time) {
    candidate->status = STCandidateStatus::kHorizonInsufficient;
  } else {
    candidate->status = STCandidateStatus::kQpSolved;
  }
}

void STCorridorPlanner::MarkDeadlineSkipped(
    STCandidateEvaluation *candidate) const {
  if (candidate == nullptr ||
      candidate->status != STCandidateStatus::kCorridorReady) {
    throw std::invalid_argument("invalid P2.4 deadline target");
  }
  candidate->status = STCandidateStatus::kDeadlineSkipped;
  candidate->qp_status = "candidate QP deadline budget exhausted";
}

void STCorridorPlanner::RefreshCounts(
    STCandidateBatchSnapshot *snapshot) const {
  if (snapshot == nullptr) {
    throw std::invalid_argument("null P2.4 candidate batch");
  }
  snapshot->evaluated_candidate_count = snapshot->candidates.size();
  snapshot->invalid_candidate_count = 0;
  snapshot->corridor_ready_candidate_count = 0;
  snapshot->corridor_empty_candidate_count = 0;
  snapshot->prediction_rejected_candidate_count = 0;
  snapshot->curvature_rejected_candidate_count = 0;
  snapshot->qp_attempted_candidate_count = 0;
  snapshot->qp_solved_candidate_count = 0;
  snapshot->qp_failed_candidate_count = 0;
  snapshot->horizon_rejected_candidate_count = 0;
  snapshot->deadline_skipped_candidate_count = 0;
  for (const STCandidateEvaluation &candidate : snapshot->candidates) {
    if (candidate.status == STCandidateStatus::kInvalidInput) {
      ++snapshot->invalid_candidate_count;
    }
    const bool corridor_was_ready =
        candidate.status == STCandidateStatus::kCorridorReady ||
        candidate.status == STCandidateStatus::kQpInfeasible ||
        candidate.status == STCandidateStatus::kQpTimeout ||
        candidate.status == STCandidateStatus::kQpResultInvalid ||
        candidate.status == STCandidateStatus::kHorizonInsufficient ||
        candidate.status == STCandidateStatus::kQpSolved ||
        candidate.status == STCandidateStatus::kDeadlineSkipped;
    if (corridor_was_ready) {
      ++snapshot->corridor_ready_candidate_count;
    }
    if (candidate.status == STCandidateStatus::kCorridorEmpty) {
      ++snapshot->corridor_empty_candidate_count;
    } else if (candidate.status ==
               STCandidateStatus::kPredictionEvidenceIncomplete) {
      ++snapshot->prediction_rejected_candidate_count;
    } else if (candidate.status ==
               STCandidateStatus::kCurvatureSpeedInfeasible) {
      ++snapshot->curvature_rejected_candidate_count;
    } else if (candidate.status == STCandidateStatus::kQpSolved) {
      ++snapshot->qp_solved_candidate_count;
    } else if (candidate.status == STCandidateStatus::kQpInfeasible ||
               candidate.status == STCandidateStatus::kQpTimeout ||
               candidate.status == STCandidateStatus::kQpResultInvalid) {
      ++snapshot->qp_failed_candidate_count;
    } else if (candidate.status ==
               STCandidateStatus::kHorizonInsufficient) {
      ++snapshot->horizon_rejected_candidate_count;
    } else if (candidate.status ==
               STCandidateStatus::kDeadlineSkipped) {
      ++snapshot->deadline_skipped_candidate_count;
    }
    if (candidate.qp_attempted) {
      ++snapshot->qp_attempted_candidate_count;
    }
  }
}
