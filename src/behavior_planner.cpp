#include "behavior_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>

#include "planning_snapshot.h"

namespace {

const double kTolerance = 1e-9;

double Clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

bool IsFinite(double value) { return std::isfinite(value); }

// Lane attribution is based only on the predicted physical contour. A vehicle
// reaches an adjacent lane only through its observed position or the measured
// lateral rate propagated by the lateral-continuation hypothesis.
double CoarseOccupiedDMin(
    const PredictedTrafficOccupancy &occupancy) {
  return occupancy.occupied_d_min_m;
}

double CoarseOccupiedDMax(
    const PredictedTrafficOccupancy &occupancy) {
  return occupancy.occupied_d_max_m;
}

bool CoarseTrafficOccupiesLane(
    const PredictedTrafficOccupancy &occupancy, int lane,
    const BehaviorPlannerConfig &config) {
  if (lane < 0) {
    return true;
  }
  const double lane_min = static_cast<double>(lane) *
                          config.lane_width_m;
  const double lane_max = lane_min + config.lane_width_m;
  return CoarseOccupiedDMax(occupancy) >= lane_min - kTolerance &&
         CoarseOccupiedDMin(occupancy) <= lane_max + kTolerance;
}

double AlignRoadS(double road_s_m, double reference_road_s_m,
                  double track_length_m) {
  return road_s_m +
         std::round((reference_road_s_m - road_s_m) / track_length_m) *
             track_length_m;
}

void AddReason(CoarseAdmissionResult *result,
               CoarseAdmissionRejectionReason reason) {
  if (!HasCoarseAdmissionRejectionReason(*result, reason)) {
    result->rejection_reasons.push_back(reason);
  }
}

struct VehicleEnvelope {
  int id = 0;
  bool safety_admissible = true;
  TrafficPredictionHypothesis hypothesis =
      TrafficPredictionHypothesis::kNominalConstantVelocity;
  double source_relative_road_s_m = 0.0;
  double occupied_road_s_min_m = 0.0;
  double occupied_road_s_max_m = 0.0;
  double occupied_d_min_m = 0.0;
  double occupied_d_max_m = 0.0;
  double minimum_road_s_rate_mps = 0.0;
  double maximum_road_s_rate_mps = 0.0;
  double maximum_longitudinal_uncertainty_m = 0.0;
};

struct EnvelopeAccumulator {
  bool initialized = false;
  VehicleEnvelope envelope;
};

bool SampleTrajectory(
    const TrafficPredictionTrajectory &trajectory, double time_s,
    std::vector<const PredictedTrafficOccupancy *> *samples) {
  samples->clear();
  if (trajectory.occupancies.empty() || !IsFinite(time_s) ||
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
  if (std::fabs(upper->prediction_time_s - time_s) <= kTolerance ||
      upper == trajectory.occupancies.begin()) {
    samples->push_back(&*upper);
    return true;
  }
  samples->push_back(&*(upper - 1));
  samples->push_back(&*upper);
  return true;
}

void ExtendEnvelope(const TrafficPredictionTrajectory &trajectory,
                    const PredictedTrafficOccupancy &occupancy,
                    EnvelopeAccumulator *accumulator) {
  if (!accumulator->initialized) {
    accumulator->initialized = true;
    accumulator->envelope.id = trajectory.vehicle_id;
    accumulator->envelope.hypothesis = trajectory.hypothesis;
    accumulator->envelope.source_relative_road_s_m =
        trajectory.source_relative_road_s_m;
    accumulator->envelope.safety_admissible =
        trajectory.source_track_valid && trajectory.safety_admissible;
    accumulator->envelope.occupied_road_s_min_m =
        occupancy.occupied_road_s_min_m;
    accumulator->envelope.occupied_road_s_max_m =
        occupancy.occupied_road_s_max_m;
    accumulator->envelope.occupied_d_min_m =
        CoarseOccupiedDMin(occupancy);
    accumulator->envelope.occupied_d_max_m =
        CoarseOccupiedDMax(occupancy);
    accumulator->envelope.minimum_road_s_rate_mps =
        occupancy.road_s_rate_mps;
    accumulator->envelope.maximum_road_s_rate_mps =
        occupancy.road_s_rate_mps;
    accumulator->envelope.maximum_longitudinal_uncertainty_m =
        occupancy.longitudinal_uncertainty_m;
    return;
  }

  VehicleEnvelope &envelope = accumulator->envelope;
  envelope.safety_admissible =
      envelope.safety_admissible && trajectory.source_track_valid &&
      trajectory.safety_admissible;
  envelope.occupied_road_s_min_m =
      std::min(envelope.occupied_road_s_min_m,
               occupancy.occupied_road_s_min_m);
  envelope.occupied_road_s_max_m =
      std::max(envelope.occupied_road_s_max_m,
               occupancy.occupied_road_s_max_m);
  envelope.occupied_d_min_m =
      std::min(envelope.occupied_d_min_m,
               CoarseOccupiedDMin(occupancy));
  envelope.occupied_d_max_m =
      std::max(envelope.occupied_d_max_m,
               CoarseOccupiedDMax(occupancy));
  envelope.minimum_road_s_rate_mps =
      std::min(envelope.minimum_road_s_rate_mps,
               occupancy.road_s_rate_mps);
  envelope.maximum_road_s_rate_mps =
      std::max(envelope.maximum_road_s_rate_mps,
               occupancy.road_s_rate_mps);
  envelope.maximum_longitudinal_uncertainty_m =
      std::max(envelope.maximum_longitudinal_uncertainty_m,
               occupancy.longitudinal_uncertainty_m);
}

bool EnvelopeForTrajectoryAtTime(
    const TrafficPredictionTrajectory &trajectory, double time_s,
    int lane, const BehaviorPlannerConfig &config,
    VehicleEnvelope *envelope) {
  std::vector<const PredictedTrafficOccupancy *> samples;
  if (!SampleTrajectory(trajectory, time_s, &samples)) {
    return false;
  }
  EnvelopeAccumulator accumulator;
  for (const PredictedTrafficOccupancy *occupancy : samples) {
    if (CoarseTrafficOccupiesLane(*occupancy, lane, config)) {
      ExtendEnvelope(trajectory, *occupancy, &accumulator);
    }
  }
  if (!accumulator.initialized) {
    return true;
  }
  *envelope = accumulator.envelope;
  return true;
}

std::vector<VehicleEnvelope> HypothesisEnvelopesAtTime(
    const FullLaneTrafficPredictionSnapshot &prediction, double time_s,
    int lane, const BehaviorPlannerConfig &config,
    bool *evidence_complete) {
  std::vector<VehicleEnvelope> result;
  result.reserve(prediction.trajectories.size());
  *evidence_complete = true;
  for (const TrafficPredictionTrajectory &trajectory :
       prediction.trajectories) {
    VehicleEnvelope envelope;
    if (!EnvelopeForTrajectoryAtTime(trajectory, time_s, lane,
                                     config, &envelope)) {
      *evidence_complete = false;
      continue;
    }
    std::vector<const PredictedTrafficOccupancy *> samples;
    if (!SampleTrajectory(trajectory, time_s, &samples)) {
      continue;
    }
    bool occupies_lane = lane < 0;
    for (const PredictedTrafficOccupancy *occupancy : samples) {
      occupies_lane = occupies_lane ||
                      CoarseTrafficOccupiesLane(
                          *occupancy, lane, config);
    }
    if (occupies_lane) {
      result.push_back(envelope);
    }
  }
  return result;
}

std::vector<VehicleEnvelope> EnvelopesAtTime(
    const FullLaneTrafficPredictionSnapshot &prediction, double time_s,
    int lane, const BehaviorPlannerConfig &config,
    bool *evidence_complete) {
  std::map<int, EnvelopeAccumulator> accumulators;
  *evidence_complete = true;
  for (const TrafficPredictionTrajectory &trajectory :
       prediction.trajectories) {
    std::vector<const PredictedTrafficOccupancy *> samples;
    if (!SampleTrajectory(trajectory, time_s, &samples)) {
      *evidence_complete = false;
      continue;
    }
    bool trajectory_occupies_lane = lane < 0;
    if (lane >= 0) {
      for (const PredictedTrafficOccupancy *occupancy : samples) {
        if (CoarseTrafficOccupiesLane(*occupancy, lane, config)) {
          trajectory_occupies_lane = true;
          break;
        }
      }
    }
    if (!trajectory_occupies_lane) {
      continue;
    }
    EnvelopeAccumulator &accumulator =
        accumulators[trajectory.vehicle_id];
    for (const PredictedTrafficOccupancy *occupancy : samples) {
      if (CoarseTrafficOccupiesLane(*occupancy, lane, config)) {
        ExtendEnvelope(trajectory, *occupancy, &accumulator);
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
  std::sort(result.begin(), result.end(),
            [](const VehicleEnvelope &left,
               const VehicleEnvelope &right) {
              const double left_center =
                  0.5 * (left.occupied_road_s_min_m +
                         left.occupied_road_s_max_m);
              const double right_center =
                  0.5 * (right.occupied_road_s_min_m +
                         right.occupied_road_s_max_m);
              if (left_center != right_center) {
                return left_center < right_center;
              }
              return left.id < right.id;
            });
  return result;
}

double EnvelopeCenter(const VehicleEnvelope &envelope) {
  return 0.5 * (envelope.occupied_road_s_min_m +
                envelope.occupied_road_s_max_m);
}

std::vector<VehicleEnvelope> FilterSearchWindow(
    const std::vector<VehicleEnvelope> &envelopes,
    double ego_reference_road_s_m,
    const BehaviorPlannerConfig &config) {
  std::vector<VehicleEnvelope> result;
  result.reserve(envelopes.size());
  for (const VehicleEnvelope &envelope : envelopes) {
    const double center = EnvelopeCenter(envelope);
    if (center + kTolerance >=
            ego_reference_road_s_m - config.rear_search_distance_m &&
        center <= ego_reference_road_s_m +
                      config.front_search_distance_m + kTolerance) {
      result.push_back(envelope);
    }
  }
  return result;
}

GapId GapAroundPosition(const std::vector<VehicleEnvelope> &envelopes,
                        int lane, double road_s_m) {
  GapId result;
  result.target_lane = lane;
  for (const VehicleEnvelope &envelope : envelopes) {
    if (EnvelopeCenter(envelope) + kTolerance >= road_s_m) {
      result.has_front_vehicle = true;
      result.front_vehicle_id = envelope.id;
      break;
    }
    result.has_rear_vehicle = true;
    result.rear_vehicle_id = envelope.id;
  }
  return result;
}

bool GapBehindNearestRear(const std::vector<VehicleEnvelope> &envelopes,
                          int lane, double road_s_m, GapId *gap) {
  std::size_t rear_index = envelopes.size();
  for (std::size_t index = 0; index < envelopes.size(); ++index) {
    if (EnvelopeCenter(envelopes[index]) < road_s_m - kTolerance) {
      rear_index = index;
    } else {
      break;
    }
  }
  if (rear_index == envelopes.size()) {
    return false;
  }
  *gap = GapId();
  gap->target_lane = lane;
  gap->has_front_vehicle = true;
  gap->front_vehicle_id = envelopes[rear_index].id;
  if (rear_index > 0) {
    gap->has_rear_vehicle = true;
    gap->rear_vehicle_id = envelopes[rear_index - 1].id;
  }
  return true;
}

const VehicleEnvelope *FindEnvelope(
    const std::vector<VehicleEnvelope> &envelopes, int id,
    std::size_t *index) {
  for (std::size_t current = 0; current < envelopes.size(); ++current) {
    if (envelopes[current].id == id) {
      if (index != nullptr) {
        *index = current;
      }
      return &envelopes[current];
    }
  }
  return nullptr;
}

struct BoundaryAdjacencyResult {
  const VehicleEnvelope *front = nullptr;
  const VehicleEnvelope *rear = nullptr;
  bool adjacent = false;
  GapTopologyFailureKind failure_kind =
      GapTopologyFailureKind::kNone;
  bool actual_front_present = false;
  int actual_front_vehicle_id = 0;
  bool actual_rear_present = false;
  int actual_rear_vehicle_id = 0;
  bool expected_boundaries_reversed = false;
};

BoundaryAdjacencyResult CheckBoundaryAdjacency(
    const GapId &gap,
    const std::vector<VehicleEnvelope> &envelopes) {
  BoundaryAdjacencyResult result;
  std::size_t front_index = envelopes.size();
  std::size_t rear_index = envelopes.size();
  result.front = gap.has_front_vehicle
                     ? FindEnvelope(envelopes, gap.front_vehicle_id,
                                    &front_index)
                     : nullptr;
  result.rear = gap.has_rear_vehicle
                    ? FindEnvelope(envelopes, gap.rear_vehicle_id,
                                   &rear_index)
                    : nullptr;

  const bool front_missing =
      gap.has_front_vehicle && result.front == nullptr;
  const bool rear_missing =
      gap.has_rear_vehicle && result.rear == nullptr;
  if (front_missing && rear_missing) {
    result.failure_kind =
        GapTopologyFailureKind::kExpectedFrontAndRearMissing;
  } else if (front_missing) {
    result.failure_kind =
        GapTopologyFailureKind::kExpectedFrontMissing;
  } else if (rear_missing) {
    result.failure_kind =
        GapTopologyFailureKind::kExpectedRearMissing;
  }

  if (result.front != nullptr) {
    result.actual_front_present = true;
    result.actual_front_vehicle_id = result.front->id;
    if (front_index > 0) {
      result.actual_rear_present = true;
      result.actual_rear_vehicle_id = envelopes[front_index - 1].id;
    }
  }
  if (result.rear != nullptr) {
    result.actual_rear_present = true;
    result.actual_rear_vehicle_id = result.rear->id;
    if (rear_index + 1 < envelopes.size()) {
      result.actual_front_present = true;
      result.actual_front_vehicle_id = envelopes[rear_index + 1].id;
    }
  }
  if (front_missing || rear_missing) {
    return result;
  }

  if (gap.has_front_vehicle && gap.has_rear_vehicle) {
    result.adjacent = rear_index + 1 == front_index;
    result.expected_boundaries_reversed = rear_index >= front_index;
    if (!result.adjacent) {
      result.failure_kind =
          GapTopologyFailureKind::kBoundariesNotAdjacent;
    }
  } else if (gap.has_front_vehicle) {
    result.adjacent = front_index == 0;
    if (!result.adjacent) {
      result.failure_kind =
          GapTopologyFailureKind::kUnexpectedRearBoundary;
    }
  } else if (gap.has_rear_vehicle) {
    result.adjacent = rear_index + 1 == envelopes.size();
    if (!result.adjacent) {
      result.failure_kind =
          GapTopologyFailureKind::kUnexpectedFrontBoundary;
    }
  } else {
    result.adjacent = envelopes.empty();
    if (!result.adjacent) {
      result.failure_kind =
          GapTopologyFailureKind::kUnexpectedOccupiedGap;
      result.actual_front_present = true;
      result.actual_front_vehicle_id = envelopes.front().id;
      result.actual_rear_present = true;
      result.actual_rear_vehicle_id = envelopes.back().id;
    }
  }
  return result;
}

double DistanceWithAcceleration(double speed_mps,
                                double acceleration_mps2,
                                double duration_s) {
  if (duration_s <= 0.0) {
    return 0.0;
  }
  speed_mps = std::max(0.0, speed_mps);
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

double DistanceWithAccelerationAndSpeedCap(
    double speed_mps, double acceleration_mps2, double duration_s,
    double speed_cap_mps) {
  if (acceleration_mps2 <= 0.0 || speed_mps >= speed_cap_mps) {
    return std::min(speed_mps, speed_cap_mps) * duration_s;
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

std::vector<double> GapEvaluationTimes(
    double start_time_s, const BehaviorPlannerConfig &config) {
  const double duration_s = config.coarse_lane_change_duration_s +
                            config.post_maneuver_observation_s;
  const std::size_t intervals = static_cast<std::size_t>(
      std::ceil(duration_s / config.gap_evaluation_time_step_s -
                kTolerance));
  if (intervals + 1 > config.maximum_gap_samples) {
    throw std::invalid_argument(
        "behavior gap sampling exceeds configured capacity");
  }
  std::vector<double> result;
  result.reserve(intervals + 1);
  for (std::size_t index = 0; index <= intervals; ++index) {
    result.push_back(std::min(
        start_time_s + duration_s,
        start_time_s + static_cast<double>(index) *
                           config.gap_evaluation_time_step_s));
  }
  if (result.empty() ||
      result.back() + kTolerance < start_time_s + duration_s) {
    result.push_back(start_time_s + duration_s);
  }
  return result;
}

bool FindMergeCorridorIntrusion(
    const GapId &gap, double time_s, double reachable_min_road_s_m,
    double reachable_max_road_s_m,
    const FullLaneTrafficPredictionSnapshot &prediction,
    const BehaviorPlannerConfig &config, int *vehicle_id,
    TrafficPredictionHypothesis *hypothesis,
    bool *evidence_complete) {
  const std::vector<VehicleEnvelope> envelopes =
      HypothesisEnvelopesAtTime(prediction, time_s, gap.target_lane,
                                config, evidence_complete);
  const double clearance_m =
      0.5 * config.ego_length_m + config.standstill_clearance_m +
      config.physical_collision_margin_m;
  for (const VehicleEnvelope &envelope : envelopes) {
    if ((gap.has_front_vehicle &&
         envelope.id == gap.front_vehicle_id) ||
        (gap.has_rear_vehicle &&
         envelope.id == gap.rear_vehicle_id)) {
      continue;
    }
    const bool intersects_reachable_corridor =
        envelope.occupied_road_s_max_m + clearance_m + kTolerance >=
            reachable_min_road_s_m &&
        envelope.occupied_road_s_min_m - clearance_m <=
            reachable_max_road_s_m + kTolerance;
    if (intersects_reachable_corridor) {
      *vehicle_id = envelope.id;
      *hypothesis = envelope.hypothesis;
      return true;
    }
  }
  return false;
}

TrafficGap EvaluateGap(
    const GapId &gap, double start_ego_road_s_m,
    double start_ego_speed_mps,
    const FullLaneTrafficPredictionSnapshot &prediction,
    const BehaviorPlannerConfig &config) {
  TrafficGap result;
  result.id = gap;
  result.topology_consistent = true;
  result.prediction_evidence_complete = true;
  result.minimum_ego_center_window_m =
      std::numeric_limits<double>::infinity();
  const std::vector<double> times = GapEvaluationTimes(
      prediction.retained_prefix_duration_s, config);
  const double required_end_time_s = times.back();
  if (prediction.grid_coverage_s + kTolerance < required_end_time_s) {
    result.prediction_evidence_complete = false;
  }

  const double half_ego_length_m = 0.5 * config.ego_length_m;
  const double clearance_m = config.standstill_clearance_m +
                             config.physical_collision_margin_m;
  for (double time_s : times) {
    const double relative_time_s =
        std::max(0.0, time_s - prediction.retained_prefix_duration_s);
    const double ego_reference_s =
        start_ego_road_s_m + start_ego_speed_mps * relative_time_s;
    bool time_evidence_complete = true;
    const std::vector<VehicleEnvelope> all_envelopes =
        EnvelopesAtTime(prediction, time_s, gap.target_lane,
                        config, &time_evidence_complete);
    const std::vector<VehicleEnvelope> envelopes = FilterSearchWindow(
        all_envelopes, ego_reference_s, config);
    const BoundaryAdjacencyResult boundary =
        CheckBoundaryAdjacency(gap, envelopes);
    const VehicleEnvelope *front = boundary.front;
    const VehicleEnvelope *rear = boundary.rear;
    const bool adjacent = boundary.adjacent;

    if (!adjacent && !result.topology_failure_observed) {
      result.topology_failure_observed = true;
      result.first_topology_failure_kind = boundary.failure_kind;
      result.first_topology_failure_time_s = time_s;
      result.first_topology_expected_front_found = front != nullptr;
      result.first_topology_expected_rear_found = rear != nullptr;
      result.first_topology_actual_front_present =
          boundary.actual_front_present;
      result.first_topology_actual_front_vehicle_id =
          boundary.actual_front_vehicle_id;
      result.first_topology_actual_rear_present =
          boundary.actual_rear_present;
      result.first_topology_actual_rear_vehicle_id =
          boundary.actual_rear_vehicle_id;
    }

    if (!adjacent &&
        relative_time_s <=
            config.coarse_lane_change_duration_s + kTolerance) {
      const double reachable_min_road_s_m =
          start_ego_road_s_m +
          DistanceWithAcceleration(
              start_ego_speed_mps,
              config.minimum_longitudinal_acceleration_mps2,
              relative_time_s);
      const double reachable_max_road_s_m =
          start_ego_road_s_m +
          DistanceWithAccelerationAndSpeedCap(
              start_ego_speed_mps,
              config.maximum_longitudinal_acceleration_mps2,
              relative_time_s, config.target_speed_mps);
      int intrusion_vehicle_id = 0;
      TrafficPredictionHypothesis intrusion_hypothesis =
          TrafficPredictionHypothesis::kNominalConstantVelocity;
      bool intrusion_evidence_complete = true;
      const bool intrusion = FindMergeCorridorIntrusion(
          gap, time_s, reachable_min_road_s_m,
          reachable_max_road_s_m, prediction, config,
          &intrusion_vehicle_id, &intrusion_hypothesis,
          &intrusion_evidence_complete);
      time_evidence_complete =
          time_evidence_complete && intrusion_evidence_complete;
      if (intrusion && !result.merge_corridor_intrusion_observed) {
        result.merge_corridor_intrusion_observed = true;
        result.first_merge_corridor_intrusion_time_s = time_s;
        result.first_merge_corridor_intrusion_vehicle_id =
            intrusion_vehicle_id;
        result.first_merge_corridor_intrusion_hypothesis =
            intrusion_hypothesis;
      }
      if (boundary.expected_boundaries_reversed) {
        result.topology_consistent = false;
        if (!result.expected_boundaries_reversed_observed) {
          result.expected_boundaries_reversed_observed = true;
          result.first_expected_boundaries_reversed_time_s = time_s;
        }
      }
    }

    TrafficGapSample sample;
    sample.prediction_time_s = time_s;
    sample.front_boundary_present =
        !gap.has_front_vehicle || front != nullptr;
    sample.rear_boundary_present =
        !gap.has_rear_vehicle || rear != nullptr;
    sample.boundaries_adjacent = adjacent;
    if (result.samples.empty()) {
      result.current_observation_valid =
          time_evidence_complete && sample.front_boundary_present &&
          sample.rear_boundary_present;
    }
    if (front != nullptr) {
      sample.front_track_admissible = front->safety_admissible;
      sample.front_occupied_min_road_s_m =
          front->occupied_road_s_min_m;
      sample.front_road_s_rate_mps =
          front->minimum_road_s_rate_mps;
      sample.front_uncertainty_m =
          front->maximum_longitudinal_uncertainty_m;
      sample.available_ego_center_max_road_s_m =
          front->occupied_road_s_min_m - half_ego_length_m -
          clearance_m;
    } else {
      sample.available_ego_center_max_road_s_m =
          ego_reference_s + config.front_search_distance_m -
          half_ego_length_m - clearance_m;
    }
    if (rear != nullptr) {
      sample.rear_track_admissible = rear->safety_admissible;
      sample.rear_occupied_max_road_s_m =
          rear->occupied_road_s_max_m;
      sample.rear_road_s_rate_mps =
          rear->maximum_road_s_rate_mps;
      sample.rear_uncertainty_m =
          rear->maximum_longitudinal_uncertainty_m;
      sample.available_ego_center_min_road_s_m =
          rear->occupied_road_s_max_m + half_ego_length_m +
          clearance_m;
    } else {
      sample.available_ego_center_min_road_s_m =
          ego_reference_s - config.rear_search_distance_m +
          half_ego_length_m + clearance_m;
    }
    sample.available_ego_center_window_m =
        sample.available_ego_center_max_road_s_m -
        sample.available_ego_center_min_road_s_m;
    result.minimum_ego_center_window_m =
        std::min(result.minimum_ego_center_window_m,
                 sample.available_ego_center_window_m);
    result.maximum_boundary_uncertainty_m =
        std::max(result.maximum_boundary_uncertainty_m,
                 std::max(sample.front_uncertainty_m,
                          sample.rear_uncertainty_m));
    result.prediction_evidence_complete =
        result.prediction_evidence_complete &&
        time_evidence_complete && sample.front_boundary_present &&
        sample.rear_boundary_present;
    result.samples.push_back(sample);
  }
  if (!IsFinite(result.minimum_ego_center_window_m)) {
    result.minimum_ego_center_window_m = 0.0;
  }
  return result;
}

const GapStabilityRecord *FindStability(
    const BehaviorPlannerState &state, const GapId &gap) {
  for (const GapStabilityRecord &record : state.gap_stability) {
    if (record.gap == gap) {
      return &record;
    }
  }
  return nullptr;
}

std::uint64_t HashCandidate(BehaviorType behavior, PassingOrder order,
                            const GapId &gap) {
  std::uint64_t hash = UINT64_C(1469598103934665603);
  const std::uint64_t prime = UINT64_C(1099511628211);
  const std::uint64_t values[] = {
      static_cast<std::uint64_t>(behavior),
      static_cast<std::uint64_t>(order),
      static_cast<std::uint64_t>(static_cast<std::int64_t>(
          gap.target_lane)),
      gap.has_front_vehicle ? UINT64_C(1) : UINT64_C(0),
      static_cast<std::uint64_t>(static_cast<std::int64_t>(
          gap.front_vehicle_id)),
      gap.has_rear_vehicle ? UINT64_C(1) : UINT64_C(0),
      static_cast<std::uint64_t>(static_cast<std::int64_t>(
          gap.rear_vehicle_id))};
  for (std::uint64_t value : values) {
    for (unsigned int byte = 0; byte < 8; ++byte) {
      hash ^= (value >> (byte * 8U)) & UINT64_C(0xff);
      hash *= prime;
    }
  }
  return hash == 0 ? 1 : hash;
}

double EstimatedLaneProgress(
    const GapId &gap, double ego_road_s_m, double ego_speed_mps,
    const FullLaneTrafficPredictionSnapshot &prediction,
    const BehaviorPlannerConfig &config) {
  const double horizon_s =
      std::max(prediction.planning_horizon_s, kTolerance);
  const double free_flow_progress_m =
      DistanceWithAccelerationAndSpeedCap(
          ego_speed_mps, config.maximum_longitudinal_acceleration_mps2,
          horizon_s, config.target_speed_mps);
  if (!gap.has_front_vehicle) {
    return free_flow_progress_m;
  }

  const double terminal_time_s =
      prediction.retained_prefix_duration_s + horizon_s;
  bool evidence_complete = true;
  const std::vector<VehicleEnvelope> terminal_envelopes =
      EnvelopesAtTime(prediction, terminal_time_s, gap.target_lane,
                      config, &evidence_complete);
  const VehicleEnvelope *front =
      FindEnvelope(terminal_envelopes, gap.front_vehicle_id, nullptr);
  if (!evidence_complete || front == nullptr) {
    // The admission layer separately rejects incomplete prediction evidence.
    // A missing terminal front cannot be used to claim extra utility.
    return 0.0;
  }

  const double available_ego_center_max_road_s_m =
      front->occupied_road_s_min_m - 0.5 * config.ego_length_m -
      config.standstill_clearance_m -
      config.physical_collision_margin_m;
  const double front_limited_progress_m = std::max(
      0.0, available_ego_center_max_road_s_m - ego_road_s_m);
  return std::min(free_flow_progress_m, front_limited_progress_m);
}

bool RetainedPrefixEvidence(
    const PlanningSnapshot &planning,
    const FullLaneTrafficPredictionSnapshot &prediction,
    double frontier_ego_road_s_m, double frontier_ego_speed_mps,
    double track_length_m,
    const BehaviorPlannerConfig &config, bool *conflict) {
  *conflict = false;
  if (planning.retained_prefix_points !=
          planning.retained_lateral_states.size() ||
      planning.retained_prefix_points !=
          planning.retained_longitudinal_states.size()) {
    return false;
  }
  if (planning.retained_prefix_points == 0) {
    return planning.frontier.time_from_telemetry_s <= kTolerance;
  }
  if (planning.frontier.time_from_telemetry_s >
          config.maximum_retained_prefix_s + kTolerance ||
      prediction.grid_coverage_s + kTolerance <
          planning.frontier.time_from_telemetry_s) {
    return false;
  }

  const double half_length = 0.5 * config.ego_length_m +
                             config.physical_collision_margin_m;
  const double half_width = 0.5 * config.ego_width_m +
                            config.physical_collision_margin_m;
  for (std::size_t index = 0;
       index < planning.retained_prefix_points; ++index) {
    const LateralPathState &lateral =
        planning.retained_lateral_states[index];
    if (!lateral.valid || !IsFinite(lateral.road_parameter_s) ||
        !IsFinite(lateral.planned_d)) {
      return false;
    }
    const double time_s =
        static_cast<double>(index + 1) * config.simulator_time_step_s;
    const double expected_road_s_m =
        frontier_ego_road_s_m -
        frontier_ego_speed_mps *
            std::max(0.0,
                     planning.frontier.time_from_telemetry_s - time_s);
    const double retained_road_s_m = AlignRoadS(
        lateral.road_parameter_s, expected_road_s_m, track_length_m);
    bool time_evidence_complete = true;
    const std::vector<VehicleEnvelope> envelopes = EnvelopesAtTime(
        prediction, time_s, -1, config, &time_evidence_complete);
    if (!time_evidence_complete) {
      return false;
    }
    for (const VehicleEnvelope &envelope : envelopes) {
      const bool longitudinal_overlap =
          retained_road_s_m + half_length + kTolerance >=
              envelope.occupied_road_s_min_m &&
          retained_road_s_m - half_length <=
              envelope.occupied_road_s_max_m + kTolerance;
      const bool lateral_overlap =
          lateral.planned_d + half_width + kTolerance >=
              envelope.occupied_d_min_m &&
          lateral.planned_d - half_width <=
              envelope.occupied_d_max_m + kTolerance;
      if (longitudinal_overlap && lateral_overlap) {
        *conflict = true;
        return true;
      }
    }
  }
  return true;
}

void CheckSourceLaneFront(
    const FullLaneTrafficPredictionSnapshot &prediction,
    int source_lane, double start_ego_road_s_m,
    double start_ego_speed_mps,
    const BehaviorPlannerConfig &config,
    CoarseAdmissionResult *admission) {
  std::set<int> source_front_vehicle_ids;
  bool start_evidence_complete = true;
  for (const TrafficPredictionTrajectory &trajectory :
       prediction.trajectories) {
    if (trajectory.hypothesis !=
        TrafficPredictionHypothesis::kNominalConstantVelocity) {
      continue;
    }
    VehicleEnvelope envelope;
    if (!EnvelopeForTrajectoryAtTime(
            trajectory, prediction.retained_prefix_duration_s,
            source_lane, config, &envelope)) {
      start_evidence_complete = false;
      continue;
    }
    std::vector<const PredictedTrafficOccupancy *> samples;
    if (!SampleTrajectory(trajectory,
                          prediction.retained_prefix_duration_s,
                          &samples)) {
      start_evidence_complete = false;
      continue;
    }
    bool occupies_source_lane = false;
    for (const PredictedTrafficOccupancy *occupancy : samples) {
      occupies_source_lane =
          occupies_source_lane ||
          CoarseTrafficOccupiesLane(
              *occupancy, source_lane, config);
    }
    if (occupies_source_lane &&
        EnvelopeCenter(envelope) + kTolerance >=
            start_ego_road_s_m &&
        EnvelopeCenter(envelope) <=
            start_ego_road_s_m + config.front_search_distance_m +
                kTolerance) {
      source_front_vehicle_ids.insert(trajectory.vehicle_id);
    }
  }
  if (!start_evidence_complete) {
    AddReason(admission,
              CoarseAdmissionRejectionReason::
                  kPredictionEvidenceIncomplete);
  }

  const std::vector<double> times = GapEvaluationTimes(
      prediction.retained_prefix_duration_s, config);
  const double stop_check_end_s =
      prediction.retained_prefix_duration_s +
      config.coarse_lane_change_duration_s;
  for (double time_s : times) {
    if (time_s > stop_check_end_s + kTolerance) {
      break;
    }
    const double relative_time_s =
        time_s - prediction.retained_prefix_duration_s;
    const double braking_ego_s =
        start_ego_road_s_m +
        DistanceWithAcceleration(start_ego_speed_mps,
                                 config.minimum_longitudinal_acceleration_mps2,
                                 relative_time_s);
    bool evidence_complete = true;
    const std::vector<VehicleEnvelope> envelopes =
        HypothesisEnvelopesAtTime(prediction, time_s, source_lane,
                                  config, &evidence_complete);
    if (!evidence_complete) {
      AddReason(admission,
                CoarseAdmissionRejectionReason::
                    kPredictionEvidenceIncomplete);
      continue;
    }
    for (const VehicleEnvelope &envelope : envelopes) {
      if (source_front_vehicle_ids.find(envelope.id) ==
          source_front_vehicle_ids.end()) {
        continue;
      }
      if (!envelope.safety_admissible) {
        AddReason(admission,
                  CoarseAdmissionRejectionReason::
                      kSourceFrontTrackNotAdmissible);
      }
      const double required_clearance_m =
          0.5 * config.ego_length_m +
          config.standstill_clearance_m +
          config.physical_collision_margin_m;
      const double margin_m =
          envelope.occupied_road_s_min_m - braking_ego_s -
          required_clearance_m;
      if (!admission->has_source_front_margin ||
          margin_m < admission->minimum_source_front_margin_m) {
        admission->has_source_front_margin = true;
        admission->minimum_source_front_margin_m = margin_m;
        admission->source_front_limiting_vehicle_id = envelope.id;
        admission->source_front_limiting_time_s = time_s;
        admission->source_front_limiting_ego_road_s_m = braking_ego_s;
        admission->source_front_limiting_occupied_min_road_s_m =
            envelope.occupied_road_s_min_m;
        admission->source_front_limiting_occupied_max_road_s_m =
            envelope.occupied_road_s_max_m;
        admission->source_front_limiting_min_rate_mps =
            envelope.minimum_road_s_rate_mps;
        admission->source_front_limiting_max_rate_mps =
            envelope.maximum_road_s_rate_mps;
        admission->source_front_limiting_uncertainty_m =
            envelope.maximum_longitudinal_uncertainty_m;
        admission->source_front_limiting_hypothesis =
            envelope.hypothesis;
      }
      if (margin_m < -kTolerance) {
        if (!admission->source_front_risk_observed) {
          admission->source_front_risk_observed = true;
          admission->first_source_front_risk_time_s = time_s;
          admission->first_source_front_risk_vehicle_id = envelope.id;
          admission->first_source_front_risk_margin_m = margin_m;
          admission->first_source_front_risk_hypothesis =
              envelope.hypothesis;
        }
        AddReason(admission,
                  CoarseAdmissionRejectionReason::kSourceLaneFrontRisk);
      }
    }
  }
}

struct CoarseReachableState {
  double road_s_m = 0.0;
  double speed_mps = 0.0;
};

CoarseReachableState PropagateCoarseState(
    const CoarseReachableState &state, double acceleration_mps2,
    double duration_s, const BehaviorPlannerConfig &config) {
  CoarseReachableState result;
  if (acceleration_mps2 >= 0.0) {
    result.road_s_m =
        state.road_s_m + DistanceWithAccelerationAndSpeedCap(
                               state.speed_mps, acceleration_mps2,
                               duration_s, config.target_speed_mps);
  } else {
    result.road_s_m =
        state.road_s_m + DistanceWithAcceleration(
                               state.speed_mps, acceleration_mps2,
                               duration_s);
  }
  result.speed_mps = Clamp(
      state.speed_mps + acceleration_mps2 * duration_s,
      0.0, config.target_speed_mps);
  return result;
}

std::vector<CoarseReachableState> PruneCoarseStates(
    const std::vector<CoarseReachableState> &states,
    const BehaviorPlannerConfig &config) {
  std::map<std::pair<long long, long long>, CoarseReachableState> bins;
  for (const CoarseReachableState &state : states) {
    const long long position_bin = static_cast<long long>(std::llround(
        state.road_s_m / config.coarse_position_resolution_m));
    const long long speed_bin = static_cast<long long>(std::llround(
        state.speed_mps / config.coarse_speed_resolution_mps));
    const std::pair<long long, long long> key(position_bin, speed_bin);
    if (bins.find(key) == bins.end()) {
      bins[key] = state;
    }
  }

  std::vector<CoarseReachableState> result;
  if (bins.size() <= config.maximum_coarse_reachable_states) {
    result.reserve(bins.size());
    for (const std::pair<const std::pair<long long, long long>,
                         CoarseReachableState> &entry : bins) {
      result.push_back(entry.second);
    }
    return result;
  }

  result.reserve(config.maximum_coarse_reachable_states);
  std::size_t source_index = 0;
  std::size_t selected_index = 0;
  for (const std::pair<const std::pair<long long, long long>,
                       CoarseReachableState> &entry : bins) {
    const std::size_t desired_source_index =
        config.maximum_coarse_reachable_states == 1
            ? 0
            : selected_index * (bins.size() - 1) /
                  (config.maximum_coarse_reachable_states - 1);
    if (source_index == desired_source_index &&
        selected_index < config.maximum_coarse_reachable_states) {
      result.push_back(entry.second);
      ++selected_index;
    }
    ++source_index;
  }
  return result;
}

std::vector<CoarseReachableState> PropagateCoarseStates(
    const std::vector<CoarseReachableState> &states,
    double duration_s, const BehaviorPlannerConfig &config) {
  std::vector<CoarseReachableState> propagated;
  propagated.reserve(
      states.size() * config.coarse_acceleration_samples);
  for (const CoarseReachableState &state : states) {
    for (std::size_t index = 0;
         index < config.coarse_acceleration_samples; ++index) {
      const double ratio =
          config.coarse_acceleration_samples == 1
              ? 0.0
              : static_cast<double>(index) /
                    static_cast<double>(
                        config.coarse_acceleration_samples - 1);
      const double acceleration_mps2 =
          config.minimum_longitudinal_acceleration_mps2 +
          ratio * (config.maximum_longitudinal_acceleration_mps2 -
                   config.minimum_longitudinal_acceleration_mps2);
      propagated.push_back(PropagateCoarseState(
          state, acceleration_mps2, duration_s, config));
    }
  }
  return PruneCoarseStates(propagated, config);
}

struct CoarseIntruderConstraintResult {
  bool safe = true;
  int blocking_vehicle_id = 0;
  TrafficPredictionHypothesis blocking_hypothesis =
      TrafficPredictionHypothesis::kNominalConstantVelocity;
  double minimum_margin_m = std::numeric_limits<double>::infinity();
};

CoarseIntruderConstraintResult CheckUnexpectedTargetTraffic(
    const BehaviorCandidate &candidate,
    const CoarseReachableState &state,
    const std::vector<VehicleEnvelope> &envelopes,
    const BehaviorPlannerConfig &config) {
  CoarseIntruderConstraintResult result;
  const double half_ego_length_m = 0.5 * config.ego_length_m;
  const double clearance_m = config.standstill_clearance_m +
                             config.physical_collision_margin_m;
  for (const VehicleEnvelope &envelope : envelopes) {
    if ((candidate.gap.has_front_vehicle &&
         envelope.id == candidate.gap.front_vehicle_id) ||
        (candidate.gap.has_rear_vehicle &&
         envelope.id == candidate.gap.rear_vehicle_id)) {
      continue;
    }

    double margin_m = 0.0;
    if (EnvelopeCenter(envelope) + kTolerance >= state.road_s_m) {
      margin_m = envelope.occupied_road_s_min_m - state.road_s_m -
                 half_ego_length_m - clearance_m -
                 state.speed_mps * config.target_front_time_headway_s;
    } else {
      const double rear_clearance_m =
          state.road_s_m - envelope.occupied_road_s_max_m -
          half_ego_length_m - clearance_m;
      const double rear_headway_margin_m =
          rear_clearance_m -
          std::max(0.0, envelope.maximum_road_s_rate_mps) *
              config.target_rear_time_headway_s;
      const double closing_speed_mps =
          std::max(0.0, envelope.maximum_road_s_rate_mps -
                            state.speed_mps);
      const double rear_ttc_margin_m =
          rear_clearance_m -
          closing_speed_mps * config.minimum_rear_ttc_s;
      margin_m = std::min(rear_headway_margin_m,
                          rear_ttc_margin_m);
    }
    if (margin_m < result.minimum_margin_m) {
      result.minimum_margin_m = margin_m;
      result.blocking_vehicle_id = envelope.id;
      result.blocking_hypothesis = envelope.hypothesis;
    }
  }
  result.safe = !IsFinite(result.minimum_margin_m) ||
                result.minimum_margin_m >= -kTolerance;
  return result;
}

void CheckTargetGapKinematics(
    const BehaviorCandidate &candidate, double start_ego_road_s_m,
    double start_ego_speed_mps,
    const FullLaneTrafficPredictionSnapshot &prediction,
    const BehaviorPlannerConfig &config,
    CoarseAdmissionResult *admission) {
  double minimum_front_margin = std::numeric_limits<double>::infinity();
  double minimum_rear_margin = std::numeric_limits<double>::infinity();
  double minimum_rear_ttc = std::numeric_limits<double>::infinity();
  double minimum_combined_margin =
      std::numeric_limits<double>::infinity();
  const double target_use_start_s =
      config.coarse_lane_change_duration_s;
  const double half_ego_length_m = 0.5 * config.ego_length_m;
  const double clearance_m = config.standstill_clearance_m +
                             config.physical_collision_margin_m;

  std::vector<CoarseReachableState> reachable_states(1);
  reachable_states.front().road_s_m = start_ego_road_s_m;
  reachable_states.front().speed_mps = start_ego_speed_mps;
  double previous_relative_time_s = 0.0;

  for (const TrafficGapSample &sample : candidate.traffic_gap.samples) {
    const double relative_time_s =
        sample.prediction_time_s -
        prediction.retained_prefix_duration_s;
    const double propagation_duration_s =
        std::max(0.0, relative_time_s - previous_relative_time_s);
    if (propagation_duration_s > kTolerance) {
      reachable_states = PropagateCoarseStates(
          reachable_states, propagation_duration_s, config);
    }
    previous_relative_time_s = relative_time_s;
    if (relative_time_s + kTolerance < target_use_start_s) {
      continue;
    }

    double best_front_margin =
        std::numeric_limits<double>::infinity();
    double best_rear_margin =
        std::numeric_limits<double>::infinity();
    double best_rear_ttc_margin =
        std::numeric_limits<double>::infinity();
    double best_rear_ttc = std::numeric_limits<double>::infinity();
    double best_combined_margin =
        -std::numeric_limits<double>::infinity();
    if (candidate.gap.has_front_vehicle) {
      best_front_margin = -std::numeric_limits<double>::infinity();
    }
    if (candidate.gap.has_rear_vehicle) {
      best_rear_margin = -std::numeric_limits<double>::infinity();
      best_rear_ttc_margin =
          -std::numeric_limits<double>::infinity();
      best_rear_ttc = -std::numeric_limits<double>::infinity();
    }

    std::vector<CoarseReachableState> feasible_states;
    feasible_states.reserve(reachable_states.size());
    for (const CoarseReachableState &state : reachable_states) {
      double combined_margin = std::numeric_limits<double>::infinity();
      double front_margin = std::numeric_limits<double>::infinity();
      if (candidate.gap.has_front_vehicle) {
        front_margin =
            sample.front_occupied_min_road_s_m - state.road_s_m -
            half_ego_length_m - clearance_m -
            state.speed_mps * config.target_front_time_headway_s;
        best_front_margin = std::max(best_front_margin, front_margin);
        combined_margin = std::min(combined_margin, front_margin);
      }

      double rear_margin = std::numeric_limits<double>::infinity();
      double rear_ttc_margin =
          std::numeric_limits<double>::infinity();
      double rear_ttc = std::numeric_limits<double>::infinity();
      if (candidate.gap.has_rear_vehicle) {
        const double rear_clearance_m =
            state.road_s_m - sample.rear_occupied_max_road_s_m -
            half_ego_length_m - clearance_m;
        rear_margin =
            rear_clearance_m -
            std::max(0.0, sample.rear_road_s_rate_mps) *
                config.target_rear_time_headway_s;
        const double closing_speed_mps =
            std::max(0.0, sample.rear_road_s_rate_mps -
                              state.speed_mps);
        rear_ttc_margin =
            rear_clearance_m -
            closing_speed_mps * config.minimum_rear_ttc_s;
        rear_ttc = closing_speed_mps > kTolerance
                       ? rear_clearance_m / closing_speed_mps
                       : std::numeric_limits<double>::infinity();
        best_rear_margin = std::max(best_rear_margin, rear_margin);
        best_rear_ttc_margin =
            std::max(best_rear_ttc_margin, rear_ttc_margin);
        best_rear_ttc = std::max(best_rear_ttc, rear_ttc);
        combined_margin =
            std::min(combined_margin,
                     std::min(rear_margin, rear_ttc_margin));
      }

      best_combined_margin =
          std::max(best_combined_margin, combined_margin);
      if (front_margin >= -kTolerance &&
          rear_margin >= -kTolerance &&
          rear_ttc_margin >= -kTolerance) {
        feasible_states.push_back(state);
      }
    }

    if (candidate.gap.has_front_vehicle &&
        IsFinite(best_front_margin)) {
      minimum_front_margin =
          std::min(minimum_front_margin, best_front_margin);
    }
    if (candidate.gap.has_rear_vehicle &&
        IsFinite(best_rear_margin)) {
      minimum_rear_margin =
          std::min(minimum_rear_margin, best_rear_margin);
    }
    if (candidate.gap.has_rear_vehicle &&
        IsFinite(best_rear_ttc)) {
      minimum_rear_ttc = std::min(minimum_rear_ttc, best_rear_ttc);
    }
    if (IsFinite(best_combined_margin)) {
      minimum_combined_margin =
          std::min(minimum_combined_margin, best_combined_margin);
    }
    if (sample.available_ego_center_window_m + kTolerance <
        config.minimum_gap_center_window_m) {
      AddReason(admission,
                CoarseAdmissionRejectionReason::kGapTooSmall);
    }

    if (feasible_states.empty()) {
      if (candidate.gap.has_front_vehicle &&
          best_front_margin < -kTolerance) {
        AddReason(admission,
                  CoarseAdmissionRejectionReason::
                      kInsufficientFrontHeadway);
      }
      if (candidate.gap.has_rear_vehicle &&
          best_rear_margin < -kTolerance) {
        AddReason(admission,
                  CoarseAdmissionRejectionReason::
                      kInsufficientRearHeadway);
      }
      if (candidate.gap.has_rear_vehicle &&
          best_rear_ttc_margin < -kTolerance) {
        AddReason(admission,
                  CoarseAdmissionRejectionReason::kRearTtcTooSmall);
      }
      AddReason(admission,
                CoarseAdmissionRejectionReason::kGapUnreachable);
      if (!admission->target_kinematic_failure_observed) {
        admission->target_kinematic_failure_observed = true;
        admission->first_target_kinematic_failure_time_s =
            sample.prediction_time_s;
        admission->first_target_propagated_state_count =
            reachable_states.size();
        admission->first_target_feasible_state_count = 0;
        admission->first_target_best_front_margin_m =
            candidate.gap.has_front_vehicle ? best_front_margin : 0.0;
        admission->first_target_best_rear_margin_m =
            candidate.gap.has_rear_vehicle ? best_rear_margin : 0.0;
        admission->first_target_best_rear_ttc_margin_m =
            candidate.gap.has_rear_vehicle ? best_rear_ttc_margin : 0.0;
      }
      break;
    }

    bool intruder_evidence_complete = true;
    const std::vector<VehicleEnvelope> target_envelopes =
        HypothesisEnvelopesAtTime(
            prediction, sample.prediction_time_s,
            candidate.target_lane, config,
            &intruder_evidence_complete);
    if (!intruder_evidence_complete) {
      AddReason(admission,
                CoarseAdmissionRejectionReason::
                    kPredictionEvidenceIncomplete);
    }

    struct BlockingEvidence {
      std::size_t blocked_state_count = 0;
      double minimum_margin_m =
          std::numeric_limits<double>::infinity();
    };
    typedef std::pair<int, int> BlockingKey;
    std::map<BlockingKey, BlockingEvidence> blockers;
    std::vector<CoarseReachableState> intruder_feasible_states;
    intruder_feasible_states.reserve(feasible_states.size());
    for (const CoarseReachableState &state : feasible_states) {
      const CoarseIntruderConstraintResult intruder =
          CheckUnexpectedTargetTraffic(
              candidate, state, target_envelopes, config);
      if (intruder.safe) {
        intruder_feasible_states.push_back(state);
        continue;
      }
      const BlockingKey key(
          intruder.blocking_vehicle_id,
          static_cast<int>(intruder.blocking_hypothesis));
      BlockingEvidence &evidence = blockers[key];
      ++evidence.blocked_state_count;
      evidence.minimum_margin_m =
          std::min(evidence.minimum_margin_m,
                   intruder.minimum_margin_m);
    }

    if (intruder_feasible_states.empty()) {
      AddReason(admission,
                CoarseAdmissionRejectionReason::
                    kMergeCorridorBlocked);
      if (!admission->merge_corridor_blocked_observed) {
        admission->merge_corridor_blocked_observed = true;
        admission->first_merge_corridor_blocked_time_s =
            sample.prediction_time_s;
        admission->first_merge_corridor_candidate_state_count =
            feasible_states.size();
        admission->first_merge_corridor_feasible_state_count = 0;
        BlockingKey limiting_key(0, static_cast<int>(
            TrafficPredictionHypothesis::kNominalConstantVelocity));
        BlockingEvidence limiting_evidence;
        for (const std::pair<const BlockingKey, BlockingEvidence> &entry :
             blockers) {
          if (entry.second.blocked_state_count >
                  limiting_evidence.blocked_state_count ||
              (entry.second.blocked_state_count ==
                   limiting_evidence.blocked_state_count &&
               entry.second.minimum_margin_m <
                   limiting_evidence.minimum_margin_m)) {
            limiting_key = entry.first;
            limiting_evidence = entry.second;
          }
        }
        admission->first_merge_corridor_blocking_vehicle_id =
            limiting_key.first;
        admission->first_merge_corridor_blocking_hypothesis =
            static_cast<TrafficPredictionHypothesis>(
                limiting_key.second);
        admission->first_merge_corridor_blocking_margin_m =
            IsFinite(limiting_evidence.minimum_margin_m)
                ? limiting_evidence.minimum_margin_m
                : 0.0;
      }
      break;
    }
    reachable_states =
        PruneCoarseStates(intruder_feasible_states, config);
  }

  if (IsFinite(minimum_combined_margin)) {
    admission->reachable_center_overlap_m =
        minimum_combined_margin;
  }
  if (IsFinite(minimum_front_margin)) {
    admission->has_front_margin = true;
    admission->minimum_front_margin_m = minimum_front_margin;
  }
  if (IsFinite(minimum_rear_margin)) {
    admission->has_rear_margin = true;
    admission->minimum_rear_margin_m = minimum_rear_margin;
  }
  if (IsFinite(minimum_rear_ttc)) {
    admission->has_rear_ttc = true;
    admission->minimum_rear_ttc_s = minimum_rear_ttc;
  }
}

bool IsStable(const BehaviorCandidate &candidate,
              const BehaviorPlannerConfig &config) {
  return candidate.stable_observations >=
             config.minimum_stable_observations &&
         candidate.stable_duration_s + kTolerance >=
             config.gap_stability_window_s;
}

void EvaluateAdmission(
    const PlanningSnapshot &planning,
    const FullLaneTrafficPredictionSnapshot &prediction,
    double start_ego_road_s_m, double start_ego_speed_mps,
    double track_length_m,
    const BehaviorPlannerConfig &config,
    BehaviorCandidate *candidate) {
  CoarseAdmissionResult &admission = candidate->coarse_admission;
  admission.evaluated = true;
  if (candidate->target_lane < 0 ||
      candidate->target_lane >= config.lane_count ||
      std::abs(candidate->target_lane - candidate->source_lane) != 1) {
    AddReason(&admission,
              CoarseAdmissionRejectionReason::kTargetLaneUnavailable);
  }
  if (!candidate->traffic_gap.topology_consistent) {
    AddReason(&admission,
              CoarseAdmissionRejectionReason::kGapTopologyChanged);
  }
  if (!candidate->traffic_gap.prediction_evidence_complete) {
    AddReason(&admission,
              CoarseAdmissionRejectionReason::
                  kPredictionEvidenceIncomplete);
  }
  if (!IsStable(*candidate, config)) {
    AddReason(&admission,
              CoarseAdmissionRejectionReason::kGapNotStable);
  }
  for (const TrafficGapSample &sample : candidate->traffic_gap.samples) {
    if (candidate->gap.has_front_vehicle &&
        !sample.front_track_admissible) {
      AddReason(&admission,
                CoarseAdmissionRejectionReason::
                    kFrontTrackNotAdmissible);
    }
    if (candidate->gap.has_rear_vehicle &&
        !sample.rear_track_admissible) {
      AddReason(&admission,
                CoarseAdmissionRejectionReason::
                    kRearTrackNotAdmissible);
    }
  }

  bool retained_conflict = false;
  if (!RetainedPrefixEvidence(
          planning, prediction, start_ego_road_s_m,
          start_ego_speed_mps, track_length_m, config,
          &retained_conflict)) {
    AddReason(&admission,
              CoarseAdmissionRejectionReason::
                  kRetainedPrefixEvidenceInvalid);
  } else if (retained_conflict) {
    AddReason(&admission,
              CoarseAdmissionRejectionReason::kRetainedPrefixConflict);
  }

  CheckTargetGapKinematics(*candidate, start_ego_road_s_m,
                           start_ego_speed_mps, prediction, config,
                           &admission);
  CheckSourceLaneFront(prediction, candidate->source_lane,
                       start_ego_road_s_m, start_ego_speed_mps, config,
                       &admission);
  if (!planning.control_lane_cruise_backup_valid) {
    AddReason(&admission,
              CoarseAdmissionRejectionReason::
                  kNoCurrentLaneBrakingBackup);
  }
  if (candidate->estimated_progress_gain_m + kTolerance <
      config.minimum_progress_benefit_m) {
    AddReason(&admission,
              CoarseAdmissionRejectionReason::
                  kInsufficientProgressBenefit);
  }
  const double required_end_time_s =
      prediction.retained_prefix_duration_s +
      config.coarse_lane_change_duration_s +
      config.post_maneuver_observation_s;
  if (config.coarse_lane_change_duration_s +
              config.post_maneuver_observation_s >
          config.planning_horizon_s + kTolerance ||
      prediction.grid_coverage_s + kTolerance < required_end_time_s ||
      candidate->traffic_gap.samples.empty() ||
      candidate->traffic_gap.samples.back().prediction_time_s +
              kTolerance <
          required_end_time_s) {
    AddReason(&admission,
              CoarseAdmissionRejectionReason::
                  kPostObservationInsufficient);
  }
  admission.passed = admission.rejection_reasons.empty();
  candidate->status = admission.passed
                          ? BehaviorCandidateStatus::
                                kCoarseAdmissionPassed
                          : BehaviorCandidateStatus::
                                kCoarseAdmissionRejected;
}

struct CandidateSpec {
  BehaviorType behavior = BehaviorType::kKeepLane;
  int target_lane = -1;
  GapId gap;
  PassingOrder order = PassingOrder::kStayBehindFront;
};

bool SameCandidateSpec(const CandidateSpec &left,
                       const CandidateSpec &right) {
  return left.behavior == right.behavior &&
         left.target_lane == right.target_lane &&
         left.gap == right.gap && left.order == right.order;
}

void AddCandidateSpec(const CandidateSpec &spec,
                      std::vector<CandidateSpec> *specs) {
  for (const CandidateSpec &existing : *specs) {
    if (SameCandidateSpec(existing, spec)) {
      return;
    }
  }
  specs->push_back(spec);
}

bool CandidatePreferred(const BehaviorCandidate &left,
                        const BehaviorCandidate &right) {
  const bool left_passed =
      left.status ==
      BehaviorCandidateStatus::kCoarseAdmissionPassed;
  const bool right_passed =
      right.status ==
      BehaviorCandidateStatus::kCoarseAdmissionPassed;
  if (left_passed != right_passed) {
    return left_passed;
  }
  if (left.estimated_progress_gain_m !=
      right.estimated_progress_gain_m) {
    return left.estimated_progress_gain_m >
           right.estimated_progress_gain_m;
  }
  if (left.uncertainty_cost != right.uncertainty_cost) {
    return left.uncertainty_cost < right.uncertainty_cost;
  }
  if (left.target_lane != right.target_lane) {
    return left.target_lane < right.target_lane;
  }
  if (left.order != right.order) {
    return static_cast<int>(left.order) <
           static_cast<int>(right.order);
  }
  return left.candidate_id < right.candidate_id;
}

void ValidateConfig(const BehaviorPlannerConfig &config) {
  const bool finite =
      IsFinite(config.simulator_time_step_s) &&
      IsFinite(config.gap_evaluation_time_step_s) &&
      IsFinite(config.planning_horizon_s) &&
      IsFinite(config.maximum_retained_prefix_s) &&
      IsFinite(config.coarse_lane_change_duration_s) &&
      IsFinite(config.post_maneuver_observation_s) &&
      IsFinite(config.gap_stability_window_s) &&
      IsFinite(config.target_front_time_headway_s) &&
      IsFinite(config.target_rear_time_headway_s) &&
      IsFinite(config.minimum_rear_ttc_s) &&
      IsFinite(config.standstill_clearance_m) &&
      IsFinite(config.physical_collision_margin_m) &&
      IsFinite(config.minimum_progress_benefit_m) &&
      IsFinite(config.minimum_gap_center_window_m) &&
      IsFinite(config.maximum_longitudinal_acceleration_mps2) &&
      IsFinite(config.minimum_longitudinal_acceleration_mps2) &&
      IsFinite(config.target_speed_mps) &&
      IsFinite(config.rear_search_distance_m) &&
      IsFinite(config.front_search_distance_m) &&
      IsFinite(config.ego_length_m) && IsFinite(config.ego_width_m) &&
      IsFinite(config.lane_width_m) &&
      IsFinite(config.coarse_position_resolution_m) &&
      IsFinite(config.coarse_speed_resolution_mps);
  if (!finite || config.simulator_time_step_s <= 0.0 ||
      config.gap_evaluation_time_step_s <= 0.0 ||
      config.planning_horizon_s <= 0.0 ||
      config.maximum_retained_prefix_s < 0.0 ||
      config.coarse_lane_change_duration_s <= 0.0 ||
      config.post_maneuver_observation_s < 0.0 ||
      config.gap_stability_window_s < 0.0 ||
      config.minimum_stable_observations == 0 ||
      config.target_front_time_headway_s < 0.0 ||
      config.target_rear_time_headway_s < 0.0 ||
      config.minimum_rear_ttc_s < 0.0 ||
      config.standstill_clearance_m < 0.0 ||
      config.physical_collision_margin_m < 0.0 ||
      config.minimum_progress_benefit_m < 0.0 ||
      config.minimum_gap_center_window_m < 0.0 ||
      config.maximum_longitudinal_acceleration_mps2 < 0.0 ||
      config.minimum_longitudinal_acceleration_mps2 >= 0.0 ||
      config.target_speed_mps <= 0.0 ||
      config.rear_search_distance_m <= 0.0 ||
      config.front_search_distance_m <= 0.0 ||
      config.ego_length_m <= 0.0 || config.ego_width_m <= 0.0 ||
      config.lane_width_m <= 0.0 || config.lane_count <= 0 ||
      config.lane_count > 63 || config.maximum_candidates == 0 ||
      config.maximum_gap_samples == 0 ||
      config.coarse_acceleration_samples < 2 ||
      config.maximum_coarse_reachable_states < 2 ||
      config.coarse_position_resolution_m <= 0.0 ||
      config.coarse_speed_resolution_mps <= 0.0) {
    throw std::invalid_argument("invalid behavior planner configuration");
  }
  const double gap_duration_s =
      config.coarse_lane_change_duration_s +
      config.post_maneuver_observation_s;
  const std::size_t required_gap_samples =
      static_cast<std::size_t>(
          std::ceil(gap_duration_s /
                        config.gap_evaluation_time_step_s -
                    kTolerance)) +
      1;
  if (required_gap_samples > config.maximum_gap_samples) {
    throw std::invalid_argument(
        "behavior planner gap sample capacity is insufficient");
  }
}

} // namespace

bool operator==(const GapId &left, const GapId &right) {
  return left.target_lane == right.target_lane &&
         left.has_front_vehicle == right.has_front_vehicle &&
         (!left.has_front_vehicle ||
          left.front_vehicle_id == right.front_vehicle_id) &&
         left.has_rear_vehicle == right.has_rear_vehicle &&
         (!left.has_rear_vehicle ||
          left.rear_vehicle_id == right.rear_vehicle_id);
}

bool operator!=(const GapId &left, const GapId &right) {
  return !(left == right);
}

bool operator<(const GapId &left, const GapId &right) {
  if (left.target_lane != right.target_lane) {
    return left.target_lane < right.target_lane;
  }
  if (left.has_front_vehicle != right.has_front_vehicle) {
    return left.has_front_vehicle < right.has_front_vehicle;
  }
  if (left.has_front_vehicle &&
      left.front_vehicle_id != right.front_vehicle_id) {
    return left.front_vehicle_id < right.front_vehicle_id;
  }
  if (left.has_rear_vehicle != right.has_rear_vehicle) {
    return left.has_rear_vehicle < right.has_rear_vehicle;
  }
  return left.has_rear_vehicle &&
         left.rear_vehicle_id < right.rear_vehicle_id;
}

const char *PassingOrderName(PassingOrder order) {
  switch (order) {
  case PassingOrder::kStayBehindFront:
    return "StayBehindFront";
  case PassingOrder::kMergeAheadOfRear:
    return "MergeAheadOfRear";
  case PassingOrder::kWaitBehindRear:
    return "WaitBehindRear";
  }
  return "Unknown";
}

const char *BehaviorTypeName(BehaviorType behavior) {
  switch (behavior) {
  case BehaviorType::kKeepLane:
    return "KeepLane";
  case BehaviorType::kChangeLeft:
    return "ChangeLeft";
  case BehaviorType::kChangeRight:
    return "ChangeRight";
  }
  return "Unknown";
}

const char *CoarseAdmissionRejectionReasonName(
    CoarseAdmissionRejectionReason reason) {
  switch (reason) {
  case CoarseAdmissionRejectionReason::kTargetLaneUnavailable:
    return "TargetLaneUnavailable";
  case CoarseAdmissionRejectionReason::kGapTopologyChanged:
    return "GapTopologyChanged";
  case CoarseAdmissionRejectionReason::kMergeCorridorBlocked:
    return "MergeCorridorBlocked";
  case CoarseAdmissionRejectionReason::kPredictionEvidenceIncomplete:
    return "PredictionEvidenceIncomplete";
  case CoarseAdmissionRejectionReason::kGapNotStable:
    return "GapNotStable";
  case CoarseAdmissionRejectionReason::kFrontTrackNotAdmissible:
    return "FrontTrackNotAdmissible";
  case CoarseAdmissionRejectionReason::kRearTrackNotAdmissible:
    return "RearTrackNotAdmissible";
  case CoarseAdmissionRejectionReason::kRetainedPrefixEvidenceInvalid:
    return "RetainedPrefixEvidenceInvalid";
  case CoarseAdmissionRejectionReason::kRetainedPrefixConflict:
    return "RetainedPrefixConflict";
  case CoarseAdmissionRejectionReason::kInsufficientFrontHeadway:
    return "InsufficientFrontHeadway";
  case CoarseAdmissionRejectionReason::kInsufficientRearHeadway:
    return "InsufficientRearHeadway";
  case CoarseAdmissionRejectionReason::kRearTtcTooSmall:
    return "RearTtcTooSmall";
  case CoarseAdmissionRejectionReason::kSourceFrontTrackNotAdmissible:
    return "SourceFrontTrackNotAdmissible";
  case CoarseAdmissionRejectionReason::kSourceLaneFrontRisk:
    return "SourceLaneFrontRisk";
  case CoarseAdmissionRejectionReason::kNoCurrentLaneBrakingBackup:
    return "NoCurrentLaneBrakingBackup";
  case CoarseAdmissionRejectionReason::kInsufficientProgressBenefit:
    return "InsufficientProgressBenefit";
  case CoarseAdmissionRejectionReason::kGapUnreachable:
    return "GapUnreachable";
  case CoarseAdmissionRejectionReason::kGapTooSmall:
    return "GapTooSmall";
  case CoarseAdmissionRejectionReason::kPostObservationInsufficient:
    return "PostObservationInsufficient";
  }
  return "Unknown";
}

const char *BehaviorCandidateStatusName(
    BehaviorCandidateStatus status) {
  switch (status) {
  case BehaviorCandidateStatus::kKeepLaneBaseline:
    return "KeepLaneBaseline";
  case BehaviorCandidateStatus::kCoarseAdmissionRejected:
    return "CoarseAdmissionRejected";
  case BehaviorCandidateStatus::kCoarseAdmissionPassed:
    return "CoarseAdmissionPassed";
  }
  return "Unknown";
}

const char *GapTopologyFailureKindName(
    GapTopologyFailureKind kind) {
  switch (kind) {
  case GapTopologyFailureKind::kNone:
    return "None";
  case GapTopologyFailureKind::kExpectedFrontMissing:
    return "ExpectedFrontMissing";
  case GapTopologyFailureKind::kExpectedRearMissing:
    return "ExpectedRearMissing";
  case GapTopologyFailureKind::kExpectedFrontAndRearMissing:
    return "ExpectedFrontAndRearMissing";
  case GapTopologyFailureKind::kBoundariesNotAdjacent:
    return "BoundariesNotAdjacent";
  case GapTopologyFailureKind::kUnexpectedRearBoundary:
    return "UnexpectedRearBoundary";
  case GapTopologyFailureKind::kUnexpectedFrontBoundary:
    return "UnexpectedFrontBoundary";
  case GapTopologyFailureKind::kUnexpectedOccupiedGap:
    return "UnexpectedOccupiedGap";
  }
  return "Unknown";
}

bool HasCoarseAdmissionRejectionReason(
    const CoarseAdmissionResult &result,
    CoarseAdmissionRejectionReason reason) {
  return std::find(result.rejection_reasons.begin(),
                   result.rejection_reasons.end(),
                   reason) != result.rejection_reasons.end();
}

BehaviorPlanner::BehaviorPlanner(
    const BehaviorPlannerConfig &config)
    : config_(config) {
  ValidateConfig(config_);
}

BehaviorPlanningUpdate BehaviorPlanner::Evaluate(
    const PlanningSnapshot &planning,
    const TrafficTrackingSnapshot &tracking,
    const FullLaneTrafficPredictionSnapshot &prediction,
    double track_length_m, const BehaviorPlannerState &state) const {
  if (planning.cycle == 0 || planning.cycle != tracking.cycle ||
      tracking.cycle != prediction.cycle ||
      !IsFinite(track_length_m) || track_length_m <= 0.0 ||
      !IsFinite(planning.frontier.road_s_unwrapped_m) ||
      !IsFinite(planning.frontier.longitudinal.v) ||
      !IsFinite(planning.frontier.time_from_telemetry_s) ||
      planning.frontier.time_from_telemetry_s < 0.0 ||
      !IsFinite(prediction.retained_prefix_duration_s) ||
      std::fabs(prediction.retained_prefix_duration_s -
                planning.frontier.time_from_telemetry_s) > kTolerance ||
      planning.target_lane < 0 ||
      planning.target_lane >= config_.lane_count) {
    throw std::invalid_argument("invalid behavior planner input");
  }

  BehaviorPlanningUpdate update;
  update.next_state.cycle = planning.cycle;
  update.snapshot.cycle = planning.cycle;
  update.snapshot.source_lane = planning.target_lane;
  const int source_lane = planning.target_lane;
  const double start_ego_speed_mps =
      std::max(0.0, planning.frontier.longitudinal.v);
  const double expected_frontier_s =
      tracking.ego_road_s_unwrapped_m +
      start_ego_speed_mps * planning.frontier.time_from_telemetry_s;
  const double start_ego_road_s_m = AlignRoadS(
      planning.frontier.road_s_unwrapped_m, expected_frontier_s,
      track_length_m);

  bool frontier_evidence_complete = true;
  const std::vector<VehicleEnvelope> source_envelopes =
      FilterSearchWindow(
          EnvelopesAtTime(prediction,
                          prediction.retained_prefix_duration_s,
                          source_lane, config_,
                          &frontier_evidence_complete),
          start_ego_road_s_m, config_);
  const GapId source_gap =
      GapAroundPosition(source_envelopes, source_lane,
                        start_ego_road_s_m);
  const TrafficGap source_traffic_gap =
      EvaluateGap(source_gap, start_ego_road_s_m,
                  start_ego_speed_mps, prediction, config_);

  std::vector<CandidateSpec> specs;
  CandidateSpec keep;
  keep.behavior = BehaviorType::kKeepLane;
  keep.target_lane = source_lane;
  keep.gap = source_gap;
  keep.order = PassingOrder::kStayBehindFront;
  AddCandidateSpec(keep, &specs);

  const int adjacent_lanes[] = {source_lane - 1, source_lane + 1};
  for (int target_lane : adjacent_lanes) {
    if (target_lane < 0 || target_lane >= config_.lane_count) {
      continue;
    }
    bool target_evidence_complete = true;
    const std::vector<VehicleEnvelope> target_envelopes =
        FilterSearchWindow(
            EnvelopesAtTime(prediction,
                            prediction.retained_prefix_duration_s,
                            target_lane, config_,
                            &target_evidence_complete),
            start_ego_road_s_m, config_);
    CandidateSpec immediate;
    immediate.behavior = target_lane < source_lane
                             ? BehaviorType::kChangeLeft
                             : BehaviorType::kChangeRight;
    immediate.target_lane = target_lane;
    immediate.gap = GapAroundPosition(target_envelopes, target_lane,
                                      start_ego_road_s_m);
    immediate.order = immediate.gap.has_rear_vehicle
                          ? PassingOrder::kMergeAheadOfRear
                          : PassingOrder::kStayBehindFront;
    AddCandidateSpec(immediate, &specs);

    CandidateSpec wait = immediate;
    if (GapBehindNearestRear(target_envelopes, target_lane,
                             start_ego_road_s_m, &wait.gap)) {
      wait.order = PassingOrder::kWaitBehindRear;
      AddCandidateSpec(wait, &specs);
    }
  }

  std::vector<BehaviorCandidate> candidates;
  candidates.reserve(specs.size());
  for (const CandidateSpec &spec : specs) {
    BehaviorCandidate candidate;
    candidate.behavior = spec.behavior;
    candidate.source_lane = source_lane;
    candidate.target_lane = spec.target_lane;
    candidate.gap = spec.gap;
    candidate.order = spec.order;
    candidate.candidate_id =
        HashCandidate(spec.behavior, spec.order, spec.gap);
    candidate.traffic_gap =
        spec.behavior == BehaviorType::kKeepLane
            ? source_traffic_gap
            : EvaluateGap(spec.gap, start_ego_road_s_m,
                          start_ego_speed_mps, prediction, config_);
    candidates.push_back(candidate);
  }

  const bool prior_state_contiguous =
      tracking.reset_reason == TrafficTrackerResetReason::kNone &&
      state.cycle != 0 && state.cycle + 1 == planning.cycle;
  std::set<GapId> recorded_gaps;
  for (const BehaviorCandidate &candidate : candidates) {
    if (candidate.behavior == BehaviorType::kKeepLane ||
        !candidate.traffic_gap.current_observation_valid ||
        recorded_gaps.find(candidate.gap) != recorded_gaps.end()) {
      continue;
    }
    GapStabilityRecord record;
    record.gap = candidate.gap;
    record.consecutive_observations = 1;
    record.observed_duration_s = 0.0;
    if (prior_state_contiguous) {
      const GapStabilityRecord *previous =
          FindStability(state, candidate.gap);
      if (previous != nullptr) {
        record.consecutive_observations =
            previous->consecutive_observations + 1;
        record.observed_duration_s =
            previous->observed_duration_s +
            config_.simulator_time_step_s;
      }
    }
    update.next_state.gap_stability.push_back(record);
    recorded_gaps.insert(candidate.gap);
  }

  const double benefit_horizon_s =
      std::max(prediction.planning_horizon_s, kTolerance);
  const double source_progress_m = EstimatedLaneProgress(
      source_traffic_gap.id, start_ego_road_s_m, start_ego_speed_mps,
      prediction, config_);
  for (BehaviorCandidate &candidate : candidates) {
    const GapStabilityRecord *stability =
        FindStability(update.next_state, candidate.gap);
    if (stability != nullptr) {
      candidate.stable_observations =
          stability->consecutive_observations;
      candidate.stable_duration_s = stability->observed_duration_s;
    }
    candidate.estimated_progress_m = EstimatedLaneProgress(
        candidate.traffic_gap.id, start_ego_road_s_m,
        start_ego_speed_mps, prediction, config_);
    candidate.estimated_progress_gain_m =
        candidate.behavior == BehaviorType::kKeepLane
            ? 0.0
            : candidate.estimated_progress_m - source_progress_m;
    candidate.estimated_speed_gain_mps =
        candidate.estimated_progress_gain_m / benefit_horizon_s;
    candidate.uncertainty_cost =
        candidate.traffic_gap.maximum_boundary_uncertainty_m;
    if (candidate.behavior == BehaviorType::kKeepLane) {
      candidate.status =
          BehaviorCandidateStatus::kKeepLaneBaseline;
      continue;
    }
    EvaluateAdmission(planning, prediction, start_ego_road_s_m,
                      start_ego_speed_mps, track_length_m, config_,
                      &candidate);
  }

  BehaviorCandidate keep_candidate = candidates.front();
  std::vector<BehaviorCandidate> lane_change_candidates(
      candidates.begin() + 1, candidates.end());
  std::sort(lane_change_candidates.begin(),
            lane_change_candidates.end(), CandidatePreferred);
  update.snapshot.candidates.push_back(keep_candidate);
  const std::size_t lane_change_capacity =
      config_.maximum_candidates > 0
          ? config_.maximum_candidates - 1
          : 0;
  if (lane_change_candidates.size() > lane_change_capacity) {
    lane_change_candidates.resize(lane_change_capacity);
  }
  update.snapshot.candidates.insert(
      update.snapshot.candidates.end(), lane_change_candidates.begin(),
      lane_change_candidates.end());
  update.snapshot.generated_candidate_count =
      update.snapshot.candidates.size();

  std::vector<GapStabilityRecord> retained_stability;
  for (const GapStabilityRecord &record :
       update.next_state.gap_stability) {
    bool retained = false;
    for (const BehaviorCandidate &candidate :
         update.snapshot.candidates) {
      if (candidate.behavior != BehaviorType::kKeepLane &&
          candidate.gap == record.gap) {
        retained = true;
        break;
      }
    }
    if (retained) {
      retained_stability.push_back(record);
    }
  }
  update.next_state.gap_stability.swap(retained_stability);
  update.snapshot.stable_gap_count = 0;
  update.snapshot.coarse_admitted_candidate_count = 0;
  for (const BehaviorCandidate &candidate :
       update.snapshot.candidates) {
    if (candidate.status ==
        BehaviorCandidateStatus::kCoarseAdmissionPassed) {
      ++update.snapshot.coarse_admitted_candidate_count;
    }
  }
  for (const GapStabilityRecord &record :
       update.next_state.gap_stability) {
    if (record.consecutive_observations >=
            config_.minimum_stable_observations &&
        record.observed_duration_s + kTolerance >=
            config_.gap_stability_window_s) {
      ++update.snapshot.stable_gap_count;
    }
  }

  update.snapshot.has_best_coarse_candidate = true;
  update.snapshot.best_coarse_candidate_id = keep_candidate.candidate_id;
  for (const BehaviorCandidate &candidate :
       update.snapshot.candidates) {
    if (candidate.status ==
        BehaviorCandidateStatus::kCoarseAdmissionPassed) {
      update.snapshot.best_coarse_candidate_id = candidate.candidate_id;
      break;
    }
  }
  return update;
}
