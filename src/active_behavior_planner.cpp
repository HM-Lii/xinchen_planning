#include "active_behavior_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "longitudinal_qp.h"
#include "map.h"
#include "planning_snapshot.h"
#include "trajectory_assembler.h"

namespace {

constexpr double kTolerance = 1e-6;

bool Finite(double value) { return std::isfinite(value); }

double Clamp(double value, double minimum, double maximum) {
  return std::max(minimum, std::min(value, maximum));
}

void ValidateConfig(const ActiveBehaviorPlannerConfig &config) {
  if (!Finite(config.output_time_step_s) ||
      config.output_time_step_s <= 0.0 || config.output_points == 0 ||
      config.full_trajectory_sample_count < config.output_points ||
      !Finite(config.target_speed_mps) || config.target_speed_mps <= 0.0 ||
      config.minimum_valid_proposal_cycles == 0 ||
      !Finite(config.minimum_discretionary_acceleration_mps2) ||
      config.minimum_discretionary_acceleration_mps2 <
          config.longitudinal_qp.minimum_acceleration_mps2 ||
      config.minimum_discretionary_acceleration_mps2 > 0.0 ||
      config.target_stable_cycles == 0 ||
      !Finite(config.target_center_tolerance_m) ||
      config.target_center_tolerance_m <= 0.0 ||
      !Finite(config.target_lateral_rate_tolerance_mps) ||
      config.target_lateral_rate_tolerance_mps < 0.0 ||
      !Finite(config.dynamics_tightening_factor) ||
      config.dynamics_tightening_factor <= 0.0 ||
      config.dynamics_tightening_factor >= 1.0 ||
      config.longitudinal_qp.horizon_steps !=
          config.st_corridor.qp_horizon_steps ||
      std::fabs(config.longitudinal_qp.time_step_seconds -
                config.st_corridor.qp_time_step_s) > 1e-12 ||
      std::fabs(config.output_time_step_s -
                config.trajectory_validator.time_step_s) > 1e-12) {
    throw std::invalid_argument("invalid active behavior configuration");
  }
}

const BehaviorCandidate *FindBehaviorCandidate(
    const BehaviorPlanningSnapshot &snapshot, std::uint64_t candidate_id) {
  for (const BehaviorCandidate &candidate : snapshot.candidates) {
    if (candidate.candidate_id == candidate_id) {
      return &candidate;
    }
  }
  return nullptr;
}

STCandidateEvaluation *FindMutableSTCandidate(
    STCandidateBatchSnapshot *snapshot, std::uint64_t candidate_id) {
  for (STCandidateEvaluation &candidate : snapshot->candidates) {
    if (candidate.candidate_id == candidate_id) {
      return &candidate;
    }
  }
  return nullptr;
}

bool SameProposal(std::uint64_t candidate_id, int target_lane,
                  const GapId &gap, std::uint64_t state_candidate_id,
                  int state_target_lane, const GapId &state_gap) {
  return candidate_id == state_candidate_id &&
         target_lane == state_target_lane && gap == state_gap;
}

bool DynamicsOnlyRejection(const ValidationResult &validation) {
  if (validation.valid || validation.violations.empty()) {
    return false;
  }
  for (const TrajectoryViolation &violation : validation.violations) {
    if (violation.type != ViolationType::kSpeed &&
        violation.type != ViolationType::kAcceleration &&
        violation.type != ViolationType::kJerk) {
      return false;
    }
  }
  return true;
}

std::string ValidationDetail(const ValidationResult &validation) {
  if (validation.valid) {
    return "valid";
  }
  if (validation.violations.empty()) {
    return "validator rejected without violation evidence";
  }
  const TrajectoryViolation &first = validation.violations.front();
  return std::string(ViolationTypeName(first.type)) + " at t=" +
         std::to_string(first.time_from_telemetry_s) + " (" +
         first.detail + ")";
}

struct BuiltBehaviorTrajectory {
  FullTrajectory full;
  PlannerOutput output;
  std::vector<LongitudinalState> output_longitudinal;
  std::vector<LateralPathState> output_lateral;
};

BuiltBehaviorTrajectory BuildBehaviorTrajectory(
    const PlanningSnapshot &planning,
    const SpatialPathCandidate &spatial_path,
    const LongitudinalQpResult &qp,
    const ActiveBehaviorPlannerConfig &config,
    std::uint64_t transition_id, double absolute_progress_offset_m) {
  if (!qp.success || !qp.hard_safe ||
      spatial_path.status != SpatialPathCandidateStatus::kPrecheckPassed ||
      spatial_path.geometry.samples.size() < 2) {
    throw std::invalid_argument("invalid P2.5 trajectory inputs");
  }

  TrajectoryCanonicalizationConfig assembly_config;
  assembly_config.sample_time_step_s = config.output_time_step_s;
  assembly_config.sample_count = config.full_trajectory_sample_count;
  assembly_config.minimum_acceleration_mps2 =
      config.longitudinal_qp.minimum_acceleration_mps2;
  assembly_config.maximum_acceleration_mps2 =
      config.longitudinal_qp.maximum_acceleration_mps2;
  assembly_config.maximum_jerk_mps3 = config.longitudinal_qp.maximum_jerk_mps3;
  assembly_config.maximum_progress_m = spatial_path.geometry.path_extent_m;
  assembly_config.invalid_trajectory_message =
      "candidate QP trajectory is outside P2.5 geometry";
  const std::vector<LongitudinalState> sampled =
      SampleCanonicalTrajectory(qp, assembly_config);

  BuiltBehaviorTrajectory result;
  result.full = InitializeFullTrajectory(
      planning, config.output_time_step_s,
      static_cast<double>(config.longitudinal_qp.horizon_steps) *
          config.longitudinal_qp.time_step_seconds,
      sampled.size());
  ValidateRetainedTrajectoryPrefix(planning);

  double retained_frame_offset_x = 0.0;
  double retained_frame_offset_y = 0.0;
  if (planning.retained_prefix_points > 0) {
    const LateralPathState &retained_anchor =
        planning.retained_lateral_states.back();
    retained_frame_offset_x =
        planning.input.previous_path_x[planning.retained_prefix_points - 1] -
        retained_anchor.expected_x;
    retained_frame_offset_y =
        planning.input.previous_path_y[planning.retained_prefix_points - 1] -
        retained_anchor.expected_y;
  }

  double continuation_frame_offset_x = 0.0;
  double continuation_frame_offset_y = 0.0;
  const bool continuing_committed_path =
      planning.historical_plan_aligned && planning.frontier.lateral.valid &&
      planning.frontier.lateral.transition_id == transition_id;
  if (continuing_committed_path &&
      planning.retained_prefix_points > 0 &&
      !spatial_path.geometry.samples.empty()) {
    // Simulator/controller serialization can translate the retained prefix by
    // sub-millimeter amounts. Keep the complete continuation in that same
    // Cartesian frame; snapping immediately back to nominal map geometry
    // turns the tiny position residual into a controller-rate jerk impulse.
    continuation_frame_offset_x =
        planning.input.previous_path_x[planning.retained_prefix_points - 1] -
        spatial_path.geometry.samples.front().x_m;
    continuation_frame_offset_y =
        planning.input.previous_path_y[planning.retained_prefix_points - 1] -
        spatial_path.geometry.samples.front().y_m;
  }

  // Persist the same rigid retained-frame normalization used by the hard
  // validator so the next cycle keeps a single Cartesian serialization frame.
  AppendRetainedTrajectoryPrefix(planning, config.output_time_step_s,
                                 retained_frame_offset_x,
                                 retained_frame_offset_y, true, &result.full);

  for (std::size_t index = 0; index < sampled.size(); ++index) {
    const SpatialPathGeometrySample geometry =
        SampleSpatialPathAtProgress(spatial_path.geometry,
                                    sampled[index].s);
    TrajectoryPoint point;
    point.time_from_telemetry_s =
        planning.frontier.time_from_telemetry_s +
        static_cast<double>(index + 1) * config.output_time_step_s;
    point.x = geometry.x_m + continuation_frame_offset_x;
    point.y = geometry.y_m + continuation_frame_offset_y;
    point.longitudinal = sampled[index];
    point.lateral.valid = true;
    point.lateral.transition_id = transition_id;
    point.lateral.correction_progress_m =
        absolute_progress_offset_m + geometry.path_progress_m;
    point.lateral.residual_correction_progress_m = 0.0;
    point.lateral.road_parameter_s = geometry.road_s_unwrapped_m;
    point.lateral.planned_d = geometry.d_m;
    point.lateral.expected_x = point.x;
    point.lateral.expected_y = point.y;
    point.lateral.exact_tangent_valid = true;
    point.lateral.tangent_x = geometry.tangent_x;
    point.lateral.tangent_y = geometry.tangent_y;
    point.lateral.exact_curvature_valid = true;
    point.lateral.curvature_x_per_m = geometry.curvature_x_per_m;
    point.lateral.curvature_y_per_m = geometry.curvature_y_per_m;
    result.full.points.push_back(point);
  }
  TrajectoryOutputSlice output =
      SliceTrajectoryOutput(result.full, config.output_points,
                            "P2.5 trajectory cannot satisfy output contract");
  result.output = std::move(output.output);
  result.output_longitudinal = std::move(output.longitudinal);
  result.output_lateral = std::move(output.lateral);
  return result;
}

double ComputeCandidateCost(const FinalBehaviorCandidate &candidate) {
  // Hard margins are considered by CandidatePreferred first. This scalar is
  // the deterministic secondary objective and remains diagnostic-friendly.
  const double comfort = 0.20 * candidate.maximum_acceleration_mps2 +
                         0.05 * candidate.maximum_jerk_mps3;
  return -candidate.terminal_progress_m -
         candidate.progress_benefit_m + comfort;
}

bool CandidatePreferred(const FinalBehaviorCandidate &left,
                        const FinalBehaviorCandidate &right) {
  if (left.status != right.status) {
    return left.status == FinalBehaviorCandidateStatus::kValid;
  }
  if (std::fabs(left.minimum_physical_margin_m -
                right.minimum_physical_margin_m) > 1e-9) {
    return left.minimum_physical_margin_m >
           right.minimum_physical_margin_m;
  }
  if (std::fabs(left.minimum_operational_margin_m -
                right.minimum_operational_margin_m) > 1e-9) {
    return left.minimum_operational_margin_m >
           right.minimum_operational_margin_m;
  }
  if (std::fabs(left.minimum_road_margin_m -
                right.minimum_road_margin_m) > 1e-9) {
    return left.minimum_road_margin_m > right.minimum_road_margin_m;
  }
  if (std::fabs(left.cost - right.cost) > 1e-9) {
    return left.cost < right.cost;
  }
  if (left.target_lane != right.target_lane) {
    return left.target_lane < right.target_lane;
  }
  return left.candidate_id < right.candidate_id;
}

double InvertGeometryProgress(const SpatialPathGeometryTable &geometry,
                              double road_s_unwrapped_m) {
  if (geometry.samples.empty() || !Finite(road_s_unwrapped_m)) {
    throw std::invalid_argument("cannot locate committed path progress");
  }
  if (road_s_unwrapped_m <=
      geometry.samples.front().road_s_unwrapped_m + kTolerance) {
    return 0.0;
  }
  if (road_s_unwrapped_m >=
      geometry.samples.back().road_s_unwrapped_m - kTolerance) {
    return geometry.path_extent_m;
  }
  const std::vector<SpatialPathGeometrySample>::const_iterator upper =
      std::lower_bound(
          geometry.samples.begin(), geometry.samples.end(),
          road_s_unwrapped_m,
          [](const SpatialPathGeometrySample &sample, double road_s) {
            return sample.road_s_unwrapped_m < road_s;
          });
  if (upper == geometry.samples.begin()) {
    return upper->path_progress_m;
  }
  const SpatialPathGeometrySample &lower = *(upper - 1);
  const double road_span =
      upper->road_s_unwrapped_m - lower.road_s_unwrapped_m;
  if (road_span <= kTolerance) {
    return lower.path_progress_m;
  }
  const double fraction =
      (road_s_unwrapped_m - lower.road_s_unwrapped_m) / road_span;
  return lower.path_progress_m +
         fraction * (upper->path_progress_m - lower.path_progress_m);
}

SpatialPathCandidate ShiftCommittedPath(
    const SpatialPathCandidate &committed, double offset_progress_m) {
  if (committed.geometry.samples.size() < 2 ||
      offset_progress_m < -kTolerance ||
      offset_progress_m > committed.geometry.path_extent_m + kTolerance) {
    throw std::invalid_argument("invalid committed path shift");
  }
  offset_progress_m = Clamp(offset_progress_m, 0.0,
                            committed.geometry.path_extent_m);
  const SpatialPathGeometrySample start =
      SampleSpatialPathAtProgress(committed.geometry, offset_progress_m);

  SpatialPathCandidate shifted = committed;
  shifted.start_road_s_unwrapped_m = start.road_s_unwrapped_m;
  shifted.start_d_m = start.d_m;
  shifted.start_d_first_derivative = start.d_first_derivative;
  shifted.start_d_second_derivative_per_m =
      start.d_second_derivative_per_m;
  shifted.start_d_third_derivative_per_m2 =
      start.d_third_derivative_per_m2;
  shifted.geometry.samples.clear();
  SpatialPathGeometrySample shifted_start = start;
  shifted_start.path_progress_m = 0.0;
  shifted_start.construction_progress_m = 0.0;
  shifted.geometry.samples.push_back(shifted_start);
  for (const SpatialPathGeometrySample &sample : committed.geometry.samples) {
    if (sample.path_progress_m <= offset_progress_m + kTolerance) {
      continue;
    }
    SpatialPathGeometrySample value = sample;
    value.path_progress_m -= offset_progress_m;
    value.construction_progress_m =
        std::max(0.0, sample.construction_progress_m -
                          start.construction_progress_m);
    shifted.geometry.samples.push_back(value);
  }
  shifted.geometry.sample_count = shifted.geometry.samples.size();
  shifted.geometry.path_extent_m =
      committed.geometry.path_extent_m - offset_progress_m;
  shifted.geometry.construction_extent_m =
      std::max(0.0, committed.geometry.construction_extent_m -
                        start.construction_progress_m);
  shifted.geometry.transition_completion_path_progress_m = std::max(
      0.0, committed.geometry.transition_completion_path_progress_m -
               offset_progress_m);
  shifted.occupancy.target_lane_coverage_start_path_progress_m = std::max(
      0.0,
      committed.occupancy.target_lane_coverage_start_path_progress_m -
          offset_progress_m);
  shifted.occupancy.source_lane_departure_path_progress_m = std::max(
      0.0, committed.occupancy.source_lane_departure_path_progress_m -
               offset_progress_m);
  shifted.occupancy.lane_change_completion_path_progress_m = std::max(
      0.0, committed.occupancy.lane_change_completion_path_progress_m -
               offset_progress_m);
  shifted.requested_path_extent_m = shifted.geometry.path_extent_m;
  if (shifted.geometry.samples.size() < 2 ||
      shifted.geometry.path_extent_m <= kTolerance) {
    throw std::runtime_error("committed path has insufficient remaining extent");
  }
  return shifted;
}

double EstimateLateralRate(const PlanningSnapshot &planning,
                           double output_time_step_s) {
  const std::size_t count = planning.retained_lateral_states.size();
  if (count >= 2) {
    const LateralPathState &last =
        planning.retained_lateral_states[count - 1];
    const LateralPathState &previous =
        planning.retained_lateral_states[count - 2];
    if (last.valid && previous.valid) {
      return (last.planned_d - previous.planned_d) / output_time_step_s;
    }
  }
  return 0.0;
}

FinalBehaviorCandidateStatus StatusFromSt(
    STCandidateStatus status) {
  switch (status) {
  case STCandidateStatus::kCorridorEmpty:
  case STCandidateStatus::kPredictionEvidenceIncomplete:
  case STCandidateStatus::kCurvatureSpeedInfeasible:
    return FinalBehaviorCandidateStatus::kCorridorEmpty;
  case STCandidateStatus::kHorizonInsufficient:
    return FinalBehaviorCandidateStatus::kHorizonInsufficient;
  case STCandidateStatus::kQpInfeasible:
  case STCandidateStatus::kQpTimeout:
  case STCandidateStatus::kQpResultInvalid:
  case STCandidateStatus::kDeadlineSkipped:
    return FinalBehaviorCandidateStatus::kQpInfeasible;
  default:
    return FinalBehaviorCandidateStatus::kGenerated;
  }
}

} // namespace

const char *FinalBehaviorCandidateStatusName(
    FinalBehaviorCandidateStatus status) {
  switch (status) {
  case FinalBehaviorCandidateStatus::kGenerated:
    return "Generated";
  case FinalBehaviorCandidateStatus::kAdmissionRejected:
    return "AdmissionRejected";
  case FinalBehaviorCandidateStatus::kLateralRejected:
    return "LateralRejected";
  case FinalBehaviorCandidateStatus::kCorridorEmpty:
    return "CorridorEmpty";
  case FinalBehaviorCandidateStatus::kQpInfeasible:
    return "QpInfeasible";
  case FinalBehaviorCandidateStatus::kHorizonInsufficient:
    return "HorizonInsufficient";
  case FinalBehaviorCandidateStatus::kComfortRejected:
    return "ComfortRejected";
  case FinalBehaviorCandidateStatus::kValidationRejected:
    return "ValidationRejected";
  case FinalBehaviorCandidateStatus::kValid:
    return "Valid";
  }
  return "Unknown";
}

const char *BehaviorManeuverPhaseName(BehaviorManeuverPhase phase) {
  switch (phase) {
  case BehaviorManeuverPhase::kKeepLane:
    return "KeepLane";
  case BehaviorManeuverPhase::kPrepareCandidate:
    return "PrepareCandidate";
  case BehaviorManeuverPhase::kCommitted:
    return "Committed";
  case BehaviorManeuverPhase::kSettling:
    return "Settling";
  }
  return "Unknown";
}

class ActiveBehaviorPlanner::Impl {
public:
  explicit Impl(const ActiveBehaviorPlannerConfig &config)
      : config_(config), tracker_(config.traffic_tracker),
        predictor_(config.traffic_prediction),
        behavior_planner_(config.behavior_planner),
        spatial_path_planner_(config.spatial_path),
        st_corridor_planner_(config.st_corridor),
        candidate_qp_(config.longitudinal_qp),
        validator_(config.trajectory_validator) {
    ValidateConfig(config_);
  }

  struct MachineState {
    BehaviorManeuverPhase phase = BehaviorManeuverPhase::kKeepLane;
    std::uint64_t proposal_candidate_id = 0;
    int proposal_target_lane = -1;
    GapId proposal_gap;
    std::size_t stable_proposal_cycles = 0;
    std::size_t target_stable_cycles = 0;
    std::uint64_t commit_cycle = 0;
    std::uint64_t maneuver_transition_id = 0;
    BehaviorCandidate committed_behavior;
    SpatialPathCandidate committed_path;
    double source_lane_departure_time_budget_s = 0.0;
    double completion_time_budget_s = 0.0;
    std::size_t last_retained_prefix_points = 0;
  };

  struct PendingTransaction {
    std::uint64_t cycle = 0;
    TrafficTrackerState tracking_state;
    BehaviorPlannerState behavior_state;
    MachineState machine_observation;
    MachineState machine_dispatch;
    bool qp_valid = false;
    LongitudinalQpResult qp_result;
    std::uint64_t oscillation_count = 0;
    std::uint64_t completed_maneuver_count = 0;
    std::uint64_t cancelled_proposal_count = 0;
    ActiveBehaviorCycleResult result;

    bool open() const { return cycle != 0; }

    void Begin(std::uint64_t next_cycle, const MachineState &state) {
      cycle = next_cycle;
      machine_observation = state;
      machine_dispatch = state;
      qp_valid = false;
      qp_result = LongitudinalQpResult();
      oscillation_count = 0;
      completed_maneuver_count = 0;
      cancelled_proposal_count = 0;
      result = ActiveBehaviorCycleResult();
    }

    // A successful control cycle immediately reuses both machine-state
    // buffers. Reset only transaction authority and diagnostics here; a full
    // reset is reserved for discard and lifecycle reset paths.
    void CloseAfterCommit() {
      cycle = 0;
      qp_valid = false;
      oscillation_count = 0;
      completed_maneuver_count = 0;
      cancelled_proposal_count = 0;
      result = ActiveBehaviorCycleResult();
    }

    void Clear() { *this = PendingTransaction(); }
  };

  ActiveBehaviorCycleResult Evaluate(const PlanningSnapshot &planning,
                                     const MapData &map,
                                     bool allow_new_maneuver) {
    if (planning.cycle == 0 || pending_.open()) {
      throw std::logic_error("active behavior Evaluate transaction overlap");
    }
    ActiveBehaviorCycleResult result;
    result.cycle = planning.cycle;
    result.phase_before = machine_state_.phase;
    result.allow_new_maneuver = allow_new_maneuver;
    pending_.Begin(planning.cycle, machine_state_);

    TrafficTrackingUpdate tracking =
        tracker_.Evaluate(planning, map, tracking_state_);
    result.tracking = tracking.snapshot;
    result.prediction = predictor_.Predict(
        tracking.snapshot, planning.frontier.time_from_telemetry_s);
    pending_.tracking_state = std::move(tracking.next_state);

    if (machine_state_.phase == BehaviorManeuverPhase::kCommitted ||
        machine_state_.phase == BehaviorManeuverPhase::kSettling) {
      pending_.behavior_state = behavior_state_;
      EvaluateCommitted(planning, map, result.prediction, &result);
    } else {
      BehaviorPlanningUpdate behavior = behavior_planner_.Evaluate(
          planning, result.tracking, result.prediction, map.track_length,
          behavior_state_);
      pending_.behavior_state = std::move(behavior.next_state);
      result.behavior = std::move(behavior.snapshot);
      result.spatial_paths = spatial_path_planner_.Generate(
          planning, result.behavior, map);
      result.st_candidates = st_corridor_planner_.Build(
          planning, result.behavior, result.spatial_paths,
          result.prediction);
      EvaluateFreshCandidates(planning, map, &result);
      UpdateProposalState(planning, allow_new_maneuver, &result);
    }

    result.phase_after = pending_.machine_observation.phase;
    if (result.first_commit) {
      result.phase_after = pending_.machine_dispatch.phase;
    }
    pending_.result = result;
    return result;
  }

  void Commit(std::uint64_t cycle, bool active_candidate_dispatched) {
    if (cycle == 0 || cycle != pending_.cycle) {
      throw std::logic_error("active behavior commit has no matching Evaluate");
    }
    if (pending_.result.first_commit && !active_candidate_dispatched) {
      machine_state_ = pending_.machine_observation;
    } else {
      machine_state_ = active_candidate_dispatched
                           ? pending_.machine_dispatch
                           : pending_.machine_observation;
    }
    tracking_state_ = std::move(pending_.tracking_state);
    behavior_state_ = std::move(pending_.behavior_state);
    if (pending_.qp_valid) {
      candidate_qp_.CommitWarmStart(pending_.qp_result);
    }
    diagnostics_.oscillation_count += pending_.oscillation_count;
    diagnostics_.completed_maneuver_count += pending_.completed_maneuver_count;
    diagnostics_.cancelled_proposal_count += pending_.cancelled_proposal_count;

    diagnostics_.phase = machine_state_.phase;
    diagnostics_.last_cycle = cycle;
    diagnostics_.commit_cycle = machine_state_.commit_cycle;
    diagnostics_.committed_candidate_id =
        machine_state_.committed_behavior.candidate_id;
    diagnostics_.source_lane =
        machine_state_.committed_behavior.source_lane;
    diagnostics_.target_lane =
        machine_state_.committed_behavior.target_lane;
    diagnostics_.committed_gap = machine_state_.committed_behavior.gap;
    diagnostics_.stable_proposal_cycles =
        machine_state_.stable_proposal_cycles;
    diagnostics_.target_stable_cycles =
        machine_state_.target_stable_cycles;
    diagnostics_.latest_result_committed = true;
    diagnostics_.has_latest_result = true;
    diagnostics_.latest_result = pending_.result;
    for (const FinalBehaviorCandidate &candidate :
         pending_.result.final_candidates) {
      if (candidate.status ==
          FinalBehaviorCandidateStatus::kValidationRejected) {
        ++diagnostics_.full_validation_rejection_count;
      }
      if (candidate.tightening_attempted) {
        ++diagnostics_.tightening_attempt_count;
      }
      if (candidate.tightening_succeeded) {
        ++diagnostics_.tightening_success_count;
      }
    }

    pending_.CloseAfterCommit();
  }

  void Discard(std::uint64_t cycle) noexcept {
    if (pending_.cycle != cycle) {
      return;
    }
    // Retain failure evidence without committing tracker, behavior-machine or
    // QP state. This makes a no-control transaction replayable from runtime
    // diagnostics instead of erasing the exact ST/final rejection.
    diagnostics_.phase = machine_state_.phase;
    diagnostics_.last_cycle = cycle;
    diagnostics_.latest_result_committed = false;
    diagnostics_.has_latest_result = pending_.result.cycle == cycle;
    if (diagnostics_.has_latest_result) {
      diagnostics_.latest_result = pending_.result;
    }
    pending_.Clear();
  }

  void Reset() {
    candidate_qp_.ResetWarmStart();
    tracking_state_ = TrafficTrackerState();
    behavior_state_ = BehaviorPlannerState();
    machine_state_ = MachineState();
    pending_.Clear();
    diagnostics_ = ActiveBehaviorDiagnostics();
  }

  void HandleControlHistoryReset() {
    if (pending_.open()) {
      throw std::logic_error(
          "control-history reset during active behavior transaction");
    }
    candidate_qp_.ResetWarmStart();
    if (machine_state_.phase != BehaviorManeuverPhase::kKeepLane) {
      ++diagnostics_.cancelled_proposal_count;
    }
    behavior_state_ = BehaviorPlannerState();
    machine_state_ = MachineState();
    pending_.Clear();

    diagnostics_.phase = BehaviorManeuverPhase::kKeepLane;
    diagnostics_.commit_cycle = 0;
    diagnostics_.committed_candidate_id = 0;
    diagnostics_.source_lane = -1;
    diagnostics_.target_lane = -1;
    diagnostics_.committed_gap = GapId();
    diagnostics_.stable_proposal_cycles = 0;
    diagnostics_.target_stable_cycles = 0;
    diagnostics_.has_latest_result = false;
    diagnostics_.latest_result = ActiveBehaviorCycleResult();
  }

  bool ManeuverInProgress() const {
    return machine_state_.phase == BehaviorManeuverPhase::kCommitted ||
           machine_state_.phase == BehaviorManeuverPhase::kSettling;
  }

  BehaviorManeuverPhase phase() const { return machine_state_.phase; }

  ActiveBehaviorDiagnostics diagnostics() const { return diagnostics_; }

private:
  FinalBehaviorCandidate FinalizeCandidate(
      const PlanningSnapshot &planning, const MapData &map,
      const BehaviorCandidate &behavior,
      const SpatialPathCandidate &spatial_path,
      const STCandidateEvaluation &ready_candidate,
      double absolute_progress_offset_m,
      std::uint64_t transition_id) {
    FinalBehaviorCandidate final;
    final.candidate_id = behavior.candidate_id;
    final.behavior = behavior.behavior;
    final.source_lane = behavior.source_lane;
    final.target_lane = behavior.target_lane;
    final.gap = behavior.gap;
    final.order = behavior.order;
    final.spatial_path = spatial_path;
    final.st_candidate = ready_candidate;
    final.progress_benefit_m = behavior.estimated_progress_gain_m;
    final.speed_benefit_mps = behavior.estimated_speed_gain_mps;

    if (ready_candidate.status != STCandidateStatus::kCorridorReady) {
      final.status = StatusFromSt(ready_candidate.status);
      final.rejection_detail =
          STCandidateStatusName(ready_candidate.status);
      return final;
    }

    LongitudinalQpInput qp_input = st_corridor_planner_.MakeQpInput(
        planning, ready_candidate, config_.target_speed_mps);
    final.reference_speed_mps = qp_input.reference_speed_mps;
    const QpWarmStartState common_warm_start = candidate_qp_.warm_start();
    LongitudinalQpResult qp =
        candidate_qp_.Evaluate(qp_input, common_warm_start);
    final.qp_attempted = true;
    STCandidateEvaluation evaluated = ready_candidate;
    st_corridor_planner_.AttachQpResult(qp, &evaluated);
    final.st_candidate = evaluated;
    final.qp = qp;
    if (evaluated.status != STCandidateStatus::kQpSolved) {
      final.status = StatusFromSt(evaluated.status);
      final.rejection_detail = STCandidateStatusName(evaluated.status);
      return final;
    }

    EvaluateCartesian(planning, map, transition_id,
                      absolute_progress_offset_m, &final);
    if (final.status == FinalBehaviorCandidateStatus::kValidationRejected &&
        DynamicsOnlyRejection(final.validation)) {
      final.tightening_attempted = true;
      for (std::size_t index = 1;
           index < qp_input.maximum_speed_mps.size(); ++index) {
        qp_input.maximum_speed_mps[index] = std::max(
            0.0, qp_input.maximum_speed_mps[index] *
                     config_.dynamics_tightening_factor);
      }
      if (!qp_input.maximum_speed_mps.empty()) {
        qp_input.maximum_speed_mps[0] = std::max(
            qp_input.maximum_speed_mps[0],
            planning.frontier.longitudinal.v);
      }
      LongitudinalQpResult tightened =
          candidate_qp_.Evaluate(qp_input, common_warm_start);
      STCandidateEvaluation tightened_st = ready_candidate;
      st_corridor_planner_.AttachQpResult(tightened, &tightened_st);
      if (tightened_st.status == STCandidateStatus::kQpSolved) {
        final.qp = tightened;
        final.st_candidate = tightened_st;
        final.reference_speed_mps = qp_input.reference_speed_mps;
        EvaluateCartesian(planning, map, transition_id,
                          absolute_progress_offset_m, &final);
        final.tightening_succeeded =
            final.status == FinalBehaviorCandidateStatus::kValid;
      }
    }
    return final;
  }

  FinalBehaviorCandidate BuildCommittedEmergencyCandidate(
      const PlanningSnapshot &planning, const MapData &map,
      const BehaviorCandidate &behavior,
      const SpatialPathCandidate &spatial_path,
      double absolute_progress_offset_m,
      std::uint64_t transition_id) const {
    FinalBehaviorCandidate final;
    final.candidate_id = behavior.candidate_id;
    final.behavior = behavior.behavior;
    final.source_lane = behavior.source_lane;
    final.target_lane = behavior.target_lane;
    final.gap = behavior.gap;
    final.order = behavior.order;
    final.spatial_path = spatial_path;
    final.progress_benefit_m = behavior.estimated_progress_gain_m;
    final.speed_benefit_mps = behavior.estimated_speed_gain_mps;
    final.emergency_continuation = true;

    LongitudinalQpInput qp_input;
    qp_input.initial_speed_mps =
        std::max(0.0, planning.frontier.longitudinal.v);
    qp_input.initial_acceleration_mps2 =
        planning.frontier.longitudinal.a;
    qp_input.initial_jerk_valid =
        planning.frontier.state_source ==
        PlanningStateSource::kExactInherited;
    qp_input.initial_jerk_mps3 = planning.frontier.longitudinal.j;
    qp_input.reference_speed_mps.assign(
        config_.longitudinal_qp.horizon_steps + 1, 0.0);
    qp_input.minimum_progress_m.assign(
        config_.longitudinal_qp.horizon_steps + 1, 0.0);
    qp_input.maximum_progress_m.assign(
        config_.longitudinal_qp.horizon_steps + 1,
        spatial_path.geometry.path_extent_m);
    qp_input.safety_policy =
        LongitudinalSafetyPolicy::kMaximumBraking;
    final.reference_speed_mps = qp_input.reference_speed_mps;
    final.qp = candidate_qp_.Evaluate(qp_input, QpWarmStartState());
    final.qp_attempted = true;
    final.qp.trajectory.emergency = true;
    if (!final.qp.success || !final.qp.hard_safe) {
      final.status = FinalBehaviorCandidateStatus::kQpInfeasible;
      final.rejection_detail =
          std::string("CommittedEmergencyQp: ") + final.qp.status;
      return final;
    }

    EvaluateCartesian(planning, map, transition_id,
                      absolute_progress_offset_m, &final);
    if (final.status != FinalBehaviorCandidateStatus::kValid) {
      final.rejection_detail =
          std::string("CommittedEmergencyValidation: ") +
          final.rejection_detail;
    }
    return final;
  }

  void EvaluateCartesian(const PlanningSnapshot &planning,
                         const MapData &map,
                         std::uint64_t transition_id,
                         double absolute_progress_offset_m,
                         FinalBehaviorCandidate *final) const {
    try {
      if (!final->qp.trajectory.states.empty()) {
        final->minimum_longitudinal_acceleration_mps2 =
            final->qp.trajectory.states.front().a;
        for (const LongitudinalState &state :
             final->qp.trajectory.states) {
          final->minimum_longitudinal_acceleration_mps2 =
              std::min(final->minimum_longitudinal_acceleration_mps2,
                       state.a);
        }
      }
      const BuiltBehaviorTrajectory built = BuildBehaviorTrajectory(
          planning, final->spatial_path, final->qp, config_, transition_id,
          absolute_progress_offset_m);
      final->full_trajectory = built.full;
      final->output = built.output;
      final->output_longitudinal = built.output_longitudinal;
      final->output_lateral = built.output_lateral;
      TrajectoryValidationContext context;
      context.input = &planning.input;
      context.map = &map;
      context.trajectory = &final->full_trajectory;
      context.prediction_coverage_s =
          planning.frontier.time_from_telemetry_s +
          static_cast<double>(config_.longitudinal_qp.horizon_steps) *
              config_.longitudinal_qp.time_step_seconds;
      final->validation = validator_.Validate(context);
      final->terminal_progress_m =
          final->qp.trajectory.states.empty()
              ? 0.0
              : final->qp.trajectory.states.back().s;
      final->terminal_speed_mps =
          final->qp.trajectory.states.empty()
              ? 0.0
              : final->qp.trajectory.states.back().v;
      final->minimum_physical_margin_m = std::min(
          final->qp.minimum_physical_margin_meters,
          final->validation.minimum_collision_margin_m);
      final->minimum_operational_margin_m =
          final->qp.minimum_operational_margin_meters;
      final->minimum_road_margin_m =
          final->validation.minimum_road_margin_m;
      final->maximum_acceleration_mps2 =
          final->validation.maximum_acceleration_mps2;
      final->maximum_jerk_mps3 = final->validation.maximum_jerk_mps3;
      final->status = final->validation.valid
                          ? FinalBehaviorCandidateStatus::kValid
                          : FinalBehaviorCandidateStatus::kValidationRejected;
      final->rejection_detail = ValidationDetail(final->validation);
      final->cost = ComputeCandidateCost(*final);
    } catch (const std::exception &error) {
      final->status = FinalBehaviorCandidateStatus::kValidationRejected;
      final->rejection_detail = error.what();
    }
  }

  void EvaluateFreshCandidates(const PlanningSnapshot &planning,
                               const MapData &map,
                               ActiveBehaviorCycleResult *result) {
    const std::uint64_t transition_seed =
        UINT64_C(0x8000000000000000) | planning.cycle;
    for (const STCandidateEvaluation &st :
         result->st_candidates.candidates) {
      const BehaviorCandidate *behavior =
          FindBehaviorCandidate(result->behavior, st.candidate_id);
      const SpatialPathCandidate *path =
          FindSpatialPathCandidate(result->spatial_paths, st.candidate_id);
      if (behavior == nullptr || path == nullptr) {
        continue;
      }
      FinalBehaviorCandidate final = FinalizeCandidate(
          planning, map, *behavior, *path, st, 0.0,
          transition_seed ^ behavior->candidate_id);
      if (final.status == FinalBehaviorCandidateStatus::kValid &&
          final.minimum_longitudinal_acceleration_mps2 + kTolerance <
              config_.minimum_discretionary_acceleration_mps2) {
        final.status = FinalBehaviorCandidateStatus::kComfortRejected;
        std::ostringstream detail;
        detail << "minimum longitudinal acceleration "
               << final.minimum_longitudinal_acceleration_mps2
               << " m/s^2 is below discretionary floor "
               << config_.minimum_discretionary_acceleration_mps2
               << " m/s^2";
        final.rejection_detail = detail.str();
      }
      STCandidateEvaluation *evaluated = FindMutableSTCandidate(
          &result->st_candidates, final.candidate_id);
      if (evaluated != nullptr) {
        *evaluated = final.st_candidate;
      }
      result->final_candidates.push_back(std::move(final));
    }
    st_corridor_planner_.RefreshCounts(&result->st_candidates);
    SelectBest(result);
  }

  void SelectBest(ActiveBehaviorCycleResult *result) {
    const FinalBehaviorCandidate *best = nullptr;
    for (const FinalBehaviorCandidate &candidate :
         result->final_candidates) {
      if (candidate.status != FinalBehaviorCandidateStatus::kValid) {
        continue;
      }
      if (best == nullptr || CandidatePreferred(candidate, *best)) {
        best = &candidate;
      }
    }
    if (best != nullptr) {
      result->has_best_valid_candidate = true;
      result->best_candidate_id = best->candidate_id;
      result->best_target_lane = best->target_lane;
      result->best_gap = best->gap;
    }
  }

  const FinalBehaviorCandidate *BestCandidate(
      const ActiveBehaviorCycleResult &result) const {
    for (const FinalBehaviorCandidate &candidate : result.final_candidates) {
      if (candidate.status == FinalBehaviorCandidateStatus::kValid &&
          candidate.candidate_id == result.best_candidate_id) {
        return &candidate;
      }
    }
    return nullptr;
  }

  void UpdateProposalState(const PlanningSnapshot &planning,
                           bool allow_new_maneuver,
                           ActiveBehaviorCycleResult *result) {
    MachineState next = machine_state_;
    const FinalBehaviorCandidate *best = BestCandidate(*result);
    if (!allow_new_maneuver || best == nullptr) {
      if (next.phase == BehaviorManeuverPhase::kPrepareCandidate) {
        ++pending_.cancelled_proposal_count;
      }
      next = MachineState();
      pending_.machine_observation = next;
      pending_.machine_dispatch = next;
      result->stable_proposal_cycles = 0;
      return;
    }

    const bool same =
        next.phase == BehaviorManeuverPhase::kPrepareCandidate &&
        SameProposal(best->candidate_id, best->target_lane, best->gap,
                     next.proposal_candidate_id,
                     next.proposal_target_lane, next.proposal_gap);
    if (!same) {
      if (next.phase == BehaviorManeuverPhase::kPrepareCandidate &&
          next.proposal_target_lane != -1 &&
          next.proposal_target_lane != best->target_lane) {
        ++pending_.oscillation_count;
      }
      next.phase = BehaviorManeuverPhase::kPrepareCandidate;
      next.proposal_candidate_id = best->candidate_id;
      next.proposal_target_lane = best->target_lane;
      next.proposal_gap = best->gap;
      next.stable_proposal_cycles = 1;
    } else {
      ++next.stable_proposal_cycles;
    }
    pending_.machine_observation = next;
    pending_.machine_dispatch = next;
    result->stable_proposal_cycles = next.stable_proposal_cycles;

    if (next.stable_proposal_cycles >=
        config_.minimum_valid_proposal_cycles) {
      result->would_commit = true;
      result->first_commit = true;
      result->has_control_candidate = true;
      result->control_candidate = *best;
      pending_.machine_dispatch.phase = BehaviorManeuverPhase::kCommitted;
      pending_.machine_dispatch.commit_cycle = result->cycle;
      pending_.machine_dispatch.maneuver_transition_id =
          result->control_candidate.output_lateral.empty()
              ? result->control_candidate.candidate_id
              : result->control_candidate.output_lateral.back().transition_id;
      const BehaviorCandidate *behavior =
          FindBehaviorCandidate(result->behavior, best->candidate_id);
      const SpatialPathCandidate *path = FindSpatialPathCandidate(
          result->spatial_paths, best->candidate_id);
      if (behavior == nullptr || path == nullptr) {
        throw std::logic_error("best active candidate lost its identity");
      }
      pending_.machine_dispatch.committed_behavior = *behavior;
      pending_.machine_dispatch.committed_path = *path;
      pending_.machine_dispatch.source_lane_departure_time_budget_s =
          best->st_candidate.source_lane_departure_time_s;
      pending_.machine_dispatch.completion_time_budget_s =
          best->st_candidate.lane_change_completion_time_s;
      pending_.machine_dispatch.last_retained_prefix_points =
          planning.retained_prefix_points;
      pending_.qp_valid = best->qp.success;
      pending_.qp_result = best->qp;
    }
  }

  void EvaluateCommitted(
      const PlanningSnapshot &planning, const MapData &map,
      const FullLaneTrafficPredictionSnapshot &prediction,
      ActiveBehaviorCycleResult *result) {
    MachineState next = machine_state_;
    double absolute_progress_m = InvertGeometryProgress(
        next.committed_path.geometry,
        planning.frontier.road_s_unwrapped_m);
    if (planning.historical_plan_aligned && planning.frontier.lateral.valid &&
        planning.frontier.lateral.transition_id ==
            next.maneuver_transition_id &&
        Finite(planning.frontier.lateral.correction_progress_m)) {
      // The inherited active output already carries the exact spatial arc
      // progress. Re-inverting RoadS through the discretized geometry table
      // introduces a small origin error that becomes a large Cartesian jerk
      // impulse at controller rate on curved, high-speed lane changes.
      absolute_progress_m = Clamp(
          planning.frontier.lateral.correction_progress_m, 0.0,
          next.committed_path.geometry.path_extent_m);
    }
    const std::size_t bounded_previous_points =
        std::min(config_.output_points,
                 planning.original_previous_path_points);
    const std::size_t consumed_points =
        config_.output_points - bounded_previous_points;
    const long long frontier_advance_points =
        static_cast<long long>(consumed_points) +
        static_cast<long long>(planning.retained_prefix_points) -
        static_cast<long long>(next.last_retained_prefix_points);
    const double frontier_advance_s =
        static_cast<double>(std::max<long long>(0, frontier_advance_points)) *
        config_.output_time_step_s;
    next.source_lane_departure_time_budget_s = std::max(
        0.0, next.source_lane_departure_time_budget_s - frontier_advance_s);
    next.completion_time_budget_s = std::max(
        0.0, next.completion_time_budget_s - frontier_advance_s);
    next.last_retained_prefix_points = planning.retained_prefix_points;
    const double completion_progress_m =
        next.committed_path.occupancy
            .lane_change_completion_path_progress_m;
    const double target_center_d =
        next.committed_path.target_d_m;
    const double lateral_rate_mps =
        EstimateLateralRate(planning, config_.output_time_step_s);
    result->target_only =
        absolute_progress_m + kTolerance >= completion_progress_m;

    if (result->target_only ||
        next.phase == BehaviorManeuverPhase::kSettling) {
      next.phase = BehaviorManeuverPhase::kSettling;
      const bool stable_now =
          std::fabs(planning.frontier.d_m - target_center_d) <=
              config_.target_center_tolerance_m &&
          std::fabs(lateral_rate_mps) <=
              config_.target_lateral_rate_tolerance_mps;
      if (!stable_now) {
        next.target_stable_cycles = 0;
      } else if (next.target_stable_cycles < config_.target_stable_cycles) {
        ++next.target_stable_cycles;
      }
      result->target_stable =
          next.target_stable_cycles >= config_.target_stable_cycles;
      if (result->target_stable &&
          planning.control_lane_cruise_handoff_valid) {
        ++pending_.completed_maneuver_count;
        // Relinquish active control only when the target is stable and the
        // same-cycle normal-operational lane-cruise backup has already passed
        // its complete hard gate. Degraded/maximum-braking backups do not
        // qualify as a nominal ownership handoff. If that Cartesian handoff
        // is not ready, Settling continues to dispatch the immutable committed
        // geometry below.
        pending_.machine_observation = MachineState();
        pending_.machine_dispatch = MachineState();
        pending_.behavior_state = BehaviorPlannerState();
        result->stable_proposal_cycles = 0;
        return;
      }
      pending_.machine_observation = next;
      pending_.machine_dispatch = next;
    } else {
      pending_.machine_observation = next;
      pending_.machine_dispatch = next;
    }

    // Reuse the immutable committed geometry throughout Settling. target_only
    // changes the ST occupancy to the target lane; it must not suppress the
    // continuously validated control output before target stability is
    // committed.
    SpatialPathCandidate shifted =
        ShiftCommittedPath(next.committed_path, absolute_progress_m);
    BehaviorCandidate behavior = next.committed_behavior;
    behavior.coarse_admission.evaluated = true;
    behavior.coarse_admission.passed = true;
    behavior.status =
        BehaviorCandidateStatus::kCoarseAdmissionPassed;

    result->behavior.cycle = planning.cycle;
    result->behavior.source_lane = behavior.source_lane;
    result->behavior.generated_candidate_count = 1;
    result->behavior.stable_gap_count = 1;
    result->behavior.coarse_admitted_candidate_count = 1;
    result->behavior.has_best_coarse_candidate = true;
    result->behavior.best_coarse_candidate_id = behavior.candidate_id;
    result->behavior.candidates.push_back(behavior);
    result->spatial_paths.cycle = planning.cycle;
    result->spatial_paths.evaluated_candidate_count = 1;
    result->spatial_paths.generated_candidate_count = 1;
    result->spatial_paths.precheck_passed_candidate_count = 1;
    result->spatial_paths.candidates.push_back(shifted);
    const bool source_lane_released =
        absolute_progress_m + kTolerance >=
        next.committed_path.occupancy
            .source_lane_departure_path_progress_m;
    STManeuverSchedule schedule;
    schedule.committed_continuation = true;
    schedule.source_lane_released = source_lane_released;
    schedule.has_source_lane_departure_deadline =
        !source_lane_released;
    schedule.source_lane_departure_deadline_s =
        next.source_lane_departure_time_budget_s;
    schedule.has_completion_deadline = !result->target_only;
    schedule.completion_deadline_s =
        next.completion_time_budget_s;
    result->st_candidates = st_corridor_planner_.Build(
        planning, result->behavior, result->spatial_paths, prediction,
        schedule);
    if (!result->st_candidates.candidates.empty()) {
      FinalBehaviorCandidate final = FinalizeCandidate(
          planning, map, behavior, shifted,
          result->st_candidates.candidates.front(), absolute_progress_m,
          next.maneuver_transition_id);
      result->st_candidates.candidates.front() = final.st_candidate;
      result->final_candidates.push_back(final);
      if (final.status == FinalBehaviorCandidateStatus::kValid) {
        result->has_best_valid_candidate = true;
        result->best_candidate_id = final.candidate_id;
        result->best_target_lane = final.target_lane;
        result->best_gap = final.gap;
        result->would_commit = true;
        result->continuation = true;
        result->has_control_candidate = true;
        result->control_candidate = final;
        // Keep the continuously decremented commitment deadlines.  Replacing
        // them with a newly observed 0.1 s QP node would round the budget back
        // up every 0.02 s controller cycle and could make the deadline stop
        // advancing altogether.
        pending_.qp_valid = final.qp.success;
        pending_.qp_result = final.qp;
      }
    }
    if (!result->has_control_candidate) {
      FinalBehaviorCandidate emergency = BuildCommittedEmergencyCandidate(
          planning, map, behavior, shifted, absolute_progress_m,
          next.maneuver_transition_id);
      result->final_candidates.push_back(emergency);
      if (emergency.status == FinalBehaviorCandidateStatus::kValid) {
        result->continuation = true;
        result->has_control_candidate = true;
        result->control_candidate = emergency;
      } else if (emergency.validation.HasOnlyCollisionViolations()) {
        result->continuation = true;
        result->has_minimum_risk_candidate = true;
        result->minimum_risk_candidate = emergency;
      }
    }
    st_corridor_planner_.RefreshCounts(&result->st_candidates);
    result->stable_proposal_cycles = next.stable_proposal_cycles;
  }

  ActiveBehaviorPlannerConfig config_;
  TrafficTracker tracker_;
  FullLaneTrafficPredictor predictor_;
  BehaviorPlanner behavior_planner_;
  SpatialPathPlanner spatial_path_planner_;
  STCorridorPlanner st_corridor_planner_;
  LongitudinalQp candidate_qp_;
  TrajectoryValidator validator_;

  TrafficTrackerState tracking_state_;
  BehaviorPlannerState behavior_state_;
  MachineState machine_state_;

  PendingTransaction pending_;
  ActiveBehaviorDiagnostics diagnostics_;
};

ActiveBehaviorPlanner::ActiveBehaviorPlanner(
    const ActiveBehaviorPlannerConfig &config)
    : impl_(new Impl(config)) {}

ActiveBehaviorPlanner::~ActiveBehaviorPlanner() = default;

ActiveBehaviorCycleResult ActiveBehaviorPlanner::Evaluate(
    const PlanningSnapshot &planning, const MapData &map,
    bool allow_new_maneuver) {
  return impl_->Evaluate(planning, map, allow_new_maneuver);
}

void ActiveBehaviorPlanner::Commit(
    std::uint64_t cycle, bool active_candidate_dispatched) {
  impl_->Commit(cycle, active_candidate_dispatched);
}

void ActiveBehaviorPlanner::Discard(std::uint64_t cycle) noexcept {
  impl_->Discard(cycle);
}

void ActiveBehaviorPlanner::HandleControlHistoryReset() {
  impl_->HandleControlHistoryReset();
}

void ActiveBehaviorPlanner::Reset() { impl_->Reset(); }

bool ActiveBehaviorPlanner::ManeuverInProgress() const {
  return impl_->ManeuverInProgress();
}

BehaviorManeuverPhase ActiveBehaviorPlanner::phase() const {
  return impl_->phase();
}

ActiveBehaviorDiagnostics ActiveBehaviorPlanner::diagnostics() const {
  return impl_->diagnostics();
}
