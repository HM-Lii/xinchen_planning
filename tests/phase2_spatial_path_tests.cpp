#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

#include "map.h"
#include "planner.h"
#include "spatial_path.h"

namespace {

int failures = 0;

void Expect(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
    ++failures;
  }
}

void RunTest(void (*test)(), const std::string &name) {
  try {
    test();
  } catch (const std::exception &error) {
    Expect(false, name + " threw: " + error.what());
  }
}

PlanningSnapshot Planning(double road_s_m, double d_m, double speed_mps) {
  PlanningSnapshot result;
  result.cycle = 3;
  result.target_lane = static_cast<int>(std::floor(d_m / 4.0));
  result.frontier.road_s_unwrapped_m = road_s_m;
  result.frontier.d_m = d_m;
  result.frontier.longitudinal.v = speed_mps;
  result.frontier.lateral.valid = true;
  result.frontier.lateral.road_parameter_s = road_s_m;
  result.frontier.lateral.planned_d = d_m;
  for (int index = 3; index >= 0; --index) {
    LateralPathState state;
    state.valid = true;
    state.road_parameter_s = road_s_m - 0.5 * index;
    state.planned_d = d_m;
    result.retained_lateral_states.push_back(state);
  }
  return result;
}

BehaviorCandidate LaneChange(std::uint64_t id, int source_lane, int target_lane,
                             double estimated_progress_m = 160.0) {
  BehaviorCandidate result;
  result.candidate_id = id;
  result.behavior = target_lane < source_lane ? BehaviorType::kChangeLeft
                                              : BehaviorType::kChangeRight;
  result.source_lane = source_lane;
  result.target_lane = target_lane;
  result.estimated_progress_m = estimated_progress_m;
  result.coarse_admission.evaluated = true;
  result.coarse_admission.passed = true;
  result.status = BehaviorCandidateStatus::kCoarseAdmissionPassed;
  return result;
}

BehaviorPlanningSnapshot BehaviorWith(const BehaviorCandidate &candidate) {
  BehaviorPlanningSnapshot result;
  result.cycle = 3;
  result.source_lane = candidate.source_lane;
  result.generated_candidate_count = 1;
  result.coarse_admitted_candidate_count = 1;
  result.candidates.push_back(candidate);
  return result;
}

const SpatialPathGeometrySample *
TransitionSample(const SpatialPathCandidate &candidate) {
  for (const SpatialPathGeometrySample &sample : candidate.geometry.samples) {
    if (std::fabs(sample.construction_progress_m -
                  candidate.transition_length_m) < 1e-8) {
      return &sample;
    }
  }
  return nullptr;
}

void TestSepticC3ArcLengthAndOccupancy() {
  const MapData map = LoadMap("data/highway_map.csv");
  SpatialPathPlannerConfig config;
  config.speed_upper_bound_mps = 20.0;
  SpatialPathPlanner planner(config);
  const SpatialPathBatchSnapshot batch = planner.Generate(
      Planning(100.0, 6.0, 20.0), BehaviorWith(LaneChange(101, 1, 0)), map);
  Expect(batch.evaluated_candidate_count == 1 &&
             batch.generated_candidate_count == 1 &&
             batch.precheck_passed_candidate_count == 1 &&
             batch.candidates.size() == 1,
         "one coarse-admitted lane change produces one valid spatial path");
  if (batch.candidates.empty()) {
    return;
  }
  const SpatialPathCandidate &candidate = batch.candidates.front();
  Expect(candidate.status == SpatialPathCandidateStatus::kPrecheckPassed &&
             candidate.precheck.passed &&
             std::fabs(candidate.precheck.speed_upper_bound_mps - 20.0) < 1e-12,
         "nominal septic passes every P2.3 precheck at its declared hard speed "
         "upper bound");
  Expect(candidate.precheck.c2_position_residual_m < 1e-9 &&
             candidate.precheck.c2_first_derivative_residual < 1e-9 &&
             candidate.precheck.c2_second_derivative_residual_per_m < 1e-9 &&
             candidate.precheck.c3_third_derivative_residual_per_m2 < 1e-9,
         "septic matches start and terminal d/d'/d''/d''' exactly");
  Expect(!candidate.geometry.samples.empty() &&
             std::fabs(candidate.geometry.samples.front().d_m - 6.0) < 1e-9,
         "geometry starts at the planning-frontier lateral position");
  const SpatialPathGeometrySample *finish = TransitionSample(candidate);
  Expect(finish != nullptr && std::fabs(finish->d_m - 2.0) < 1e-8 &&
             std::fabs(finish->d_first_derivative) < 1e-8 &&
             std::fabs(finish->d_second_derivative_per_m) < 1e-8 &&
             std::fabs(finish->d_third_derivative_per_m2) < 1e-8,
         "transition reaches the target center with zero first/second/third "
         "derivative");

  bool monotonic = true;
  bool arc_length_bounds_chord = true;
  for (std::size_t index = 1; index < candidate.geometry.samples.size();
       ++index) {
    const SpatialPathGeometrySample &previous =
        candidate.geometry.samples[index - 1];
    const SpatialPathGeometrySample &sample = candidate.geometry.samples[index];
    monotonic = monotonic &&
                sample.path_progress_m > previous.path_progress_m &&
                sample.road_s_unwrapped_m > previous.road_s_unwrapped_m;
    const double chord =
        std::hypot(sample.x_m - previous.x_m, sample.y_m - previous.y_m);
    arc_length_bounds_chord =
        arc_length_bounds_chord &&
        sample.path_progress_m - previous.path_progress_m + 1e-6 >= chord;
  }
  Expect(monotonic && arc_length_bounds_chord &&
             candidate.geometry.path_extent_m + 1e-8 >=
                 candidate.requested_path_extent_m,
         "PathProgress is monotonic physical arc length with full requested "
         "coverage");
  const SpatialPathGeometrySample interpolated = SampleSpatialPathAtProgress(
      candidate.geometry,
      0.5 * candidate.geometry.transition_completion_path_progress_m);
  Expect(
      std::fabs(interpolated.path_progress_m -
                0.5 *
                    candidate.geometry.transition_completion_path_progress_m) <
              1e-12 &&
          interpolated.d_m < 6.0 && interpolated.d_m > 2.0 &&
          std::fabs(std::hypot(interpolated.tangent_x, interpolated.tangent_y) -
                    1.0) < 1e-12,
      "downstream consumers can map physical PathProgress to geometry");
  Expect(candidate.occupancy.target_lane_coverage_started &&
             candidate.occupancy.source_lane_departed &&
             candidate.occupancy.lane_change_completed &&
             candidate.occupancy.target_lane_coverage_start_path_progress_m <
                 candidate.occupancy.source_lane_departure_path_progress_m &&
             candidate.occupancy.source_lane_departure_path_progress_m <
                 candidate.occupancy.lane_change_completion_path_progress_m,
         "body occupancy milestones are present and ordered");
}

void TestHardSpeedUpperBoundIsNotWeakenedByCandidatePace() {
  const MapData map = LoadMap("data/highway_map.csv");
  SpatialPathPlanner planner;
  const SpatialPathBatchSnapshot batch =
      planner.Generate(Planning(100.0, 6.0, 20.0),
                       BehaviorWith(LaneChange(106, 1, 0, 160.0)), map);
  Expect(batch.candidates.size() == 1,
         "hard-speed-upper candidate is evaluated");
  if (batch.candidates.empty()) {
    return;
  }
  const SpatialPathPrecheckResult &precheck = batch.candidates.front().precheck;
  Expect(std::fabs(precheck.speed_upper_bound_mps - 22.12848) < 1e-9 &&
             precheck.maximum_estimated_lateral_acceleration_mps2 > 4.0 &&
             HasSpatialPathPrecheckReason(
                 precheck, SpatialPathPrecheckReason::kLateralAcceleration),
         "coarse pace cannot weaken the configured lateral-dynamics speed "
         "upper bound");

  const SpatialPathBatchSnapshot overspeed =
      planner.Generate(Planning(100.0, 6.0, 25.0),
                       BehaviorWith(LaneChange(107, 1, 0, 160.0)), map);
  Expect(overspeed.candidates.size() == 1 &&
             std::fabs(
                 overspeed.candidates.front().precheck.speed_upper_bound_mps -
                 25.0) < 1e-9,
         "an already-higher planning-frontier speed raises the hard dynamics "
         "bound");
}

void TestInheritedCubicBoundaryKeepsC3Connection() {
  const MapData map = LoadMap("data/highway_map.csv");
  PlanningSnapshot planning = Planning(100.0, 6.0, 18.0);
  const double expected_first = 0.04;
  const double expected_second = 0.004;
  const double expected_third = 0.003;
  for (LateralPathState &state : planning.retained_lateral_states) {
    const double offset = state.road_parameter_s - 100.0;
    state.planned_d = 6.0 + expected_first * offset +
                      0.5 * expected_second * offset * offset +
                      expected_third * offset * offset * offset / 6.0;
  }
  SpatialPathPlannerConfig config;
  config.maximum_lateral_jerk_mps3 = 100.0;
  SpatialPathPlanner planner(config);
  const SpatialPathBatchSnapshot batch = planner.Generate(
      planning, BehaviorWith(LaneChange(102, 1, 0, 144.0)), map);
  Expect(batch.candidates.size() == 1,
         "nonzero-boundary candidate is evaluated");
  if (batch.candidates.empty() ||
      batch.candidates.front().geometry.samples.empty()) {
    return;
  }
  const SpatialPathCandidate &candidate = batch.candidates.front();
  const SpatialPathGeometrySample &start = candidate.geometry.samples.front();
  Expect(std::fabs(candidate.start_d_first_derivative - expected_first) <
                 1e-9 &&
             std::fabs(candidate.start_d_second_derivative_per_m -
                       expected_second) < 1e-9 &&
             std::fabs(candidate.start_d_third_derivative_per_m2 -
                       expected_third) < 1e-9 &&
             std::fabs(start.d_first_derivative -
                       candidate.start_d_first_derivative) < 1e-9 &&
             std::fabs(start.d_second_derivative_per_m -
                       candidate.start_d_second_derivative_per_m) < 1e-9 &&
             std::fabs(start.d_third_derivative_per_m2 -
                       candidate.start_d_third_derivative_per_m2) < 1e-9 &&
             candidate.precheck.c2_first_derivative_residual < 1e-9 &&
             candidate.precheck.c2_second_derivative_residual_per_m < 1e-9 &&
             candidate.precheck.c3_third_derivative_residual_per_m2 < 1e-9,
         "frontier d', d'' and d''' are inherited rather than reset to zero");

  if (candidate.geometry.samples.size() >= 2) {
    const double step = 0.5;
    const double new_d = candidate.geometry.samples[1].d_m;
    const double old_d0 = planning.retained_lateral_states[3].planned_d;
    const double old_d_minus_one =
        planning.retained_lateral_states[2].planned_d;
    const double old_d_minus_two =
        planning.retained_lateral_states[1].planned_d;
    const double stitched_third_difference =
        (new_d - 3.0 * old_d0 + 3.0 * old_d_minus_one - old_d_minus_two) /
        (step * step * step);
    Expect(std::fabs(stitched_third_difference - expected_third) < 1e-3,
           "the first new sample preserves the inherited discrete "
           "third-derivative trend");
  }
}

void TestAlignedCartesianHistoryDefinesC3Boundary() {
  const MapData map = LoadMap("data/highway_map.csv");
  PlanningSnapshot planning = Planning(100.0, 6.0, 18.0);
  planning.historical_plan_aligned = true;
  for (LateralPathState &state : planning.retained_lateral_states) {
    const double actual_d = 6.01 + 0.02 * (state.road_parameter_s - 100.0);
    const RoadGeometrySample geometry =
        EvaluateRoadGeometry(state.road_parameter_s, actual_d, map);
    state.expected_x = geometry.x;
    state.expected_y = geometry.y;
    planning.input.previous_path_x.push_back(geometry.x);
    planning.input.previous_path_y.push_back(geometry.y);
  }

  SpatialPathPlannerConfig config;
  config.maximum_lateral_jerk_mps3 = 100.0;
  SpatialPathPlanner planner(config);
  const SpatialPathBatchSnapshot batch = planner.Generate(
      planning, BehaviorWith(LaneChange(110, 1, 0, 144.0)), map);
  Expect(batch.candidates.size() == 1,
         "aligned-Cartesian-boundary candidate is evaluated");
  if (batch.candidates.empty() ||
      batch.candidates.front().geometry.samples.empty()) {
    return;
  }
  const SpatialPathCandidate &candidate = batch.candidates.front();
  Expect(std::fabs(candidate.start_d_m - 6.01) < 1e-5 &&
             std::fabs(candidate.start_d_first_derivative - 0.02) < 1e-5 &&
             std::fabs(candidate.geometry.samples.front().d_m - 6.01) < 1e-5,
         "the C3 boundary follows the issued Cartesian path rather than stale "
         "nominal planned_d");
}

void TestRightLaneChangeUsesTheSameGeometricContract() {
  const MapData map = LoadMap("data/highway_map.csv");
  SpatialPathPlanner planner;
  const SpatialPathBatchSnapshot batch = planner.Generate(
      Planning(100.0, 6.0, 20.0), BehaviorWith(LaneChange(105, 1, 2)), map);
  Expect(batch.candidates.size() == 1 &&
             batch.candidates.front().precheck.passed,
         "right lane change passes the same bounded spatial prechecks");
  if (batch.candidates.empty()) {
    return;
  }
  const SpatialPathGeometrySample *finish =
      TransitionSample(batch.candidates.front());
  Expect(finish != nullptr && std::fabs(finish->d_m - 10.0) < 1e-8 &&
             batch.candidates.front().occupancy.source_lane_departed &&
             batch.candidates.front().occupancy.lane_change_completed,
         "right transition terminates at the declared target lane center");
}

void TestRoadBoundaryAndDynamicsFailClosed() {
  const MapData map = LoadMap("data/highway_map.csv");
  SpatialPathPlanner planner;
  const SpatialPathBatchSnapshot boundary =
      planner.Generate(Planning(100.0, 0.5, 15.0),
                       BehaviorWith(LaneChange(103, 0, 1, 120.0)), map);
  Expect(boundary.candidates.size() == 1 &&
             HasSpatialPathPrecheckReason(
                 boundary.candidates.front().precheck,
                 SpatialPathPrecheckReason::kRoadBoundary) &&
             !boundary.candidates.front().precheck.passed,
         "vehicle-body road-boundary intrusion rejects the spatial path");

  SpatialPathPlannerConfig strict_config;
  strict_config.maximum_abs_lateral_slope = 1e-6;
  strict_config.maximum_abs_lateral_second_derivative_per_m = 1e-6;
  strict_config.maximum_abs_curvature_per_m = 1e-6;
  strict_config.maximum_lateral_acceleration_mps2 = 0.01;
  strict_config.maximum_lateral_jerk_mps3 = 0.01;
  SpatialPathPlanner strict_planner(strict_config);
  const SpatialPathBatchSnapshot dynamics = strict_planner.Generate(
      Planning(100.0, 6.0, 20.0), BehaviorWith(LaneChange(104, 1, 0)), map);
  Expect(
      dynamics.candidates.size() == 1 &&
          HasSpatialPathPrecheckReason(dynamics.candidates.front().precheck,
                                       SpatialPathPrecheckReason::kCurvature) &&
          HasSpatialPathPrecheckReason(
              dynamics.candidates.front().precheck,
              SpatialPathPrecheckReason::kLateralAcceleration) &&
          HasSpatialPathPrecheckReason(
              dynamics.candidates.front().precheck,
              SpatialPathPrecheckReason::kLateralJerk) &&
          HasSpatialPathPrecheckReason(
              dynamics.candidates.front().precheck,
              SpatialPathPrecheckReason::kTransitionLengthInsufficient),
      "curvature and speed-coupled lateral dynamics reject before P2.4");
}

void TestOnlyCoarseAdmittedCandidatesAreEvaluated() {
  const MapData map = LoadMap("data/highway_map.csv");
  BehaviorPlanningSnapshot behavior;
  BehaviorCandidate keep;
  keep.candidate_id = 1;
  keep.behavior = BehaviorType::kKeepLane;
  keep.status = BehaviorCandidateStatus::kKeepLaneBaseline;
  behavior.candidates.push_back(keep);
  BehaviorCandidate rejected = LaneChange(2, 1, 0);
  rejected.coarse_admission.passed = false;
  rejected.status = BehaviorCandidateStatus::kCoarseAdmissionRejected;
  behavior.candidates.push_back(rejected);
  SpatialPathPlanner planner;
  const SpatialPathBatchSnapshot batch =
      planner.Generate(Planning(100.0, 6.0, 20.0), behavior, map);
  Expect(batch.evaluated_candidate_count == 0 &&
             batch.generated_candidate_count == 0 && batch.candidates.empty(),
         "keep-lane and coarse-rejected candidates do not allocate geometry");
}

void TestMalformedCandidateAndMissingBoundaryFailClosed() {
  const MapData map = LoadMap("data/highway_map.csv");
  SpatialPathPlanner planner;
  BehaviorCandidate wrong_direction = LaneChange(108, 1, 0);
  wrong_direction.behavior = BehaviorType::kChangeRight;
  const SpatialPathBatchSnapshot malformed = planner.Generate(
      Planning(100.0, 6.0, 20.0), BehaviorWith(wrong_direction), map);
  Expect(malformed.candidates.size() == 1 &&
             malformed.candidates.front().geometry.samples.empty() &&
             HasSpatialPathPrecheckReason(
                 malformed.candidates.front().precheck,
                 SpatialPathPrecheckReason::kInvalidCandidate),
         "an inconsistent lane-change direction fails closed before geometry "
         "allocation");

  PlanningSnapshot missing_boundary = Planning(100.0, 6.0, 20.0);
  missing_boundary.retained_lateral_states.pop_back();
  const SpatialPathBatchSnapshot no_boundary = planner.Generate(
      missing_boundary, BehaviorWith(LaneChange(109, 1, 0)), map);
  Expect(no_boundary.candidates.size() == 1 &&
             no_boundary.candidates.front().geometry.samples.empty() &&
             HasSpatialPathPrecheckReason(
                 no_boundary.candidates.front().precheck,
                 SpatialPathPrecheckReason::kInitialBoundaryUnavailable),
         "insufficient planning-frontier samples fail closed without inventing "
         "zero derivatives");
}

void TestInvalidConfigurationRejected() {
  SpatialPathPlannerConfig config;
  config.maximum_geometry_samples = 4;
  bool rejected = false;
  try {
    SpatialPathPlanner planner(config);
    (void)planner;
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  Expect(rejected,
         "configured geometry capacity must cover the bounded path extent");
}

} // namespace

int main() {
  RunTest(TestSepticC3ArcLengthAndOccupancy,
          "TestSepticC3ArcLengthAndOccupancy");
  RunTest(TestInheritedCubicBoundaryKeepsC3Connection,
          "TestInheritedCubicBoundaryKeepsC3Connection");
  RunTest(TestAlignedCartesianHistoryDefinesC3Boundary,
          "TestAlignedCartesianHistoryDefinesC3Boundary");
  RunTest(TestHardSpeedUpperBoundIsNotWeakenedByCandidatePace,
          "TestHardSpeedUpperBoundIsNotWeakenedByCandidatePace");
  RunTest(TestRightLaneChangeUsesTheSameGeometricContract,
          "TestRightLaneChangeUsesTheSameGeometricContract");
  RunTest(TestRoadBoundaryAndDynamicsFailClosed,
          "TestRoadBoundaryAndDynamicsFailClosed");
  RunTest(TestOnlyCoarseAdmittedCandidatesAreEvaluated,
          "TestOnlyCoarseAdmittedCandidatesAreEvaluated");
  RunTest(TestMalformedCandidateAndMissingBoundaryFailClosed,
          "TestMalformedCandidateAndMissingBoundaryFailClosed");
  RunTest(TestInvalidConfigurationRejected, "TestInvalidConfigurationRejected");
  if (failures != 0) {
    std::cerr << failures << " P2.3 spatial-path assertion(s) failed"
              << std::endl;
    return 1;
  }
  std::cout << "All P2.3 spatial-path tests passed" << std::endl;
  return 0;
}
