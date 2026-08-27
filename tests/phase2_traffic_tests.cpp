#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "map.h"
#include "planner.h"
#include "traffic_prediction.h"
#include "traffic_tracker.h"

namespace {

int failures = 0;

void Expect(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
    ++failures;
  }
}

void ExpectNear(double actual, double expected, double tolerance,
                const std::string &message) {
  if (!std::isfinite(actual) ||
      std::fabs(actual - expected) > tolerance) {
    std::cerr << "FAILED: " << message << " (actual=" << actual
              << ", expected=" << expected << ")" << std::endl;
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

DetectedVehicle Vehicle(int id, double road_s, double d,
                        double road_s_rate_mps, double d_rate_mps,
                        const MapData &map) {
  const RoadGeometrySample road =
      EvaluateRoadGeometry(road_s, d, map);
  const RoadGeometrySample center =
      EvaluateRoadGeometry(road_s, 0.0, map);
  const RoadGeometrySample unit_offset =
      EvaluateRoadGeometry(road_s, 1.0, map);
  const double normal_x = unit_offset.x - center.x;
  const double normal_y = unit_offset.y - center.y;

  DetectedVehicle result;
  result.id = static_cast<double>(id);
  result.s = NormalizeS(road_s, map.track_length);
  result.d = d;
  result.x = road.x;
  result.y = road.y;
  result.vx_mps =
      road.first_derivative_x * road_s_rate_mps + normal_x * d_rate_mps;
  result.vy_mps =
      road.first_derivative_y * road_s_rate_mps + normal_y * d_rate_mps;
  return result;
}

PlanningSnapshot Snapshot(std::uint64_t cycle, double ego_road_s,
                          const std::vector<DetectedVehicle> &traffic,
                          double telemetry_time_s = -1.0) {
  PlanningSnapshot result;
  result.cycle = cycle;
  result.telemetry_time_s = telemetry_time_s >= 0.0
                                ? telemetry_time_s
                                : static_cast<double>(cycle - 1) * 0.02;
  result.input.ego.s = ego_road_s;
  result.input.ego.d = 6.0;
  result.input.traffic = traffic;
  result.reset_reason = PlanningStateResetReason::kNone;
  return result;
}

const TrackedVehicleState &Track(const TrafficTrackingSnapshot &snapshot,
                                 int id) {
  for (const TrackedVehicleState &track : snapshot.tracks) {
    if (track.id == id) {
      return track;
    }
  }
  throw std::runtime_error("missing expected traffic track");
}

const TrafficPredictionTrajectory &Trajectory(
    const FullLaneTrafficPredictionSnapshot &snapshot, int id,
    TrafficPredictionHypothesis hypothesis) {
  for (const TrafficPredictionTrajectory &trajectory :
       snapshot.trajectories) {
    if (trajectory.vehicle_id == id &&
        trajectory.hypothesis == hypothesis) {
      return trajectory;
    }
  }
  throw std::runtime_error("missing expected prediction hypothesis");
}

void TestLoopUnwrapAndRearRelativeCoordinate() {
  const MapData map = LoadMap("data/highway_map.csv");
  TrafficTrackerConfig config;
  config.minimum_safety_track_age_s = 0.0;
  TrafficTracker tracker(config);
  TrafficTrackerState state;
  const double length = map.track_length;

  TrafficTrackingUpdate update = tracker.Evaluate(
      Snapshot(1, length - 0.5,
               {Vehicle(1, 0.5, 2.0, 37.5, 0.0, map),
                Vehicle(2, length - 1.5, 10.0, 37.5, 0.0, map)}),
      map, state);
  ExpectNear(Track(update.snapshot, 1).relative_road_s_m, 1.0, 1e-9,
             "front vehicle uses the nearest positive loop image");
  ExpectNear(Track(update.snapshot, 2).relative_road_s_m, -1.0, 1e-9,
             "rear vehicle remains negative near loop zero");
  const double first_front_s =
      Track(update.snapshot, 1).road_s_unwrapped_m;
  state = update.next_state;

  update = tracker.Evaluate(
      Snapshot(2, 0.25,
               {Vehicle(1, 1.25, 2.0, 37.5, 0.0, map),
                Vehicle(2, length - 0.75, 10.0, 37.5, 0.0, map)}),
      map, state);
  ExpectNear(Track(update.snapshot, 1).road_s_unwrapped_m - first_front_s,
             0.75, 1e-9,
             "front track is continuous while ego crosses loop zero");
  Expect(Track(update.snapshot, 2).relative_road_s_m < 0.0,
         "rear track does not wrap into a nearly full-lap lead");
  state = update.next_state;

  update = tracker.Evaluate(
      Snapshot(3, 1.0,
               {Vehicle(1, 2.0, 2.0, 37.5, 0.0, map),
                Vehicle(2, 0.0, 10.0, 37.5, 0.0, map)}),
      map, state);
  ExpectNear(Track(update.snapshot, 2).relative_road_s_m, -1.0, 1e-9,
             "rear vehicle stays negative when it also crosses loop zero");
  Expect(Track(update.snapshot, 1).observation_count == 3 &&
             Track(update.snapshot, 2).observation_count == 3,
         "loop crossings preserve vehicle identity");
}

void TestVelocityProjectionAndBoundedAcceleration() {
  const MapData map = LoadMap("data/highway_map.csv");
  TrafficTrackerConfig config;
  config.minimum_safety_track_age_s = 0.0;
  config.maximum_abs_longitudinal_acceleration_mps2 = 4.0;
  TrafficTracker tracker(config);
  TrafficTrackerState state;

  TrafficTrackingUpdate first = tracker.Evaluate(
      Snapshot(1, 90.0, {Vehicle(7, 100.0, 6.0, 10.0, 0.4, map)}),
      map, state);
  const TrackedVehicleState &initial = Track(first.snapshot, 7);
  ExpectNear(initial.road_s_rate_mps, 10.0, 1e-8,
             "Cartesian velocity projects onto the road tangent");
  ExpectNear(initial.d_rate_mps, 0.4, 1e-8,
             "Cartesian velocity projects onto Frenet d");

  TrafficTrackingUpdate second = tracker.Evaluate(
      Snapshot(2, 90.2,
               {Vehicle(7, 100.2, 6.008, 30.0, 0.4, map)}),
      map, first.next_state);
  const TrackedVehicleState &updated = Track(second.snapshot, 7);
  ExpectNear(updated.road_s_rate_mps, 30.0, 1e-8,
             "updated tangent speed uses the new measurement");
  ExpectNear(updated.d_rate_mps, 0.4, 1e-8,
             "updated lateral rate remains projected");
  ExpectNear(updated.longitudinal_acceleration_mps2, 4.0, 1e-12,
             "finite-difference longitudinal acceleration is bounded");
}

void TestDisappearReacquireReuseAndStaleness() {
  const MapData map = LoadMap("data/highway_map.csv");
  TrafficTrackerConfig config;
  config.stale_after_s = 0.04;
  config.drop_after_s = 0.12;
  config.minimum_safety_track_age_s = 0.02;
  config.id_reuse_longitudinal_tolerance_m = 2.0;
  TrafficTracker tracker(config);
  TrafficTrackerState state;

  TrafficTrackingUpdate update = tracker.Evaluate(
      Snapshot(1, 90.0, {Vehicle(9, 100.0, 2.0, 10.0, 0.0, map)}),
      map, state);
  state = update.next_state;
  update = tracker.Evaluate(
      Snapshot(2, 90.2, {Vehicle(9, 100.2, 2.0, 10.0, 0.0, map)}),
      map, state);
  Expect(Track(update.snapshot, 9).safety_admissible,
         "a fresh track reaches its configured minimum age");
  state = update.next_state;

  update = tracker.Evaluate(Snapshot(3, 90.4, {}), map, state);
  Expect(Track(update.snapshot, 9).valid,
         "a one-cycle dropout is retained with propagated state");
  state = update.next_state;
  update = tracker.Evaluate(Snapshot(5, 90.8, {}), map, state);
  Expect(!Track(update.snapshot, 9).valid &&
             !Track(update.snapshot, 9).safety_admissible &&
             update.snapshot.stale_track_count == 1,
         "a stale track is explicitly barred from safety admission");
  state = update.next_state;

  update = tracker.Evaluate(
      Snapshot(6, 91.0, {Vehicle(9, 101.0, 2.0, 10.0, 0.0, map)}),
      map, state);
  Expect(update.snapshot.reacquired_track_count == 1 &&
             Track(update.snapshot, 9).observation_count == 1 &&
             !Track(update.snapshot, 9).safety_admissible,
         "reappearance after staleness starts a new evidence age");
  state = update.next_state;
  update = tracker.Evaluate(
      Snapshot(7, 91.2, {Vehicle(9, 101.2, 2.0, 10.0, 0.0, map)}),
      map, state);
  state = update.next_state;
  update = tracker.Evaluate(
      Snapshot(8, 91.4, {Vehicle(9, 130.0, 2.0, 10.0, 0.0, map)}),
      map, state);
  Expect(update.snapshot.reused_id_count == 1 &&
             Track(update.snapshot, 9).observation_count == 1,
         "an implausible fresh jump is treated as ID reuse");
  state = update.next_state;
  update = tracker.Evaluate(Snapshot(20, 94.0, {}), map, state);
  Expect(update.snapshot.tracks.empty(),
         "tracks are removed after the bounded retention window");
}

void TestTrackerUsesExecutedTrajectoryTime() {
  const MapData map = LoadMap("data/highway_map.csv");
  TrafficTrackerConfig config;
  config.minimum_safety_track_age_s = 0.0;
  TrafficTracker tracker(config);

  TrafficTrackingUpdate update = tracker.Evaluate(
      Snapshot(1, 100.0, {Vehicle(12, 76.0, 2.0, 30.0, 0.0, map)}, 0.0), map,
      TrafficTrackerState());
  TrafficTrackerState state = update.next_state;

  update = tracker.Evaluate(Snapshot(2, 100.0, {}, 0.0), map, state);
  const TrackedVehicleState &zero_elapsed = Track(update.snapshot, 12);
  ExpectNear(zero_elapsed.relative_road_s_m, -24.0, 1e-8,
             "a zero-consumption planning call does not move a missing track");
  ExpectNear(zero_elapsed.time_since_update_s, 0.0, 0.0,
             "a zero-consumption planning call does not age a missing track");
  state = update.next_state;

  update = tracker.Evaluate(Snapshot(3, 100.0, {}, 0.70), map, state);
  const TrackedVehicleState &consumed = Track(update.snapshot, 12);
  ExpectNear(consumed.relative_road_s_m, -3.0, 1e-8,
             "35 consumed points propagate a missing rear track by 0.7 s");
  ExpectNear(consumed.time_since_update_s, 0.70, 1e-12,
             "track staleness uses executed trajectory time");
  ExpectNear(consumed.longitudinal_uncertainty_m, 1.20, 1e-12,
             "dropout uncertainty uses executed trajectory time");
  Expect(!consumed.valid && !consumed.safety_admissible &&
             update.snapshot.stale_track_count == 1,
         "a 0.7 s dropout is stale even across one additional planner call");
}

void TestPredictionHypothesesCoverageAndLaneOccupancy() {
  TrafficTrackingSnapshot tracking;
  tracking.cycle = 42;
  TrackedVehicleState front;
  front.id = 1;
  front.road_s_unwrapped_m = 120.0;
  front.relative_road_s_m = 20.0;
  front.d_m = 2.0;
  front.road_s_rate_mps = 12.0;
  front.d_rate_mps = 0.6;
  front.length_m = 4.8;
  front.width_m = 2.0;
  front.longitudinal_uncertainty_m = 0.5;
  front.valid = true;
  front.safety_admissible = true;
  TrackedVehicleState rear = front;
  rear.id = 2;
  rear.road_s_unwrapped_m = 70.0;
  rear.relative_road_s_m = -30.0;
  rear.d_m = 6.0;
  rear.road_s_rate_mps = 15.0;
  rear.d_rate_mps = 0.0;
  tracking.tracks = {front, rear};
  tracking.valid_track_count = 2;
  tracking.safety_admissible_track_count = 2;

  FullLaneTrafficPredictionConfig config;
  ExpectNear(config.front_conservative_deceleration_mps2, -1.0, 0.0,
             "front conservative braking uses the configured default");
  ExpectNear(config.rear_conservative_acceleration_mps2, 0.5, 0.0,
             "rear conservative acceleration uses the configured default");
  ExpectNear(config.longitudinal_acceleration_duration_s, 2.0, 0.0,
             "longitudinal acceleration hypotheses last two seconds");
  ExpectNear(config.longitudinal_uncertainty_growth_mps, 0.25, 0.0,
             "longitudinal prediction uncertainty uses reduced linear growth");
  FullLaneTrafficPredictionConfig excessive_rear_acceleration = config;
  excessive_rear_acceleration.rear_conservative_acceleration_mps2 = 1.01;
  bool excessive_rear_acceleration_rejected = false;
  try {
    FullLaneTrafficPredictor invalid_predictor(
        excessive_rear_acceleration);
  } catch (const std::invalid_argument &) {
    excessive_rear_acceleration_rejected = true;
  }
  Expect(excessive_rear_acceleration_rejected,
         "rear conservative acceleration above 1 m/s2 is rejected");
  FullLaneTrafficPredictionConfig excessive_front_deceleration = config;
  excessive_front_deceleration.front_conservative_deceleration_mps2 = -2.01;
  bool excessive_front_deceleration_rejected = false;
  try {
    FullLaneTrafficPredictor invalid_predictor(
        excessive_front_deceleration);
  } catch (const std::invalid_argument &) {
    excessive_front_deceleration_rejected = true;
  }
  Expect(excessive_front_deceleration_rejected,
         "front conservative braking below -2 m/s2 is rejected");
  FullLaneTrafficPredictor predictor(config);
  const FullLaneTrafficPredictionSnapshot prediction =
      predictor.Predict(tracking, 0.3);
  ExpectNear(prediction.required_coverage_s, 10.3, 1e-12,
             "prediction covers retained + planning + post horizon");
  Expect(prediction.grid_coverage_s + 1e-12 >=
             prediction.required_coverage_s,
         "uniform prediction grid reaches the required endpoint");
  Expect(prediction.trajectories.size() == 5,
         "front, rear and lateral motion generate all required hypotheses");

  const TrafficPredictionTrajectory &front_nominal = Trajectory(
      prediction, 1,
      TrafficPredictionHypothesis::kNominalConstantVelocity);
  const TrafficPredictionTrajectory &front_braking = Trajectory(
      prediction, 1,
      TrafficPredictionHypothesis::kFrontConservativeBraking);
  const TrafficPredictionTrajectory &rear_accelerating = Trajectory(
      prediction, 2,
      TrafficPredictionHypothesis::kRearConservativeAcceleration);
  const TrafficPredictionTrajectory &rear_nominal = Trajectory(
      prediction, 2,
      TrafficPredictionHypothesis::kNominalConstantVelocity);
  const TrafficPredictionTrajectory &lateral = Trajectory(
      prediction, 1,
      TrafficPredictionHypothesis::kLateralContinuation);
  Expect(front_nominal.occupancies.size() == 104 &&
             front_nominal.occupancies.back().prediction_time_s + 1e-12 >=
                 prediction.required_coverage_s,
         "every hypothesis uses the same complete time grid");
  Expect(front_braking.occupancies.back().road_s_unwrapped_m <
             front_nominal.occupancies.back().road_s_unwrapped_m,
         "front braking envelope is behind the nominal front trajectory");
  Expect(rear_accelerating.occupancies.back().road_s_unwrapped_m >
             rear_nominal.occupancies.back().road_s_unwrapped_m,
         "rear acceleration envelope closes faster than nominal");
  ExpectNear(rear_accelerating.occupancies[43].road_s_rate_mps, 16.0,
             1e-12,
             "rear acceleration hypothesis cruises after two seconds");
  Expect(lateral.occupancies[10].d_m > lateral.occupancies.front().d_m,
         "significant d rate produces a continuing lateral hypothesis");
  Expect(front_nominal.occupancies.back().longitudinal_uncertainty_m >
             front_nominal.occupancies.front().longitudinal_uncertainty_m,
         "longitudinal uncertainty grows with prediction time");
  ExpectNear(front_nominal.occupancies[43].longitudinal_uncertainty_m,
             1.575, 1e-12,
             "longitudinal uncertainty at 4.3 s is purely linear");
  ExpectNear(front_nominal.occupancies[43].longitudinal_uncertainty_m -
                 front_nominal.occupancies[42].longitudinal_uncertainty_m,
             0.025, 1e-12,
             "longitudinal uncertainty has no quadratic growth term");
  Expect(TrafficOccupiesLane(front_nominal.occupancies.front(), 0) &&
             !TrafficOccupiesLane(front_nominal.occupancies.front(), 1),
         "a centered vehicle starts in only its physical lane");
  Expect(TrafficOccupiesLane(lateral.occupancies[20], 0) &&
             TrafficOccupiesLane(lateral.occupancies[20], 1),
         "measured lateral motion activates the adjacent lane after physical intrusion");
  Expect(!TrafficOccupiesLane(rear_nominal.occupancies.front(), 0) &&
             TrafficOccupiesLane(rear_nominal.occupancies.front(), 1) &&
             !TrafficOccupiesLane(rear_nominal.occupancies.front(), 2),
         "a lane-centered vehicle without lateral motion occupies only its physical lane");
  Expect(prediction.safety_admissible_trajectory_count == 5,
         "prediction log preserves track safety-admission validity");
}

void TestLongitudinalAccelerationStopsAfterTwoSeconds() {
  TrafficTrackingSnapshot tracking;
  tracking.cycle = 43;
  TrackedVehicleState front;
  front.id = 10;
  front.road_s_unwrapped_m = 120.0;
  front.relative_road_s_m = 20.0;
  front.d_m = 2.0;
  front.road_s_rate_mps = 12.0;
  front.length_m = 4.8;
  front.width_m = 2.0;
  front.valid = true;
  front.safety_admissible = true;
  TrackedVehicleState rear = front;
  rear.id = 11;
  rear.road_s_unwrapped_m = 70.0;
  rear.relative_road_s_m = -30.0;
  rear.d_m = 6.0;
  rear.road_s_rate_mps = 15.0;
  TrackedVehicleState stopping_front = front;
  stopping_front.id = 12;
  stopping_front.road_s_unwrapped_m = 110.0;
  stopping_front.relative_road_s_m = 10.0;
  stopping_front.road_s_rate_mps = 1.0;
  tracking.tracks = {front, rear, stopping_front};
  tracking.valid_track_count = 3;
  tracking.safety_admissible_track_count = 3;

  FullLaneTrafficPredictionConfig config;
  config.planning_horizon_s = 3.0;
  config.post_maneuver_observation_s = 0.0;
  FullLaneTrafficPredictor predictor(config);
  const FullLaneTrafficPredictionSnapshot prediction =
      predictor.Predict(tracking, 0.0);
  const TrafficPredictionTrajectory &front_braking = Trajectory(
      prediction, 10,
      TrafficPredictionHypothesis::kFrontConservativeBraking);
  const TrafficPredictionTrajectory &rear_accelerating = Trajectory(
      prediction, 11,
      TrafficPredictionHypothesis::kRearConservativeAcceleration);
  const TrafficPredictionTrajectory &front_stopping = Trajectory(
      prediction, 12,
      TrafficPredictionHypothesis::kFrontConservativeBraking);

  ExpectNear(prediction.longitudinal_acceleration_duration_s, 2.0, 0.0,
             "prediction snapshot records the acceleration duration");
  ExpectNear(front_braking.occupancies[19].road_s_rate_mps, 10.1, 1e-12,
             "front braking remains active before two seconds");
  ExpectNear(front_braking.occupancies[19]
                 .longitudinal_acceleration_mps2,
             -1.0, 1e-12,
             "front braking reports acceleration before the boundary");
  ExpectNear(front_braking.occupancies[20].road_s_unwrapped_m, 142.0,
             1e-12,
             "front braking reaches a continuous two-second boundary");
  ExpectNear(front_braking.occupancies[20].road_s_rate_mps, 10.0, 1e-12,
             "front braking reaches its two-second terminal speed");
  ExpectNear(front_braking.occupancies[20]
                 .longitudinal_acceleration_mps2,
             0.0, 0.0,
             "front braking is disabled at the two-second boundary");
  ExpectNear(front_braking.occupancies[30].road_s_unwrapped_m, 152.0,
             1e-12,
             "front vehicle cruises after the braking window");
  ExpectNear(front_braking.occupancies[30].road_s_rate_mps, 10.0, 1e-12,
             "front speed stays constant after the braking window");

  ExpectNear(rear_accelerating.occupancies[19].road_s_rate_mps, 15.95,
             1e-12,
             "rear acceleration remains active before two seconds");
  ExpectNear(rear_accelerating.occupancies[20].road_s_unwrapped_m, 101.0,
             1e-12,
             "rear acceleration reaches a continuous two-second boundary");
  ExpectNear(rear_accelerating.occupancies[20].road_s_rate_mps, 16.0,
             1e-12,
             "rear acceleration reaches its two-second terminal speed");
  ExpectNear(rear_accelerating.occupancies[20]
                 .longitudinal_acceleration_mps2,
             0.0, 0.0,
             "rear acceleration is disabled at the two-second boundary");
  ExpectNear(rear_accelerating.occupancies[30].road_s_unwrapped_m, 117.0,
             1e-12,
             "rear vehicle cruises after the acceleration window");
  ExpectNear(rear_accelerating.occupancies[30].road_s_rate_mps, 16.0,
             1e-12,
             "rear speed stays constant after the acceleration window");
  ExpectNear(front_stopping.occupancies[10].road_s_unwrapped_m, 110.5,
             1e-12,
             "a low-speed front vehicle stops before the two-second limit");
  ExpectNear(front_stopping.occupancies[10].road_s_rate_mps, 0.0, 0.0,
             "a front vehicle does not acquire negative predicted speed");
  ExpectNear(front_stopping.occupancies[30].road_s_unwrapped_m, 110.5,
             1e-12,
             "a front vehicle stopped inside the braking window stays stopped");

  FullLaneTrafficPredictionConfig invalid_config = config;
  invalid_config.longitudinal_acceleration_duration_s = 0.0;
  bool invalid_duration_rejected = false;
  try {
    FullLaneTrafficPredictor invalid_predictor(invalid_config);
  } catch (const std::invalid_argument &) {
    invalid_duration_rejected = true;
  }
  Expect(invalid_duration_rejected,
         "non-positive longitudinal acceleration duration is rejected");
}

void TestTrackerTransactionAndControlReset() {
  const MapData map = LoadMap("data/highway_map.csv");
  TrafficTrackerConfig config;
  config.minimum_safety_track_age_s = 0.0;
  TrafficTracker tracker(config);
  TrafficTrackerState committed;
  const TrafficTrackingUpdate first = tracker.Evaluate(
      Snapshot(1, 100.0, {Vehicle(3, 110.0, 6.0, 10.0, 0.0, map)}),
      map, committed);
  Expect(!committed.initialized && committed.tracks.empty(),
         "Evaluate does not mutate the caller's committed tracker state");

  const PlanningSnapshot second_snapshot =
      Snapshot(2, 100.2,
               {Vehicle(3, 110.2, 6.0, 10.0, 0.0, map)});
  const TrafficTrackingUpdate second_a =
      tracker.Evaluate(second_snapshot, map, first.next_state);
  const TrafficTrackingUpdate second_b =
      tracker.Evaluate(second_snapshot, map, first.next_state);
  ExpectNear(Track(second_a.snapshot, 3).road_s_unwrapped_m,
             Track(second_b.snapshot, 3).road_s_unwrapped_m, 0.0,
             "discarding one evaluation leaves deterministic next state");
  Expect(Track(second_a.snapshot, 3).observation_count == 2,
         "only the explicitly supplied committed state advances history");

  PlanningSnapshot reset_snapshot =
      Snapshot(3, 100.4,
               {Vehicle(3, 110.4, 6.0, 10.0, 0.0, map)});
  reset_snapshot.reset_reason =
      PlanningStateResetReason::kHistoryPositionMismatch;
  const TrafficTrackingUpdate reset =
      tracker.Evaluate(reset_snapshot, map, second_a.next_state);
  Expect(reset.snapshot.reset_reason ==
             TrafficTrackerResetReason::kControlHistoryReset &&
             Track(reset.snapshot, 3).observation_count == 1,
         "control-history alignment failure resets behavior tracking");
}

void TestBoundedTrackerAndPredictionCapacity() {
  const MapData map = LoadMap("data/highway_map.csv");
  TrafficTrackerConfig tracker_config;
  tracker_config.minimum_safety_track_age_s = 0.0;
  tracker_config.maximum_tracks = 1;
  TrafficTracker tracker(tracker_config);
  const TrafficTrackingUpdate first = tracker.Evaluate(
      Snapshot(1, 100.0, {Vehicle(1, 110.0, 2.0, 10.0, 0.0, map)}),
      map, TrafficTrackerState());
  bool tracker_capacity_rejected = false;
  try {
    tracker.Evaluate(
        Snapshot(2, 100.2,
                 {Vehicle(2, 120.0, 6.0, 10.0, 0.0, map)}),
        map, first.next_state);
  } catch (const std::invalid_argument &) {
    tracker_capacity_rejected = true;
  }
  Expect(tracker_capacity_rejected && first.next_state.tracks.size() == 1,
         "retained plus new IDs cannot exceed the bounded tracker state");

  FullLaneTrafficPredictionConfig prediction_config;
  prediction_config.maximum_source_tracks = 1;
  FullLaneTrafficPredictor predictor(prediction_config);
  TrafficTrackingSnapshot oversized;
  oversized.cycle = 1;
  oversized.tracks.resize(2);
  bool prediction_capacity_rejected = false;
  try {
    predictor.Predict(oversized, 0.0);
  } catch (const std::invalid_argument &) {
    prediction_capacity_rejected = true;
  }
  Expect(prediction_capacity_rejected,
         "prediction storage rejects an unbounded source snapshot");
}

} // namespace

int main() {
  RunTest(TestLoopUnwrapAndRearRelativeCoordinate,
          "TestLoopUnwrapAndRearRelativeCoordinate");
  RunTest(TestVelocityProjectionAndBoundedAcceleration,
          "TestVelocityProjectionAndBoundedAcceleration");
  RunTest(TestDisappearReacquireReuseAndStaleness,
          "TestDisappearReacquireReuseAndStaleness");
  RunTest(TestTrackerUsesExecutedTrajectoryTime,
          "TestTrackerUsesExecutedTrajectoryTime");
  RunTest(TestPredictionHypothesesCoverageAndLaneOccupancy,
          "TestPredictionHypothesesCoverageAndLaneOccupancy");
  RunTest(TestLongitudinalAccelerationStopsAfterTwoSeconds,
          "TestLongitudinalAccelerationStopsAfterTwoSeconds");
  RunTest(TestTrackerTransactionAndControlReset,
          "TestTrackerTransactionAndControlReset");
  RunTest(TestBoundedTrackerAndPredictionCapacity,
          "TestBoundedTrackerAndPredictionCapacity");
  if (failures != 0) {
    std::cerr << failures << " P2.1 traffic assertion(s) failed"
              << std::endl;
    return 1;
  }
  std::cout << "All P2.1 traffic tests passed" << std::endl;
  return 0;
}
