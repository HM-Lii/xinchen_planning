#ifndef TRAFFIC_PREDICTION_H
#define TRAFFIC_PREDICTION_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "traffic_tracker.h"

struct FullLaneTrafficPredictionConfig {
  double time_step_s = 0.10;
  double planning_horizon_s = 8.0;
  double post_maneuver_observation_s = 2.0;
  // Runtime validation limits braking magnitude to 2 m/s2.
  double front_conservative_deceleration_mps2 = -1.0;
  // Runtime validation caps this hypothesis at 1 m/s2.
  double rear_conservative_acceleration_mps2 = 0.5;
  // Conservative longitudinal acceleration hypotheses are applied only for
  // this duration; the speed reached at the boundary is then held constant.
  double longitudinal_acceleration_duration_s = 2.0;
  double maximum_predicted_speed_mps = 55.0;
  double lateral_motion_threshold_mps = 0.20;
  double lateral_continuation_duration_s = 2.0;
  double maximum_abs_lateral_prediction_rate_mps = 3.0;
  // Prediction uncertainty grows linearly; there is deliberately no t^2
  // acceleration-uncertainty term.
  double longitudinal_uncertainty_growth_mps = 0.25;
  double lane_width_m = 4.0;
  int lane_count = 3;
  std::size_t maximum_source_tracks = 128;
  std::size_t maximum_prediction_nodes = 256;
};

enum class TrafficPredictionHypothesis {
  kNominalConstantVelocity,
  kFrontConservativeBraking,
  kRearConservativeAcceleration,
  kLateralContinuation
};

struct PredictedTrafficOccupancy {
  double prediction_time_s = 0.0;
  double road_s_unwrapped_m = 0.0;
  double d_m = 0.0;
  double road_s_rate_mps = 0.0;
  double d_rate_mps = 0.0;
  double longitudinal_acceleration_mps2 = 0.0;
  double longitudinal_uncertainty_m = 0.0;
  double occupied_road_s_min_m = 0.0;
  double occupied_road_s_max_m = 0.0;
  double occupied_d_min_m = 0.0;
  double occupied_d_max_m = 0.0;
  // Lane i is occupied when bit i is set. P2.1 limits lane_count to 63.
  std::uint64_t occupied_lane_mask = 0;
};

struct TrafficPredictionTrajectory {
  int vehicle_id = 0;
  TrafficPredictionHypothesis hypothesis =
      TrafficPredictionHypothesis::kNominalConstantVelocity;
  bool source_track_valid = false;
  bool safety_admissible = false;
  double source_relative_road_s_m = 0.0;
  double vehicle_length_m = 0.0;
  double vehicle_width_m = 0.0;
  std::vector<PredictedTrafficOccupancy> occupancies;
};

struct FullLaneTrafficPredictionSnapshot {
  std::uint64_t cycle = 0;
  double time_step_s = 0.0;
  double retained_prefix_duration_s = 0.0;
  double planning_horizon_s = 0.0;
  double post_maneuver_observation_s = 0.0;
  double longitudinal_acceleration_duration_s = 0.0;
  double required_coverage_s = 0.0;
  double grid_coverage_s = 0.0;
  std::size_t source_track_count = 0;
  std::size_t safety_admissible_track_count = 0;
  std::size_t safety_admissible_trajectory_count = 0;
  std::vector<TrafficPredictionTrajectory> trajectories;
};

const char *TrafficPredictionHypothesisName(
    TrafficPredictionHypothesis hypothesis);
bool TrafficOccupiesLane(const PredictedTrafficOccupancy &occupancy,
                         int lane);

class FullLaneTrafficPredictor {
public:
  explicit FullLaneTrafficPredictor(
      const FullLaneTrafficPredictionConfig &config);

  FullLaneTrafficPredictionSnapshot
  Predict(const TrafficTrackingSnapshot &tracking,
          double retained_prefix_duration_s) const;

private:
  FullLaneTrafficPredictionConfig config_;
};

#endif // TRAFFIC_PREDICTION_H
