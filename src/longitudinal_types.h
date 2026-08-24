#ifndef LONGITUDINAL_TYPES_H
#define LONGITUDINAL_TYPES_H

#include <cstddef>
#include <string>
#include <vector>

struct LongitudinalState {
  double s = 0.0;
  double v = 0.0;
  double a = 0.0;
  double j = 0.0;
};

struct PredictedObstacle {
  double id = 0.0;
  double relative_s = 0.0;
  double speed_mps = 0.0;
  double d = 0.0;
  // Empty keeps the legacy meaning: the obstacle is collision-relevant at
  // every QP node. Traffic prediction fills one entry per node from the
  // vehicle's current contour; lateral occupancy is not extrapolated.
  std::vector<unsigned char> hard_collision_active;
  // Per-node speed cap caused only by a currently intruding adjacent-vehicle
  // contour. The current-frame cap is held constant over the planning horizon;
  // empty means that this obstacle contributes no intrusion speed cap.
  std::vector<double> intrusion_speed_limit_mps;
};

struct SpeedLimitEvent {
  double conflict_time_seconds = 0.0;
  double speed_limit_mps = 0.0;
};

struct LongitudinalTrajectory {
  double time_step_seconds = 0.0;
  std::vector<LongitudinalState> states;
  bool emergency = false;
};

struct LongitudinalQpConfig {
  std::size_t horizon_steps = 80;
  double time_step_seconds = 0.1;
  double maximum_speed_mps = 22.12848; // 49.5 mph
  double minimum_acceleration_mps2 = -5.0;
  double maximum_acceleration_mps2 = 3.0;
  double maximum_jerk_mps3 = 8.0;
  double time_headway_seconds = 1.5;
  double standstill_gap_meters = 5.0;
  double ego_length_meters = 4.8;
  double obstacle_length_meters = 4.8;
  double prediction_margin_meters = 1.0;
  double headway_slack_weight = 200.0;
  double speed_weight = 8.0;
  double acceleration_weight = 0.4;
  double jerk_weight = 0.08;
  double initial_jerk_continuity_weight = 0.4;
  double terminal_speed_weight = 20.0;
};

struct LongitudinalQpInput {
  double initial_speed_mps = 0.0;
  double initial_acceleration_mps2 = 0.0;
  bool initial_jerk_valid = false;
  double initial_jerk_mps3 = 0.0;
  std::vector<double> reference_speed_mps;
  std::vector<PredictedObstacle> obstacles;
  bool emergency_stop = false;
};

struct LongitudinalQpResult {
  bool success = false;
  std::string status;
  double objective = 0.0;
  double maximum_headway_slack_meters = 0.0;
  LongitudinalTrajectory trajectory;
};

struct TrafficPredictionConfig {
  double simulator_time_step_seconds = 0.02;
  std::size_t horizon_steps = 80;
  double maximum_speed_mps = 22.12848;
  double lane_width_meters = 4.0;
  double lane_boundary_margin_meters = 0.35;
  double ego_width_meters = 2.0;
  double obstacle_width_meters = 2.0;
  double lookahead_distance_meters = 250.0;
};

struct SpeedReferenceConfig {
  std::size_t horizon_steps = 80;
  double time_step_seconds = 0.1;
  double maximum_speed_mps = 22.12848;
  double minimum_acceleration_mps2 = -2.5;
  double maximum_jerk_mps3 = 2.0;
  double time_headway_seconds = 1.5;
  // Converts positive following-distance surplus into a gradual closing-speed
  // allowance above the lead vehicle speed.
  double gap_closing_time_seconds = 6.0;
  double standstill_gap_meters = 5.0;
  double ego_length_meters = 4.8;
  double obstacle_length_meters = 4.8;
  double prediction_margin_meters = 1.0;
  double speed_weight = 20.0;
  double acceleration_weight = 0.2;
  double jerk_weight = 0.05;
};

struct SpeedReferenceResult {
  bool success = false;
  std::string status;
  std::vector<SpeedLimitEvent> events;
  std::vector<double> raw_speed_limits_mps;
  LongitudinalTrajectory trajectory;
};

#endif // LONGITUDINAL_TYPES_H
