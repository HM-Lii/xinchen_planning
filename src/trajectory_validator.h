#ifndef TRAJECTORY_VALIDATOR_H
#define TRAJECTORY_VALIDATOR_H

#include <cstddef>
#include <string>
#include <vector>

#include "longitudinal_types.h"
#include "path_stitcher.h"
#include "planner_types.h"

enum class ViolationType {
  kNone,
  kNonFinite,
  kSpeed,
  kAcceleration,
  kJerk,
  kRoadBoundary,
  kCollision,
  kPredictionCoverage,
  kStitching,
  kStateMismatch
};

struct TrajectoryPoint {
  double time_from_telemetry_s = 0.0;
  double x = 0.0;
  double y = 0.0;
  LongitudinalState longitudinal;
  LateralPathState lateral;
  bool retained_prefix = false;
};

struct FullTrajectory {
  double time_step_s = 0.02;
  std::vector<TrajectoryPoint> points;
  std::size_t retained_prefix_points = 0;
  double planning_frontier_delay_s = 0.0;
  double planning_horizon_s = 0.0;
  bool exact_retained_state = false;
  std::vector<double> kinematic_seed_x;
  std::vector<double> kinematic_seed_y;
};

struct TrajectoryViolation {
  ViolationType type = ViolationType::kNone;
  std::size_t point_index = 0;
  double time_from_telemetry_s = 0.0;
  double magnitude = 0.0;
  double object_id = 0.0;
  std::string detail;
};

enum class CollisionCheckKind {
  kEndpoint,
  kSweptInterval
};

// Evidence captured at the first predicted body overlap for one traffic
// object.  This is deliberately richer than TrajectoryViolation so an online
// log can distinguish a same-lane longitudinal conflict from an adjacent-lane
// or retained-prefix geometry conflict without replaying the simulator.
struct CollisionEvidence {
  double object_id = 0.0;
  CollisionCheckKind check_kind = CollisionCheckKind::kEndpoint;
  std::size_t point_index = 0;
  double time_from_telemetry_s = 0.0;
  double box_separation_m = 0.0;
  double overlap_m = 0.0;
  bool retained_prefix = false;
  bool stitch_boundary = false;

  double ego_x_m = 0.0;
  double ego_y_m = 0.0;
  double ego_road_s_m = 0.0;
  double ego_d_m = 0.0;
  double ego_speed_mps = 0.0;

  double obstacle_x_m = 0.0;
  double obstacle_y_m = 0.0;
  double obstacle_road_s_m = 0.0;
  double obstacle_d_m = 0.0;
  double obstacle_speed_mps = 0.0;
  double relative_road_s_m = 0.0;
  double relative_d_m = 0.0;
  double closing_speed_mps = 0.0;
  double center_distance_m = 0.0;

  // Original sensor-fusion evidence, before road-following prediction.
  double observed_x_m = 0.0;
  double observed_y_m = 0.0;
  double observed_road_s_m = 0.0;
  double observed_d_m = 0.0;
  double observed_vx_mps = 0.0;
  double observed_vy_mps = 0.0;
  double observed_speed_mps = 0.0;
  double observed_road_s_speed_mps = 0.0;
  double observed_d_rate_mps = 0.0;
};

struct ValidationResult {
  bool valid = false;
  std::vector<TrajectoryViolation> violations;
  double minimum_road_margin_m = 0.0;
  double minimum_collision_margin_m = 0.0;
  double maximum_speed_mps = 0.0;
  double maximum_acceleration_mps2 = 0.0;
  double maximum_jerk_mps3 = 0.0;
  std::vector<CollisionEvidence> collision_evidence;
  // Same-lane traffic whose body is fully behind the ego body in the current
  // telemetry frame is intentionally excluded from the hard collision
  // timeline. Keep the IDs so this control policy remains observable.
  std::vector<double> shielded_same_lane_rear_vehicle_ids;

  ViolationType FirstViolationType() const;
  double FirstViolationTimeS() const;
  bool HasViolation(ViolationType type) const;
  bool HasOnlyCollisionViolations() const;
};

struct TrajectoryValidatorConfig {
  double time_step_s = 0.02;
  double maximum_speed_mps = 22.12848;
  double minimum_acceleration_mps2 = -5.0;
  double maximum_acceleration_mps2 = 3.0;
  double maximum_jerk_mps3 = 8.0;
  double maximum_cartesian_acceleration_mps2 = 10.0;
  double maximum_cartesian_jerk_mps3 = 10.0;
  double speed_tolerance_mps = 0.05;
  double acceleration_tolerance_mps2 = 0.05;
  double jerk_tolerance_mps3 = 0.05;
  double safety_tolerance_m = 0.05;
  double ego_length_m = 4.8;
  double ego_width_m = 2.0;
  double obstacle_length_m = 4.8;
  double obstacle_width_m = 2.0;
  double physical_collision_margin_m = 0.0;
  double lane_boundary_margin_m = 0.35;
  double lane_width_m = 4.0;
  int lane_count = 3;
};

struct TrajectoryValidationContext {
  const PlannerInput *input = nullptr;
  const MapData *map = nullptr;
  const FullTrajectory *trajectory = nullptr;
  double prediction_coverage_s = 0.0;
};

class TrajectoryValidator {
public:
  explicit TrajectoryValidator(
      const TrajectoryValidatorConfig &config = TrajectoryValidatorConfig());

  ValidationResult
  Validate(const TrajectoryValidationContext &context) const;

private:
  TrajectoryValidatorConfig config_;
};

const char *ViolationTypeName(ViolationType type);
const char *CollisionCheckKindName(CollisionCheckKind kind);

#endif // TRAJECTORY_VALIDATOR_H
