#include "trajectory_validator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "map.h"

namespace {

constexpr double kMilesPerHourToMetersPerSecond = 0.44704;
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

struct Vector2d {
  Vector2d(double x_value = 0.0, double y_value = 0.0)
      : x(x_value), y(y_value) {}

  double x;
  double y;
};

struct OrientedBox {
  Vector2d center;
  Vector2d longitudinal_axis;
  Vector2d lateral_axis;
  double half_length_m = 0.0;
  double half_width_m = 0.0;
};

bool Finite(double value) { return std::isfinite(value); }

double Dot(const Vector2d &left, const Vector2d &right) {
  return left.x * right.x + left.y * right.y;
}

double CartesianLateralCoordinate(double x, double y,
                                  double road_s_unwrapped_m,
                                  const MapData &map) {
  const RoadGeometrySample center =
      EvaluateRoadGeometryOnValidatedMap(road_s_unwrapped_m, 0.0, map);
  const RoadGeometrySample unit_offset =
      EvaluateRoadGeometryOnValidatedMap(road_s_unwrapped_m, 1.0, map);
  const double normal_x = unit_offset.x - center.x;
  const double normal_y = unit_offset.y - center.y;
  const double normal_squared = normal_x * normal_x + normal_y * normal_y;
  if (!Finite(normal_squared) || normal_squared <= 1e-12) {
    throw std::runtime_error("validator road normal is degenerate");
  }
  return ((x - center.x) * normal_x + (y - center.y) * normal_y) /
         normal_squared;
}

OrientedBox MakeBox(double x, double y, double heading, double length,
                    double width, double longitudinal_inflation = 0.0,
                    double safety_inflation = 0.0) {
  OrientedBox box;
  box.center = {x, y};
  box.longitudinal_axis = {std::cos(heading), std::sin(heading)};
  box.lateral_axis = {-box.longitudinal_axis.y, box.longitudinal_axis.x};
  box.half_length_m = 0.5 * length + longitudinal_inflation + safety_inflation;
  box.half_width_m = 0.5 * width + safety_inflation;
  return box;
}

double ProjectionRadius(const OrientedBox &box, const Vector2d &axis) {
  return box.half_length_m * std::fabs(Dot(box.longitudinal_axis, axis)) +
         box.half_width_m * std::fabs(Dot(box.lateral_axis, axis));
}

double BoxSeparation(const OrientedBox &left, const OrientedBox &right) {
  const Vector2d center_delta = {right.center.x - left.center.x,
                                 right.center.y - left.center.y};
  const Vector2d axes[4] = {left.longitudinal_axis, left.lateral_axis,
                            right.longitudinal_axis, right.lateral_axis};
  double maximum_axis_separation = -std::numeric_limits<double>::infinity();
  for (const Vector2d &axis : axes) {
    const double separation =
        std::fabs(Dot(center_delta, axis)) - ProjectionRadius(left, axis) -
        ProjectionRadius(right, axis);
    maximum_axis_separation = std::max(maximum_axis_separation, separation);
  }
  return maximum_axis_separation;
}

struct VehicleRoadVelocity {
  double longitudinal_speed_mps = 0.0;
  double d_rate_mps = 0.0;
};

VehicleRoadVelocity VehicleVelocityInRoadFrame(
    const DetectedVehicle &vehicle, const MapData &map) {
  const RoadGeometrySample road =
      EvaluateRoadGeometryOnValidatedMap(vehicle.s, vehicle.d, map);
  const RoadGeometrySample center =
      EvaluateRoadGeometryOnValidatedMap(vehicle.s, 0.0, map);
  const RoadGeometrySample offset =
      EvaluateRoadGeometryOnValidatedMap(vehicle.s, 1.0, map);
  const double normal_x = offset.x - center.x;
  const double normal_y = offset.y - center.y;
  const double determinant = road.first_derivative_x * normal_y -
                             road.first_derivative_y * normal_x;
  if (!Finite(determinant) || std::fabs(determinant) <= 1e-10) {
    throw std::runtime_error("validator traffic basis is degenerate");
  }
  const double parameter_rate =
      (vehicle.vx_mps * normal_y - vehicle.vy_mps * normal_x) / determinant;
  VehicleRoadVelocity result;
  result.longitudinal_speed_mps =
      std::max(0.0, parameter_rate *
                        std::hypot(road.first_derivative_x,
                                   road.first_derivative_y));
  result.d_rate_mps =
      (road.first_derivative_x * vehicle.vy_mps -
       road.first_derivative_y * vehicle.vx_mps) /
      determinant;
  return result;
}

OrientedBox ObstacleBoxAtRoadParameter(double road_s, double d,
                                       double longitudinal_inflation,
                                       double lateral_inflation,
                                       const TrajectoryValidatorConfig &config,
                                       const MapData &map) {
  const RoadGeometrySample geometry =
      EvaluateRoadGeometryOnValidatedMap(road_s, d, map);
  const double heading =
      std::atan2(geometry.first_derivative_y, geometry.first_derivative_x);
  OrientedBox box =
      MakeBox(geometry.x, geometry.y, heading, config.obstacle_length_m,
              config.obstacle_width_m, longitudinal_inflation,
              0.5 * config.physical_collision_margin_m);
  box.half_width_m += lateral_inflation;
  return box;
}

struct ObstacleBoxTimeline {
  struct Pose {
    OrientedBox box;
    double road_s_m = 0.0;
    double d_m = 0.0;
    double speed_mps = 0.0;
  };

  double object_id = 0.0;
  DetectedVehicle observation;
  double observed_road_s_speed_mps = 0.0;
  double observed_d_rate_mps = 0.0;
  std::vector<Pose> endpoints;
  std::vector<Pose> midpoint_sweeps;
};

ObstacleBoxTimeline BuildObstacleBoxTimeline(
    const DetectedVehicle &vehicle, const FullTrajectory &trajectory,
    const TrajectoryValidatorConfig &config, const MapData &map) {
  ObstacleBoxTimeline timeline;
  timeline.object_id = vehicle.id;
  timeline.observation = vehicle;
  timeline.endpoints.reserve(trajectory.points.size());
  timeline.midpoint_sweeps.reserve(trajectory.points.size());

  // The measured Cartesian velocity and its Frenet projection are constant for
  // this prediction model.  Compute the projection once per obstacle rather
  // than once for every endpoint and midpoint.
  const VehicleRoadVelocity road_velocity =
      VehicleVelocityInRoadFrame(vehicle, map);
  const double speed = road_velocity.longitudinal_speed_mps;
  const double d_rate = road_velocity.d_rate_mps;
  timeline.observed_road_s_speed_mps = speed;
  timeline.observed_d_rate_mps = d_rate;
  const double maximum_time =
      trajectory.points.empty()
          ? 0.0
          : trajectory.points.back().time_from_telemetry_s;
  const RoadArcLengthIndex arc_index = BuildRoadArcLengthIndexOnValidatedMap(
      vehicle.s, speed * maximum_time, vehicle.d, map);

  if (speed <= 1e-12 && std::fabs(d_rate) <= 1e-12) {
    ObstacleBoxTimeline::Pose stationary;
    stationary.box =
        ObstacleBoxAtRoadParameter(vehicle.s, vehicle.d, 0.0, 0.0, config, map);
    stationary.road_s_m = vehicle.s;
    stationary.d_m = vehicle.d;
    stationary.speed_mps = 0.0;
    timeline.endpoints.assign(trajectory.points.size(), stationary);
    timeline.midpoint_sweeps.assign(trajectory.points.size(), stationary);
    return timeline;
  }

  double previous_time = 0.0;
  for (const TrajectoryPoint &point : trajectory.points) {
    const double endpoint_d = vehicle.d + d_rate * point.time_from_telemetry_s;
    const double endpoint_s =
        RoadParameterAtArcLength(arc_index,
                                 speed * point.time_from_telemetry_s);
    ObstacleBoxTimeline::Pose endpoint;
    endpoint.box = ObstacleBoxAtRoadParameter(endpoint_s, endpoint_d, 0.0, 0.0,
                                              config, map);
    endpoint.road_s_m = endpoint_s;
    endpoint.d_m = endpoint_d;
    endpoint.speed_mps = speed;
    timeline.endpoints.push_back(endpoint);

    const double midpoint_time =
        0.5 * (previous_time + point.time_from_telemetry_s);
    const double midpoint_s =
        RoadParameterAtArcLength(arc_index, speed * midpoint_time);
    const double midpoint_d = vehicle.d + d_rate * midpoint_time;
    const double longitudinal_interval_inflation =
        0.5 * speed * (point.time_from_telemetry_s - previous_time);
    const double lateral_interval_inflation =
        0.5 * std::fabs(d_rate) * (point.time_from_telemetry_s - previous_time);
    ObstacleBoxTimeline::Pose midpoint;
    midpoint.box = ObstacleBoxAtRoadParameter(
        midpoint_s, midpoint_d, longitudinal_interval_inflation,
        lateral_interval_inflation, config, map);
    midpoint.road_s_m = midpoint_s;
    midpoint.d_m = midpoint_d;
    midpoint.speed_mps = speed;
    timeline.midpoint_sweeps.push_back(midpoint);
    previous_time = point.time_from_telemetry_s;
  }
  return timeline;
}

Vector2d CanonicalPosition(const FullTrajectory &trajectory,
                           std::size_t index,
                           double retained_frame_offset_x,
                           double retained_frame_offset_y) {
  const TrajectoryPoint &point = trajectory.points[index];
  if (index < trajectory.retained_prefix_points && point.lateral.valid) {
    return {point.lateral.expected_x + retained_frame_offset_x,
            point.lateral.expected_y + retained_frame_offset_y};
  }
  return {point.x, point.y};
}

double PointHeading(const FullTrajectory &trajectory, std::size_t index,
                    const PlannerInput &input,
                    double retained_frame_offset_x,
                    double retained_frame_offset_y) {
  const LateralPathState &lateral = trajectory.points[index].lateral;
  if (lateral.valid && lateral.exact_tangent_valid &&
      Finite(lateral.tangent_x) && Finite(lateral.tangent_y) &&
      std::hypot(lateral.tangent_x, lateral.tangent_y) > 1e-8) {
    return std::atan2(lateral.tangent_y, lateral.tangent_x);
  }
  if (trajectory.points.size() == 1) {
    return input.ego.yaw_deg * kDegreesToRadians;
  }

  const Vector2d current = CanonicalPosition(
      trajectory, index, retained_frame_offset_x, retained_frame_offset_y);
  std::size_t lower = index;
  while (lower > 0) {
    --lower;
    const Vector2d candidate = CanonicalPosition(
        trajectory, lower, retained_frame_offset_x, retained_frame_offset_y);
    if (std::hypot(current.x - candidate.x, current.y - candidate.y) > 1e-6) {
      break;
    }
  }
  std::size_t upper = index;
  while (upper + 1 < trajectory.points.size()) {
    ++upper;
    const Vector2d candidate = CanonicalPosition(
        trajectory, upper, retained_frame_offset_x, retained_frame_offset_y);
    if (std::hypot(candidate.x - current.x, candidate.y - current.y) > 1e-6) {
      break;
    }
  }
  const bool has_lower = lower < index;
  const bool has_upper = upper > index;
  if (has_lower || has_upper) {
    const Vector2d from = CanonicalPosition(
        trajectory, has_lower ? lower : index, retained_frame_offset_x,
        retained_frame_offset_y);
    const Vector2d to = CanonicalPosition(
        trajectory, has_upper ? upper : index, retained_frame_offset_x,
        retained_frame_offset_y);
    if (std::hypot(to.x - from.x, to.y - from.y) > 1e-8) {
      return std::atan2(to.y - from.y, to.x - from.x);
    }
  }
  return input.ego.yaw_deg * kDegreesToRadians;
}

void AddViolationOnce(ValidationResult *result, ViolationType type,
                      std::size_t index, double time_s, double magnitude,
                      const std::string &detail, double object_id = 0.0) {
  if (result->HasViolation(type)) {
    return;
  }
  TrajectoryViolation violation;
  violation.type = type;
  violation.point_index = index;
  violation.time_from_telemetry_s = time_s;
  violation.magnitude = magnitude;
  violation.object_id = object_id;
  violation.detail = detail;
  result->violations.push_back(violation);
}

void AddCollisionViolation(ValidationResult *result, std::size_t index,
                           double time_s, double magnitude,
                           const std::string &detail, double object_id) {
  for (TrajectoryViolation &violation : result->violations) {
    if (violation.type == ViolationType::kCollision &&
        violation.object_id == object_id) {
      if (time_s < violation.time_from_telemetry_s) {
        violation.point_index = index;
        violation.time_from_telemetry_s = time_s;
        violation.magnitude = magnitude;
        violation.detail = detail;
      }
      return;
    }
  }
  TrajectoryViolation violation;
  violation.type = ViolationType::kCollision;
  violation.point_index = index;
  violation.time_from_telemetry_s = time_s;
  violation.magnitude = magnitude;
  violation.object_id = object_id;
  violation.detail = detail;
  result->violations.push_back(violation);
}

double AlignRoadS(double road_s_m, double reference_road_s_m,
                  double track_length_m) {
  return road_s_m +
         std::round((reference_road_s_m - road_s_m) / track_length_m) *
             track_length_m;
}

int LaneIndexAtCurrentD(double d_m,
                        const TrajectoryValidatorConfig &config) {
  const double road_width_m =
      static_cast<double>(config.lane_count) * config.lane_width_m;
  if (!Finite(d_m) || d_m < 0.0 || d_m >= road_width_m) {
    return -1;
  }
  return static_cast<int>(std::floor(d_m / config.lane_width_m));
}

bool IsShieldedSameLaneRearVehicle(
    const PlannerInput &input, const DetectedVehicle &vehicle,
    const TrajectoryValidatorConfig &config, const MapData &map) {
  const int ego_lane = LaneIndexAtCurrentD(input.ego.d, config);
  const int vehicle_lane = LaneIndexAtCurrentD(vehicle.d, config);
  if (ego_lane < 0 || vehicle_lane != ego_lane) {
    return false;
  }

  const double aligned_vehicle_s =
      AlignRoadS(vehicle.s, input.ego.s, map.track_length);
  const double relative_road_s_m = aligned_vehicle_s - input.ego.s;
  const double minimum_nonoverlap_center_distance_m =
      0.5 * (config.ego_length_m + config.obstacle_length_m) +
      config.physical_collision_margin_m + config.safety_tolerance_m;

  // Do not shield a vehicle whose body already touches/overlaps the ego body.
  // Once a same-lane vehicle is unambiguously behind in the current frame,
  // however, its entire future timeline is outside the hard collision gate.
  return relative_road_s_m <= -minimum_nonoverlap_center_distance_m;
}

void AddCollisionEvidence(ValidationResult *result,
                          const ObstacleBoxTimeline &timeline,
                          const ObstacleBoxTimeline::Pose &obstacle,
                          const OrientedBox &ego_box,
                          CollisionCheckKind check_kind,
                          std::size_t point_index, double time_s,
                          double separation_m, double ego_road_s_m,
                          double ego_d_m, double ego_speed_mps,
                          std::size_t retained_prefix_points,
                          double track_length_m) {
  CollisionEvidence evidence;
  evidence.object_id = timeline.object_id;
  evidence.check_kind = check_kind;
  evidence.point_index = point_index;
  evidence.time_from_telemetry_s = time_s;
  evidence.box_separation_m = separation_m;
  evidence.overlap_m = std::max(0.0, -separation_m);
  evidence.retained_prefix = point_index < retained_prefix_points;
  evidence.stitch_boundary = retained_prefix_points > 0 &&
                             point_index == retained_prefix_points;
  evidence.ego_x_m = ego_box.center.x;
  evidence.ego_y_m = ego_box.center.y;
  evidence.ego_road_s_m = ego_road_s_m;
  evidence.ego_d_m = ego_d_m;
  evidence.ego_speed_mps = ego_speed_mps;
  evidence.obstacle_x_m = obstacle.box.center.x;
  evidence.obstacle_y_m = obstacle.box.center.y;
  evidence.obstacle_road_s_m = AlignRoadS(
      obstacle.road_s_m, ego_road_s_m, track_length_m);
  evidence.obstacle_d_m = obstacle.d_m;
  evidence.obstacle_speed_mps = obstacle.speed_mps;
  evidence.relative_road_s_m =
      evidence.obstacle_road_s_m - evidence.ego_road_s_m;
  evidence.relative_d_m = evidence.obstacle_d_m - evidence.ego_d_m;
  evidence.closing_speed_mps =
      evidence.ego_speed_mps - evidence.obstacle_speed_mps;
  evidence.center_distance_m =
      std::hypot(evidence.obstacle_x_m - evidence.ego_x_m,
                 evidence.obstacle_y_m - evidence.ego_y_m);
  evidence.observed_x_m = timeline.observation.x;
  evidence.observed_y_m = timeline.observation.y;
  evidence.observed_road_s_m = timeline.observation.s;
  evidence.observed_d_m = timeline.observation.d;
  evidence.observed_vx_mps = timeline.observation.vx_mps;
  evidence.observed_vy_mps = timeline.observation.vy_mps;
  evidence.observed_speed_mps =
      std::hypot(timeline.observation.vx_mps,
                 timeline.observation.vy_mps);
  evidence.observed_road_s_speed_mps =
      timeline.observed_road_s_speed_mps;
  evidence.observed_d_rate_mps = timeline.observed_d_rate_mps;

  for (CollisionEvidence &existing : result->collision_evidence) {
    if (existing.object_id == evidence.object_id) {
      if (evidence.time_from_telemetry_s <
              existing.time_from_telemetry_s ||
          (evidence.time_from_telemetry_s ==
               existing.time_from_telemetry_s &&
           evidence.overlap_m > existing.overlap_m)) {
        existing = evidence;
      }
      return;
    }
  }
  result->collision_evidence.push_back(evidence);
}

void ValidateConfig(const TrajectoryValidatorConfig &config) {
  if (!Finite(config.time_step_s) || config.time_step_s <= 0.0 ||
      !Finite(config.maximum_speed_mps) || config.maximum_speed_mps <= 0.0 ||
      !Finite(config.minimum_acceleration_mps2) ||
      !Finite(config.maximum_acceleration_mps2) ||
      config.minimum_acceleration_mps2 >= config.maximum_acceleration_mps2 ||
      !Finite(config.maximum_jerk_mps3) || config.maximum_jerk_mps3 <= 0.0 ||
      !Finite(config.maximum_cartesian_acceleration_mps2) ||
      config.maximum_cartesian_acceleration_mps2 <= 0.0 ||
      !Finite(config.maximum_cartesian_jerk_mps3) ||
      config.maximum_cartesian_jerk_mps3 <= 0.0 ||
      config.speed_tolerance_mps < 0.0 ||
      config.acceleration_tolerance_mps2 < 0.0 ||
      config.jerk_tolerance_mps3 < 0.0 || config.safety_tolerance_m < 0.0 ||
      config.ego_length_m <= 0.0 || config.ego_width_m <= 0.0 ||
      config.obstacle_length_m <= 0.0 || config.obstacle_width_m <= 0.0 ||
      config.physical_collision_margin_m < 0.0 ||
      config.lane_boundary_margin_m < 0.0 || config.lane_width_m <= 0.0 ||
      config.lane_count <= 0) {
    throw std::invalid_argument("invalid trajectory validator configuration");
  }
}

} // namespace

ViolationType ValidationResult::FirstViolationType() const {
  if (violations.empty()) {
    return ViolationType::kNone;
  }
  const TrajectoryViolation *first = &violations.front();
  for (const TrajectoryViolation &violation : violations) {
    if (violation.time_from_telemetry_s < first->time_from_telemetry_s) {
      first = &violation;
    }
  }
  return first->type;
}

double ValidationResult::FirstViolationTimeS() const {
  if (violations.empty()) {
    return 0.0;
  }
  double first_time = violations.front().time_from_telemetry_s;
  for (const TrajectoryViolation &violation : violations) {
    first_time = std::min(first_time, violation.time_from_telemetry_s);
  }
  return first_time;
}

bool ValidationResult::HasViolation(ViolationType type) const {
  for (const TrajectoryViolation &violation : violations) {
    if (violation.type == type) {
      return true;
    }
  }
  return false;
}

bool ValidationResult::HasOnlyCollisionViolations() const {
  if (violations.empty()) {
    return false;
  }
  for (const TrajectoryViolation &violation : violations) {
    if (violation.type != ViolationType::kCollision) {
      return false;
    }
  }
  return true;
}

TrajectoryValidator::TrajectoryValidator(
    const TrajectoryValidatorConfig &config)
    : config_(config) {
  ValidateConfig(config_);
}

ValidationResult TrajectoryValidator::Validate(
    const TrajectoryValidationContext &context) const {
  ValidationResult result;
  result.minimum_road_margin_m = std::numeric_limits<double>::infinity();
  result.minimum_collision_margin_m =
      std::numeric_limits<double>::infinity();
  if (context.input == nullptr || context.map == nullptr ||
      context.trajectory == nullptr) {
    AddViolationOnce(&result, ViolationType::kStateMismatch, 0, 0.0, 0.0,
                     "validator context is incomplete");
    return result;
  }
  const PlannerInput &input = *context.input;
  const MapData &map = *context.map;
  const FullTrajectory &trajectory = *context.trajectory;
  std::string map_error;
  if (!ValidateMap(map, &map_error)) {
    AddViolationOnce(&result, ViolationType::kStateMismatch, 0, 0.0, 0.0,
                     "validator map is invalid: " + map_error);
    return result;
  }
  if (!Finite(input.ego.x) || !Finite(input.ego.y) ||
      !Finite(input.ego.s) || !Finite(input.ego.d) ||
      !Finite(input.ego.yaw_deg) || !Finite(input.ego.speed_mph) ||
      !Finite(input.end_path_s) || !Finite(input.end_path_d)) {
    AddViolationOnce(&result, ViolationType::kNonFinite, 0, 0.0, 0.0,
                     "ego telemetry is non-finite");
    return result;
  }
  for (double coordinate : input.previous_path_x) {
    if (!Finite(coordinate)) {
      AddViolationOnce(&result, ViolationType::kNonFinite, 0, 0.0, 0.0,
                       "retained x coordinate is non-finite");
      return result;
    }
  }
  for (double coordinate : input.previous_path_y) {
    if (!Finite(coordinate)) {
      AddViolationOnce(&result, ViolationType::kNonFinite, 0, 0.0, 0.0,
                       "retained y coordinate is non-finite");
      return result;
    }
  }
  for (const DetectedVehicle &vehicle : input.traffic) {
    if (!Finite(vehicle.id) || !Finite(vehicle.x) || !Finite(vehicle.y) ||
        !Finite(vehicle.vx_mps) || !Finite(vehicle.vy_mps) ||
        !Finite(vehicle.s) || !Finite(vehicle.d)) {
      AddViolationOnce(&result, ViolationType::kNonFinite, 0, 0.0, 0.0,
                       "traffic evidence is non-finite");
      return result;
    }
  }
  if (!Finite(trajectory.time_step_s) || trajectory.time_step_s <= 0.0 ||
      trajectory.points.empty() ||
      trajectory.retained_prefix_points > trajectory.points.size()) {
    AddViolationOnce(&result, ViolationType::kStateMismatch, 0, 0.0, 0.0,
                     "full trajectory shape is invalid");
    return result;
  }

  const double required_end_time =
      trajectory.planning_frontier_delay_s + trajectory.planning_horizon_s;
  const double actual_end_time = trajectory.points.back().time_from_telemetry_s;
  if (!Finite(context.prediction_coverage_s) ||
      context.prediction_coverage_s + 1e-9 < required_end_time ||
      actual_end_time + 1e-9 < required_end_time) {
    AddViolationOnce(&result, ViolationType::kPredictionCoverage,
                     trajectory.points.size() - 1, actual_end_time,
                     required_end_time -
                         std::min(context.prediction_coverage_s,
                                  actual_end_time),
                     "prediction or trajectory does not cover the horizon");
  }

  if (trajectory.retained_prefix_points != input.previous_path_x.size() ||
      input.previous_path_x.size() != input.previous_path_y.size()) {
    AddViolationOnce(&result, ViolationType::kStateMismatch, 0, 0.0, 0.0,
                     "retained prefix count does not match the snapshot");
  }

  const double telemetry_speed =
      std::max(0.0, input.ego.speed_mph * kMilesPerHourToMetersPerSecond);
  const double allowed_speed = std::max(config_.maximum_speed_mps,
                                        telemetry_speed);
  double retained_frame_offset_x = 0.0;
  double retained_frame_offset_y = 0.0;
  if (trajectory.retained_prefix_points > 0 &&
      trajectory.retained_prefix_points <= trajectory.points.size()) {
    const TrajectoryPoint &anchor =
        trajectory.points[trajectory.retained_prefix_points - 1];
    if (anchor.lateral.valid) {
      retained_frame_offset_x = anchor.x - anchor.lateral.expected_x;
      retained_frame_offset_y = anchor.y - anchor.lateral.expected_y;
    }
  }
  std::vector<Vector2d> kinematic_positions;
  kinematic_positions.reserve(trajectory.points.size());
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    const TrajectoryPoint &point = trajectory.points[index];
    const double expected_time =
        static_cast<double>(index + 1) * trajectory.time_step_s;
    if (!Finite(point.time_from_telemetry_s) || !Finite(point.x) ||
        !Finite(point.y) || !Finite(point.longitudinal.s) ||
        !Finite(point.longitudinal.v) || !Finite(point.longitudinal.a) ||
        !Finite(point.longitudinal.j) ||
        std::fabs(point.time_from_telemetry_s - expected_time) > 1e-8) {
      AddViolationOnce(&result, ViolationType::kNonFinite, index,
                       point.time_from_telemetry_s, 0.0,
                       "trajectory point is non-finite or has invalid time");
    }
    if (index < trajectory.retained_prefix_points) {
      if (index >= input.previous_path_x.size() ||
          std::hypot(point.x - input.previous_path_x[index],
                     point.y - input.previous_path_y[index]) > 1e-9 ||
          !point.retained_prefix) {
        AddViolationOnce(&result, ViolationType::kStitching, index,
                         point.time_from_telemetry_s, 0.0,
                         "retained prefix was modified");
      }
    } else if (point.retained_prefix) {
      AddViolationOnce(&result, ViolationType::kStateMismatch, index,
                       point.time_from_telemetry_s, 0.0,
                       "new point is marked as retained");
    }

    if (point.longitudinal.v < -config_.speed_tolerance_mps ||
        point.longitudinal.v > allowed_speed + config_.speed_tolerance_mps) {
      AddViolationOnce(&result, ViolationType::kSpeed, index,
                       point.time_from_telemetry_s,
                       std::max(-point.longitudinal.v,
                                point.longitudinal.v - allowed_speed),
                       "longitudinal speed bound is violated");
    }
    if (point.longitudinal.a <
            config_.minimum_acceleration_mps2 -
                config_.acceleration_tolerance_mps2 ||
        point.longitudinal.a >
            config_.maximum_acceleration_mps2 +
                config_.acceleration_tolerance_mps2) {
      AddViolationOnce(&result, ViolationType::kAcceleration, index,
                       point.time_from_telemetry_s,
                       std::max(config_.minimum_acceleration_mps2 -
                                    point.longitudinal.a,
                                point.longitudinal.a -
                                    config_.maximum_acceleration_mps2),
                       "longitudinal acceleration bound is violated");
    }
    if (std::fabs(point.longitudinal.j) >
        config_.maximum_jerk_mps3 + config_.jerk_tolerance_mps3) {
      AddViolationOnce(&result, ViolationType::kJerk, index,
                       point.time_from_telemetry_s,
                       std::fabs(point.longitudinal.j) -
                           config_.maximum_jerk_mps3,
                       "longitudinal jerk bound is violated");
    }
    result.maximum_speed_mps =
        std::max(result.maximum_speed_mps, std::fabs(point.longitudinal.v));

    const bool translated_retained =
        index < trajectory.retained_prefix_points && point.lateral.valid;
    const double validation_x =
        translated_retained
            ? point.lateral.expected_x + retained_frame_offset_x
            : point.x;
    const double validation_y =
        translated_retained
            ? point.lateral.expected_y + retained_frame_offset_y
            : point.y;
    kinematic_positions.push_back({validation_x, validation_y});

    if (!point.lateral.valid ||
        !Finite(point.lateral.correction_progress_m) ||
        !Finite(point.lateral.residual_correction_progress_m) ||
        !Finite(point.lateral.road_parameter_s) ||
        !Finite(point.lateral.planned_d) ||
        !Finite(point.lateral.expected_x) ||
        !Finite(point.lateral.expected_y)) {
      AddViolationOnce(&result, ViolationType::kStateMismatch, index,
                       point.time_from_telemetry_s, 0.0,
                       "trajectory point has no exact lateral state");
      continue;
    }
    if ((!point.retained_prefix || !trajectory.exact_retained_state) &&
        std::hypot(point.x - point.lateral.expected_x,
                   point.y - point.lateral.expected_y) > 1e-6) {
      AddViolationOnce(&result, ViolationType::kStateMismatch, index,
                       point.time_from_telemetry_s,
                       std::hypot(point.x - point.lateral.expected_x,
                                  point.y - point.lateral.expected_y),
                       "Cartesian point disagrees with its lateral state");
    }
    const double heading = PointHeading(
        trajectory, index, input, retained_frame_offset_x,
        retained_frame_offset_y);
    const double actual_d = CartesianLateralCoordinate(
        validation_x, validation_y, point.lateral.road_parameter_s, map);
    const RoadGeometrySample road = EvaluateRoadGeometryOnValidatedMap(
        point.lateral.road_parameter_s, actual_d, map);
    const double road_heading =
        std::atan2(road.first_derivative_y, road.first_derivative_x);
    const double heading_delta = heading - road_heading;
    const double lateral_extent =
        0.5 * config_.ego_width_m * std::fabs(std::cos(heading_delta)) +
        0.5 * config_.ego_length_m * std::fabs(std::sin(heading_delta));
    const double lower_margin =
        actual_d - lateral_extent - config_.lane_boundary_margin_m;
    const double upper_margin =
        static_cast<double>(config_.lane_count) * config_.lane_width_m -
        actual_d - lateral_extent - config_.lane_boundary_margin_m;
    const double road_margin = std::min(lower_margin, upper_margin);
    result.minimum_road_margin_m =
        std::min(result.minimum_road_margin_m, road_margin);
    if (road_margin < -config_.safety_tolerance_m) {
      AddViolationOnce(&result, ViolationType::kRoadBoundary, index,
                       point.time_from_telemetry_s, -road_margin,
                       "ego body crosses the drivable road boundary");
    }
  }

  if (trajectory.kinematic_seed_x.size() !=
      trajectory.kinematic_seed_y.size()) {
    AddViolationOnce(&result, ViolationType::kStateMismatch, 0, 0.0, 0.0,
                     "kinematic seed arrays have different lengths");
  }
  std::vector<Vector2d> all_kinematic_positions;
  const std::size_t seed_count =
      std::min(trajectory.kinematic_seed_x.size(),
               trajectory.kinematic_seed_y.size());
  all_kinematic_positions.reserve(seed_count + kinematic_positions.size());
  for (std::size_t index = 0; index < seed_count; ++index) {
    if (!Finite(trajectory.kinematic_seed_x[index]) ||
        !Finite(trajectory.kinematic_seed_y[index])) {
      AddViolationOnce(&result, ViolationType::kNonFinite, 0, 0.0, 0.0,
                       "kinematic seed is non-finite");
      continue;
    }
    all_kinematic_positions.push_back(
        {trajectory.kinematic_seed_x[index] + retained_frame_offset_x,
         trajectory.kinematic_seed_y[index] + retained_frame_offset_y});
  }
  all_kinematic_positions.insert(all_kinematic_positions.end(),
                                 kinematic_positions.begin(),
                                 kinematic_positions.end());

  Vector2d previous_position = {input.ego.x, input.ego.y};
  Vector2d previous_velocity =
      {telemetry_speed * std::cos(input.ego.yaw_deg * kDegreesToRadians),
       telemetry_speed * std::sin(input.ego.yaw_deg * kDegreesToRadians)};
  std::size_t first_position_index = 0;
  if (seed_count >= 2 && all_kinematic_positions.size() >= 2) {
    previous_position = all_kinematic_positions[0];
    previous_velocity =
        {(all_kinematic_positions[1].x - all_kinematic_positions[0].x) /
             trajectory.time_step_s,
         (all_kinematic_positions[1].y - all_kinematic_positions[0].y) /
             trajectory.time_step_s};
    first_position_index = 1;
  }
  Vector2d previous_acceleration;
  std::size_t acceleration_sample_count = 0;
  for (std::size_t combined_index = first_position_index;
       combined_index < all_kinematic_positions.size(); ++combined_index) {
    const Vector2d position = all_kinematic_positions[combined_index];
    const Vector2d velocity =
        {(position.x - previous_position.x) / trajectory.time_step_s,
         (position.y - previous_position.y) / trajectory.time_step_s};
    const double velocity_magnitude = std::hypot(velocity.x, velocity.y);
    const Vector2d acceleration =
        {(velocity.x - previous_velocity.x) / trajectory.time_step_s,
         (velocity.y - previous_velocity.y) / trajectory.time_step_s};
    const double acceleration_magnitude =
        std::hypot(acceleration.x, acceleration.y);
    const bool is_trajectory_point = combined_index >= seed_count;
    const std::size_t trajectory_index =
        is_trajectory_point ? combined_index - seed_count : 0;
    const bool validate_cartesian_dynamics =
        is_trajectory_point &&
        (!trajectory.exact_retained_state ||
         trajectory_index >= trajectory.retained_prefix_points);
    if (validate_cartesian_dynamics) {
      result.maximum_speed_mps =
          std::max(result.maximum_speed_mps, velocity_magnitude);
      if (velocity_magnitude >
          allowed_speed + config_.speed_tolerance_mps) {
        AddViolationOnce(
            &result, ViolationType::kSpeed, trajectory_index,
            trajectory.points[trajectory_index].time_from_telemetry_s,
            velocity_magnitude - allowed_speed,
            "Cartesian path speed bound is violated");
      }
      result.maximum_acceleration_mps2 =
          std::max(result.maximum_acceleration_mps2,
                   acceleration_magnitude);
      if (acceleration_magnitude >
          config_.maximum_cartesian_acceleration_mps2 +
              config_.acceleration_tolerance_mps2) {
        AddViolationOnce(
            &result, ViolationType::kAcceleration, trajectory_index,
            trajectory.points[trajectory_index].time_from_telemetry_s,
            acceleration_magnitude -
                config_.maximum_cartesian_acceleration_mps2,
            "Cartesian total acceleration bound is violated");
      }
    }
    if (acceleration_sample_count >= 2) {
      const double jerk =
          std::hypot(acceleration.x - previous_acceleration.x,
                     acceleration.y - previous_acceleration.y) /
          trajectory.time_step_s;
      if (validate_cartesian_dynamics) {
        result.maximum_jerk_mps3 =
            std::max(result.maximum_jerk_mps3, jerk);
        if (jerk > config_.maximum_cartesian_jerk_mps3 +
                       config_.jerk_tolerance_mps3) {
          std::ostringstream detail;
          detail << "Cartesian total jerk bound is violated; acceleration "
                 << "changed from (" << previous_acceleration.x << ", "
                 << previous_acceleration.y << ") to (" << acceleration.x
                 << ", " << acceleration.y << ") m/s^2";
          AddViolationOnce(
              &result, ViolationType::kJerk, trajectory_index,
              trajectory.points[trajectory_index].time_from_telemetry_s,
              jerk - config_.maximum_cartesian_jerk_mps3,
              detail.str());
        }
      }
    }
    previous_position = position;
    previous_velocity = velocity;
    previous_acceleration = acceleration;
    ++acceleration_sample_count;
  }

  std::vector<ObstacleBoxTimeline> obstacle_timelines;
  obstacle_timelines.reserve(input.traffic.size());
  for (const DetectedVehicle &vehicle : input.traffic) {
    if (IsShieldedSameLaneRearVehicle(input, vehicle, config_, map)) {
      result.shielded_same_lane_rear_vehicle_ids.push_back(vehicle.id);
      continue;
    }
    obstacle_timelines.push_back(
        BuildObstacleBoxTimeline(vehicle, trajectory, config_, map));
  }

  OrientedBox previous_ego = MakeBox(
      input.ego.x, input.ego.y, input.ego.yaw_deg * kDegreesToRadians,
      config_.ego_length_m, config_.ego_width_m, 0.0,
      0.5 * config_.physical_collision_margin_m);
  double previous_time = 0.0;
  double previous_ego_road_s_m = input.ego.s;
  double previous_ego_d_m = input.ego.d;
  double previous_ego_speed_mps = telemetry_speed;
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    const TrajectoryPoint &point = trajectory.points[index];
    const double heading = PointHeading(
        trajectory, index, input, retained_frame_offset_x,
        retained_frame_offset_y);
    const OrientedBox ego =
        MakeBox(point.x, point.y, heading, config_.ego_length_m,
                config_.ego_width_m, 0.0,
                0.5 * config_.physical_collision_margin_m);
    const double ego_road_s_m = point.lateral.valid
                                    ? point.lateral.road_parameter_s
                                    : previous_ego_road_s_m;
    const double ego_d_m =
        point.lateral.valid ? point.lateral.planned_d : previous_ego_d_m;
    for (const ObstacleBoxTimeline &timeline : obstacle_timelines) {
      const ObstacleBoxTimeline::Pose &obstacle = timeline.endpoints[index];
      const double separation = BoxSeparation(ego, obstacle.box);
      result.minimum_collision_margin_m =
          std::min(result.minimum_collision_margin_m, separation);
      if (separation < -config_.safety_tolerance_m) {
        AddCollisionViolation(&result, index, point.time_from_telemetry_s,
                              -separation,
                              "ego and obstacle rectangles overlap",
                              timeline.object_id);
        AddCollisionEvidence(
            &result, timeline, obstacle, ego, CollisionCheckKind::kEndpoint,
            index, point.time_from_telemetry_s, separation, ego_road_s_m,
            ego_d_m, point.longitudinal.v,
            trajectory.retained_prefix_points, map.track_length);
      }

      const double midpoint_time =
          0.5 * (previous_time + point.time_from_telemetry_s);
      const double ego_sweep =
          0.5 * std::hypot(ego.center.x - previous_ego.center.x,
                           ego.center.y - previous_ego.center.y);
      const ObstacleBoxTimeline::Pose &midpoint_obstacle =
          timeline.midpoint_sweeps[index];
      const double midpoint_heading = std::atan2(
          ego.center.y - previous_ego.center.y,
          ego.center.x - previous_ego.center.x);
      const OrientedBox midpoint_ego = MakeBox(
          0.5 * (ego.center.x + previous_ego.center.x),
          0.5 * (ego.center.y + previous_ego.center.y), midpoint_heading,
          config_.ego_length_m, config_.ego_width_m, ego_sweep,
          0.5 * config_.physical_collision_margin_m);
      const double swept_separation =
          BoxSeparation(midpoint_ego, midpoint_obstacle.box);
      result.minimum_collision_margin_m =
          std::min(result.minimum_collision_margin_m, swept_separation);
      if (swept_separation < -config_.safety_tolerance_m) {
        AddCollisionViolation(
            &result, index, midpoint_time, -swept_separation,
            "swept ego and obstacle rectangles overlap", timeline.object_id);
        AddCollisionEvidence(
            &result, timeline, midpoint_obstacle, midpoint_ego,
            CollisionCheckKind::kSweptInterval, index, midpoint_time,
            swept_separation,
            0.5 * (previous_ego_road_s_m + ego_road_s_m),
            0.5 * (previous_ego_d_m + ego_d_m),
            0.5 * (previous_ego_speed_mps + point.longitudinal.v),
            trajectory.retained_prefix_points, map.track_length);
      }
    }
    previous_ego = ego;
    previous_time = point.time_from_telemetry_s;
    previous_ego_road_s_m = ego_road_s_m;
    previous_ego_d_m = ego_d_m;
    previous_ego_speed_mps = point.longitudinal.v;
  }

  if (!Finite(result.minimum_road_margin_m)) {
    result.minimum_road_margin_m = 0.0;
  }
  if (!Finite(result.minimum_collision_margin_m)) {
    result.minimum_collision_margin_m =
        std::numeric_limits<double>::infinity();
  }
  std::stable_sort(
      result.violations.begin(), result.violations.end(),
      [](const TrajectoryViolation &left,
         const TrajectoryViolation &right) {
        return left.time_from_telemetry_s < right.time_from_telemetry_s;
      });
  std::stable_sort(
      result.collision_evidence.begin(), result.collision_evidence.end(),
      [](const CollisionEvidence &left, const CollisionEvidence &right) {
        return left.time_from_telemetry_s < right.time_from_telemetry_s;
      });
  result.valid = result.violations.empty();
  return result;
}

const char *ViolationTypeName(ViolationType type) {
  switch (type) {
  case ViolationType::kNone:
    return "None";
  case ViolationType::kNonFinite:
    return "NonFinite";
  case ViolationType::kSpeed:
    return "Speed";
  case ViolationType::kAcceleration:
    return "Acceleration";
  case ViolationType::kJerk:
    return "Jerk";
  case ViolationType::kRoadBoundary:
    return "RoadBoundary";
  case ViolationType::kCollision:
    return "Collision";
  case ViolationType::kPredictionCoverage:
    return "PredictionCoverage";
  case ViolationType::kStitching:
    return "Stitching";
  case ViolationType::kStateMismatch:
    return "StateMismatch";
  }
  return "Unknown";
}

const char *CollisionCheckKindName(CollisionCheckKind kind) {
  switch (kind) {
  case CollisionCheckKind::kEndpoint:
    return "Endpoint";
  case CollisionCheckKind::kSweptInterval:
    return "SweptInterval";
  }
  return "Unknown";
}
