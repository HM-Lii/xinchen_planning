#include "spatial_path.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "map.h"
#include "planning_snapshot.h"

namespace {

constexpr double kNumericalTolerance = 1e-10;
constexpr double kMinimumTangentMagnitude = 1e-8;

bool Finite(double value) { return std::isfinite(value); }

double Clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

void AddReason(SpatialPathPrecheckResult *result,
               SpatialPathPrecheckReason reason) {
  if (std::find(result->rejection_reasons.begin(),
                result->rejection_reasons.end(),
                reason) == result->rejection_reasons.end()) {
    result->rejection_reasons.push_back(reason);
  }
}

void ValidateConfig(const SpatialPathPlannerConfig &config) {
  const bool finite =
      Finite(config.planning_horizon_s) &&
      Finite(config.post_maneuver_observation_s) &&
      Finite(config.nominal_transition_duration_s) &&
      Finite(config.minimum_transition_length_m) &&
      Finite(config.maximum_transition_length_m) &&
      Finite(config.maximum_path_extent_m) &&
      Finite(config.geometry_sample_step_m) &&
      Finite(config.committed_continuation_horizon_s) &&
      Finite(config.speed_upper_bound_mps) &&
      Finite(config.maximum_abs_longitudinal_acceleration_mps2) &&
      Finite(config.maximum_abs_lateral_slope) &&
      Finite(config.maximum_abs_lateral_second_derivative_per_m) &&
      Finite(config.maximum_abs_curvature_per_m) &&
      Finite(config.maximum_lateral_acceleration_mps2) &&
      Finite(config.maximum_lateral_jerk_mps3) &&
      Finite(config.c2_position_tolerance_m) &&
      Finite(config.c2_first_derivative_tolerance) &&
      Finite(config.c2_second_derivative_tolerance_per_m) &&
      Finite(config.c3_third_derivative_tolerance_per_m2) &&
      Finite(config.road_boundary_tolerance_m) &&
      Finite(config.minimum_boundary_sample_spacing_m) &&
      Finite(config.ego_length_m) && Finite(config.ego_width_m) &&
      Finite(config.lane_boundary_margin_m) && Finite(config.lane_width_m);
  const double maximum_regular_samples =
      std::ceil(config.maximum_path_extent_m / config.geometry_sample_step_m) +
      3.0;
  if (!finite || config.planning_horizon_s <= 0.0 ||
      config.post_maneuver_observation_s < 0.0 ||
      config.nominal_transition_duration_s <= 0.0 ||
      config.minimum_transition_length_m <= 0.0 ||
      config.maximum_transition_length_m < config.minimum_transition_length_m ||
      config.maximum_path_extent_m < config.maximum_transition_length_m ||
      config.geometry_sample_step_m <= 0.0 ||
      config.maximum_geometry_samples < 3 ||
      config.committed_continuation_horizon_s < 0.0 ||
      maximum_regular_samples >
          static_cast<double>(config.maximum_geometry_samples) ||
      config.speed_upper_bound_mps <= 0.0 ||
      config.maximum_abs_longitudinal_acceleration_mps2 < 0.0 ||
      config.maximum_abs_lateral_slope <= 0.0 ||
      config.maximum_abs_lateral_second_derivative_per_m <= 0.0 ||
      config.maximum_abs_curvature_per_m <= 0.0 ||
      config.maximum_lateral_acceleration_mps2 <= 0.0 ||
      config.maximum_lateral_jerk_mps3 <= 0.0 ||
      config.c2_position_tolerance_m < 0.0 ||
      config.c2_first_derivative_tolerance < 0.0 ||
      config.c2_second_derivative_tolerance_per_m < 0.0 ||
      config.c3_third_derivative_tolerance_per_m2 < 0.0 ||
      config.road_boundary_tolerance_m < 0.0 ||
      config.minimum_boundary_sample_spacing_m <= 0.0 ||
      config.ego_length_m <= 0.0 || config.ego_width_m <= 0.0 ||
      config.lane_boundary_margin_m < 0.0 || config.lane_width_m <= 0.0 ||
      config.lane_count <= 0 || config.lane_count > 63 ||
      config.lane_boundary_margin_m >= 0.5 * config.lane_width_m) {
    throw std::invalid_argument("invalid spatial path configuration");
  }
}

struct BoundaryState {
  bool valid = false;
  double road_s_unwrapped_m = 0.0;
  double d_m = 0.0;
  double first_derivative = 0.0;
  double second_derivative_per_m = 0.0;
  double third_derivative_per_m2 = 0.0;
};

BoundaryState EstimateInitialBoundary(const PlanningSnapshot &planning,
                                      const SpatialPathPlannerConfig &config,
                                      const MapData &map) {
  BoundaryState result;
  if (!Finite(planning.frontier.road_s_unwrapped_m) ||
      !Finite(planning.frontier.d_m) ||
      planning.retained_lateral_states.size() < 4) {
    return result;
  }

  const std::size_t count = planning.retained_lateral_states.size();
  const LateralPathState &first = planning.retained_lateral_states[count - 4];
  const LateralPathState &second = planning.retained_lateral_states[count - 3];
  const LateralPathState &third = planning.retained_lateral_states[count - 2];
  const LateralPathState &fourth = planning.retained_lateral_states[count - 1];
  if (!first.valid || !second.valid || !third.valid || !fourth.valid ||
      !Finite(first.road_parameter_s) || !Finite(second.road_parameter_s) ||
      !Finite(third.road_parameter_s) || !Finite(fourth.road_parameter_s) ||
      !Finite(first.planned_d) || !Finite(second.planned_d) ||
      !Finite(third.planned_d) || !Finite(fourth.planned_d) ||
      std::fabs(fourth.road_parameter_s -
                planning.frontier.road_s_unwrapped_m) > 1e-4 ||
      std::fabs(fourth.planned_d - planning.frontier.d_m) > 1e-4) {
    return result;
  }

  double x0 = first.road_parameter_s;
  double x1 = second.road_parameter_s;
  double x2 = third.road_parameter_s;
  double x3 = fourth.road_parameter_s;
  double y0 = first.planned_d;
  double y1 = second.planned_d;
  double y2 = third.planned_d;
  double y3 = fourth.planned_d;

  // The validator intentionally evaluates inherited kinematics from the
  // previously issued smooth Cartesian states plus one rigid frame offset,
  // not from independently quantized simulator points. Build the new Frenet
  // boundary from that same frame so a millimetre input residual cannot
  // become a position jump at the first fresh point.
  if (planning.historical_plan_aligned &&
      planning.input.previous_path_x.size() ==
          planning.retained_lateral_states.size() &&
      planning.input.previous_path_y.size() ==
          planning.retained_lateral_states.size()) {
    const double frame_offset_x =
        planning.input.previous_path_x.back() - fourth.expected_x;
    const double frame_offset_y =
        planning.input.previous_path_y.back() - fourth.expected_y;
    const LateralPathState *states[4] = {&first, &second, &third, &fourth};
    double projected_s[4] = {0.0, 0.0, 0.0, 0.0};
    double projected_d[4] = {0.0, 0.0, 0.0, 0.0};
    try {
      for (std::size_t index = 0; index < 4; ++index) {
        if (!Finite(states[index]->expected_x) ||
            !Finite(states[index]->expected_y)) {
          return result;
        }
        const RoadProjection projection =
            ProjectCartesianToRoad(states[index]->expected_x + frame_offset_x,
                                   states[index]->expected_y + frame_offset_y,
                                   states[index]->road_parameter_s, map);
        projected_s[index] = projection.road_s_unwrapped_m;
        projected_d[index] = projection.d_m;
      }
    } catch (const std::exception &) {
      return result;
    }
    x0 = projected_s[0];
    x1 = projected_s[1];
    x2 = projected_s[2];
    x3 = projected_s[3];
    y0 = projected_d[0];
    y1 = projected_d[1];
    y2 = projected_d[2];
    y3 = projected_d[3];
  }
  if (x1 - x0 < config.minimum_boundary_sample_spacing_m ||
      x2 - x1 < config.minimum_boundary_sample_spacing_m ||
      x3 - x2 < config.minimum_boundary_sample_spacing_m) {
    return result;
  }
  // Fit the inherited path locally with a cubic. Its derivatives at the
  // frontier are the C3 boundary for the newly generated lane-change path.
  const double divided01 = (y1 - y0) / (x1 - x0);
  const double divided12 = (y2 - y1) / (x2 - x1);
  const double divided23 = (y3 - y2) / (x3 - x2);
  const double divided012 = (divided12 - divided01) / (x2 - x0);
  const double divided123 = (divided23 - divided12) / (x3 - x1);
  const double divided0123 = (divided123 - divided012) / (x3 - x0);
  if (!Finite(divided01) || !Finite(divided012) || !Finite(divided0123)) {
    return result;
  }

  result.road_s_unwrapped_m = x3;
  result.d_m = y3;
  const double frontier_from_x0 = x3 - x0;
  const double frontier_from_x1 = x3 - x1;
  const double frontier_from_x2 = x3 - x2;
  result.first_derivative = divided01 +
                            divided012 * (frontier_from_x0 + frontier_from_x1) +
                            divided0123 * (frontier_from_x1 * frontier_from_x2 +
                                           frontier_from_x0 * frontier_from_x2 +
                                           frontier_from_x0 * frontier_from_x1);
  result.second_derivative_per_m =
      2.0 * divided012 +
      2.0 * divided0123 *
          (frontier_from_x0 + frontier_from_x1 + frontier_from_x2);
  result.third_derivative_per_m2 = 6.0 * divided0123;
  result.valid = Finite(result.first_derivative) &&
                 Finite(result.second_derivative_per_m) &&
                 Finite(result.third_derivative_per_m2);
  return result;
}

std::array<double, 8>
FitNormalizedSeptic(double start_value, double start_first_derivative,
                    double start_second_derivative,
                    double start_third_derivative, double end_value,
                    double end_first_derivative, double end_second_derivative,
                    double end_third_derivative, double length_m) {
  std::array<double, 8> coefficients = {
      {start_value, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  const double length_squared = length_m * length_m;
  const double length_cubed = length_squared * length_m;
  coefficients[1] = start_first_derivative * length_m;
  coefficients[2] = 0.5 * start_second_derivative * length_squared;
  coefficients[3] = start_third_derivative * length_cubed / 6.0;
  const double value_residual = end_value - coefficients[0] - coefficients[1] -
                                coefficients[2] - coefficients[3];
  const double first_residual = end_first_derivative * length_m -
                                coefficients[1] - 2.0 * coefficients[2] -
                                3.0 * coefficients[3];
  const double second_residual = end_second_derivative * length_squared -
                                 2.0 * coefficients[2] - 6.0 * coefficients[3];
  const double third_residual =
      end_third_derivative * length_cubed - 6.0 * coefficients[3];
  coefficients[4] = 35.0 * value_residual - 15.0 * first_residual +
                    2.5 * second_residual - third_residual / 6.0;
  coefficients[5] = -84.0 * value_residual + 39.0 * first_residual -
                    7.0 * second_residual + 0.5 * third_residual;
  coefficients[6] = 70.0 * value_residual - 34.0 * first_residual +
                    6.5 * second_residual - 0.5 * third_residual;
  coefficients[7] = -20.0 * value_residual + 10.0 * first_residual -
                    2.0 * second_residual + third_residual / 6.0;
  return coefficients;
}

template <std::size_t N>
double PolynomialValue(const std::array<double, N> &coefficients, double u) {
  double value = coefficients[N - 1];
  for (int index = static_cast<int>(N) - 2; index >= 0; --index) {
    value = value * u + coefficients[static_cast<std::size_t>(index)];
  }
  return value;
}

template <std::size_t N>
double PolynomialFirst(const std::array<double, N> &coefficients, double u) {
  double value = static_cast<double>(N - 1) * coefficients[N - 1];
  for (int index = static_cast<int>(N) - 2; index >= 1; --index) {
    value = value * u + static_cast<double>(index) *
                            coefficients[static_cast<std::size_t>(index)];
  }
  return value;
}

template <std::size_t N>
double PolynomialSecond(const std::array<double, N> &coefficients, double u) {
  double value = static_cast<double>((N - 1) * (N - 2)) * coefficients[N - 1];
  for (int index = static_cast<int>(N) - 2; index >= 2; --index) {
    value = value * u + static_cast<double>(index * (index - 1)) *
                            coefficients[static_cast<std::size_t>(index)];
  }
  return value;
}

template <std::size_t N>
double PolynomialThird(const std::array<double, N> &coefficients, double u) {
  double value =
      static_cast<double>((N - 1) * (N - 2) * (N - 3)) * coefficients[N - 1];
  for (int index = static_cast<int>(N) - 2; index >= 3; --index) {
    value = value * u + static_cast<double>(index * (index - 1) * (index - 2)) *
                            coefficients[static_cast<std::size_t>(index)];
  }
  return value;
}

struct LateralPolynomialSample {
  double d_m = 0.0;
  double first_derivative = 0.0;
  double second_derivative_per_m = 0.0;
  double third_derivative_per_m2 = 0.0;
};

LateralPolynomialSample EvaluateLateral(const SpatialPathCandidate &candidate,
                                        double progress_m) {
  LateralPolynomialSample result;
  if (progress_m > candidate.transition_length_m) {
    result.d_m = candidate.target_d_m;
    return result;
  }
  const double u = Clamp(progress_m / candidate.transition_length_m, 0.0, 1.0);
  result.d_m = PolynomialValue(candidate.d_coefficients, u);
  result.first_derivative = PolynomialFirst(candidate.d_coefficients, u) /
                            candidate.transition_length_m;
  result.second_derivative_per_m =
      PolynomialSecond(candidate.d_coefficients, u) /
      (candidate.transition_length_m * candidate.transition_length_m);
  result.third_derivative_per_m2 =
      PolynomialThird(candidate.d_coefficients, u) /
      (candidate.transition_length_m * candidate.transition_length_m *
       candidate.transition_length_m);
  return result;
}

struct RawGeometrySample {
  SpatialPathGeometrySample sample;
  double metric = 0.0;
  double body_lateral_extent_with_margin_m = 0.0;
};

RawGeometrySample EvaluateGeometry(const SpatialPathCandidate &candidate,
                                   double construction_progress_m,
                                   const SpatialPathPlannerConfig &config,
                                   const MapData &map) {
  const double road_s =
      candidate.start_road_s_unwrapped_m + construction_progress_m;
  const LateralPolynomialSample lateral =
      EvaluateLateral(candidate, construction_progress_m);
  const RoadGeometrySample center =
      EvaluateRoadGeometryOnValidatedMap(road_s, 0.0, map);
  const RoadGeometrySample unit_offset =
      EvaluateRoadGeometryOnValidatedMap(road_s, 1.0, map);
  const RoadGeometrySample road =
      EvaluateRoadGeometryOnValidatedMap(road_s, lateral.d_m, map);

  const double normal_x = unit_offset.x - center.x;
  const double normal_y = unit_offset.y - center.y;
  const double normal_first_x =
      unit_offset.first_derivative_x - center.first_derivative_x;
  const double normal_first_y =
      unit_offset.first_derivative_y - center.first_derivative_y;
  const double first_x =
      road.first_derivative_x + normal_x * lateral.first_derivative;
  const double first_y =
      road.first_derivative_y + normal_y * lateral.first_derivative;
  const double second_x = road.second_derivative_x +
                          2.0 * normal_first_x * lateral.first_derivative +
                          normal_x * lateral.second_derivative_per_m;
  const double second_y = road.second_derivative_y +
                          2.0 * normal_first_y * lateral.first_derivative +
                          normal_y * lateral.second_derivative_per_m;
  const double metric = std::hypot(first_x, first_y);
  const double road_metric =
      std::hypot(road.first_derivative_x, road.first_derivative_y);
  if (!Finite(road.x) || !Finite(road.y) || !Finite(metric) ||
      metric <= kMinimumTangentMagnitude || !Finite(road_metric) ||
      road_metric <= kMinimumTangentMagnitude) {
    throw std::runtime_error("spatial path geometry is degenerate");
  }

  const double first_second_dot = first_x * second_x + first_y * second_y;
  const double metric_squared = metric * metric;
  const double curvature_x =
      second_x / metric_squared -
      first_x * first_second_dot / (metric_squared * metric_squared);
  const double curvature_y =
      second_y / metric_squared -
      first_y * first_second_dot / (metric_squared * metric_squared);
  const double signed_curvature =
      (first_x * second_y - first_y * second_x) / (metric_squared * metric);
  const double tangent_x = first_x / metric;
  const double tangent_y = first_y / metric;
  const double road_tangent_x = road.first_derivative_x / road_metric;
  const double road_tangent_y = road.first_derivative_y / road_metric;
  const double absolute_heading_cosine =
      std::fabs(tangent_x * road_tangent_x + tangent_y * road_tangent_y);
  const double absolute_heading_sine =
      std::fabs(tangent_x * road_tangent_y - tangent_y * road_tangent_x);
  const double body_lateral_extent =
      0.5 * config.ego_width_m * absolute_heading_cosine +
      0.5 * config.ego_length_m * absolute_heading_sine;
  const double lower_margin =
      lateral.d_m - body_lateral_extent - config.lane_boundary_margin_m;
  const double upper_margin =
      static_cast<double>(config.lane_count) * config.lane_width_m -
      lateral.d_m - body_lateral_extent - config.lane_boundary_margin_m;

  RawGeometrySample result;
  result.metric = metric;
  result.body_lateral_extent_with_margin_m =
      body_lateral_extent + config.lane_boundary_margin_m;
  result.sample.construction_progress_m = construction_progress_m;
  result.sample.road_s_unwrapped_m = road_s;
  result.sample.d_m = lateral.d_m;
  result.sample.d_first_derivative = lateral.first_derivative;
  result.sample.d_second_derivative_per_m = lateral.second_derivative_per_m;
  result.sample.d_third_derivative_per_m2 = lateral.third_derivative_per_m2;
  result.sample.x_m = road.x;
  result.sample.y_m = road.y;
  result.sample.tangent_x = tangent_x;
  result.sample.tangent_y = tangent_y;
  result.sample.curvature_x_per_m = curvature_x;
  result.sample.curvature_y_per_m = curvature_y;
  result.sample.curvature_per_m = signed_curvature;
  result.sample.road_margin_m = std::min(lower_margin, upper_margin);
  return result;
}

void EvaluateC3(const BoundaryState &boundary,
                SpatialPathCandidate *candidate) {
  const double length_m = candidate->transition_length_m;
  const LateralPolynomialSample start = EvaluateLateral(*candidate, 0.0);
  const LateralPolynomialSample finish = EvaluateLateral(*candidate, length_m);
  SpatialPathPrecheckResult &precheck = candidate->precheck;
  precheck.c2_position_residual_m =
      std::max(std::fabs(start.d_m - boundary.d_m),
               std::fabs(finish.d_m - candidate->target_d_m));
  precheck.c2_first_derivative_residual =
      std::max(std::fabs(start.first_derivative - boundary.first_derivative),
               std::fabs(finish.first_derivative));
  precheck.c2_second_derivative_residual_per_m =
      std::max(std::fabs(start.second_derivative_per_m -
                         boundary.second_derivative_per_m),
               std::fabs(finish.second_derivative_per_m));
  precheck.c3_third_derivative_residual_per_m2 =
      std::max(std::fabs(start.third_derivative_per_m2 -
                         boundary.third_derivative_per_m2),
               std::fabs(finish.third_derivative_per_m2));
}

void UpdateLaneOccupancy(const RawGeometrySample &raw,
                         const SpatialPathCandidate &candidate,
                         const SpatialPathPlannerConfig &config,
                         LaneOccupancyProfile *occupancy) {
  const double body_min_d =
      raw.sample.d_m - raw.body_lateral_extent_with_margin_m;
  const double body_max_d =
      raw.sample.d_m + raw.body_lateral_extent_with_margin_m;
  const double target_min_d =
      static_cast<double>(candidate.target_lane) * config.lane_width_m;
  const double target_max_d = target_min_d + config.lane_width_m;
  const double source_min_d =
      static_cast<double>(candidate.source_lane) * config.lane_width_m;
  const double source_max_d = source_min_d + config.lane_width_m;
  const bool target_overlap =
      body_max_d + kNumericalTolerance >= target_min_d &&
      body_min_d <= target_max_d + kNumericalTolerance;
  const bool source_overlap =
      body_max_d + kNumericalTolerance >= source_min_d &&
      body_min_d <= source_max_d + kNumericalTolerance;
  if (!occupancy->target_lane_coverage_started && target_overlap) {
    occupancy->target_lane_coverage_started = true;
    occupancy->target_lane_coverage_start_path_progress_m =
        raw.sample.path_progress_m;
  }
  if (!occupancy->source_lane_departed &&
      occupancy->target_lane_coverage_started && !source_overlap) {
    occupancy->source_lane_departed = true;
    occupancy->source_lane_departure_path_progress_m =
        raw.sample.path_progress_m;
  }
  const bool body_inside_target =
      body_min_d + kNumericalTolerance >= target_min_d &&
      body_max_d <= target_max_d + kNumericalTolerance;
  if (!occupancy->lane_change_completed && body_inside_target &&
      raw.sample.construction_progress_m + kNumericalTolerance >=
          candidate.transition_length_m) {
    occupancy->lane_change_completed = true;
    occupancy->lane_change_completion_path_progress_m =
        raw.sample.path_progress_m;
  }
}

SpatialPathCandidate GenerateCandidate(const PlanningSnapshot &planning,
                                       const BehaviorCandidate &behavior,
                                       const SpatialPathPlannerConfig &config,
                                       const MapData &map) {
  SpatialPathCandidate candidate;
  candidate.candidate_id = behavior.candidate_id;
  candidate.source_lane = behavior.source_lane;
  candidate.target_lane = behavior.target_lane;
  candidate.precheck.evaluated = true;
  candidate.precheck.minimum_road_margin_m =
      std::numeric_limits<double>::infinity();
  candidate.start_road_s_unwrapped_m = planning.frontier.road_s_unwrapped_m;

  const bool valid_candidate =
      behavior.behavior != BehaviorType::kKeepLane &&
      behavior.status == BehaviorCandidateStatus::kCoarseAdmissionPassed &&
      behavior.coarse_admission.evaluated && behavior.coarse_admission.passed &&
      Finite(planning.frontier.longitudinal.v) &&
      planning.frontier.longitudinal.v >= 0.0 &&
      Finite(behavior.estimated_progress_m) &&
      behavior.estimated_progress_m >= 0.0 && behavior.source_lane >= 0 &&
      behavior.source_lane < config.lane_count && behavior.target_lane >= 0 &&
      behavior.target_lane < config.lane_count &&
      behavior.source_lane == planning.target_lane &&
      ((behavior.behavior == BehaviorType::kChangeLeft &&
        behavior.target_lane == behavior.source_lane - 1) ||
       (behavior.behavior == BehaviorType::kChangeRight &&
        behavior.target_lane == behavior.source_lane + 1));
  if (!valid_candidate) {
    AddReason(&candidate.precheck,
              SpatialPathPrecheckReason::kInvalidCandidate);
    return candidate;
  }

  const BoundaryState boundary = EstimateInitialBoundary(planning, config, map);
  if (!boundary.valid) {
    AddReason(&candidate.precheck,
              SpatialPathPrecheckReason::kInitialBoundaryUnavailable);
    return candidate;
  }
  candidate.start_d_m = boundary.d_m;
  candidate.start_road_s_unwrapped_m = boundary.road_s_unwrapped_m;
  candidate.start_d_first_derivative = boundary.first_derivative;
  candidate.start_d_second_derivative_per_m = boundary.second_derivative_per_m;
  candidate.start_d_third_derivative_per_m2 = boundary.third_derivative_per_m2;
  candidate.target_d_m =
      (static_cast<double>(candidate.target_lane) + 0.5) * config.lane_width_m;
  // P2.3 has no candidate QP yet.  The configured control speed limit and an
  // already-higher frontier speed are hard lower requirements for the bound;
  // a coarse pace estimate is not a hard bound and must not weaken curvature
  // or lateral-dynamics screening.
  candidate.precheck.speed_upper_bound_mps =
      std::max(config.speed_upper_bound_mps, planning.frontier.longitudinal.v);
  const double transition_design_speed_mps =
      config.use_frontier_speed_for_transition_length
          ? std::max(planning.frontier.longitudinal.v,
                     config.minimum_transition_length_m /
                         config.nominal_transition_duration_s)
          : candidate.precheck.speed_upper_bound_mps;
  candidate.transition_length_m = Clamp(
      transition_design_speed_mps * config.nominal_transition_duration_s,
      config.minimum_transition_length_m, config.maximum_transition_length_m);
  candidate.d_coefficients = FitNormalizedSeptic(
      candidate.start_d_m, candidate.start_d_first_derivative,
      candidate.start_d_second_derivative_per_m,
      candidate.start_d_third_derivative_per_m2, candidate.target_d_m, 0.0, 0.0,
      0.0, candidate.transition_length_m);
  EvaluateC3(boundary, &candidate);

  RawGeometrySample previous = EvaluateGeometry(candidate, 0.0, config, map);
  previous.sample.path_progress_m = 0.0;
  candidate.geometry.samples.push_back(previous.sample);
  std::vector<double> body_extents;
  body_extents.push_back(previous.body_lateral_extent_with_margin_m);
  double construction_progress_m = 0.0;
  double path_progress_m = 0.0;
  double required_path_extent_m = 0.0;
  bool transition_sampled = false;

  while (candidate.geometry.samples.size() < config.maximum_geometry_samples) {
    if (transition_sampled &&
        path_progress_m + kNumericalTolerance >= required_path_extent_m) {
      break;
    }
    if (construction_progress_m + kNumericalTolerance >=
        config.maximum_path_extent_m) {
      break;
    }
    double next_progress_m =
        std::min(config.maximum_path_extent_m,
                 construction_progress_m + config.geometry_sample_step_m);
    if (!transition_sampled &&
        candidate.transition_length_m >
            construction_progress_m + kNumericalTolerance &&
        candidate.transition_length_m < next_progress_m - kNumericalTolerance) {
      next_progress_m = candidate.transition_length_m;
    }
    if (next_progress_m <= construction_progress_m + kNumericalTolerance) {
      break;
    }

    RawGeometrySample midpoint = EvaluateGeometry(
        candidate, 0.5 * (construction_progress_m + next_progress_m), config,
        map);
    RawGeometrySample next =
        EvaluateGeometry(candidate, next_progress_m, config, map);
    const double interval_m = next_progress_m - construction_progress_m;
    path_progress_m += interval_m *
                       (previous.metric + 4.0 * midpoint.metric + next.metric) /
                       6.0;
    next.sample.path_progress_m = path_progress_m;
    candidate.geometry.samples.push_back(next.sample);
    body_extents.push_back(next.body_lateral_extent_with_margin_m);
    construction_progress_m = next_progress_m;
    previous = next;

    if (!transition_sampled && construction_progress_m + kNumericalTolerance >=
                                   candidate.transition_length_m) {
      transition_sampled = true;
      candidate.geometry.transition_completion_path_progress_m =
          path_progress_m;
      required_path_extent_m =
          std::max(std::max(0.0, behavior.estimated_progress_m),
                   path_progress_m + candidate.precheck.speed_upper_bound_mps *
                                         config.post_maneuver_observation_s);
      candidate.requested_path_extent_m = required_path_extent_m;
    }
  }

  candidate.geometry.path_extent_m = path_progress_m;
  candidate.geometry.construction_extent_m = construction_progress_m;
  candidate.geometry.sample_count = candidate.geometry.samples.size();
  if (!transition_sampled || path_progress_m + kNumericalTolerance <
                                 candidate.requested_path_extent_m) {
    AddReason(&candidate.precheck,
              SpatialPathPrecheckReason::kGeometryCoverageInsufficient);
  }

  const std::size_t sample_count = candidate.geometry.samples.size();
  for (std::size_t index = 0; index < sample_count; ++index) {
    SpatialPathGeometrySample &sample = candidate.geometry.samples[index];
    if (sample_count == 1) {
      sample.curvature_rate_per_m2 = 0.0;
    } else if (index == 0) {
      const SpatialPathGeometrySample &next = candidate.geometry.samples[1];
      sample.curvature_rate_per_m2 =
          (next.curvature_per_m - sample.curvature_per_m) /
          std::max(next.path_progress_m - sample.path_progress_m,
                   kNumericalTolerance);
    } else if (index + 1 == sample_count) {
      const SpatialPathGeometrySample &previous_sample =
          candidate.geometry.samples[index - 1];
      sample.curvature_rate_per_m2 =
          (sample.curvature_per_m - previous_sample.curvature_per_m) /
          std::max(sample.path_progress_m - previous_sample.path_progress_m,
                   kNumericalTolerance);
    } else {
      const SpatialPathGeometrySample &previous_sample =
          candidate.geometry.samples[index - 1];
      const SpatialPathGeometrySample &next =
          candidate.geometry.samples[index + 1];
      sample.curvature_rate_per_m2 =
          (next.curvature_per_m - previous_sample.curvature_per_m) /
          std::max(next.path_progress_m - previous_sample.path_progress_m,
                   kNumericalTolerance);
    }
    candidate.precheck.maximum_abs_lateral_slope =
        std::max(candidate.precheck.maximum_abs_lateral_slope,
                 std::fabs(sample.d_first_derivative));
    candidate.precheck.maximum_abs_lateral_second_derivative_per_m =
        std::max(candidate.precheck.maximum_abs_lateral_second_derivative_per_m,
                 std::fabs(sample.d_second_derivative_per_m));
    candidate.precheck.maximum_abs_curvature_per_m =
        std::max(candidate.precheck.maximum_abs_curvature_per_m,
                 std::fabs(sample.curvature_per_m));
    candidate.precheck.maximum_abs_curvature_rate_per_m2 =
        std::max(candidate.precheck.maximum_abs_curvature_rate_per_m2,
                 std::fabs(sample.curvature_rate_per_m2));
    candidate.precheck.minimum_road_margin_m = std::min(
        candidate.precheck.minimum_road_margin_m, sample.road_margin_m);

    RawGeometrySample occupancy_sample;
    occupancy_sample.sample = sample;
    occupancy_sample.body_lateral_extent_with_margin_m = body_extents[index];
    UpdateLaneOccupancy(occupancy_sample, candidate, config,
                        &candidate.occupancy);
  }

  if (!Finite(candidate.precheck.minimum_road_margin_m)) {
    candidate.precheck.minimum_road_margin_m = 0.0;
  }
  const double speed_mps = candidate.precheck.speed_upper_bound_mps;
  candidate.precheck.maximum_estimated_lateral_acceleration_mps2 =
      speed_mps * speed_mps * candidate.precheck.maximum_abs_curvature_per_m;
  candidate.precheck.maximum_estimated_lateral_jerk_mps3 =
      speed_mps * speed_mps * speed_mps *
          candidate.precheck.maximum_abs_curvature_rate_per_m2 +
      2.0 * speed_mps * config.maximum_abs_longitudinal_acceleration_mps2 *
          candidate.precheck.maximum_abs_curvature_per_m;

  if (candidate.precheck.c2_position_residual_m >
          config.c2_position_tolerance_m ||
      candidate.precheck.c2_first_derivative_residual >
          config.c2_first_derivative_tolerance ||
      candidate.precheck.c2_second_derivative_residual_per_m >
          config.c2_second_derivative_tolerance_per_m) {
    AddReason(&candidate.precheck, SpatialPathPrecheckReason::kC2Discontinuity);
  }
  if (candidate.precheck.c3_third_derivative_residual_per_m2 >
      config.c3_third_derivative_tolerance_per_m2) {
    AddReason(&candidate.precheck, SpatialPathPrecheckReason::kC3Discontinuity);
  }
  const bool lateral_derivative_failed =
      candidate.precheck.maximum_abs_lateral_slope >
          config.maximum_abs_lateral_slope ||
      candidate.precheck.maximum_abs_lateral_second_derivative_per_m >
          config.maximum_abs_lateral_second_derivative_per_m;
  if (lateral_derivative_failed) {
    AddReason(&candidate.precheck,
              SpatialPathPrecheckReason::kLateralDerivative);
  }
  if (candidate.precheck.maximum_abs_curvature_per_m >
      config.maximum_abs_curvature_per_m) {
    AddReason(&candidate.precheck, SpatialPathPrecheckReason::kCurvature);
  }
  if (candidate.precheck.minimum_road_margin_m <
      -config.road_boundary_tolerance_m) {
    AddReason(&candidate.precheck, SpatialPathPrecheckReason::kRoadBoundary);
  }
  const bool lateral_acceleration_failed =
      candidate.precheck.maximum_estimated_lateral_acceleration_mps2 >
      config.maximum_lateral_acceleration_mps2;
  if (lateral_acceleration_failed) {
    AddReason(&candidate.precheck,
              SpatialPathPrecheckReason::kLateralAcceleration);
  }
  const bool lateral_jerk_failed =
      candidate.precheck.maximum_estimated_lateral_jerk_mps3 >
      config.maximum_lateral_jerk_mps3;
  if (lateral_jerk_failed) {
    AddReason(&candidate.precheck, SpatialPathPrecheckReason::kLateralJerk);
  }
  if (candidate.transition_length_m + kNumericalTolerance <
          config.minimum_transition_length_m ||
      lateral_derivative_failed) {
    AddReason(&candidate.precheck,
              SpatialPathPrecheckReason::kTransitionLengthInsufficient);
  }
  if (!candidate.occupancy.target_lane_coverage_started ||
      !candidate.occupancy.source_lane_departed ||
      !candidate.occupancy.lane_change_completed ||
      candidate.occupancy.target_lane_coverage_start_path_progress_m >
          candidate.occupancy.source_lane_departure_path_progress_m +
              kNumericalTolerance ||
      candidate.occupancy.source_lane_departure_path_progress_m >
          candidate.occupancy.lane_change_completion_path_progress_m +
              kNumericalTolerance) {
    AddReason(&candidate.precheck,
              SpatialPathPrecheckReason::kLaneOccupancyInconsistent);
  }

  // Do not make the transition-only P2.3 dynamics gate depend on far-ahead
  // road curvature. P2.4 applies node-wise curvature speed limits to this
  // appended committed segment, and P2.5 validates its Cartesian dynamics.
  if (candidate.precheck.rejection_reasons.empty() &&
      config.committed_continuation_horizon_s > 0.0) {
    candidate.requested_path_extent_m =
        std::max(candidate.requested_path_extent_m,
                 candidate.geometry.transition_completion_path_progress_m +
                     candidate.precheck.speed_upper_bound_mps *
                         config.committed_continuation_horizon_s);
    while (candidate.geometry.samples.size() <
               config.maximum_geometry_samples &&
           path_progress_m + kNumericalTolerance <
               candidate.requested_path_extent_m &&
           construction_progress_m + kNumericalTolerance <
               config.maximum_path_extent_m) {
      const double next_progress_m =
          std::min(config.maximum_path_extent_m,
                   construction_progress_m + config.geometry_sample_step_m);
      if (next_progress_m <= construction_progress_m + kNumericalTolerance) {
        break;
      }
      const RawGeometrySample midpoint = EvaluateGeometry(
          candidate, 0.5 * (construction_progress_m + next_progress_m), config,
          map);
      RawGeometrySample next =
          EvaluateGeometry(candidate, next_progress_m, config, map);
      const double interval_m = next_progress_m - construction_progress_m;
      path_progress_m +=
          interval_m * (previous.metric + 4.0 * midpoint.metric + next.metric) /
          6.0;
      next.sample.path_progress_m = path_progress_m;
      candidate.geometry.samples.push_back(next.sample);
      construction_progress_m = next_progress_m;
      previous = next;
    }
    candidate.geometry.path_extent_m = path_progress_m;
    candidate.geometry.construction_extent_m = construction_progress_m;
    candidate.geometry.sample_count = candidate.geometry.samples.size();
    if (candidate.geometry.path_extent_m + kNumericalTolerance <
        candidate.requested_path_extent_m) {
      AddReason(&candidate.precheck,
                SpatialPathPrecheckReason::kGeometryCoverageInsufficient);
    }

    // The appended points need curvature-rate evidence for P2.5 diagnostics.
    // Recompute the bounded table once so the old terminal one-sided value is
    // also consistent with its new successor.
    const std::size_t extended_count = candidate.geometry.samples.size();
    for (std::size_t index = 0; index < extended_count; ++index) {
      SpatialPathGeometrySample &sample = candidate.geometry.samples[index];
      if (extended_count == 1) {
        sample.curvature_rate_per_m2 = 0.0;
      } else if (index == 0) {
        const SpatialPathGeometrySample &next_sample =
            candidate.geometry.samples[1];
        sample.curvature_rate_per_m2 =
            (next_sample.curvature_per_m - sample.curvature_per_m) /
            std::max(next_sample.path_progress_m - sample.path_progress_m,
                     kNumericalTolerance);
      } else if (index + 1 == extended_count) {
        const SpatialPathGeometrySample &previous_sample =
            candidate.geometry.samples[index - 1];
        sample.curvature_rate_per_m2 =
            (sample.curvature_per_m - previous_sample.curvature_per_m) /
            std::max(sample.path_progress_m - previous_sample.path_progress_m,
                     kNumericalTolerance);
      } else {
        const SpatialPathGeometrySample &previous_sample =
            candidate.geometry.samples[index - 1];
        const SpatialPathGeometrySample &next_sample =
            candidate.geometry.samples[index + 1];
        sample.curvature_rate_per_m2 =
            (next_sample.curvature_per_m - previous_sample.curvature_per_m) /
            std::max(next_sample.path_progress_m -
                         previous_sample.path_progress_m,
                     kNumericalTolerance);
      }
    }
  }

  candidate.precheck.passed = candidate.precheck.rejection_reasons.empty();
  candidate.status = candidate.precheck.passed
                         ? SpatialPathCandidateStatus::kPrecheckPassed
                         : SpatialPathCandidateStatus::kPrecheckRejected;
  return candidate;
}

} // namespace

const char *SpatialPathPrecheckReasonName(SpatialPathPrecheckReason reason) {
  switch (reason) {
  case SpatialPathPrecheckReason::kInvalidCandidate:
    return "InvalidCandidate";
  case SpatialPathPrecheckReason::kInitialBoundaryUnavailable:
    return "InitialBoundaryUnavailable";
  case SpatialPathPrecheckReason::kC2Discontinuity:
    return "C2Discontinuity";
  case SpatialPathPrecheckReason::kC3Discontinuity:
    return "C3Discontinuity";
  case SpatialPathPrecheckReason::kTransitionLengthInsufficient:
    return "TransitionLengthInsufficient";
  case SpatialPathPrecheckReason::kLateralDerivative:
    return "LateralDerivative";
  case SpatialPathPrecheckReason::kCurvature:
    return "Curvature";
  case SpatialPathPrecheckReason::kRoadBoundary:
    return "RoadBoundary";
  case SpatialPathPrecheckReason::kLateralAcceleration:
    return "LateralAcceleration";
  case SpatialPathPrecheckReason::kLateralJerk:
    return "LateralJerk";
  case SpatialPathPrecheckReason::kLaneOccupancyInconsistent:
    return "LaneOccupancyInconsistent";
  case SpatialPathPrecheckReason::kGeometryCoverageInsufficient:
    return "GeometryCoverageInsufficient";
  }
  return "Unknown";
}

const char *SpatialPathCandidateStatusName(SpatialPathCandidateStatus status) {
  switch (status) {
  case SpatialPathCandidateStatus::kPrecheckRejected:
    return "PrecheckRejected";
  case SpatialPathCandidateStatus::kPrecheckPassed:
    return "PrecheckPassed";
  }
  return "Unknown";
}

bool HasSpatialPathPrecheckReason(const SpatialPathPrecheckResult &result,
                                  SpatialPathPrecheckReason reason) {
  return std::find(result.rejection_reasons.begin(),
                   result.rejection_reasons.end(),
                   reason) != result.rejection_reasons.end();
}

const SpatialPathCandidate *
FindSpatialPathCandidate(const SpatialPathBatchSnapshot &snapshot,
                         std::uint64_t candidate_id) {
  for (const SpatialPathCandidate &candidate : snapshot.candidates) {
    if (candidate.candidate_id == candidate_id) {
      return &candidate;
    }
  }
  return nullptr;
}

SpatialPathGeometrySample
SampleSpatialPathAtProgress(const SpatialPathGeometryTable &geometry,
                            double path_progress_m) {
  if (!Finite(path_progress_m) || geometry.samples.empty() ||
      path_progress_m < -kNumericalTolerance ||
      path_progress_m > geometry.path_extent_m + kNumericalTolerance) {
    throw std::out_of_range("PathProgress is outside spatial geometry");
  }
  const double bounded_progress_m =
      Clamp(path_progress_m, 0.0, geometry.path_extent_m);
  const std::vector<SpatialPathGeometrySample>::const_iterator upper =
      std::lower_bound(
          geometry.samples.begin(), geometry.samples.end(), bounded_progress_m,
          [](const SpatialPathGeometrySample &sample, double progress_m) {
            return sample.path_progress_m < progress_m;
          });
  if (upper == geometry.samples.begin()) {
    return *upper;
  }
  if (upper == geometry.samples.end()) {
    return geometry.samples.back();
  }
  if (std::fabs(upper->path_progress_m - bounded_progress_m) <=
      kNumericalTolerance) {
    return *upper;
  }
  const SpatialPathGeometrySample &lower = *(upper - 1);
  const double span_m = upper->path_progress_m - lower.path_progress_m;
  if (span_m <= kNumericalTolerance) {
    throw std::runtime_error("spatial geometry PathProgress is not monotonic");
  }
  const double fraction = (bounded_progress_m - lower.path_progress_m) / span_m;
  const auto interpolate = [fraction](double first, double second) {
    return first + fraction * (second - first);
  };
  const double lower_curvature_squared =
      lower.curvature_per_m * lower.curvature_per_m;
  const double upper_curvature_squared =
      upper->curvature_per_m * upper->curvature_per_m;
  const double lower_third_x = -lower_curvature_squared * lower.tangent_x -
                               lower.curvature_rate_per_m2 * lower.tangent_y;
  const double lower_third_y = -lower_curvature_squared * lower.tangent_y +
                               lower.curvature_rate_per_m2 * lower.tangent_x;
  const double upper_third_x = -upper_curvature_squared * upper->tangent_x -
                               upper->curvature_rate_per_m2 * upper->tangent_y;
  const double upper_third_y = -upper_curvature_squared * upper->tangent_y +
                               upper->curvature_rate_per_m2 * upper->tangent_x;
  const std::array<double, 8> x_coefficients =
      FitNormalizedSeptic(lower.x_m, lower.tangent_x, lower.curvature_x_per_m,
                          lower_third_x, upper->x_m, upper->tangent_x,
                          upper->curvature_x_per_m, upper_third_x, span_m);
  const std::array<double, 8> y_coefficients =
      FitNormalizedSeptic(lower.y_m, lower.tangent_y, lower.curvature_y_per_m,
                          lower_third_y, upper->y_m, upper->tangent_y,
                          upper->curvature_y_per_m, upper_third_y, span_m);
  const double first_x = PolynomialFirst(x_coefficients, fraction) / span_m;
  const double first_y = PolynomialFirst(y_coefficients, fraction) / span_m;
  const double second_x =
      PolynomialSecond(x_coefficients, fraction) / (span_m * span_m);
  const double second_y =
      PolynomialSecond(y_coefficients, fraction) / (span_m * span_m);
  const double third_x =
      PolynomialThird(x_coefficients, fraction) / (span_m * span_m * span_m);
  const double third_y =
      PolynomialThird(y_coefficients, fraction) / (span_m * span_m * span_m);
  const double derivative_magnitude = std::hypot(first_x, first_y);
  if (derivative_magnitude <= kMinimumTangentMagnitude) {
    throw std::runtime_error("C2 spatial interpolation is degenerate");
  }
  const double derivative_squared = derivative_magnitude * derivative_magnitude;
  const double first_second_dot = first_x * second_x + first_y * second_y;
  SpatialPathGeometrySample result;
  result.construction_progress_m = interpolate(lower.construction_progress_m,
                                               upper->construction_progress_m);
  result.path_progress_m = bounded_progress_m;
  result.road_s_unwrapped_m =
      interpolate(lower.road_s_unwrapped_m, upper->road_s_unwrapped_m);
  result.d_m = interpolate(lower.d_m, upper->d_m);
  result.d_first_derivative =
      interpolate(lower.d_first_derivative, upper->d_first_derivative);
  result.d_second_derivative_per_m = interpolate(
      lower.d_second_derivative_per_m, upper->d_second_derivative_per_m);
  result.d_third_derivative_per_m2 = interpolate(
      lower.d_third_derivative_per_m2, upper->d_third_derivative_per_m2);
  // Linear Cartesian interpolation creates a false heading corner at every
  // geometry-table node. Match position, unit tangent, arc-curvature and its
  // derivative at both endpoints so dense P2.5 sampling remains C3 and the
  // validator sees physical jerk rather than lookup-table quantization.
  result.x_m = PolynomialValue(x_coefficients, fraction);
  result.y_m = PolynomialValue(y_coefficients, fraction);
  result.tangent_x = first_x / derivative_magnitude;
  result.tangent_y = first_y / derivative_magnitude;
  result.curvature_x_per_m =
      second_x / derivative_squared -
      first_x * first_second_dot / (derivative_squared * derivative_squared);
  result.curvature_y_per_m =
      second_y / derivative_squared -
      first_y * first_second_dot / (derivative_squared * derivative_squared);
  const double first_second_cross = first_x * second_y - first_y * second_x;
  result.curvature_per_m =
      first_second_cross / (derivative_squared * derivative_magnitude);
  const double first_third_cross = first_x * third_y - first_y * third_x;
  const double derivative_fourth = derivative_squared * derivative_squared;
  const double derivative_sixth = derivative_fourth * derivative_squared;
  result.curvature_rate_per_m2 =
      first_third_cross / derivative_fourth -
      3.0 * first_second_cross * first_second_dot / derivative_sixth;
  result.road_margin_m = interpolate(lower.road_margin_m, upper->road_margin_m);
  return result;
}

SpatialPathPlanner::SpatialPathPlanner(const SpatialPathPlannerConfig &config)
    : config_(config) {
  ValidateConfig(config_);
}

SpatialPathBatchSnapshot
SpatialPathPlanner::Generate(const PlanningSnapshot &planning,
                             const BehaviorPlanningSnapshot &behavior,
                             const MapData &map) const {
  std::string map_error;
  if (!ValidateMap(map, &map_error)) {
    throw std::invalid_argument("invalid spatial path map: " + map_error);
  }
  SpatialPathBatchSnapshot result;
  result.cycle = planning.cycle;
  result.candidates.reserve(behavior.coarse_admitted_candidate_count);
  for (const BehaviorCandidate &behavior_candidate : behavior.candidates) {
    if (behavior_candidate.status !=
        BehaviorCandidateStatus::kCoarseAdmissionPassed) {
      continue;
    }
    SpatialPathCandidate candidate;
    try {
      candidate = GenerateCandidate(planning, behavior_candidate, config_, map);
    } catch (const std::exception &) {
      candidate.candidate_id = behavior_candidate.candidate_id;
      candidate.source_lane = behavior_candidate.source_lane;
      candidate.target_lane = behavior_candidate.target_lane;
      candidate.precheck.evaluated = true;
      AddReason(&candidate.precheck,
                SpatialPathPrecheckReason::kGeometryCoverageInsufficient);
    }
    ++result.evaluated_candidate_count;
    if (!candidate.geometry.samples.empty()) {
      ++result.generated_candidate_count;
    }
    if (candidate.precheck.passed) {
      ++result.precheck_passed_candidate_count;
    }
    result.candidates.push_back(std::move(candidate));
  }
  return result;
}
