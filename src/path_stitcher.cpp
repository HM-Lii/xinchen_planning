#include "path_stitcher.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "map.h"

namespace {

constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
constexpr double kMilesPerHourToMetersPerSecond = 0.44704;
constexpr double kMinimumTangentLength = 1e-8;
constexpr double kArcLengthTableStepMeters = 0.025;
constexpr double kMinimumTransitionLengthMeters = 15.0;
constexpr double kCorrectionJerkBudgetMps3 = 8.0;
constexpr double kPositionJerkShapeBound = 100.0;
constexpr double kTangentJerkShapeBound = 40.0;
constexpr double kCurvatureJerkShapeBound = 10.0;
constexpr double kLateralJerkShapeBound = 60.0;
constexpr double kMaximumHistoricalCurvaturePerMeter = 0.1;
constexpr double kPathAlignmentToleranceMeters = 0.02;
constexpr double kMaximumContinuationResidualMeters = 0.2;
constexpr double kMinimumResidualTransitionLengthMeters = 4.0;
constexpr double kMaximumResidualTransitionLengthMeters = 30.0;
constexpr double kCorrectionMagnitudeTolerance = 1e-6;
constexpr double kMinimumRollingLengthMeters = 1e-6;
constexpr std::size_t kMaximumArcLengthTableEntries = 20000;

struct Vector2d {
  Vector2d(double x_value = 0.0, double y_value = 0.0)
      : x(x_value), y(y_value) {}

  double x;
  double y;
};

struct BoundaryError {
  Vector2d position;
  Vector2d tangent;
  Vector2d curvature;
};

struct CurveSample {
  Vector2d position;
  Vector2d derivative;
  double d = 0.0;
};

struct ResidualCorrection {
  Vector2d position;
  double transition_length_m = 0.0;
};

struct ArcLengthTableEntry {
  double local_nominal_distance_m = 0.0;
  double correction_progress_m = 0.0;
  double road_parameter_s = 0.0;
  double corrected_arc_length_m = 0.0;
  double derivative_magnitude = 0.0;
};

double Length(const Vector2d &vector) { return std::hypot(vector.x, vector.y); }

Vector2d Add(const Vector2d &left, const Vector2d &right) {
  return {left.x + right.x, left.y + right.y};
}

Vector2d Subtract(const Vector2d &left, const Vector2d &right) {
  return {left.x - right.x, left.y - right.y};
}

Vector2d Scale(const Vector2d &vector, double scale) {
  return {scale * vector.x, scale * vector.y};
}

Vector2d UnitVector(const Vector2d &vector, const Vector2d &fallback) {
  const double length = Length(vector);
  if (length > kMinimumTangentLength) {
    return {vector.x / length, vector.y / length};
  }
  const double fallback_length = Length(fallback);
  if (fallback_length <= kMinimumTangentLength) {
    throw std::runtime_error("cannot determine path tangent");
  }
  return {fallback.x / fallback_length, fallback.y / fallback_length};
}

Vector2d LimitMagnitude(const Vector2d &vector, double maximum_magnitude) {
  const double magnitude = Length(vector);
  if (magnitude <= maximum_magnitude || magnitude <= kMinimumTangentLength) {
    return vector;
  }
  return Scale(vector, maximum_magnitude / magnitude);
}

Vector2d HistoricalAnchor(const PlannerInput &input) {
  if (!input.previous_path_x.empty()) {
    return {input.previous_path_x.back(), input.previous_path_y.back()};
  }
  return {input.ego.x, input.ego.y};
}

Vector2d HistoricalTangent(const PlannerInput &input,
                           const Vector2d &road_tangent) {
  const std::size_t size = input.previous_path_x.size();
  if (size >= 3) {
    const Vector2d previous_tangent = UnitVector(
        {input.previous_path_x[size - 2] - input.previous_path_x[size - 3],
         input.previous_path_y[size - 2] - input.previous_path_y[size - 3]},
        road_tangent);
    const Vector2d last_tangent = UnitVector(
        {input.previous_path_x[size - 1] - input.previous_path_x[size - 2],
         input.previous_path_y[size - 1] - input.previous_path_y[size - 2]},
        previous_tangent);
    return UnitVector({1.5 * last_tangent.x - 0.5 * previous_tangent.x,
                       1.5 * last_tangent.y - 0.5 * previous_tangent.y},
                      last_tangent);
  }
  if (size == 2) {
    const Vector2d previous_tangent =
        UnitVector({input.previous_path_x[0] - input.ego.x,
                    input.previous_path_y[0] - input.ego.y},
                   road_tangent);
    const Vector2d last_tangent =
        UnitVector({input.previous_path_x[1] - input.previous_path_x[0],
                    input.previous_path_y[1] - input.previous_path_y[0]},
                   previous_tangent);
    return UnitVector({1.5 * last_tangent.x - 0.5 * previous_tangent.x,
                       1.5 * last_tangent.y - 0.5 * previous_tangent.y},
                      last_tangent);
  }
  if (size == 1) {
    return UnitVector({input.previous_path_x.back() - input.ego.x,
                       input.previous_path_y.back() - input.ego.y},
                      road_tangent);
  }
  const double yaw_radians = input.ego.yaw_deg * kDegreesToRadians;
  return UnitVector({std::cos(yaw_radians), std::sin(yaw_radians)},
                    road_tangent);
}

Vector2d RoadArcCurvature(const RoadGeometrySample &geometry) {
  const Vector2d first = {geometry.first_derivative_x,
                          geometry.first_derivative_y};
  const Vector2d second = {geometry.second_derivative_x,
                           geometry.second_derivative_y};
  const double squared_speed = first.x * first.x + first.y * first.y;
  if (squared_speed <= kMinimumTangentLength * kMinimumTangentLength) {
    throw std::runtime_error("road spline has a degenerate tangent");
  }
  const double projection = first.x * second.x + first.y * second.y;
  return {second.x / squared_speed -
              first.x * projection / (squared_speed * squared_speed),
          second.y / squared_speed -
              first.y * projection / (squared_speed * squared_speed)};
}

Vector2d HistoricalCurvature(const PlannerInput &input,
                             const Vector2d &fallback) {
  Vector2d first_point;
  Vector2d second_point;
  Vector2d third_point;
  const std::size_t size = input.previous_path_x.size();
  if (size >= 3) {
    first_point = {input.previous_path_x[size - 3],
                   input.previous_path_y[size - 3]};
    second_point = {input.previous_path_x[size - 2],
                    input.previous_path_y[size - 2]};
    third_point = {input.previous_path_x[size - 1],
                   input.previous_path_y[size - 1]};
  } else if (size == 2) {
    first_point = {input.ego.x, input.ego.y};
    second_point = {input.previous_path_x[0], input.previous_path_y[0]};
    third_point = {input.previous_path_x[1], input.previous_path_y[1]};
  } else {
    return fallback;
  }

  const Vector2d first_segment = {second_point.x - first_point.x,
                                  second_point.y - first_point.y};
  const Vector2d second_segment = {third_point.x - second_point.x,
                                   third_point.y - second_point.y};
  const double first_length = Length(first_segment);
  const double second_length = Length(second_segment);
  const double average_length = 0.5 * (first_length + second_length);
  if (first_length <= kMinimumTangentLength ||
      second_length <= kMinimumTangentLength ||
      average_length <= kMinimumTangentLength) {
    return fallback;
  }
  const Vector2d first_tangent = {first_segment.x / first_length,
                                  first_segment.y / first_length};
  const Vector2d second_tangent = {second_segment.x / second_length,
                                   second_segment.y / second_length};
  return LimitMagnitude({(second_tangent.x - first_tangent.x) / average_length,
                         (second_tangent.y - first_tangent.y) / average_length},
                        kMaximumHistoricalCurvaturePerMeter);
}

double MaximumPlanningSpeed(const PlannerInput &input,
                            const std::vector<LongitudinalState> &states,
                            double configured_maximum_speed) {
  double maximum_speed = std::max(
      configured_maximum_speed,
      std::max(0.0, input.ego.speed_mph * kMilesPerHourToMetersPerSecond));
  for (const LongitudinalState &state : states) {
    maximum_speed = std::max(maximum_speed, std::max(0.0, state.v));
  }
  return maximum_speed;
}

double CorrectionTransitionLength(double maximum_speed, double lateral_error,
                                  const BoundaryError &error) {
  const double position_length =
      maximum_speed *
      std::cbrt(kPositionJerkShapeBound * Length(error.position) /
                kCorrectionJerkBudgetMps3);
  const double tangent_length =
      maximum_speed *
      std::sqrt(kTangentJerkShapeBound * Length(error.tangent) /
                kCorrectionJerkBudgetMps3);
  const double curvature_length =
      maximum_speed * maximum_speed * maximum_speed *
      kCurvatureJerkShapeBound * Length(error.curvature) /
      kCorrectionJerkBudgetMps3;
  const double lateral_length =
      maximum_speed *
      std::cbrt(kLateralJerkShapeBound * std::fabs(lateral_error) /
                kCorrectionJerkBudgetMps3);
  return std::max(kMinimumTransitionLengthMeters,
                  std::max(std::max(position_length, tangent_length),
                           std::max(curvature_length, lateral_length)));
}

double SmoothStep5(double u) {
  if (u <= 0.0) {
    return 0.0;
  }
  if (u >= 1.0) {
    return 1.0;
  }
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  return 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
}

double SmoothStep5Derivative(double u) {
  if (u <= 0.0 || u >= 1.0) {
    return 0.0;
  }
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  return 30.0 * u2 - 60.0 * u3 + 30.0 * u4;
}

BoundaryError BoundaryFromPlan(const LateralCorrectionPlan &plan) {
  BoundaryError error;
  error.position = {plan.position_error_x, plan.position_error_y};
  error.tangent = {plan.tangent_error_x, plan.tangent_error_y};
  error.curvature = {plan.curvature_error_x, plan.curvature_error_y};
  return error;
}

Vector2d PositionCorrection(double progress, double transition_length,
                            const BoundaryError &error) {
  if (transition_length <= 0.0 || progress >= transition_length) {
    return {};
  }
  const double u = std::max(0.0, progress / transition_length);
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  const double position_basis = 1.0 - 10.0 * u3 + 15.0 * u4 - 6.0 * u5;
  const double tangent_basis = u - 6.0 * u3 + 8.0 * u4 - 3.0 * u5;
  const double curvature_basis = 0.5 * u2 - 1.5 * u3 + 1.5 * u4 - 0.5 * u5;
  return Add(Add(Scale(error.position, position_basis),
                 Scale(error.tangent, transition_length * tangent_basis)),
             Scale(error.curvature,
                   transition_length * transition_length * curvature_basis));
}

Vector2d CorrectionDerivative(double progress, double transition_length,
                              const BoundaryError &error) {
  if (transition_length <= 0.0 || progress >= transition_length) {
    return {};
  }
  const double u = std::max(0.0, progress / transition_length);
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double position_basis_derivative = -30.0 * u2 + 60.0 * u3 - 30.0 * u4;
  const double tangent_basis_derivative =
      1.0 - 18.0 * u2 + 32.0 * u3 - 15.0 * u4;
  const double curvature_basis_derivative = u - 4.5 * u2 + 6.0 * u3 - 2.5 * u4;
  return Add(
      Add(Scale(error.position, position_basis_derivative / transition_length),
          Scale(error.tangent, tangent_basis_derivative)),
      Scale(error.curvature, transition_length * curvature_basis_derivative));
}

std::array<double, 6> FitNormalizedQuintic(
    double start_value, double start_first_derivative,
    double start_second_derivative, double end_value,
    double end_first_derivative, double end_second_derivative,
    double length_meters) {
  std::array<double, 6> coefficients =
      {{end_value, 0.0, 0.0, 0.0, 0.0, 0.0}};
  if (length_meters <= kMinimumRollingLengthMeters) {
    return coefficients;
  }

  coefficients[0] = start_value;
  coefficients[1] = start_first_derivative * length_meters;
  coefficients[2] =
      0.5 * start_second_derivative * length_meters * length_meters;
  const double value_residual =
      end_value - coefficients[0] - coefficients[1] - coefficients[2];
  const double first_residual =
      end_first_derivative * length_meters - coefficients[1] -
      2.0 * coefficients[2];
  const double second_residual =
      end_second_derivative * length_meters * length_meters -
      2.0 * coefficients[2];
  coefficients[3] =
      10.0 * value_residual - 4.0 * first_residual +
      0.5 * second_residual;
  coefficients[4] =
      -15.0 * value_residual + 7.0 * first_residual - second_residual;
  coefficients[5] =
      6.0 * value_residual - 3.0 * first_residual +
      0.5 * second_residual;
  return coefficients;
}

double EvaluatePolynomial(const std::array<double, 6> &coefficients,
                          double u) {
  double value = coefficients[5];
  for (int index = 4; index >= 0; --index) {
    value = value * u + coefficients[static_cast<std::size_t>(index)];
  }
  return value;
}

double EvaluatePolynomialFirstDerivative(
    const std::array<double, 6> &coefficients, double u) {
  return coefficients[1] +
         u * (2.0 * coefficients[2] +
              u * (3.0 * coefficients[3] +
                   u * (4.0 * coefficients[4] +
                        u * 5.0 * coefficients[5])));
}

double EvaluatePolynomialSecondDerivative(
    const std::array<double, 6> &coefficients, double u) {
  return 2.0 * coefficients[2] +
         u * (6.0 * coefficients[3] +
              u * (12.0 * coefficients[4] +
                   u * 20.0 * coefficients[5]));
}

double PolynomialSpan(const LateralCorrectionPlan &plan) {
  return plan.transition_length_m - plan.polynomial_origin_progress_m;
}

double PolynomialU(const LateralCorrectionPlan &plan, double progress) {
  const double span = PolynomialSpan(plan);
  if (span <= kMinimumRollingLengthMeters) {
    return 1.0;
  }
  return std::max(
      0.0, std::min(1.0,
                    (progress - plan.polynomial_origin_progress_m) / span));
}

double PlannedD(const LateralCorrectionPlan &plan, double progress) {
  if (plan.transition_length_m <= 0.0 ||
      progress >= plan.transition_length_m) {
    return plan.target_d;
  }
  return EvaluatePolynomial(plan.d_coefficients, PolynomialU(plan, progress));
}

double PlannedDDerivative(const LateralCorrectionPlan &plan,
                          double progress) {
  const double span = PolynomialSpan(plan);
  if (span <= kMinimumRollingLengthMeters ||
      progress >= plan.transition_length_m) {
    return 0.0;
  }
  return EvaluatePolynomialFirstDerivative(plan.d_coefficients,
                                           PolynomialU(plan, progress)) /
         span;
}

double PlannedDSecondDerivative(const LateralCorrectionPlan &plan,
                                double progress) {
  const double span = PolynomialSpan(plan);
  if (span <= kMinimumRollingLengthMeters ||
      progress >= plan.transition_length_m) {
    return 0.0;
  }
  return EvaluatePolynomialSecondDerivative(plan.d_coefficients,
                                            PolynomialU(plan, progress)) /
         (span * span);
}

LateralCorrectionPlan RollPlanAtFixedTerminal(
    const LateralCorrectionPlan &plan, double progress,
    bool *rolling_replanned) {
  if (rolling_replanned != nullptr) {
    *rolling_replanned = false;
  }
  if (!plan.initialized || !plan.has_correction ||
      progress >= plan.transition_length_m - kMinimumRollingLengthMeters) {
    return plan;
  }

  const double current_d = PlannedD(plan, progress);
  const double current_d_derivative = PlannedDDerivative(plan, progress);
  const double current_d_second_derivative =
      PlannedDSecondDerivative(plan, progress);
  const double remaining_length = plan.transition_length_m - progress;

  LateralCorrectionPlan rolled = plan;
  rolled.start_d = current_d;
  rolled.polynomial_origin_progress_m = progress;
  rolled.d_coefficients = FitNormalizedQuintic(
      current_d, current_d_derivative, current_d_second_derivative,
      plan.target_d, 0.0, 0.0, remaining_length);
  ++rolled.rolling_replan_count;
  if (rolling_replanned != nullptr) {
    *rolling_replanned = true;
  }
  return rolled;
}

Vector2d RoadNormal(double road_s, const MapData &map) {
  const RoadGeometrySample center = EvaluateRoadGeometry(road_s, 0.0, map);
  const RoadGeometrySample offset = EvaluateRoadGeometry(road_s, 1.0, map);
  return {offset.x - center.x, offset.y - center.y};
}

Vector2d ResidualPosition(double local_distance,
                          const ResidualCorrection &residual) {
  if (residual.transition_length_m <= 0.0 ||
      local_distance >= residual.transition_length_m) {
    return {};
  }
  const double u = std::max(0.0, local_distance / residual.transition_length_m);
  return Scale(residual.position, 1.0 - SmoothStep5(u));
}

Vector2d ResidualDerivative(double local_distance,
                            const ResidualCorrection &residual) {
  if (residual.transition_length_m <= 0.0 ||
      local_distance >= residual.transition_length_m) {
    return {};
  }
  const double u = std::max(0.0, local_distance / residual.transition_length_m);
  return Scale(residual.position,
               -SmoothStep5Derivative(u) / residual.transition_length_m);
}

CurveSample EvaluateCurve(const LateralCorrectionPlan &plan, double progress,
                          double road_parameter, double local_distance,
                          const ResidualCorrection &residual,
                          const MapData &map) {
  const double d = PlannedD(plan, progress);
  const double d_derivative = PlannedDDerivative(plan, progress);
  const RoadGeometrySample road = EvaluateRoadGeometry(road_parameter, d, map);
  const double road_parameter_derivative =
      1.0 / RoadParameterMetric(road_parameter, plan.target_d, map);
  const Vector2d normal = RoadNormal(road_parameter, map);
  const BoundaryError boundary_error = BoundaryFromPlan(plan);

  CurveSample sample;
  sample.d = d;
  sample.position =
      Add(Add({road.x, road.y},
              PositionCorrection(progress, plan.transition_length_m,
                                 boundary_error)),
          ResidualPosition(local_distance, residual));
  sample.derivative =
      Add(Add({road.first_derivative_x * road_parameter_derivative +
                   normal.x * d_derivative,
               road.first_derivative_y * road_parameter_derivative +
                   normal.y * d_derivative},
              CorrectionDerivative(progress, plan.transition_length_m,
                                   boundary_error)),
          ResidualDerivative(local_distance, residual));
  if (!std::isfinite(sample.position.x) || !std::isfinite(sample.position.y) ||
      !std::isfinite(sample.derivative.x) ||
      !std::isfinite(sample.derivative.y) ||
      Length(sample.derivative) <= kMinimumTangentLength) {
    throw std::runtime_error("stitched lateral curve is degenerate");
  }
  return sample;
}

double ResidualTransitionLength(double residual_magnitude,
                                double maximum_speed) {
  if (residual_magnitude <= kCorrectionMagnitudeTolerance) {
    return 0.0;
  }
  const double jerk_limited_length =
      maximum_speed *
      std::cbrt(kLateralJerkShapeBound * residual_magnitude /
                kCorrectionJerkBudgetMps3);
  return std::min(kMaximumResidualTransitionLengthMeters,
                  std::max(kMinimumResidualTransitionLengthMeters,
                           jerk_limited_length));
}

std::vector<ArcLengthTableEntry> BuildArcLengthTable(
    const LateralCorrectionPlan &plan, double start_progress,
    double start_road_parameter, double required_arc_length,
    const ResidualCorrection &residual, const MapData &map) {
  std::vector<ArcLengthTableEntry> table;
  ArcLengthTableEntry first;
  first.correction_progress_m = start_progress;
  first.road_parameter_s = start_road_parameter;
  first.derivative_magnitude =
      Length(EvaluateCurve(plan, start_progress, start_road_parameter, 0.0,
                           residual, map)
                 .derivative);
  table.push_back(first);

  while (table.size() < 2 ||
         table.back().corrected_arc_length_m + 1e-10 < required_arc_length) {
    if (table.size() >= kMaximumArcLengthTableEntries) {
      throw std::runtime_error("stitched path arc-length table did not converge");
    }
    const ArcLengthTableEntry &previous = table.back();
    double step = kArcLengthTableStepMeters;
    const double plan_boundary =
        plan.transition_length_m - previous.correction_progress_m;
    if (plan_boundary > 1e-12 && plan_boundary < step) {
      step = plan_boundary;
    }
    const double residual_boundary =
        residual.transition_length_m - previous.local_nominal_distance_m;
    if (residual_boundary > 1e-12 && residual_boundary < step) {
      step = residual_boundary;
    }

    ArcLengthTableEntry next;
    next.local_nominal_distance_m =
        previous.local_nominal_distance_m + step;
    next.correction_progress_m = previous.correction_progress_m + step;
    next.road_parameter_s = AdvanceRoadParameter(
        previous.road_parameter_s, step, plan.target_d, map);
    next.derivative_magnitude = Length(
        EvaluateCurve(plan, next.correction_progress_m, next.road_parameter_s,
                      next.local_nominal_distance_m, residual, map)
            .derivative);
    const double corrected_step =
        0.5 * (previous.derivative_magnitude + next.derivative_magnitude) * step;
    if (!std::isfinite(corrected_step) || corrected_step <= 0.0) {
      throw std::runtime_error("stitched path has invalid arc length");
    }
    next.corrected_arc_length_m =
        previous.corrected_arc_length_m + corrected_step;
    table.push_back(next);
  }
  return table;
}

double InterpolateLocalDistance(const ArcLengthTableEntry &lower,
                                const ArcLengthTableEntry &upper,
                                double target_arc_length) {
  const double step =
      upper.local_nominal_distance_m - lower.local_nominal_distance_m;
  const double arc_offset = target_arc_length - lower.corrected_arc_length_m;
  const double speed_slope =
      (upper.derivative_magnitude - lower.derivative_magnitude) / step;
  double distance =
      step * arc_offset /
      (upper.corrected_arc_length_m - lower.corrected_arc_length_m);
  for (int iteration = 0; iteration < 4; ++iteration) {
    const double interpolation_error =
        lower.derivative_magnitude * distance +
        0.5 * speed_slope * distance * distance - arc_offset;
    const double derivative = lower.derivative_magnitude + speed_slope * distance;
    if (std::fabs(derivative) <= kMinimumTangentLength) {
      break;
    }
    distance = std::max(
        0.0, std::min(step, distance - interpolation_error / derivative));
  }
  return lower.local_nominal_distance_m + distance;
}

bool StatesAlignWithInput(const std::vector<LateralPathState> &last_states,
                          const PlannerInput &input,
                          std::size_t output_point_count) {
  const std::size_t previous_size = input.previous_path_x.size();
  if (last_states.size() != output_point_count ||
      previous_size > last_states.size()) {
    return false;
  }
  const std::size_t consumed = last_states.size() - previous_size;
  if (previous_size == 0) {
    if (last_states.empty()) {
      return false;
    }
    return std::hypot(input.ego.x - last_states.back().expected_x,
                      input.ego.y - last_states.back().expected_y) <=
           kMaximumContinuationResidualMeters;
  }
  for (std::size_t index = 0; index < previous_size; ++index) {
    const LateralPathState &expected = last_states[consumed + index];
    if (std::hypot(input.previous_path_x[index] - expected.expected_x,
                   input.previous_path_y[index] - expected.expected_y) >
        kPathAlignmentToleranceMeters) {
      return false;
    }
  }
  return true;
}

LateralCorrectionPlan InitializePlan(
    const PlannerInput &input, double plan_start_s, double current_d,
    double target_d, double maximum_speed, const MapData &map,
    std::uint64_t transition_id) {
  LateralCorrectionPlan plan;
  plan.initialized = true;
  plan.id = transition_id;
  plan.start_d = current_d;
  plan.target_d = target_d;

  const RoadGeometrySample road =
      EvaluateRoadGeometry(plan_start_s, current_d, map);
  const Vector2d road_tangent =
      UnitVector({road.first_derivative_x, road.first_derivative_y}, {1.0, 0.0});
  const double road_parameter_derivative =
      1.0 / RoadParameterMetric(plan_start_s, target_d, map);
  const Vector2d base_derivative = {
      road.first_derivative_x * road_parameter_derivative,
      road.first_derivative_y * road_parameter_derivative};
  const Vector2d road_curvature = RoadArcCurvature(road);
  const Vector2d historical_anchor = HistoricalAnchor(input);
  const Vector2d historical_tangent = HistoricalTangent(input, road_tangent);
  const Vector2d historical_curvature =
      HistoricalCurvature(input, road_curvature);

  BoundaryError error;
  error.position = Subtract(historical_anchor, {road.x, road.y});
  error.tangent = Subtract(historical_tangent, base_derivative);
  error.curvature = Subtract(historical_curvature, road_curvature);
  plan.position_error_x = error.position.x;
  plan.position_error_y = error.position.y;
  plan.tangent_error_x = error.tangent.x;
  plan.tangent_error_y = error.tangent.y;
  plan.curvature_error_x = error.curvature.x;
  plan.curvature_error_y = error.curvature.y;
  plan.has_correction =
      std::fabs(current_d - target_d) > kCorrectionMagnitudeTolerance ||
      Length(error.position) > kCorrectionMagnitudeTolerance ||
      Length(error.tangent) > kCorrectionMagnitudeTolerance ||
      Length(error.curvature) > kCorrectionMagnitudeTolerance;
  if (plan.has_correction) {
    plan.transition_length_m = CorrectionTransitionLength(
        maximum_speed, target_d - current_d, error);
    plan.d_coefficients = FitNormalizedQuintic(
        current_d, 0.0, 0.0, target_d, 0.0, 0.0,
        plan.transition_length_m);
  } else {
    plan.d_coefficients[0] = target_d;
  }
  return plan;
}

void ValidateInput(const PlannerInput &input, double plan_start_s,
                   double lane_d,
                   const std::vector<LongitudinalState> &states,
                   double maximum_planning_speed_mps) {
  if (!std::isfinite(plan_start_s) || !std::isfinite(lane_d) ||
      !std::isfinite(maximum_planning_speed_mps) ||
      maximum_planning_speed_mps < 0.0 ||
      input.previous_path_x.size() != input.previous_path_y.size()) {
    throw std::invalid_argument("invalid stitched-path input");
  }
  double previous_relative_s = 0.0;
  for (const LongitudinalState &state : states) {
    if (!std::isfinite(state.s) || !std::isfinite(state.v) ||
        state.s + 1e-7 < previous_relative_s) {
      throw std::invalid_argument(
          "stitched-path longitudinal states are not monotonic");
    }
    previous_relative_s = state.s;
  }
}

} // namespace

StitchedRoadPathResult PathStitcher::Sample(
    const PlannerInput &input, double plan_start_s, double lane_d,
    const std::vector<LongitudinalState> &states,
    double maximum_planning_speed_mps, const MapData &map) {
  ValidateInput(input, plan_start_s, lane_d, states,
                maximum_planning_speed_mps);

  const std::size_t previous_size = input.previous_path_x.size();
  const std::size_t output_point_count = previous_size + states.size();
  const bool state_aligned =
      StatesAlignWithInput(last_output_states_, input, output_point_count);

  std::vector<LateralPathState> retained_states;
  LateralPathState continuation_state;
  bool has_continuation_state = false;
  if (state_aligned) {
    const std::size_t consumed = last_output_states_.size() - previous_size;
    retained_states.assign(last_output_states_.begin() + consumed,
                           last_output_states_.end());
    if (previous_size > 0 && !retained_states.empty() &&
        retained_states.back().valid) {
      continuation_state = retained_states.back();
      has_continuation_state = true;
    } else if (previous_size == 0 && !last_output_states_.empty() &&
               last_output_states_.back().valid) {
      continuation_state = last_output_states_.back();
      has_continuation_state = true;
    }
  } else {
    retained_states.resize(previous_size);
    for (std::size_t index = 0; index < previous_size; ++index) {
      retained_states[index].expected_x = input.previous_path_x[index];
      retained_states[index].expected_y = input.previous_path_y[index];
    }
  }

  LateralCorrectionPlan next_plan = plan_;
  std::uint64_t next_transition_id = next_transition_id_;
  bool reset = false;
  bool rolling_replanned = false;
  const double maximum_speed =
      MaximumPlanningSpeed(input, states, maximum_planning_speed_mps);
  const double current_d =
      previous_size == 0 ? input.ego.d : input.end_path_d;
  double start_progress = 0.0;
  double start_road_parameter = plan_start_s;
  ResidualCorrection residual;

  const bool continuation_matches_plan =
      has_continuation_state && next_plan.initialized &&
      continuation_state.transition_id == next_plan.id &&
      std::fabs(next_plan.target_d - lane_d) <= 1e-9;
  if (continuation_matches_plan) {
    start_progress = continuation_state.correction_progress_m;
    start_road_parameter = continuation_state.road_parameter_s;
    next_plan = RollPlanAtFixedTerminal(next_plan, start_progress,
                                        &rolling_replanned);
    const CurveSample expected =
        EvaluateCurve(next_plan, start_progress, start_road_parameter, 0.0,
                      ResidualCorrection(), map);
    residual.position = Subtract(HistoricalAnchor(input), expected.position);
    const double residual_magnitude = Length(residual.position);
    if (residual_magnitude <= kMaximumContinuationResidualMeters) {
      residual.transition_length_m =
          ResidualTransitionLength(residual_magnitude, maximum_speed);
    } else {
      has_continuation_state = false;
    }
  }

  if (!continuation_matches_plan || !has_continuation_state) {
    next_plan = InitializePlan(input, plan_start_s, current_d, lane_d,
                               maximum_speed, map, next_transition_id++);
    start_progress = 0.0;
    start_road_parameter = plan_start_s;
    residual = ResidualCorrection();
    reset = true;
    rolling_replanned = false;
  }

  StitchedRoadPathResult result;
  result.new_points.reserve(states.size());
  std::vector<LateralPathState> new_lateral_states;
  new_lateral_states.reserve(states.size());
  if (!states.empty()) {
    const std::vector<ArcLengthTableEntry> arc_table = BuildArcLengthTable(
        next_plan, start_progress, start_road_parameter,
        std::max(0.0, states.back().s), residual, map);
    std::size_t upper_index = 1;
    double previous_target_arc_length = 0.0;
    for (const LongitudinalState &state : states) {
      const double target_arc_length =
          std::max(previous_target_arc_length, std::max(0.0, state.s));
      while (upper_index < arc_table.size() &&
             arc_table[upper_index].corrected_arc_length_m <
                 target_arc_length) {
        ++upper_index;
      }
      if (upper_index >= arc_table.size()) {
        throw std::logic_error("stitched path arc-length lookup is incomplete");
      }
      const ArcLengthTableEntry &lower = arc_table[upper_index - 1];
      const ArcLengthTableEntry &upper = arc_table[upper_index];
      const double local_distance =
          InterpolateLocalDistance(lower, upper, target_arc_length);
      const double progress = start_progress + local_distance;
      const double road_parameter = AdvanceRoadParameter(
          lower.road_parameter_s,
          local_distance - lower.local_nominal_distance_m,
          next_plan.target_d, map);
      const CurveSample sample = EvaluateCurve(
          next_plan, progress, road_parameter, local_distance, residual, map);
      result.new_points.push_back({sample.position.x, sample.position.y});

      LateralPathState lateral_state;
      lateral_state.valid = true;
      lateral_state.transition_id = next_plan.id;
      lateral_state.correction_progress_m = progress;
      lateral_state.road_parameter_s = road_parameter;
      lateral_state.planned_d = sample.d;
      lateral_state.expected_x = sample.position.x;
      lateral_state.expected_y = sample.position.y;
      new_lateral_states.push_back(lateral_state);
      previous_target_arc_length = target_arc_length;
    }
  }

  result.output_states = retained_states;
  result.output_states.insert(result.output_states.end(),
                              new_lateral_states.begin(),
                              new_lateral_states.end());
  if (result.output_states.size() != output_point_count) {
    throw std::logic_error("lateral state queue has the wrong size");
  }

  double frontier_progress = start_progress;
  double frontier_d = PlannedD(next_plan, start_progress);
  if (!new_lateral_states.empty()) {
    frontier_progress = new_lateral_states.back().correction_progress_m;
    frontier_d = new_lateral_states.back().planned_d;
  } else if (has_continuation_state) {
    frontier_d = continuation_state.planned_d;
  }
  result.diagnostics.transition_id = next_plan.id;
  result.diagnostics.transition_length_m = next_plan.transition_length_m;
  result.diagnostics.progress_m =
      std::min(frontier_progress, next_plan.transition_length_m);
  result.diagnostics.remaining_m =
      std::max(0.0, next_plan.transition_length_m - frontier_progress);
  result.diagnostics.transition_active =
      next_plan.has_correction && result.diagnostics.remaining_m > 1e-6;
  result.diagnostics.state_reset = reset;
  result.diagnostics.state_aligned = state_aligned;
  result.diagnostics.rolling_replanned = rolling_replanned;
  result.diagnostics.rolling_replan_count = next_plan.rolling_replan_count;
  result.diagnostics.rolling_origin_m =
      next_plan.polynomial_origin_progress_m;
  result.diagnostics.position_residual_m = Length(residual.position);
  if (reset) {
    result.diagnostics.position_residual_m = std::hypot(
        next_plan.position_error_x, next_plan.position_error_y);
  }
  result.diagnostics.frontier_d = frontier_d;

  plan_ = next_plan;
  next_transition_id_ = next_transition_id;
  last_output_states_ = result.output_states;
  return result;
}
