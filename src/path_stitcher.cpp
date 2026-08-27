#include "path_stitcher.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "map.h"

namespace {

constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
constexpr double kMinimumTangentLength = 1e-8;
// Simpson integration plus the quadratic local inverse remains well resolved
// at 10 cm: even the shortest allowed lateral transition spans 150 intervals.
constexpr double kArcLengthTableStepMeters = 0.1;
constexpr double kMinimumTransitionLengthMeters = 15.0;
// Reserve Cartesian jerk headroom for longitudinal and road-curvature terms.
constexpr double kCorrectionJerkBudgetMps3 = 4.0;
constexpr double kPositionJerkShapeBound = 100.0;
constexpr double kTangentJerkShapeBound = 40.0;
constexpr double kCurvatureJerkShapeBound = 10.0;
constexpr double kLateralJerkShapeBound = 60.0;
constexpr double kMaximumHistoricalCurvaturePerMeter = 0.1;
constexpr double kMaximumContinuationResidualMeters = 0.2;
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
  BoundaryError error;
  double transition_length_m = 0.0;
  double start_progress_m = 0.0;
};

struct ArcLengthTableEntry {
  double local_nominal_distance_m = 0.0;
  double correction_progress_m = 0.0;
  double road_parameter_s = 0.0;
  double corrected_arc_length_m = 0.0;
  double derivative_magnitude = 0.0;
  // Derivative at the midpoint of the interval ending at this entry.
  double interval_midpoint_derivative_magnitude = 0.0;
};

enum class ArcLengthTableTarget {
  kCorrectedArcLength,
  kLocalNominalDistance
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

Vector2d CorrectionSecondDerivative(double progress,
                                    double transition_length,
                                    const BoundaryError &error) {
  if (transition_length <= 0.0 || progress >= transition_length) {
    return {};
  }
  const double u = std::max(0.0, progress / transition_length);
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double position_basis_second = -60.0 * u + 180.0 * u2 - 120.0 * u3;
  const double tangent_basis_second = -36.0 * u + 96.0 * u2 - 60.0 * u3;
  const double curvature_basis_second = 1.0 - 9.0 * u + 18.0 * u2 - 10.0 * u3;
  return Add(
      Add(Scale(error.position,
                position_basis_second /
                    (transition_length * transition_length)),
          Scale(error.tangent,
                tangent_basis_second / transition_length)),
      Scale(error.curvature, curvature_basis_second));
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

RoadGeometrySample InterpolateRoadGeometry(
    const RoadGeometrySample &center, const RoadGeometrySample &unit_offset,
    double d) {
  RoadGeometrySample result;
  result.x = center.x + d * (unit_offset.x - center.x);
  result.y = center.y + d * (unit_offset.y - center.y);
  result.first_derivative_x =
      center.first_derivative_x +
      d * (unit_offset.first_derivative_x - center.first_derivative_x);
  result.first_derivative_y =
      center.first_derivative_y +
      d * (unit_offset.first_derivative_y - center.first_derivative_y);
  result.second_derivative_x =
      center.second_derivative_x +
      d * (unit_offset.second_derivative_x - center.second_derivative_x);
  result.second_derivative_y =
      center.second_derivative_y +
      d * (unit_offset.second_derivative_y - center.second_derivative_y);
  return result;
}

Vector2d ResidualPosition(double local_distance,
                          const ResidualCorrection &residual) {
  return PositionCorrection(residual.start_progress_m + local_distance,
                            residual.transition_length_m, residual.error);
}

Vector2d ResidualDerivative(double local_distance,
                            const ResidualCorrection &residual) {
  return CorrectionDerivative(residual.start_progress_m + local_distance,
                              residual.transition_length_m, residual.error);
}

CurveSample EvaluateCurve(const LateralCorrectionPlan &plan, double progress,
                          double road_parameter, double local_distance,
                          const ResidualCorrection &residual,
                          const MapData &map) {
  const double d = PlannedD(plan, progress);
  const double d_derivative = PlannedDDerivative(plan, progress);
  // Frenet road geometry is affine in d.  Evaluate the four road splines once
  // at d={0, 1}, then derive the current/target offsets and normal from those
  // samples.  This is algebraically equivalent to four independent geometry
  // evaluations and avoids repeating spline lookup and map validation.
  const RoadGeometrySample center =
      EvaluateRoadGeometry(road_parameter, 0.0, map);
  const RoadGeometrySample unit_offset =
      EvaluateRoadGeometry(road_parameter, 1.0, map);
  const RoadGeometrySample road =
      InterpolateRoadGeometry(center, unit_offset, d);
  const RoadGeometrySample target_road =
      InterpolateRoadGeometry(center, unit_offset, plan.target_d);
  const double target_metric =
      std::hypot(target_road.first_derivative_x,
                 target_road.first_derivative_y);
  if (!std::isfinite(target_metric) ||
      target_metric <= kMinimumTangentLength) {
    throw std::runtime_error("road spline has a degenerate tangent");
  }
  const double road_parameter_derivative = 1.0 / target_metric;
  const Vector2d normal = {unit_offset.x - center.x,
                           unit_offset.y - center.y};
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

std::vector<ArcLengthTableEntry> BuildArcLengthTable(
    const LateralCorrectionPlan &plan, double start_progress,
    double start_road_parameter, double required_extent,
    ArcLengthTableTarget target, const ResidualCorrection &residual,
    const MapData &map) {
  std::vector<ArcLengthTableEntry> table;
  ArcLengthTableEntry first;
  first.correction_progress_m = start_progress;
  first.road_parameter_s = start_road_parameter;
  first.derivative_magnitude =
      Length(EvaluateCurve(plan, start_progress, start_road_parameter, 0.0,
                           residual, map)
                 .derivative);
  table.push_back(first);

  const auto needs_more_entries = [&table, required_extent, target]() {
    if (table.size() < 2) {
      return true;
    }
    return target == ArcLengthTableTarget::kCorrectedArcLength
               ? table.back().corrected_arc_length_m + 1e-10 <
                     required_extent
               : table.back().local_nominal_distance_m + 1e-10 <
                     required_extent;
  };
  while (needs_more_entries()) {
    if (table.size() >= kMaximumArcLengthTableEntries) {
      throw std::runtime_error("stitched path arc-length table did not converge");
    }
    const ArcLengthTableEntry &previous = table.back();
    double step = kArcLengthTableStepMeters;
    if (target == ArcLengthTableTarget::kLocalNominalDistance) {
      const double target_remaining =
          required_extent - previous.local_nominal_distance_m;
      if (target_remaining > 1e-12 && target_remaining < step) {
        step = target_remaining;
      }
    }
    const double plan_boundary =
        plan.transition_length_m - previous.correction_progress_m;
    if (plan_boundary > 1e-12 && plan_boundary < step) {
      step = plan_boundary;
    }
    const double residual_boundary =
        residual.transition_length_m - residual.start_progress_m -
        previous.local_nominal_distance_m;
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
    const double midpoint_local_distance =
        previous.local_nominal_distance_m + 0.5 * step;
    const double midpoint_progress = previous.correction_progress_m +
                                     0.5 * step;
    const double midpoint_road_parameter = AdvanceRoadParameter(
        previous.road_parameter_s, 0.5 * step, plan.target_d, map);
    next.interval_midpoint_derivative_magnitude = Length(
        EvaluateCurve(plan, midpoint_progress, midpoint_road_parameter,
                      midpoint_local_distance, residual, map)
            .derivative);
    const double corrected_step =
        step * (previous.derivative_magnitude +
                4.0 * next.interval_midpoint_derivative_magnitude +
                next.derivative_magnitude) /
        6.0;
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
  const double lower_derivative = lower.derivative_magnitude;
  const double midpoint_derivative =
      upper.interval_midpoint_derivative_magnitude;
  const double upper_derivative = upper.derivative_magnitude;
  // Quadratic Lagrange interpolation through u={0, 0.5, 1}.  Its integral
  // is Simpson's rule at u=1 and provides a phase-stable local inverse.
  const double quadratic =
      2.0 * (lower_derivative + upper_derivative -
             2.0 * midpoint_derivative);
  const double linear =
      4.0 * midpoint_derivative - 3.0 * lower_derivative -
      upper_derivative;
  double u = arc_offset /
             (upper.corrected_arc_length_m - lower.corrected_arc_length_m);
  u = std::max(0.0, std::min(1.0, u));
  for (int iteration = 0; iteration < 6; ++iteration) {
    const double integral =
        step * (quadratic * u * u * u / 3.0 +
                linear * u * u / 2.0 + lower_derivative * u);
    const double derivative =
        step * (quadratic * u * u + linear * u + lower_derivative);
    if (std::fabs(derivative) <= kMinimumTangentLength) {
      break;
    }
    u = std::max(0.0,
                 std::min(1.0, u - (integral - arc_offset) / derivative));
  }
  return lower.local_nominal_distance_m + u * step;
}

LateralCorrectionPlan InitializePlan(
    const PlannerInput &input, double plan_start_s, double current_d,
    double target_d, double maximum_speed, const MapData &map,
    std::uint64_t transition_id,
    const LateralPathState *exact_boundary_state) {
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
  const bool has_exact_tangent =
      exact_boundary_state != nullptr &&
      exact_boundary_state->exact_tangent_valid &&
      std::isfinite(exact_boundary_state->tangent_x) &&
      std::isfinite(exact_boundary_state->tangent_y) &&
      std::hypot(exact_boundary_state->tangent_x,
                 exact_boundary_state->tangent_y) > kMinimumTangentLength;
  const bool has_exact_curvature =
      exact_boundary_state != nullptr &&
      exact_boundary_state->exact_curvature_valid &&
      std::isfinite(exact_boundary_state->curvature_x_per_m) &&
      std::isfinite(exact_boundary_state->curvature_y_per_m);
  const Vector2d historical_tangent =
      has_exact_tangent
          ? UnitVector({exact_boundary_state->tangent_x,
                        exact_boundary_state->tangent_y},
                       road_tangent)
          : HistoricalTangent(input, road_tangent);
  const Vector2d historical_curvature =
      has_exact_curvature
          ? Vector2d(exact_boundary_state->curvature_x_per_m,
                     exact_boundary_state->curvature_y_per_m)
          : HistoricalCurvature(input, road_curvature);

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
        state.s + 1e-4 < previous_relative_s) {
      throw std::invalid_argument(
          "stitched-path longitudinal states are not monotonic");
    }
    previous_relative_s = std::max(previous_relative_s, state.s);
  }
}

} // namespace

FixedSpatialPath PathStitcher::Prepare(
    const PlannerInput &input, double plan_start_s, double current_d,
    double lane_d, double maximum_planning_speed_mps, const MapData &map,
    const PathStitcherState &current_state,
    const std::vector<LateralPathState> &retained_states,
    bool historical_state_aligned) const {
  ValidateInput(input, plan_start_s, lane_d, {}, maximum_planning_speed_mps);
  if (!std::isfinite(current_d) ||
      retained_states.size() != input.previous_path_x.size()) {
    throw std::invalid_argument("invalid fixed spatial path state");
  }

  const std::size_t previous_size = input.previous_path_x.size();
  LateralPathState continuation_state;
  bool has_continuation_state = false;
  if (historical_state_aligned) {
    if (previous_size > 0 && !retained_states.empty() &&
        retained_states.back().valid) {
      continuation_state = retained_states.back();
      has_continuation_state = true;
    } else if (previous_size == 0 &&
               !current_state.last_output_states.empty() &&
               current_state.last_output_states.back().valid) {
      continuation_state = current_state.last_output_states.back();
      has_continuation_state = true;
    }
  }

  LateralCorrectionPlan next_plan = current_state.plan;
  std::uint64_t next_transition_id = current_state.next_transition_id;
  bool reset = false;
  bool rolling_replanned = false;
  const double maximum_speed = maximum_planning_speed_mps;
  double start_progress = 0.0;
  double start_road_parameter = plan_start_s;
  ResidualCorrection residual;
  double received_position_residual_m = 0.0;

  const bool continuation_matches_plan =
      has_continuation_state && next_plan.initialized &&
      continuation_state.transition_id == next_plan.id &&
      std::fabs(next_plan.target_d - lane_d) <= 1e-9;
  if (continuation_matches_plan) {
    start_progress = continuation_state.correction_progress_m;
    start_road_parameter = continuation_state.road_parameter_s;
    const LateralCorrectionPlan previous_plan = next_plan;
    const CurveSample previous_base = EvaluateCurve(
        previous_plan, start_progress, start_road_parameter, 0.0,
        ResidualCorrection(), map);
    next_plan = RollPlanAtFixedTerminal(next_plan, start_progress,
                                        &rolling_replanned);
    const CurveSample next_base =
        EvaluateCurve(next_plan, start_progress, start_road_parameter, 0.0,
                      ResidualCorrection(), map);
    const Vector2d received_offset =
        Subtract(HistoricalAnchor(input),
                 {continuation_state.expected_x,
                  continuation_state.expected_y});
    received_position_residual_m = Length(received_offset);
    if (received_position_residual_m <=
        kMaximumContinuationResidualMeters) {
      BoundaryError previous_residual_error;
      previous_residual_error.position = {
          current_state.residual_correction.position_x_m,
          current_state.residual_correction.position_y_m};
      previous_residual_error.tangent = {
          current_state.residual_correction.tangent_x,
          current_state.residual_correction.tangent_y};
      previous_residual_error.curvature = {
          current_state.residual_correction.curvature_x_per_m,
          current_state.residual_correction.curvature_y_per_m};
      const double previous_residual_progress =
          continuation_state.residual_correction_progress_m;
      const double previous_residual_length =
          current_state.residual_correction.transition_length_m;
      const Vector2d previous_residual_derivative = CorrectionDerivative(
          previous_residual_progress, previous_residual_length,
          previous_residual_error);
      const Vector2d previous_residual_curvature =
          CorrectionSecondDerivative(previous_residual_progress,
                                     previous_residual_length,
                                     previous_residual_error);

      residual.error.position = Subtract(
          Add({continuation_state.expected_x,
               continuation_state.expected_y},
              received_offset),
          next_base.position);
      residual.error.tangent = Subtract(
          Add(previous_base.derivative, previous_residual_derivative),
          next_base.derivative);
      residual.error.curvature = previous_residual_curvature;
      const double previous_remaining =
          std::max(0.0, previous_residual_length -
                            previous_residual_progress);
      residual.transition_length_m = std::max(
          previous_remaining,
          CorrectionTransitionLength(maximum_speed, 0.0, residual.error));
    } else {
      has_continuation_state = false;
    }
  }

  if (!continuation_matches_plan || !has_continuation_state) {
    next_plan = InitializePlan(input, plan_start_s, current_d, lane_d,
                               maximum_speed, map, next_transition_id++,
                               has_continuation_state ? &continuation_state
                                                      : nullptr);
    start_progress = 0.0;
    start_road_parameter = plan_start_s;
    residual = ResidualCorrection();
    reset = true;
    rolling_replanned = false;
  }

  FixedSpatialPath result;
  result.plan = next_plan;
  result.retained_states = retained_states;
  result.next_transition_id = next_transition_id;
  result.start_progress_m = start_progress;
  result.start_road_parameter_s = start_road_parameter;
  result.residual_correction.position_x_m = residual.error.position.x;
  result.residual_correction.position_y_m = residual.error.position.y;
  result.residual_correction.tangent_x = residual.error.tangent.x;
  result.residual_correction.tangent_y = residual.error.tangent.y;
  result.residual_correction.curvature_x_per_m =
      residual.error.curvature.x;
  result.residual_correction.curvature_y_per_m =
      residual.error.curvature.y;
  result.residual_correction.transition_length_m =
      residual.transition_length_m;
  result.residual_start_progress_m = 0.0;
  result.diagnostics.transition_id = next_plan.id;
  result.diagnostics.transition_length_m = next_plan.transition_length_m;
  result.diagnostics.progress_m =
      std::min(start_progress, next_plan.transition_length_m);
  result.diagnostics.remaining_m =
      std::max(0.0, next_plan.transition_length_m - start_progress);
  result.diagnostics.transition_active =
      next_plan.has_correction && result.diagnostics.remaining_m > 1e-6;
  result.diagnostics.state_reset = reset;
  result.diagnostics.state_aligned = historical_state_aligned;
  result.diagnostics.rolling_replanned = rolling_replanned;
  result.diagnostics.rolling_replan_count = next_plan.rolling_replan_count;
  result.diagnostics.rolling_origin_m = next_plan.polynomial_origin_progress_m;
  result.diagnostics.position_residual_m = received_position_residual_m;
  if (reset) {
    result.diagnostics.position_residual_m = std::hypot(
        next_plan.position_error_x, next_plan.position_error_y);
  }
  result.diagnostics.frontier_d =
      has_continuation_state ? continuation_state.planned_d
                             : PlannedD(next_plan, start_progress);
  return result;
}

StitchedRoadPathResult
PathStitcher::Sample(const FixedSpatialPath &path,
                     const std::vector<LongitudinalState> &states,
                     const MapData &map) const {
  double previous_relative_s = 0.0;
  for (const LongitudinalState &state : states) {
    if (!std::isfinite(state.s) || !std::isfinite(state.v) ||
        !std::isfinite(state.a) || !std::isfinite(state.j) ||
        state.s + 1e-7 < previous_relative_s) {
      throw std::invalid_argument(
          "fixed-path longitudinal states are invalid");
    }
    previous_relative_s = state.s;
  }

  ResidualCorrection residual;
  residual.error.position = {path.residual_correction.position_x_m,
                             path.residual_correction.position_y_m};
  residual.error.tangent = {path.residual_correction.tangent_x,
                            path.residual_correction.tangent_y};
  residual.error.curvature = {
      path.residual_correction.curvature_x_per_m,
      path.residual_correction.curvature_y_per_m};
  residual.transition_length_m =
      path.residual_correction.transition_length_m;
  residual.start_progress_m = path.residual_start_progress_m;

  StitchedRoadPathResult result;
  result.diagnostics = path.diagnostics;
  result.new_points.reserve(states.size());
  std::vector<LateralPathState> new_lateral_states;
  new_lateral_states.reserve(states.size());
  if (!states.empty()) {
    const std::vector<ArcLengthTableEntry> arc_table = BuildArcLengthTable(
        path.plan, path.start_progress_m, path.start_road_parameter_s,
        std::max(0.0, states.back().s),
        ArcLengthTableTarget::kCorrectedArcLength, residual, map);
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
      const double progress = path.start_progress_m + local_distance;
      const double road_parameter = AdvanceRoadParameter(
          lower.road_parameter_s,
          local_distance - lower.local_nominal_distance_m,
          path.plan.target_d, map);
      const CurveSample sample = EvaluateCurve(
          path.plan, progress, road_parameter, local_distance, residual, map);
      result.new_points.push_back({sample.position.x, sample.position.y});

      LateralPathState lateral_state;
      lateral_state.valid = true;
      lateral_state.transition_id = path.plan.id;
      lateral_state.correction_progress_m = progress;
      lateral_state.residual_correction_progress_m =
          path.residual_start_progress_m + local_distance;
      lateral_state.road_parameter_s = road_parameter;
      lateral_state.planned_d = sample.d;
      lateral_state.expected_x = sample.position.x;
      lateral_state.expected_y = sample.position.y;
      lateral_state.exact_tangent_valid = true;
      const Vector2d tangent = UnitVector(sample.derivative, {1.0, 0.0});
      lateral_state.tangent_x = tangent.x;
      lateral_state.tangent_y = tangent.y;
      new_lateral_states.push_back(lateral_state);
      previous_target_arc_length = target_arc_length;
    }
  }

  result.output_states = path.retained_states;
  result.output_states.insert(result.output_states.end(),
                              new_lateral_states.begin(),
                              new_lateral_states.end());
  if (result.output_states.size() !=
      path.retained_states.size() + states.size()) {
    throw std::logic_error("lateral state queue has the wrong size");
  }
  result.next_state.plan = path.plan;
  result.next_state.residual_correction = path.residual_correction;
  result.next_state.next_transition_id = path.next_transition_id;
  result.next_state.last_output_states = result.output_states;
  return result;
}

double FixedPathProgressToRoadParameterDistance(
    const FixedSpatialPath &path, double target_road_s_unwrapped_m,
    const MapData &map) {
  if (!std::isfinite(target_road_s_unwrapped_m)) {
    throw std::invalid_argument("invalid fixed-path projection target");
  }
  const double start = NormalizeS(path.start_road_parameter_s, map.track_length);
  const double target = NormalizeS(target_road_s_unwrapped_m, map.track_length);
  double parameter_distance = target - start;
  if (parameter_distance < 0.0) {
    parameter_distance += map.track_length;
  }
  const double nominal_distance = RoadArcLength(
      path.start_road_parameter_s, parameter_distance, path.plan.target_d, map);
  if (nominal_distance <= 1e-12) {
    return 0.0;
  }
  ResidualCorrection residual;
  residual.error.position = {path.residual_correction.position_x_m,
                             path.residual_correction.position_y_m};
  residual.error.tangent = {path.residual_correction.tangent_x,
                            path.residual_correction.tangent_y};
  residual.error.curvature = {
      path.residual_correction.curvature_x_per_m,
      path.residual_correction.curvature_y_per_m};
  residual.transition_length_m =
      path.residual_correction.transition_length_m;
  residual.start_progress_m = path.residual_start_progress_m;
  const std::vector<ArcLengthTableEntry> table = BuildArcLengthTable(
      path.plan, path.start_progress_m, path.start_road_parameter_s,
      nominal_distance, ArcLengthTableTarget::kLocalNominalDistance,
      residual, map);
  return table.empty() ? 0.0 : table.back().corrected_arc_length_m;
}
