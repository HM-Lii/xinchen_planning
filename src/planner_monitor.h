#ifndef PLANNER_MONITOR_H
#define PLANNER_MONITOR_H

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "longitudinal_types.h"
#include "path_stitcher.h"
#include "planner_types.h"

struct PlannerMonitorConfig {
  bool enabled = false;
  bool write_csv = true;
  std::string log_directory = "logs";
  std::size_t console_summary_interval_cycles = 50;
  std::size_t violation_report_interval_cycles = 10;
  std::size_t detail_csv_interval_cycles = 50;
  double maximum_cartesian_acceleration_mps2 = 10.0;
  double maximum_cartesian_jerk_mps3 = 10.0;
  double speed_tolerance_mps = 0.05;
  double acceleration_tolerance_mps2 = 0.05;
  double jerk_tolerance_mps3 = 0.05;
  double safety_tolerance_meters = 0.05;
};

struct IndexedMetric {
  bool valid = false;
  double value = 0.0;
  std::size_t index = 0;
};

struct CartesianKinematicSample {
  std::size_t output_index = 0;
  bool is_previous_path = false;
  double x = 0.0;
  double y = 0.0;
  double velocity_x_mps = 0.0;
  double velocity_y_mps = 0.0;
  double speed_mps = 0.0;
  double acceleration_x_mps2 = 0.0;
  double acceleration_y_mps2 = 0.0;
  double acceleration_mps2 = 0.0;
  double tangential_acceleration_mps2 = 0.0;
  bool jerk_valid = false;
  double jerk_x_mps3 = 0.0;
  double jerk_y_mps3 = 0.0;
  double jerk_mps3 = 0.0;
  double tangential_jerk_mps3 = 0.0;
};

struct QpNodeMonitorSample {
  std::size_t node_index = 0;
  double time_seconds = 0.0;
  LongitudinalState state;
  double reference_speed_mps = 0.0;
  bool safety_margin_valid = false;
  double minimum_safety_margin_meters = 0.0;
  double limiting_obstacle_id = 0.0;
};

struct PlannerMonitorLimits {
  double output_time_step_seconds = 0.02;
  double maximum_speed_mps = 0.0;
  double minimum_acceleration_mps2 = 0.0;
  double maximum_acceleration_mps2 = 0.0;
  double maximum_jerk_mps3 = 0.0;
  double maximum_cartesian_acceleration_mps2 = 10.0;
  double maximum_cartesian_jerk_mps3 = 10.0;
  double speed_tolerance_mps = 0.05;
  double acceleration_tolerance_mps2 = 0.05;
  double jerk_tolerance_mps3 = 0.05;
  double safety_tolerance_meters = 0.05;
  double maximum_lateral_deviation_meters = 1.65;
  double time_headway_seconds = 0.0;
  double fixed_safety_gap_meters = 0.0;
};

struct PlannerCycleDiagnostics {
  std::uint64_t cycle = 0;
  std::size_t previous_path_size = 0;
  std::size_t new_point_count = 0;
  double ego_speed_mps = 0.0;
  double initial_speed_mps = 0.0;
  double initial_acceleration_mps2 = 0.0;
  double initial_jerk_mps3 = 0.0;
  double plan_start_s = 0.0;
  double plan_start_d = 0.0;
  double lane_center_d = 0.0;
  bool historical_plan_aligned = false;
  LateralStitchDiagnostics lateral;
  std::size_t relevant_obstacle_count = 0;
  IndexedMetric nearest_obstacle_distance;

  double reference_first_mps = 0.0;
  double reference_minimum_mps = 0.0;
  double reference_last_mps = 0.0;
  std::string qp_status;
  double qp_objective = 0.0;
  bool emergency = false;

  IndexedMetric qp_minimum_speed;
  IndexedMetric qp_maximum_speed;
  IndexedMetric qp_minimum_acceleration;
  IndexedMetric qp_maximum_acceleration;
  IndexedMetric qp_maximum_absolute_jerk;
  IndexedMetric qp_minimum_safety_margin;
  double qp_limiting_obstacle_id = 0.0;

  IndexedMetric cartesian_maximum_speed;
  IndexedMetric cartesian_minimum_tangential_acceleration;
  IndexedMetric cartesian_maximum_tangential_acceleration;
  IndexedMetric cartesian_maximum_acceleration;
  IndexedMetric cartesian_maximum_absolute_tangential_jerk;
  IndexedMetric cartesian_maximum_jerk;
  IndexedMetric new_path_junction_speed;
  IndexedMetric historical_path_position_residual;
  bool cartesian_metrics_new_path_only = false;

  bool qp_speed_violation = false;
  bool qp_acceleration_violation = false;
  bool qp_jerk_violation = false;
  bool qp_safety_violation = false;
  bool cartesian_speed_violation = false;
  bool cartesian_acceleration_violation = false;
  bool cartesian_jerk_violation = false;
  bool lane_deviation_violation = false;

  std::vector<CartesianKinematicSample> cartesian_samples;
  std::vector<LongitudinalState> output_longitudinal_states;
  std::vector<LateralPathState> output_lateral_states;
  std::vector<QpNodeMonitorSample> qp_samples;

  bool HasViolation() const;
};

std::vector<CartesianKinematicSample> ComputeCartesianKinematics(
    const PlannerInput &input, const PlannerOutput &output,
    double time_step_seconds, std::size_t previous_path_size);

PlannerCycleDiagnostics BuildPlannerDiagnostics(
    std::uint64_t cycle, const PlannerInput &input, const PlannerOutput &output,
    std::size_t previous_path_size, const LongitudinalState &initial_state,
    double plan_start_s, double plan_start_d, double lane_center_d,
    const std::vector<PredictedObstacle> &obstacles,
    const std::vector<double> &reference_speed_mps,
    const LongitudinalQpResult &qp_result,
    const std::vector<LongitudinalState> &output_longitudinal_states,
    const PlannerMonitorLimits &limits,
    const LateralStitchDiagnostics &lateral = LateralStitchDiagnostics(),
    const std::vector<LateralPathState> &output_lateral_states =
        std::vector<LateralPathState>(),
    bool historical_plan_aligned = false);

class PlannerRuntimeMonitor {
public:
  explicit PlannerRuntimeMonitor(
      const PlannerMonitorConfig &config = PlannerMonitorConfig());

  void Record(const PlannerCycleDiagnostics &diagnostics);

private:
  void EnsureCsvStreams();
  void WriteCycleCsv(const PlannerCycleDiagnostics &diagnostics);
  void WriteDetailCsv(const PlannerCycleDiagnostics &diagnostics);
  void PrintSummary(const PlannerCycleDiagnostics &diagnostics) const;
  void PrintViolation(const PlannerCycleDiagnostics &diagnostics) const;

  PlannerMonitorConfig config_;
  std::uint64_t session_id_ = 0;
  std::uint64_t last_violation_report_cycle_ = 0;
  unsigned int previous_violation_mask_ = 0;
  bool csv_initialization_attempted_ = false;
  bool csv_available_ = false;
  std::ofstream cycle_csv_;
  std::ofstream point_csv_;
  std::ofstream qp_csv_;
};

#endif // PLANNER_MONITOR_H
