#ifndef TRAJECTORY_ASSEMBLER_H
#define TRAJECTORY_ASSEMBLER_H

#include <cstddef>
#include <limits>
#include <vector>

#include "longitudinal_qp.h"
#include "planning_snapshot.h"
#include "trajectory_validator.h"

struct TrajectoryCanonicalizationConfig {
  double sample_time_step_s = 0.02;
  std::size_t sample_count = 0;
  double minimum_acceleration_mps2 = 0.0;
  double maximum_acceleration_mps2 = 0.0;
  double maximum_jerk_mps3 = 0.0;
  double maximum_progress_m = std::numeric_limits<double>::infinity();
  // Callers use static diagnostic text so candidate construction does not
  // allocate merely to prepare an error path.
  const char *invalid_trajectory_message = "invalid canonical trajectory";
};

struct TrajectoryOutputSlice {
  PlannerOutput output;
  std::vector<LongitudinalState> longitudinal;
  std::vector<LateralPathState> lateral;
};

std::vector<LongitudinalState>
SampleCanonicalTrajectory(const LongitudinalQpResult &qp,
                          const TrajectoryCanonicalizationConfig &config);

FullTrajectory InitializeFullTrajectory(const PlanningSnapshot &snapshot,
                                        double time_step_s,
                                        double planning_horizon_s,
                                        std::size_t generated_point_count);

void ValidateRetainedTrajectoryPrefix(const PlanningSnapshot &snapshot);

void AppendRetainedTrajectoryPrefix(const PlanningSnapshot &snapshot,
                                    double time_step_s,
                                    double expected_frame_offset_x,
                                    double expected_frame_offset_y,
                                    bool normalize_expected_frame,
                                    FullTrajectory *trajectory);

TrajectoryOutputSlice SliceTrajectoryOutput(const FullTrajectory &trajectory,
                                            std::size_t output_points,
                                            const char *error_message);

#endif // TRAJECTORY_ASSEMBLER_H
