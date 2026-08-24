#ifndef PLANNER_TYPES_H
#define PLANNER_TYPES_H

#include <vector>

struct MapData {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  std::vector<double> dx;
  std::vector<double> dy;
  // Periodic cubic-spline second derivatives. LoadMap() populates these once
  // so runtime road evaluation is O(log N). Hand-built maps may leave them
  // empty; map evaluation will then build a temporary spline.
  std::vector<double> spline_x_second;
  std::vector<double> spline_y_second;
  std::vector<double> spline_dx_second;
  std::vector<double> spline_dy_second;
  double track_length = 0.0;
};

struct EgoState {
  double x = 0.0;
  double y = 0.0;
  double s = 0.0;
  double d = 0.0;
  double yaw_deg = 0.0;
  double speed_mph = 0.0;
};

struct DetectedVehicle {
  double id = 0.0;
  double x = 0.0;
  double y = 0.0;
  double vx_mps = 0.0;
  double vy_mps = 0.0;
  double s = 0.0;
  double d = 0.0;
};

struct PlannerInput {
  EgoState ego;
  std::vector<double> previous_path_x;
  std::vector<double> previous_path_y;
  double end_path_s = 0.0;
  double end_path_d = 0.0;
  std::vector<DetectedVehicle> traffic;
};

struct PlannerOutput {
  std::vector<double> next_x;
  std::vector<double> next_y;
};

#endif // PLANNER_TYPES_H
