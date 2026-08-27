#ifndef MAP_H
#define MAP_H

#include <string>
#include <utility>
#include <vector>

#include "planner_types.h"

struct RoadGeometrySample {
  double x = 0.0;
  double y = 0.0;
  double first_derivative_x = 0.0;
  double first_derivative_y = 0.0;
  double second_derivative_x = 0.0;
  double second_derivative_y = 0.0;
};

struct RoadProjection {
  double road_s_unwrapped_m = 0.0;
  double d_m = 0.0;
  double residual_m = 0.0;
};

// A bounded, monotonic mapping from physical arc length to the unwrapped road
// parameter for one fixed lateral offset.  The index is built once and can be
// reused for every prediction timestamp in a planning cycle.
struct RoadArcLengthIndex {
  double start_road_parameter_s = 0.0;
  double lateral_offset_m = 0.0;
  std::vector<double> road_parameter_s;
  std::vector<double> cumulative_arc_length_m;
  std::vector<double> parameter_metric;
  // Midpoint metric for the interval ending at the corresponding entry.  The
  // first entry has no preceding interval and stores zero.
  std::vector<double> interval_midpoint_metric;
};

MapData LoadMap(const std::string &path);
void PrepareMapSplines(MapData *map);
bool ValidateMap(const MapData &map, std::string *error);
double NormalizeS(double s, double track_length);
RoadGeometrySample EvaluateRoadGeometry(double s, double d, const MapData &map);
// Requires a successful ValidateMap() for the same immutable map snapshot.
// This is the hot-path counterpart of EvaluateRoadGeometry(): it intentionally
// does not rescan all map arrays on every query.
RoadGeometrySample EvaluateRoadGeometryOnValidatedMap(double s, double d,
                                                       const MapData &map);
double RoadParameterMetric(double s, double d, const MapData &map);
double RoadArcLength(double start_s, double parameter_distance, double d,
                     const MapData &map);
double AdvanceRoadParameter(double start_s, double distance_meters, double d,
                            const MapData &map);
RoadArcLengthIndex BuildRoadArcLengthIndex(double start_s,
                                           double maximum_distance_meters,
                                           double d, const MapData &map);
// Requires a successful ValidateMap() for the same immutable map snapshot.
RoadArcLengthIndex BuildRoadArcLengthIndexOnValidatedMap(
    double start_s, double maximum_distance_meters, double d,
    const MapData &map);
double RoadParameterAtArcLength(const RoadArcLengthIndex &index,
                                double distance_meters);
RoadProjection ProjectCartesianToRoad(double x, double y,
                                      double initial_road_s_unwrapped_m,
                                      const MapData &map);
std::pair<double, double> FrenetToCartesian(double s, double d,
                                            const MapData &map);

#endif // MAP_H
