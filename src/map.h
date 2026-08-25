#ifndef MAP_H
#define MAP_H

#include <string>
#include <utility>

#include "planner_types.h"

struct RoadGeometrySample {
  double x = 0.0;
  double y = 0.0;
  double first_derivative_x = 0.0;
  double first_derivative_y = 0.0;
  double second_derivative_x = 0.0;
  double second_derivative_y = 0.0;
};

MapData LoadMap(const std::string &path);
void PrepareMapSplines(MapData *map);
bool ValidateMap(const MapData &map, std::string *error);
double NormalizeS(double s, double track_length);
RoadGeometrySample EvaluateRoadGeometry(double s, double d, const MapData &map);
double RoadParameterMetric(double s, double d, const MapData &map);
double RoadArcLength(double start_s, double parameter_distance, double d,
                     const MapData &map);
double AdvanceRoadParameter(double start_s, double distance_meters, double d,
                            const MapData &map);
std::pair<double, double> FrenetToCartesian(double s, double d,
                                            const MapData &map);

#endif // MAP_H
