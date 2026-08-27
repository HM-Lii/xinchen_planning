#ifndef TRAFFIC_PREDICTOR_H
#define TRAFFIC_PREDICTOR_H

#include <vector>

#include "longitudinal_types.h"
#include "planner_types.h"

struct FixedSpatialPath;

double ForwardTrackDistance(double from_s, double to_s, double track_length);

std::vector<PredictedObstacle>
PredictRelevantTraffic(const PlannerInput &input, double plan_start_s,
                       double plan_start_d, double lane_center_d,
                       const MapData &map,
                       const TrafficPredictionConfig &config,
                       const FixedSpatialPath *fixed_path = nullptr);

#endif // TRAFFIC_PREDICTOR_H
