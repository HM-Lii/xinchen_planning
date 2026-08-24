#ifndef TRAFFIC_PREDICTOR_H
#define TRAFFIC_PREDICTOR_H

#include <vector>

#include "longitudinal_types.h"
#include "planner_types.h"

double ForwardTrackDistance(double from_s, double to_s, double track_length);

std::vector<PredictedObstacle>
PredictRelevantTraffic(const PlannerInput &input, double plan_start_s,
                       double lane_center_d, double track_length,
                       const TrafficPredictionConfig &config);

#endif // TRAFFIC_PREDICTOR_H
