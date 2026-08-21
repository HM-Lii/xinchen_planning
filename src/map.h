#ifndef MAP_H
#define MAP_H

#include <string>
#include <utility>

#include "planner_types.h"

MapData LoadMap(const std::string &path);
bool ValidateMap(const MapData &map, std::string *error);
double NormalizeS(double s, double track_length);
std::pair<double, double> FrenetToCartesian(double s, double d,
                                            const MapData &map);

#endif // MAP_H
