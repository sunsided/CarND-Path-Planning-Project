#ifndef PATH_PLANNING_WAYPOINTS_H
#define PATH_PLANNING_WAYPOINTS_H

#include <cstddef>
#include <vector>

size_t closestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
size_t nextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

#endif //PATH_PLANNING_WAYPOINTS_H
