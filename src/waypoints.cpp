#include <limits>
#include <cmath>
#include "math.h"
#include "waypoints.h"

size_t closestWaypoint(const double x, const double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y) {
    size_t closest = 0;
    auto closestLen = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < maps_x.size(); ++i) {
        const auto map_x = maps_x[i];
        const auto map_y = maps_y[i];
        const auto dist = distance(x, y, map_x, map_y);
        if (dist < closestLen) {
            closestLen = dist;
            closest = i;
        }
    }

    return closest;
}

size_t nextWaypoint(const double x, const double y, const double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y) {
    auto closest = closestWaypoint(x, y, maps_x, maps_y);

    const auto map_x = maps_x[closest];
    const auto map_y = maps_y[closest];
    const auto heading = atan2((map_y - y), (map_x - x));

    auto angle = fabs(theta - heading);
    angle = std::min(2 * pi() - angle, angle);

    if (angle > pi() / 4) {
        ++closest;
        if (closest == maps_x.size()) {
            closest = 0;
        }
    }

    return closest;
}