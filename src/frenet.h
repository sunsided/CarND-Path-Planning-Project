#ifndef PATH_PLANNING_FRENET_H
#define PATH_PLANNING_FRENET_H

#include <vector>

/**
 * Transform from Cartesian (x,y) coordinates to Frenet (s,d) coordinates
 * @param x
 * @param y
 * @param theta
 * @param maps_x
 * @param maps_y
 * @return
 */
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

#endif //PATH_PLANNING_FRENET_H
