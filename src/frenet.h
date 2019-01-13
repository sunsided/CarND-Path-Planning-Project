#ifndef PATH_PLANNING_FRENET_H
#define PATH_PLANNING_FRENET_H

#include <vector>

/**
 * Transforms from Cartesian (x,y) coordinates to Frenet (s,d) coordinates
 * @param x
 * @param y
 * @param theta
 * @param maps_x
 * @param maps_y
 * @return
 */
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

/**
 * Transform from Frenet (s,d) coordinates to Cartesian (x,y) coordinates
 * @param s
 * @param d
 * @param maps_x
 * @param maps_y
 * @return
 */
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

#endif //PATH_PLANNING_FRENET_H
