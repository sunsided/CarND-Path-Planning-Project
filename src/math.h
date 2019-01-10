//
// Created by markus on 11.01.19.
//

#ifndef PATH_PLANNING_MATH_H
#define PATH_PLANNING_MATH_H

#include <type_traits>

inline double distance(const double x1, const double y1, const double x2, const double y2) {
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline constexpr double deg2rad(const double x) { return x * pi() / 180; }
inline constexpr double rad2deg(const double x) { return x * 180 / pi(); }

#endif //PATH_PLANNING_MATH_H
