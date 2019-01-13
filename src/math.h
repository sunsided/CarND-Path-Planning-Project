#ifndef PATH_PLANNING_MATH_H
#define PATH_PLANNING_MATH_H

#include <cmath>
#include <type_traits>

inline double distance(const double x1, const double y1, const double x2, const double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

inline constexpr double pi() { return M_PI; }
inline constexpr double deg2rad(const double x) { return x * pi() / 180; }
inline constexpr double rad2deg(const double x) { return x * 180 / pi(); }

inline constexpr double mph_to_mps(const double mph) {
    return mph * 1609.344 /*meters per mile*/ / 3600 /*sec per hour*/;
}

inline constexpr double mps_to_mph(const double mps) {
    return mps / mph_to_mps(1.0);
}


#endif //PATH_PLANNING_MATH_H
