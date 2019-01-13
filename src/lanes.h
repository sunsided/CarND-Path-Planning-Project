#ifndef PATH_PLANNING_LANES_H
#define PATH_PLANNING_LANES_H

/**
 * Gets the offset to the lane center.
 * @param lane The lane index.
 * @param lane_width The width of a lane.
 * @return The d value of the lane center.
 */
inline constexpr double get_lane_center(const int lane, const int lane_width) {
    return lane_width * 0.5 + lane_width * lane;
}

/**
 * Determines the lane number from a given d value.
 * @param d The lateral offset in Frenet coordinates.
 * @param lane_width The width of a lane.
 * @return The index of the lane.
 */
inline constexpr int get_lane_number(const double d, const int lane_width) {
    return static_cast<int>(d / lane_width);
}

#endif //PATH_PLANNING_LANES_H
