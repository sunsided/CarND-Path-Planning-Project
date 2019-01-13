#ifndef PATH_PLANNING_LANEINFO_H
#define PATH_PLANNING_LANEINFO_H

#include <algorithm>
#include <vector>
#include "TrackedVehicle.h"

/**
 * The LaneInfo struct keeps track of all observed
 * vehicles in front and rear of the ego car.
 *
 * There will always be at least one entry in the
 * front and rear lists; validity of that entry
 * needs to be evaluated separately.
 */
struct LaneInfo {
public:
    LaneInfo() noexcept
        : lane_index{-1}
    {
        // Add dummy entry so the list is never empty.
        const auto max = std::numeric_limits<double>::max();
        const auto nan = std::numeric_limits<double>::quiet_NaN();
        front.emplace_back(-1, max, nan);
        rear.emplace_back(-1, 0, nan);
    }

    /**
     * Sorts the list by increasing clearance.
     */
    void sort() {
        auto sort_fn = [](const TrackedVehicle &a, const TrackedVehicle &b) -> bool
        {
            if (std::isnan(a.clearance())) return !std::isnan(b.clearance());
            if (std::isnan(b.clearance())) return true;
            return a.clearance() < b.clearance();
        };

        std::sort(std::begin(front), std::end(front), sort_fn);
        std::sort(std::begin(rear), std::end(rear), sort_fn);
    }

public:
    /**
     * The lane ID.
     */
    int lane_index;

    /**
     * The tracked vehicles in front of ego.
     */
    std::vector<TrackedVehicle> front;

    /**
     * The tracked vehicles behind ego.
     */
    std::vector<TrackedVehicle> rear;
};

#endif //PATH_PLANNING_LANEINFO_H
