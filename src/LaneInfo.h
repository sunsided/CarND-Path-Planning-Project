#ifndef PATH_PLANNING_LANEINFO_H
#define PATH_PLANNING_LANEINFO_H

struct LaneInfo {
    struct end {
        int car_id = -1;
        double speed = -1;
        double clearance = -1;
    } front, rear;

    static double getClearance(LaneInfo::end e) {
        return e.car_id < 0 ? std::numeric_limits<double>::max() : e.clearance;
    }
};

#endif //PATH_PLANNING_LANEINFO_H
