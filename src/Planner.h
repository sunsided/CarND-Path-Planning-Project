#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <limits>
#include <vector>
#include "json.hpp"

#include "Map.h"
#include "EnvContext.h"
#include "LaneInfo.h"

class Planner {
private:
    // Finite state machine for motion planning.
    enum {
        LANE_KEEPING,
        LANE_CHANGING
    } fsm = LANE_KEEPING;

    // Highway map.
    const Map& map;

    // Starting states are set by the simulator.
    double velocity = 0;
    int target_lane = 1;

    double planMotion(const EnvContext &context, const std::vector<LaneInfo> &lane_infos);
    void updateVelocity(const EnvContext &context, double target_speed);
    std::vector<std::vector<double>> generateTrajectory(const EnvContext &context) const;

public:
    explicit Planner(const Map &map): map(map) {}
    std::vector<std::vector<double>> planPath(const EnvContext &context);
};

#endif //PATH_PLANNING_PLANNER_H
