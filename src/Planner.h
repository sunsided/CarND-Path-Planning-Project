#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <limits>
#include <array>
#include <vector>
#include <json.hpp>

#include "Map.h"
#include "EnvContext.h"
#include "LaneInfo.h"
#include "PlanningState.h"
#include "Trajectory.h"

class Planner {
public:
    explicit Planner(const Map &map) noexcept
        : map{map} {}

    /**
     * Generates a plath plan according to the current environment.
     * @param context The environment.
     * @return The generated path.
     */
    Trajectory planPath(const EnvContext &context);

private:
    /**
     * Builds the lane information
     * @param context
     * @return
     */
    std::vector<LaneInfo> buildLaneInfos(const EnvContext &context) const;

    /**
     * Prepares according to the current environment and lane assessment.
     * @param context The environment.
     * @param lane_infos The prepared lane information.
     * @return The target speed.
     */
    double planMotion(const EnvContext &context, const std::vector<LaneInfo> &lane_infos);

    /**
     * Decides whether a lane change should take place.
     * @param current_lane The current lane.
     * @param lane_infos The lane information.
     * @param context The environment.
     * @return The target lane.
     */
    int decideLaneChange(int current_lane, const std::vector<LaneInfo> &lane_infos,
                         const EnvContext &context) const;

    /**
     * Updates velocity based on the target speed.
     * @param context The environment.
     * @param target_speed The target speed.
     */
    void updateVelocity(const EnvContext &context, double target_speed);

    /**
     * Generates a smooth and jerk-minimizing trajectory.
     * @param context The environment.
     * @return The generated trajectory in global cartesian coordinates.
     */
    Trajectory generateTrajectory(const EnvContext &context) const;

private:
    /**
     * The current state.
     */
    PlanningState state = PlanningState::KeepLane;

    /**
     * The map we're navigating in.
     */
    const Map &map;

    /**
     * Initial velocity as given by the simulator.
     */
    double velocity = 0;

    /**
     * The lane we're staying at or are navigating to.
     */
    int target_lane = 1;
};

#endif //PATH_PLANNING_PLANNER_H
