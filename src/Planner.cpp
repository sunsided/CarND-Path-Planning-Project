#include <cstdio>
#include <cassert>
#include <exception>
#include <iostream>
#include <fstream>
#include "spline.h"
#include "math.h"

#include "lanes.h"
#include "Planner.h"

using namespace std;

/**
 * Builds information about each lane.
 * @param context
 * @return a vector of lane information.
 */
static vector<LaneInfo> buildLaneInfos(const EnvContext &context) {
    // Initialize the returned vector.
    vector<LaneInfo> ret((unsigned) context.num_lanes);

    const auto prev_size = context.previous_path_x.size();
    const double car_s = prev_size > 0 ? context.end_path_s : context.car_s;

    // Loop through each tracked vehicle.
    for (auto check_car : context.sensor_fusion) {
        auto id = (int) check_car[0];
        auto d = check_car[6];
        auto lane = get_lane_number(d, context.lane_width);
        if (lane < 0 || lane >= 3) {
            // Around the start, the simulator can return mal-valued car information.
            // printf("Ignored invalid sensor_fusion data: car=%d, d=%f\n", id, d);
            continue;
        }
        auto vx = check_car[3];
        auto vy = check_car[4];
        auto check_speed = sqrt(vx * vx + vy * vy);
        auto check_car_s = check_car[5];
        // Take the simulator delay into account to predict where the car is.
        check_car_s += prev_size * context.sim_update_freq * check_speed;

        if (abs(check_car_s - car_s) < 2) {
            // The car is too close to us.
            ret[lane].front.car_id = ret[lane].rear.car_id = id;
            ret[lane].front.clearance = ret[lane].rear.clearance = 0;
            ret[lane].front.speed = ret[lane].rear.speed = check_speed;
        }

        auto &obj = check_car_s < car_s ? ret[lane].rear : ret[lane].front;
        if (obj.car_id < 0 || abs(car_s - check_car_s) < obj.clearance) {
            obj.car_id = id;
            obj.clearance = abs(car_s - check_car_s);
            obj.speed = check_speed;
        }
    }
    return ret;
}

/**
 * Decide if we want a lane change.
 * @param current_lane
 * @param lane_infos
 * @param context
 * @return the target lane
 */
int decideLaneChange(const int current_lane, const vector<LaneInfo> &lane_infos,
                     const EnvContext &context) {
    vector<int> candidates;
    if (current_lane > 0) {
        // Not the left most lane.
        candidates.emplace_back(current_lane - 1);
    }
    if (current_lane < context.num_lanes - 1) {
        // Not the right most lane.
        candidates.emplace_back(current_lane + 1);
    }

    vector<int> checked_candidates;
    for (auto candidate : candidates) {
        // Check if it is okay to switch to the candidate lane.
        const auto &front = lane_infos[candidate].front;
        if (front.car_id > 0) {
            if (front.clearance < 30) {
                printf("Skip lane %d: front clearance %.2f too small\n",
                       candidate, front.clearance);
                continue;
            }
            if (front.clearance < 45 && front.speed < lane_infos[current_lane].front.speed) {
                printf("Skip lane %d: front clearance %.2f, but speed %.2f < current lane %.2f\n",
                       candidate, front.clearance, front.speed, lane_infos[current_lane].front.speed);
                continue;
            }
        }

        const auto &rear = lane_infos[candidate].rear;
        if (rear.car_id > 0) {
            if (rear.clearance < 5) {
                printf("Skip lane %d: rear clearance %.2f too small\n",
                       candidate, rear.clearance);
                continue;
            }
            if (rear.speed > max(context.car_speed * 1.1, context.car_speed + 5) &&
                rear.clearance < 15) {
                printf("Skip lane %d: rear clearance %.2f too small for relative speed (%.2f vs %.2f)\n",
                       candidate, rear.clearance, rear.speed, context.car_speed);
                continue;
            }
        }

        printf("Take lane %d: front car=%2d clearance=%.2f speed=%.2f, "
               "rear car=%2d clearance=%.2f speed=%.2f\n",
               candidate, front.car_id, front.clearance, front.speed,
               rear.car_id, rear.clearance, rear.speed);
        checked_candidates.emplace_back(candidate);
    }

    if (checked_candidates.empty()) {
        // Stay at the current lane.
        return current_lane;
    } else {
        // Find the best candidate.
        sort(checked_candidates.begin(), checked_candidates.end(),
             [&lane_infos](const int lhs, const int rhs) {
                 double lhs_front = LaneInfo::getClearance(lane_infos[lhs].front);
                 double rhs_front = LaneInfo::getClearance(lane_infos[rhs].front);
                 if (lhs_front != rhs_front) {
                     return lhs_front > rhs_front;
                 }
                 double lhs_rear = LaneInfo::getClearance(lane_infos[lhs].rear);
                 double rhs_rear = LaneInfo::getClearance(lane_infos[rhs].rear);
                 return lhs_rear > rhs_rear;

             });
        return checked_candidates[0];
    }
}

/**
 * Plan the motion by determining target lane and speed.
 * @param context
 * @param lane_infos
 * @return target speed (target lane is a class variable)
 */
double Planner::planMotion(const EnvContext &context, const vector<LaneInfo> &lane_infos) {
    int current_lane = get_lane_number(context.car_d, context.lane_width);
    assert(abs(target_lane - current_lane) <= 1);

    double target_speed = 0;
    if (fsm == LANE_KEEPING) {
        const auto &front = lane_infos[current_lane].front;
        if (front.car_id < 0 || front.clearance > 30) {
            // No car is in the safe distance in front of our lane.
            target_speed = mph_to_mps(context.speed_limit_mph);
        } else if (front.clearance < 15) {
            // Critically close to the car in front.
            if (front.clearance < 5) {
                target_speed = min(front.speed * 0.75, front.speed - 10);
            } else {
                target_speed = min(front.speed * 0.9, front.speed - 5);
            }
            printf("Critically close to front car %d, clearance %.2f, set speed from %.2f to %.2f\n",
                   front.car_id, front.clearance, context.car_speed, target_speed);
        } else {

            // Decide if we want to change lane.
            target_lane = decideLaneChange(current_lane, lane_infos, context);
            if (target_lane != current_lane) {
                printf("Switch to LANE_CHANGING mode (from %d to %d)\n", current_lane, target_lane);
                fsm = LANE_CHANGING;
            } else {
                // Match speed with the car in front in the current lane.
                target_speed = min(front.speed, mph_to_mps(context.speed_limit_mph));
            }
        }
    }
    if (fsm == LANE_CHANGING) {
        // TODO: check for collision and abort lane changing if necessary.

        // Match speed with the target lane.
        const auto &front = lane_infos[target_lane].front;
        if (front.car_id < 0 || front.clearance > 30) {
            // No car is within the safe distance in the target lane.
            target_speed = mph_to_mps(context.speed_limit_mph);
        } else {
            // Match speed with the car in front in the target lane.
            target_speed = min(front.speed, mph_to_mps(context.speed_limit_mph));
        }

        // Check if we are done with lane changing.
        if (abs(get_lane_center(target_lane, context.lane_width) - context.car_d) < 0.2) {
            printf("Switch back to LANE_KEEPING mode\n");
            fsm = LANE_KEEPING;
        }
    }

    return target_speed;
}

/**
 * Updates velocity based on the target speed.
 * @param context
 * @param target_speed
 */
void Planner::updateVelocity(const EnvContext &context, double target_speed) {
    if (abs(velocity - target_speed) < context.acc_limit * context.sim_update_freq) {
        // Can reach the target speed in this update frame.
        velocity = target_speed;
    } else {
        // Gradually reach the target speed so that we don't exceed max jerk limit.
        velocity += context.acc_limit * context.sim_update_freq
                    * (velocity < target_speed ? 1.0 : -1.0);
    }
}

/**
 * Generates smooth and jerk-minimizing trajectory using spline.
 * @param context
 * @return trajectory in x, y global coordinates.
 */
vector<vector<double>> Planner::generateTrajectory(const EnvContext &context) const {
    const auto prev_size = context.previous_path_x.size();
    const double car_s = prev_size > 0 ? context.end_path_s : context.car_s;

    vector<double> pts_x, pts_y;

    double ref_x;
    double ref_y;
    double ref_yaw;

    if (prev_size < 2) {
        // Previous size is almost empty, use car pose as the starting reference.
        ref_x = context.car_x;
        ref_y = context.car_y;
        ref_yaw = deg2rad(context.car_yaw);

        // Just use two points that make the path tangent to the car.
        pts_x.push_back(context.car_x - cos(context.car_yaw));
        pts_x.push_back(context.car_x);
        pts_y.push_back(context.car_y - sin(context.car_yaw));
        pts_y.push_back(context.car_y);
    } else {
        // Use the previous path as the starting reference.
        ref_x = context.previous_path_x[prev_size - 1];
        ref_y = context.previous_path_y[prev_size - 1];

        double prev_ref_x = context.previous_path_x[prev_size - 2];
        double prev_ref_y = context.previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

        // Use two points that make the path tangent to the previous path's end point.
        pts_x.push_back(prev_ref_x);
        pts_x.push_back(ref_x);
        pts_y.push_back(prev_ref_y);
        pts_y.push_back(ref_y);
    }

    // In Frenet space, add a few evenly 30m spaced points ahead of the starting reference.
    for (int i = 0; i < 3; i++) {
        auto xy = map.toCartesian(car_s + 30 * (i + 1), get_lane_center(target_lane, context.lane_width));
        pts_x.push_back(xy[0]);
        pts_y.push_back(xy[1]);
    }

    // Convert the points from global to car coordinate system (to make later math easier).
    for (int i = 0; i < pts_x.size(); i++) {
        double shift_x = pts_x[i] - ref_x;
        double shift_y = pts_y[i] - ref_y;
        pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }

    // Define the spline.
    tk::spline spline;
    spline.set_points(pts_x, pts_y);

    // Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    // Start with the previous way points.
    for (int i = 0; i < prev_size; i++) {
        next_x_vals.push_back(context.previous_path_x[i]);
        next_y_vals.push_back(context.previous_path_y[i]);
    }

    // Set up the horizon.
    double horizon_x = 30.0;
    double horizon_y = spline(horizon_x);
    double horizon_dist = distance(horizon_x, horizon_y, 0, 0);
    // The spacing of the points depends on reference speed and simulator update frequency.
    double N = horizon_dist / (context.sim_update_freq * velocity);

    double x_add_on = 0;
    for (int i = 1; i <= 50 - prev_size; i++) {
        double point_x = x_add_on + horizon_x / N;
        double point_y = spline(point_x);
        x_add_on = point_x;

        // Convert the point from car to global coordinate system.
        double shift_x = point_x;
        double shift_y = point_y;
        point_x = shift_x * cos(ref_yaw) - shift_y * sin(ref_yaw);
        point_y = shift_x * sin(ref_yaw) + shift_y * cos(ref_yaw);
        point_x += ref_x;
        point_y += ref_y;

        next_x_vals.push_back(point_x);
        next_y_vals.push_back(point_y);
    }
    return {next_x_vals, next_y_vals};
}

/**
 * Public interface of the path planner.
 * @param context
 * @return trajectory in x, y global coordinates.
 */
vector<vector<double>> Planner::planPath(const EnvContext &context) {
    // 1. Build information about each lane.
    const vector<LaneInfo> lane_infos = buildLaneInfos(context);

    // 2. Determine target lane and velocity.
    auto target_speed = planMotion(context, lane_infos);

    // 3. Update current speed.
    updateVelocity(context, target_speed);

    // 4. Generate trajectory.
    return generateTrajectory(context);
}