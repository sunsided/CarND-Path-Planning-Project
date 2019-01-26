#include <cstdio>
#include <cassert>
#include <exception>
#include <iostream>
#include <fstream>
#include <spline.h>
#include "math.h"

#include "lanes.h"
#include "Planner.h"
#include "CostFunction.h"

using namespace std;

Trajectory Planner::planPath(const EnvContext &context) {
    const auto lanes = buildLaneInfos(context);
    const auto target_speed = planMotion(context, lanes);
    updateVelocity(context, target_speed);
    return generateTrajectory(context);
}

vector<LaneInfo> Planner::buildLaneInfos(const EnvContext &context) const {
    // The effective simulator delay is determined by the number of steps
    // already sent to the planner and the update frequency.
    const auto simulator_lag = context.simulator_lag;

    // If we have sent steps to the simulator before, we're going to
    // initialize our own position from the already sent track.
    const auto own_s = context.car_s_adjusted;

    // Determine which tracked vehicle belongs to which lane,
    // and calculate clearance.
    vector<LaneInfo> lane_infos{static_cast<size_t>(context.num_lanes)};
    for (size_t l=0; l<context.num_lanes; ++l) {
        lane_infos.at(l).lane_index = static_cast<int>(l);
    }

    for (const auto &tracked : context.tracked_vehicles) {
        const auto id = tracked.id;

        // Obtain and sanity-check the lane of the tracked car
        const auto d = tracked.d;
        const auto lane_idx = get_lane_number(d, context.lane_width);
        if (lane_idx < 0 || lane_idx >= context.num_lanes) {
            continue;
        }

        // Obtain an estimate of the current position of the tracked
        // car, given the estimator update delay.
        const auto velocity = tracked.velocity();
        const auto s = tracked.s + simulator_lag * velocity;

        // Select the correct list according to relative clearance.
        const auto clearance_rel = s - own_s;
        const auto tracked_in_front = clearance_rel > 0;
        auto &vehicles = tracked_in_front
                      ? lane_infos[lane_idx].front
                      : lane_infos[lane_idx].rear;

        // Ensure the car is not too close to ego.
        const auto clearance_abs = abs(clearance_rel);
        if (clearance_abs >= context.min_clearance) {
            vehicles.emplace_back(id, velocity, clearance_abs);
            continue;
        }

        // The car is too close; mark it with zero clearance.
        vehicles.emplace_back(id, velocity, 0);
    }

    // Ensure we track the closest car.
    for (auto &lane : lane_infos) {
        lane.sort();
        assert(!(lane.front.empty() || lane.rear.empty()));
    }

    return lane_infos;
}

double Planner::planMotion(const EnvContext &context, const vector<LaneInfo> &lane_infos) {
    int current_lane_idx = get_lane_number(context.car_d, context.lane_width);

    LaneCostEvaluation cost_function{};
    std::vector<double> lane_costs;

    auto best_cost = context.highest_cost;
    auto optimal_lane_idx = current_lane_idx;

    for (const auto &lane : lane_infos) {
        const auto cost = cost_function(lane, context);
        lane_costs.push_back(cost);
        if (cost >= best_cost) continue;

        best_cost = cost;
        optimal_lane_idx = lane.lane_index;
    }

    // Ignore any assumptions if they're still bad
    // or not better than our current strategy.
    auto current_cost = lane_costs.at(static_cast<size_t>(current_lane_idx));
    if (best_cost >= context.highest_cost || best_cost >= current_cost) {
        optimal_lane_idx = current_lane_idx;
    }

    auto target_speed = 0.01;

    if (state == PlanningState::KeepLane) {
        const auto &current_lane = lane_infos[current_lane_idx];
        const auto &front = current_lane.front.at(0);

        // Decide if we want to change lane.
        target_lane = optimal_lane_idx;

        if (target_lane != current_lane_idx) {
            printf("Preparing lane change from %d (score %f) to %d (score %f)\n",
                    current_lane_idx, current_cost,
                    target_lane, best_cost);
            state = PlanningState::PrepareLaneChange;
        }

        // Initialize speed with current lane
        if (front.clearance() > context.safety_margin) {
            target_speed = mph_to_mps(context.speed_limit_mph);
        }
        else if (front.clearance() < 15) { // TODO: Extract magic number
            if (front.clearance() < 5) { // TODO: Extract magic number
                // TODO: We might need to become drastically slower!
                target_speed = min(front.speed() * 0.75, front.speed() - 10);
            }
            else {
                target_speed = min(front.speed() * 0.9, front.speed() - 5);
            }
            printf("Critically close to front car %d, clearance %.2f, set speed from %.2f to %.2f\n",
                   front.id(), front.clearance(), context.car_speed, target_speed);
        }
        else {
            // Match speed with the car in front in the current lane.
            target_speed = min(front.speed(), mph_to_mps(context.speed_limit_mph));
        }
    }
    else if (state == PlanningState::PrepareLaneChange) {
        // TODO: Adjust to the speed on the target lane.
        // TODO: Include a wait state here; during that time, monitor the previous condition.
        if (lane_costs.at(target_lane) >= context.highest_cost) {
            printf("Aborting lane change from %d to %d\n", current_lane_idx, target_lane);
            state = PlanningState::KeepLane;
            return target_speed;
        }

        printf("Performing lane change from %d to %d\n", current_lane_idx, target_lane);
        state = PlanningState::ChangeLane;
    }
    else if (state == PlanningState::ChangeLane) {
        // TODO: check for collision and abort lane changing if necessary.

        // Match speed with the target lane.
        const auto &front = lane_infos[target_lane].front.at(0);
        if (!front.valid() || front.clearance() > context.safety_margin) {
            // No car is within the safe distance in the target lane.
            target_speed = mph_to_mps(context.speed_limit_mph);
        } else {
            // Match speed with the car in front in the target lane.
            target_speed = min(front.speed(), mph_to_mps(context.speed_limit_mph));
        }

        // Check if we are done with lane changing.
        const auto lane_center = get_lane_center(target_lane, context.lane_width);
        const auto distance_lane_center = abs(lane_center - context.car_d);
        if (distance_lane_center <= context.lane_reached_threshold) {
            printf("Target lane reached. Now keeping lane.\n");
            state = PlanningState::KeepLane;
        }
    }

    return target_speed;
}

void Planner::updateVelocity(const EnvContext &context, double target_speed) {
    const auto acceleration_step = context.acc_limit * context.sim_update_freq;

    // If we can reach the target velocity within the next simulator update,
    // set the value directly to prevent jumping.
    if (abs(velocity - target_speed) < acceleration_step) {
        velocity = target_speed;
        return;
    }

    // Increase or decrease velocity until the former condition is satisfied.
    const auto acceleration_direction = (velocity < target_speed) ? 1.0 : -1.0;
    velocity += acceleration_step * acceleration_direction;
}

Trajectory Planner::generateTrajectory(const EnvContext &context) const {
    const auto prev_size = context.previous_path_x.size();
    const auto car_s = prev_size > 0 ? context.end_path_s : context.car_s;

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

        const auto prev_ref_x = context.previous_path_x[prev_size - 2];
        const auto prev_ref_y = context.previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

        // Use two points that make the path tangent to the previous path's end point.
        pts_x.push_back(prev_ref_x);
        pts_x.push_back(ref_x);
        pts_y.push_back(prev_ref_y);
        pts_y.push_back(ref_y);
    }

    // In Frenet space, add a few evenly 30m spaced points ahead of the starting reference.
    for (int i = 0; i < 3; i++) {
        const auto xy = map.toCartesian(car_s + 30 * (i + 1), get_lane_center(target_lane, context.lane_width));
        pts_x.push_back(xy[0]);
        pts_y.push_back(xy[1]);
    }

    // Convert the points from global to car coordinate system (to make later math easier).
    for (int i = 0; i < pts_x.size(); i++) {
        const auto shift_x = pts_x[i] - ref_x;
        const auto shift_y = pts_y[i] - ref_y;
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
    auto horizon_dist = distance(horizon_x, horizon_y, 0, 0);
    // The spacing of the points depends on reference speed and simulator update frequency.
    const auto N = horizon_dist / (context.sim_update_freq * velocity);

    double x_add_on = 0;
    for (int i = 1; i <= 50 - prev_size; ++i) {
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
    return {std::move(next_x_vals), std::move(next_y_vals)};
}
