#ifndef PATH_PLANNING_ENVCONTEXT_H
#define PATH_PLANNING_ENVCONTEXT_H

#include <vector>

#include "json.hpp"

struct EnvContext {
    // Constant values.

    // The max s value before wrapping around the track back to 0
    const double max_s = 6945.554;
    // Number of lanes at one side of the highway.
    const int num_lanes = 3;
    // Lane width in meters.
    const int lane_width = 4;
    // Simulator's frequency to update car pose (seconds).
    const double sim_update_freq = 0.02;
    // Speed limit in MPH.
    const double speed_limit_mph = 49.5;
    // Acceleration/deceleration limit.
    const double acc_limit = 8 /*meters per second^2*/;
    // Safety margin in meters.
    const double safety_margin = 30;

    // Ego car's pose information.
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;

    // Previous path data given to the planner.
    std::vector<double> previous_path_x;
    std::vector<double> previous_path_y;

    // Previous path's end s and d values.
    double end_path_s;
    double end_path_d;

    // Sensor fusion data: a list of all other cars on the same side of the road.
    std::vector<std::vector<double>> sensor_fusion;

    explicit EnvContext(const nlohmann::json& j):
            car_x(j[1]["x"]),
            car_y(j[1]["y"]),
            car_s(j[1]["s"]),
            car_d(j[1]["d"]),
            car_yaw(j[1]["yaw"]),
            car_speed(j[1]["speed"]),
            end_path_s(j[1]["end_path_s"]),
            end_path_d(j[1]["end_path_d"]) {
        for (const auto& x : j[1]["previous_path_x"]) {
            previous_path_x.emplace_back(x);
        }
        for (const auto& y : j[1]["previous_path_y"]) {
            previous_path_y.emplace_back(y);
        }
        for (const auto& list : j[1]["sensor_fusion"]) {
            std::vector<double> l = list;
            sensor_fusion.emplace_back(l);
        }
    }
};

#endif //PATH_PLANNING_ENVCONTEXT_H
