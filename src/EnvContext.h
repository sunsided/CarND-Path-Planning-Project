#ifndef PATH_PLANNING_ENVCONTEXT_H
#define PATH_PLANNING_ENVCONTEXT_H

#include <vector>
#include <json.hpp>
#include "SensorData.h"

struct EnvContext {
public:
    explicit EnvContext(const nlohmann::json &j) :
            car_x{j[1]["x"]},
            car_y{j[1]["y"]},
            car_s{j[1]["s"]},
            car_d{j[1]["d"]},
            car_yaw{j[1]["yaw"]},
            car_speed{j[1]["speed"]},
            end_path_s{j[1]["end_path_s"]},
            end_path_d{j[1]["end_path_d"]} {

        for (const auto &x : j[1]["previous_path_x"]) {
            previous_path_x.emplace_back(x);
        }

        for (const auto &y : j[1]["previous_path_y"]) {
            previous_path_y.emplace_back(y);
        }

        for (const auto &list : j[1]["sensor_fusion"]) {
            std::vector<double> l = list;
            tracked_vehicles.emplace_back(l);
        }

        simulator_steps = previous_path_x.size();;
        simulator_lag = simulator_steps * sim_update_freq;
        car_s_adjusted = simulator_steps > 0 ? end_path_s : car_s;
    }

public:
    /** The max s value before wrapping around the track back to 0 */
    const double max_s = 6945.554;

    /** Number of lanes at one side of the highway. */
    const int num_lanes = 3;

    /**
     * The lane that is considered the base. The car will try to get here.
     * Set to negative value to disable.
     * */
    const int base_lane = 1;

    /** Lane width in meters. */
    const int lane_width = 4;

    /** Simulator's frequency to update car pose; in [s]. */
    const double sim_update_freq = 0.02;

    /** Speed limit; [miles/h]. */
    const double speed_limit_mph = 49.5;

    /** Acceleration/deceleration limit, in [m/(s^2)]. */
    const double acc_limit = 10;

    /** Lower boundary for clearance; in [m]. */
    const double min_clearance = 4;

    /** Safety margin; in [m]. */
    const double safety_margin = 30;

    /** Rear safety margin; in [m]. */
    const double rear_safety_margin = 10;

    /** The threshold used to decide whether the target lane was reached; in [m]. */
    const double lane_reached_threshold = 0.2;

    /** The highest allowed cost value */
    const double highest_cost = 1.0;

    /** The lowerst possible cost value */
    const double no_cost = 0.0;

    /** The number of steps already issued to the simulator */
    size_t simulator_steps;

    /** The current simulator lag */
    double simulator_lag;

    /** Ego car's pose information. */
    double car_x, car_y;
    double car_s, car_d, car_s_adjusted;
    double car_yaw;
    double car_speed;

    /** Previous path's end s and d values. */
    double end_path_s;
    double end_path_d;

    /** Previous path data given to the planner. */
    std::vector<double> previous_path_x;
    std::vector<double> previous_path_y;

    /** Sensor fusion data: a list of all other cars on the same side of the road. */
    std::vector<SensorData> tracked_vehicles;
};

#endif //PATH_PLANNING_ENVCONTEXT_H
