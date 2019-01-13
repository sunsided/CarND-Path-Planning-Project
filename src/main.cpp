#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "math.h"
#include "frenet.h"
#include "socketio_helpers.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Get lane Id given Frenet d coordinates
int getLane (const double d) {
    if (d > 0 && d < 4) return 0;
    else if (d > 4 && d < 8) return 1;
    else if (d > 8 && d < 12) return 2;
    return -1;
}

// Check for cars ahead and behind given a lane
vector<double> getLaneBehaviour(vector<vector<double >> sensor_fusion, int lane, int prev_size, double car_end_s, double car_s, double car_v, double forward_horizon, double backward_horizon, bool print){

    double status_car_ahead = 0; // 0: no car ahead, 1: car ahead in range, 2: collision warning
    double status_car_behind = 0; // 0: no car behind, 1: car behind in range, 2: collision warning
    double dist_car_ahead = 10000;
    double dist_car_behind = 10000;
    double v_car_ahead = 49.5;
    double v_car_behind = 49.5;
    double id_car_ahead = -1;
    double id_car_behind = -1;

    for(int i = 0; i < sensor_fusion.size(); i++){

        // [id, x, y, v_x, v_y, s, d]
        vector<double> i_car = sensor_fusion[i];
        double i_car_d = i_car[6];

        if (i_car_d > (2 + 4 * lane - 2) && i_car_d < (2 + 4 * lane + 2)){

            // ith car is in the same lane as in the parameter lane
            double i_car_v_x = i_car[3];
            double i_car_v_y = i_car[4];
            double i_car_v = sqrt(i_car_v_x * i_car_v_x + i_car_v_y * i_car_v_y); // ith car speed in m/s
            double i_car_s = i_car[5];
            double i_car_end_s = i_car_s + (double) prev_size * 0.02 * i_car_v; // predicted ith car end s location

            if (car_v >= i_car_v * 2.24){
                // If the trailing car is travelling at a lower speed, hence, decrease the backward horizon
                backward_horizon = 20;
            }

            if (i_car_s > car_s && i_car_s < (car_s + forward_horizon/2)){
                // The i_th car is ahead and in collision warning distance
                status_car_ahead = 2;
                double i_dist_car_ahead = i_car_s - car_s;
                dist_car_ahead = i_dist_car_ahead;
                v_car_ahead = i_car_v * 2.24;
                id_car_ahead = i_car[0];
                break;

            }
            else if (i_car_s < car_s && car_s < (i_car_s + backward_horizon/2)){
                // The i_th car is behind and in collision warning distance
                status_car_behind = 2;
                double i_dist_car_behind = car_s - i_car_s;
                dist_car_behind = i_dist_car_behind;
                v_car_behind = i_car_v * 2.24;
                id_car_behind = i_car[0];
                break;

            }
            else if (i_car_end_s > car_end_s && i_car_end_s < (car_end_s + forward_horizon)){
                // The i_th car is ahead and in the path planning horizon
                status_car_ahead = 1;
                double i_dist_car_ahead = i_car_end_s - car_end_s;

                if (i_dist_car_ahead < dist_car_ahead){
                    dist_car_ahead = i_dist_car_ahead;
                    v_car_ahead = i_car_v * 2.24;
                    id_car_ahead = i_car[0];
                }
            }
            else if (i_car_end_s < car_end_s && car_end_s < (i_car_end_s + backward_horizon)){
                // The i_th car is behind and in the path planning horizon
                status_car_behind = 1;
                double i_dist_car_behind = car_end_s - i_car_end_s;

                if (i_dist_car_behind < dist_car_behind){
                    dist_car_behind = i_dist_car_behind;
                    v_car_behind = i_car_v * 2.24;
                    id_car_behind = i_car[0];
                }
            }
        }
    }

    if (print){
        /*if (status_car_ahead == 1){
            cout << " lane: " << lane << " dist car ahead: " << dist_car_ahead << " speed car ahead: " << v_car_ahead << endl;
        }
        if (status_car_behind == 1){
            cout << " lane: " << lane << " dist car behind: " << dist_car_behind << " speed car behind: " << v_car_behind << endl;
        }*/

        cout << " lane: " << lane << " car ahead " << status_car_ahead << " dist car ahead: " << dist_car_ahead << " speed car ahead: " << v_car_ahead << endl;
        cout << " lane: " << lane << " car behind " << status_car_behind << " dist car behind: " << dist_car_behind << " speed car behind: " << v_car_behind << endl;
    }

    return {status_car_ahead, status_car_behind, dist_car_ahead, dist_car_behind, v_car_ahead, v_car_behind, id_car_ahead, id_car_behind};
}

// Get the cost of changing lane from the current lane
vector<double> getLaneChangeCost(vector<vector<double >> sensor_fusion, int current_lane, int prev_size, double car_end_s, double car_s, double car_v){

    double new_lane = -1;
    double new_lane_cost = 10000;
    double speed_car_ahead = 49.5;

    if (current_lane == 0 || current_lane == 2){

        // Car is currently in the left or right lane, so get the cost for the middle lane
        double middle_lane = 1;

        // Add offset so as to avoid lane changes that are not really useful for overtaking, i.e. when the car ahead in the new lane is slightly ahead than the car ahead in the current lane
        vector<double> middle_lane_behaviour = getLaneBehaviour(sensor_fusion, middle_lane, prev_size, car_end_s, car_s, car_v, 40, 30, true);
        double dist_car_ahead = middle_lane_behaviour[2];
        double dist_car_behind = middle_lane_behaviour[3];
        // The new lane change cost is the sum of the individual costs of the distance with the car ahead and behind in the new lne
        double middle_lane_cost = (1 - exp(-1/dist_car_ahead)) + (1 - exp(-1/dist_car_behind));

        new_lane = middle_lane;
        new_lane_cost = middle_lane_cost;
        speed_car_ahead = middle_lane_behaviour[4];
    }
    else{

        // Car is in the middle lane, so compare the costs of changing to the left and right lane

        // Left lane change cost
        double left_lane = 0;

        // Also add offset for the forward horizon.
        vector<double> left_lane_behaviour = getLaneBehaviour(sensor_fusion, left_lane, prev_size, car_end_s, car_s, car_v, 40, 30, true);
        double l_dist_car_ahead = left_lane_behaviour[2];
        double l_dist_car_behind = left_lane_behaviour[3];
        // The new lane change cost is the sum of the individual costs of the distance with the car ahead and behind in the new lne
        double left_lane_cost = (1 - exp(-1/l_dist_car_ahead)) + (1 - exp(-1/l_dist_car_behind));

        if (left_lane_behaviour[0] == 2 || left_lane_behaviour[0] == 2){
            left_lane_cost = 1;
        }

        // Right lane change cost
        double right_lane = 2;

        vector<double> right_lane_behaviour = getLaneBehaviour(sensor_fusion, right_lane, prev_size, car_end_s, car_s, car_v, 40, 30, true);
        double r_dist_car_ahead = right_lane_behaviour[2];
        double r_dist_car_behind = right_lane_behaviour[3];
        // The new lane change cost is the sum of the individual costs of the distance with the car ahead and behind in the new lne
        double right_lane_cost = (1 - exp(-1/r_dist_car_ahead)) + (1 - exp(-1/r_dist_car_behind));

        if (right_lane_behaviour[0] == 2 || right_lane_behaviour[0] == 2){
            right_lane_cost = 1;
        }

        // Choose the lane change with the lower cost
        if (left_lane_cost <= right_lane_cost){
            new_lane = left_lane;
            new_lane_cost = left_lane_cost;
            speed_car_ahead = left_lane_behaviour[4];
        }
        else{
            new_lane = right_lane;
            new_lane_cost = right_lane_cost;
            speed_car_ahead = right_lane_behaviour[4];
        }
    }

    return {new_lane, new_lane_cost, speed_car_ahead};
}

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    const auto map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    const auto max_s = 6945.554;

    ifstream in_map_(map_file_, ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    // Lane the car is currently on
    auto lane = 1;

    // Target speed
    auto ref_speed = 0.0; // mph
    auto ref_max_speed = 49.5; // mph

    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &lane, ref_max_speed, &ref_speed](
            uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
            uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length <= 2 || data[0] != '4' || data[1] != '2') return;

        auto s = hasData(data);
        if (!(s != NoData)) {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            return;
        }

        auto j = json::parse(s);
        string event = j[0].get<string>();

        if (event != "telemetry") return;
        // j[1] is the data JSON object

        // Main car's localization Data
        double car_x = j[1]["x"];
        double car_y = j[1]["y"];
        double car_s = j[1]["s"];
        double car_d = j[1]["d"];
        double car_yaw = j[1]["yaw"];
        double car_speed = j[1]["speed"];

        // Previous path data given to the Planner
        const auto previous_path_x = j[1]["previous_path_x"];
        const auto previous_path_y = j[1]["previous_path_y"];
        // Previous path's end s and d values
        double end_path_s = j[1]["end_path_s"];
        double end_path_d = j[1]["end_path_d"];

        // Sensor Fusion Data, a list of all other cars on the same side of the road.
        const auto sensor_fusion = j[1]["sensor_fusion"];

        int prev_size = previous_path_x.size();

        // Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

        // Update car reference. Too smooth trajectories, use end_path of previous path as the starting reference point in the current iteration
        double current_car_s = car_s;
        if(prev_size > 0){
            car_s = end_path_s;
        }

        cout << " current_car_s  / car_s / delta " << current_car_s << " / " <<  car_s <<  " / " <<  car_s - current_car_s << endl;

        // Check keep lane state
        bool speed_up = false;
        bool speed_down = false;
        double ref_new_speed = ref_max_speed;

        int current_lane = getLane(car_d); //
        vector<double> current_lane_behaviour = getLaneBehaviour(sensor_fusion, current_lane, prev_size, car_s, current_car_s, car_speed, 30, 30, false);
        double speed_car_ahead = current_lane_behaviour[4];

        if (current_lane_behaviour[0] == 1){

            cout << " car ahead " << endl;

            // There is a car ahead, two complementary options: change lane and/or reduce speed
            ref_new_speed = speed_car_ahead;

            // 1. Evalute the cost of keeping in the same lane
            double dist_car_ahead = current_lane_behaviour[2];
            double keep_lane_cost = (1 - exp(-1/dist_car_ahead));

            // 2. Evaluate the cost of changing lane
            vector<double> change_lane_result = getLaneChangeCost(sensor_fusion, current_lane, prev_size, car_s, current_car_s, car_speed);
            double change_lane_cost = change_lane_result[1];

            if (change_lane_cost + 0.03 < keep_lane_cost){
                // If the cost of changing lane, plus a threshold value, is lower than the cost of keeping lane, then update the lane and maintain the speed
                lane = static_cast<int>(change_lane_result[0]);
                cout << " change_lane " << endl;
                cout << " keep_lane_cost " << keep_lane_cost << endl;
                cout << " change_lane_cost " << change_lane_cost << endl;

                // Update ref speeds
                ref_new_speed = (change_lane_result[2] > ref_new_speed + 5) ? ref_new_speed : change_lane_result[2];

                /*if (ref_speed < ref_new_speed){
                    ref_speed += 0.05;
                    speed_up = true;
                }*/

                cout << " ref_new_speed " << ref_new_speed << endl;
                cout << " car_speed " << car_speed << endl;
                cout << " ref_speed " << ref_speed << endl;

            }
            else{
                // If keep lane, then reduce speed, using the speed of the car ahead as the minimum value
                if (ref_speed > ref_new_speed){
                    ref_speed -= 0.224;
                    speed_down = true;
                }

                //cout << " keep_lane " << endl;
            }

        }
        else if (current_lane_behaviour[0] == 2){

            cout << " car ahead too close: collision warning " << endl;

            // Collision warning: speed down
            ref_new_speed = speed_car_ahead;

            if (ref_speed > ref_new_speed){
                ref_speed -= 0.224;
                speed_down = true;
            }
        }
        else if (ref_speed < ref_new_speed){
            ref_speed += 0.224;
            speed_up = true;
        }

        // As in walkthrough video, create a auxiliary list of sparse (x,y) waypoints (aux_wps), evenly spaced at 30m
        // We will later use this auxiliary waypoints to interporlate and create more waypoints so as to smooth the trajectory
        vector<double> pts_x;
        vector<double> pts_y;

        double ref_x = car_x;
        double ref_y = car_y;
        double ref_yaw = deg2rad(car_yaw);

        // If previous path is almost empty, use car coordinates and heading as a starting reference to fill auxiliary list
        if(prev_size < 2){

            // Use points tangent to car heading and add them to the auxiliary list
            // TODO: ¿Why not multiply cos/sin by car speed and time?
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            pts_x.push_back(prev_car_x);
            pts_x.push_back(car_x);

            pts_y.push_back(prev_car_y);
            pts_y.push_back(car_y);

        }
        else{

            // Redefine reference state using the latest 2 points in the previous path
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            pts_x.push_back(ref_x_prev);
            pts_x.push_back(ref_x);

            pts_y.push_back(ref_y_prev);
            pts_y.push_back(ref_y);
        }

        // Using Frenet coordinates, add evenly spaced points (30m) ahead of the starting reference
        vector<double> next_wp_0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> next_wp_1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> next_wp_2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

        pts_x.push_back(next_wp_0[0]);
        pts_x.push_back(next_wp_1[0]);
        pts_x.push_back(next_wp_2[0]);

        pts_y.push_back(next_wp_0[1]);
        pts_y.push_back(next_wp_1[1]);
        pts_y.push_back(next_wp_2[1]);

        // Define the points that will be used by the trajectory planner
        vector<double> next_x_vals;
        vector<double> next_y_vals;

        // Interpolate the auxiliary waypoints (aux_wps) using a spline, but before shift the aux_wps to vehicle coordinates (x-axis is aligned with the car heading)
        for(int i = 0; i < pts_x.size(); i++){
            double shift_x = pts_x[i]-ref_x;
            double shift_y = pts_y[i]-ref_y;

            pts_x[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
            pts_y[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
        }

        // Create spline and set (x,y) points to the spline
        tk::spline sp;

        sp.set_points(pts_x,pts_y);

        // Fill first the previous path points from the last iterarion, i.e., the remaining points not used by the simulator
        for(int i = 0; i < previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
        }

        // Use the method in the walkthrough in which the spline points projected along the x-axis are splitted into equal sizes so as to travel at the target speed
        double sp_target_x = 30.0;
        double sp_target_y = sp(sp_target_x);
        double sp_target_dist = sqrt((sp_target_x)*(sp_target_x) + (sp_target_y)*(sp_target_y));

        double x_add_on = 0;

        // Fill up the remaining waypoints with of the trajectory planner
        for(int i = 1; i <= 50 - previous_path_x.size(); i++){

            // N is the number of splits, which is calculated using the linearized distance (sp_target_dist), the time interval in which a waypoint is visited (0.02) and the target speed (using the 2.24 to convert from mph to mps).
            // TODO: update target speed in this loop as well
            double N = (sp_target_dist/(0.02 * ref_speed / 2.24));
            double x_point = x_add_on + sp_target_x / N;
            double y_point = sp(x_point);

            x_add_on = x_point;

            // Rotate back to normal coordinates
            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

            // TODO: Update ref_speed using the speed_up and speed_down booleans and the ref_new_speed
        }

        // END

        json msgJson;
        msgJson["next_x"] = next_x_vals;
        msgJson["next_y"] = next_y_vals;

        // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
        msgJson["next_x"] = next_x_vals;
        msgJson["next_y"] = next_y_vals;

        auto msg1 = "42[\"control\"," + msgJson.dump() + "]";

        //this_thread::sleep_for(chrono::milliseconds(1000));
        ws.send(msg1.data(), msg1.length(), uWS::OpCode::TEXT);
    });

#if QUESTIONABLE_UPSTREAM_CODE

    // We don't need this since we're not using HTTP but if it's removed the
    // program doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

#endif

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
