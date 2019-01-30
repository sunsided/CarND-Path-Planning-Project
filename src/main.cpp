#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <Eigen/Core>
#include <Eigen/QR>
#include <json.hpp>

#include "socketio_helpers.h"
#include "math.h"
#include "Map.h"
#include "Planner.h"

using namespace std;

// for convenience
using json = nlohmann::json;

int main() {
    uWS::Hub h;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    Map map {map_file_};
    Planner planner {map};

    h.onMessage([&planner](
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
        const EnvContext context {j};
        auto output = planner.planPath(context);

        json msgJson;
        msgJson["next_x"] = output.x();
        msgJson["next_y"] = output.y();

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
