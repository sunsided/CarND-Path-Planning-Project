#include <cmath>
#include <limits>
#include <fstream>
#include <iostream>
#include <sstream>

#include "math.h"
#include "Map.h"

Map::Map(const std::string &dataFile) {
    load(dataFile);
}

void Map::load(const std::string &dataFile) {
    using namespace std;
    ifstream in_map_(dataFile.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x, y;
        float s, d_x, d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        _x.push_back(x);
        _y.push_back(y);
        _s.push_back(s);
        _dx.push_back(d_x);
        _dy.push_back(d_y);
    }
}

std::vector<double> Map::toFrenet(double x, double y, double theta) const noexcept {
    const auto next_wp = nextWaypoint(x, y, theta);

    auto prev_wp = next_wp - 1;
    if (next_wp == 0) {
        prev_wp = static_cast<int>(_x.size()) - 1;
    }

    const auto n_x = _x[next_wp] - _x[prev_wp];
    const auto n_y = _y[next_wp] - _y[prev_wp];
    const auto x_x = x - _x[prev_wp];
    const auto x_y = y - _y[prev_wp];

    // find the projection of x onto n
    const auto proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    const auto proj_x = proj_norm * n_x;
    const auto proj_y = proj_norm * n_y;

    auto frenet_d = distance(x_x, x_y, proj_x, proj_y);

    // see if d value is positive or negative by comparing it to a center point

    const auto center_x = 1000 - _x[prev_wp];
    const auto center_y = 2000 - _y[prev_wp];
    const auto centerToPos = distance(center_x, center_y, x_x, x_y);
    const auto centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }

    // calculate s value
    auto frenet_s = 0.0;
    for (int i = 0; i < prev_wp; i++) {
        frenet_s += distance(_x[i], _y[i], _x[i + 1], _y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);
    return {frenet_s, frenet_d};
}

std::vector<double> Map::toCartesian(double s, double d) const noexcept {
    int prev_wp = -1;

    while (s > _s[prev_wp + 1] && prev_wp < static_cast<int>(_s.size()) - 1) {
        ++prev_wp;
    }

    const auto wp2 = (prev_wp + 1) % static_cast<int>(_s.size());

    const auto heading = atan2(_y[wp2] - _y[prev_wp],
                               _x[wp2] - _x[prev_wp]);

    // The x,y,s along the segment
    const auto seg_s = s - _s[prev_wp];

    const auto seg_x = _x[prev_wp] + seg_s * cos(heading);
    const auto seg_y = _y[prev_wp] + seg_s * sin(heading);

    const auto perp_heading = heading - pi() / 2;

    const auto x = seg_x + d * cos(perp_heading);
    const auto y = seg_y + d * sin(perp_heading);

    return {x, y};
}

int Map::closestWaypoint(double x, double y) const noexcept {
    auto closestLen = std::numeric_limits<double>::max();
    int closestWaypoint = 0;
    for (int i = 0; i < static_cast<int>(_x.size()); i++) {
        const auto map_x = _x[i];
        const auto map_y = _y[i];
        const auto dist = distance(x, y, map_x, map_y);
        if (dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }
    }
    return closestWaypoint;
}

/**
 * Gets the next waypoint.
 * @param x
 * @param y
 * @param theta
 * @return: waypoint ID.
 */
int Map::nextWaypoint(double x, double y, double theta) const noexcept {
    auto closestWaypoint = this->closestWaypoint(x, y);
    const auto heading = atan2(_y[closestWaypoint] - y,
                               _x[closestWaypoint] - x);

    auto angle = fabs(theta - heading);
    angle = std::min(2 * pi() - angle, angle);

    if (angle > pi() / 4) {
        closestWaypoint++;
        if (closestWaypoint == _x.size()) {
            closestWaypoint = 0;
        }
    }
    return closestWaypoint;
}
