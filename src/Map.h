#ifndef PATH_PLANNING_HIGHWAYMAP_H
#define PATH_PLANNING_HIGHWAYMAP_H

#include <string>
#include <vector>

class Map {
public:
    explicit Map(const std::string &dataFile);

    Map(Map &&) = default;
    Map(Map const &) = delete;

    Map& operator=(Map &&) = default;
    Map& operator=(Map const &) = delete;

    /**
     * Transform from Cartesian (x,y) coordinates to Frenet (s,d) coordinates.
     * @param x The X coordinate.
     * @param y The Y coordinate.
     * @param theta The orientation.
     * @return The coordinates in a Frenet system.
     */
    std::vector<double> toFrenet(double x, double y, double theta) const noexcept;

    /**
     * Transform from Frenet (s,d) coordinates to Cartesian (x,y) coordinates.
     * @param s The s coordinate (distance along trajectory).
     * @param d The d coordinate (distance orthogonal to trajectory).
     * @return The coordinates in a cartesian system.
     */
    std::vector<double> toCartesian(double s, double d) const noexcept;

    int closestWaypoint(double x, double y) const noexcept;

    int nextWaypoint(double x, double y, double theta) const noexcept;

private:
    void load(const std::string &dataFile);

    std::vector<double> _x;
    std::vector<double> _y;
    std::vector<double> _s;
    std::vector<double> _dx;
    std::vector<double> _dy;
};

#endif //PATH_PLANNING_HIGHWAYMAP_H
