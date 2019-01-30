#include <utility>

#include <utility>

#include <utility>

#include <utility>

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
#include <cassert>

/**
 * A (generated) trajectory.
 */
struct Trajectory {
public:
    Trajectory(std::vector<double> &&x, std::vector<double> &&y) noexcept
            : _x{std::move(x)}, _y{std::move(y)}
    {
        assert(_x.size() == _y.size());
    }

    Trajectory(Trajectory &&other) noexcept
            : _x{std::move(other._x)}, _y{std::move(other._y)}
    {
        assert(_x.size() == _y.size());
    }

    Trajectory(const Trajectory &other) noexcept = delete;

    inline Trajectory& operator=(const Trajectory& other) noexcept = delete;

    inline Trajectory& operator=(Trajectory&& other) noexcept {
        _x = std::move(other._x);
        _y = std::move(other._y);
        return *this;
    }

    /**
     * Gets the X coordinates of the trajectory.
     * @return The X coordinates.
     */
    inline const std::vector<double>& x() const { return _x; }

    /**
     * Gets the Y coordinates of the trajectory.
     * @return The Y coordinates.
     */
    inline const std::vector<double>& y() const { return _y; }

private:
    /**
     * The x coordinates.
     */
    std::vector<double> _x;

    /**
     * The y coordinates.
     */
    std::vector<double> _y;
};

#endif //PATH_PLANNING_TRAJECTORY_H
