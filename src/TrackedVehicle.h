#ifndef PATH_PLANNING_TRACKEDVEHICLE_H
#define PATH_PLANNING_TRACKEDVEHICLE_H

struct TrackedVehicle {
public:
    TrackedVehicle() noexcept
        : _id{-1}, _speed{0}, _clearance{0} {}

    TrackedVehicle(int id, double speed, double clearance) noexcept
        : _id{id}, _speed{speed}, _clearance{clearance} {}

    TrackedVehicle(const TrackedVehicle &) noexcept = default;

    TrackedVehicle &operator=(const TrackedVehicle &) noexcept = default;

    inline bool valid() const noexcept {
        return (_id >= 0) && (!std::isnan(_clearance));
    }

    inline bool valid(const size_t num_lanes) const noexcept {
        return valid() && (_id < num_lanes);
    }

    inline int id() const noexcept { return _id; }

    inline double speed() const noexcept { return _speed; }

    /**
     * Determines the clearance to the car.
     * @return The clearance in [m] or NaN if the track is invalid.
     */
    inline double clearance() const noexcept {
        return valid() ? _clearance : std::numeric_limits<double>::quiet_NaN();
    }

    TrackedVehicle& set(const int id, const double speed, const double clearance) noexcept {
        _id = id;
        _speed = speed;
        _clearance = clearance;
        return *this;
    }

private:
    int _id;
    double _speed;
    double _clearance;
};

#endif //PATH_PLANNING_TRACKEDVEHICLE_H
