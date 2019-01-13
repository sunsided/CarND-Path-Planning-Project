#ifndef PATH_PLANNING_SENSORDATA_H
#define PATH_PLANNING_SENSORDATA_H

#include <vector>

/**
 * Data from the sensor fusion.
 */
struct SensorData {
public:
    /**
     * Initializes a new instance of the /ref SensorData struct.
     * @param data The sensor fusion data.
     */
    explicit SensorData(const std::vector<double> &data) :
        id{static_cast<int>(data[0])},
        x{data[1]},
        y{data[2]},
        vx{data[3]},
        vy{data[4]},
        s{data[5]},
        d{data[6]}
    {}

    /**
     * Obtains the tracked vehicle's absolute velocity.
     * @return The velocity of the vehicle.
     */
    inline double velocity() const noexcept {
        return sqrt(vx * vx + vy * vy);
    }

public:
    /** The ID of the tracked vehicle. */
    const int id;

    /** The X coordinate of the tracked vehicle; in [m]. */
    const double x;

    /** The Y coordinate of the tracked vehicle; in [m]. */
    const double y;

    /** The velocity in X direction of the tracked vehicle; cartesian coordinates, in [m/s]. */
    const double vx;

    /** The velocity in Y direction of the tracked vehicle; cartesian coordinates, in [m/s]. */
    const double vy;

    /** The position in S (forward) direction; Frenet coordinates. */
    const double s;

    /** The position in D (lateral) direction; Frenet coordinates. */
    const double d;
};

#endif //PATH_PLANNING_SENSORDATA_H
