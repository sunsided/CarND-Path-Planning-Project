#ifndef PATH_PLANNING_SOCKETIO_HELPERS_H
#define PATH_PLANNING_SOCKETIO_HELPERS_H

#include <string>

/**
 * The value to be returned if not data is available from \ref hasData.
 */
static const std::string NoData;

/**
 * Checks if the SocketIO event has JSON data.
 * If there is data the JSON object in string format will be returned,
 * else the empty string "" will be returned.
 * @param s The data element.
 * @return The stringified JSON data; \ref NoData if no data was available.
 */
std::string hasData(const std::string &s);

#endif //PATH_PLANNING_SOCKETIO_HELPERS_H
