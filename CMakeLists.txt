project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/math.h src/socketio_helpers.cpp src/socketio_helpers.h src/Map.cpp src/Map.h src/Planner.cpp src/Planner.h src/LaneInfo.h vendor/spline/spline.h src/EnvContext.h src/lanes.h src/TrackedVehicle.h src/SensorData.h src/PlanningState.h src/Trajectory.h src/CostFunction.h src/CostFunction.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})
target_include_directories(path_planning
        PRIVATE
        vendor/Eigen-3.3
        vendor/json
        vendor/spline)

target_link_libraries(path_planning z ssl uv uWS)
