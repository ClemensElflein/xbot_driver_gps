cmake_minimum_required(VERSION 3.0.2)
project(xbot_driver_gps_ublox)


find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        xbot_msgs
        roscpp
        serial
        robot_localization
        tf2_geometry_msgs
        tf2
        )

find_package(
        Boost
)


catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES mower_comms
        #  CATKIN_DEPENDS mower_msgs roscpp serial
        #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_executable(driver_gps_node
        src/driver_gps_node.cpp
        src/ublox_gps_interface.h
        src/ublox_gps_interface.cpp
        src/ubx_datatypes.h)

add_dependencies(driver_gps_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(driver_gps_node ${catkin_LIBRARIES})
