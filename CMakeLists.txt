cmake_minimum_required(VERSION 3.0.2)
project(openbot_driver_gps_ublox)


find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        openbot_msgs
        roscpp
        serial
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

add_executable(driver_gps_ublox_node
        src/driver_gps_ublox_node.cpp
        src/gps_interface.h
        src/gps_interface.cpp
        )

add_dependencies(driver_gps_ublox_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(driver_gps_ublox_node ${catkin_LIBRARIES})
