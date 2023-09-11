cmake_minimum_required(VERSION 3.5)
project(ublox_f9p)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

find_package(Boost REQUIRED)
find_package(ament_cmake_auto REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_localization REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial_driver REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# Build the executable
add_executable(ublox_f9p
        src/ubx_datatypes.hpp
        src/node.cpp
        src/node_main.cpp
        src/ublox.hpp
        src/ublox.cpp
)

ament_target_dependencies(ublox_f9p
        geometry_msgs
        rclcpp
        robot_localization
        sensor_msgs
        serial_driver
        std_msgs
        rtcm_msgs
        tf2
        tf2_geometry_msgs
)

# Installation and export info
install(TARGETS
        ublox_f9p
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()