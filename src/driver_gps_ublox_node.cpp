//
// Created by Clemens Elflein on 14.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "ros/ros.h"
#include "gps_interface.h"
#include "openbot_msgs/WheelTick.h"

ros::Publisher pose_pub;


using namespace openbot::driver_gps_ublox;

GpsInterface gpsInterface;

bool allow_verbose_logging = false;


void gps_log(std::string text, GpsInterface::Level level) {
    switch (level) {
        case GpsInterface::Level::VERBOSE:
            if(!allow_verbose_logging) {
                return;
            }
            ROS_INFO_STREAM("[driver_gps_ublox] " << text);
            break;
        case GpsInterface::Level::INFO:
            ROS_INFO_STREAM("[driver_gps_ublox] " << text);
            break;
        case GpsInterface::Level::WARN:
            ROS_WARN_STREAM("[driver_gps_ublox] " << text);
            break;
        default:
            ROS_ERROR_STREAM("[driver_gps_ublox] " << text);
            break;
    }
}

void wheelTicksReceived(const openbot_msgs::WheelTick::ConstPtr &msg) {
    gpsInterface.send_raw(nullptr, 0);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mower_comms");

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    allow_verbose_logging = paramNh.param("verbose", false);
    if(allow_verbose_logging) {
        ROS_WARN("GPS node has verbose logging enabled");
    }

    gpsInterface.set_log_function(gps_log);
    gpsInterface.set_baudrate(paramNh.param("baudrate", 38400));
    gpsInterface.set_serial_port(paramNh.param("serial_port", std::string("/dev/ttyACM0")));
    gpsInterface.start();

    // Subscribe to wheel ticks
    ros::Subscriber cmd_vel_sub = n.subscribe("wheel_ticks", 0, wheelTicksReceived, ros::TransportHints().tcpNoDelay(true));

    while(ros::ok()) {
        ros::spin();
    }

    gpsInterface.stop();
    return 0;
}
