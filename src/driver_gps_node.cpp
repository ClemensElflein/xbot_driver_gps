//
// Created by Clemens Elflein on 14.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "ros/ros.h"
#include "ublox_gps_interface.h"
#include "xbot_msgs/WheelTick.h"
#include "geometry_msgs/Pose.h"

ros::Publisher pose_pub;


using namespace xbot::driver::gps;

GpsInterface gpsInterface;

bool allow_verbose_logging = false;


void gps_log(std::string text, GpsInterface::Level level) {
    switch (level) {
        case GpsInterface::Level::VERBOSE:
            if(!allow_verbose_logging) {
                return;
            }
            ROS_INFO_STREAM("[driver_gps] " << text);
            break;
        case GpsInterface::Level::INFO:
            ROS_INFO_STREAM("[driver_gps] " << text);
            break;
        case GpsInterface::Level::WARN:
            ROS_WARN_STREAM("[driver_gps] " << text);
            break;
        default:
            ROS_ERROR_STREAM("[driver_gps] " << text);
            break;
    }
}

void wheelTicksReceived(const xbot_msgs::WheelTick::ConstPtr &msg) {
//    gpsInterface.send_raw(nullptr, 0);
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

    std::string mode = paramNh.param("mode", std::string("absolute"));
    if(mode == "absolute") {
        ROS_INFO_STREAM("Using absolute mode for GPS");
        gpsInterface.set_mode(xbot::driver::gps::GpsInterface::ABSOLUTE);
        double datum_lat, datum_long, datum_height;
        bool has_datum = true;
        has_datum &= paramNh.getParam("datum_lat", datum_lat);
        has_datum &= paramNh.getParam("datum_long", datum_long);
        has_datum &= paramNh.getParam("datum_height", datum_height);
        if(!has_datum) {
            ROS_ERROR_STREAM("You need to provide datum_lat and datum_long and datum_height in order to use the absolute mode");
            return 2;
        }
        gpsInterface.set_datum(datum_lat, datum_long, datum_height);
    } else if(mode == "relative") {
        ROS_INFO_STREAM("Using relative mode for GPS");
        gpsInterface.set_mode(xbot::driver::gps::GpsInterface::RELATIVE);
    }

    if(!gpsInterface.start()) {
        return 1;
    }

    // Subscribe to wheel ticks
    ros::Subscriber wheel_tick_sub = n.subscribe("wheel_ticks", 0, wheelTicksReceived, ros::TransportHints().tcpNoDelay(true));

    pose_pub = n.advertise<geometry_msgs::Pose>("pose", 1);

    GpsInterface::GpsState state = {0};
    geometry_msgs::Pose pose_result;

    while(ros::ok()) {
        ros::spinOnce();
        if(gpsInterface.get_gps_result(&state)) {
            // new state received, publish
            pose_result.position.x = state.posE;
            pose_result.position.y = state.posN;
            pose_result.position.z = state.posU;
            pose_pub.publish(pose_result);
        }
    }

    gpsInterface.stop();
    return 0;
}
