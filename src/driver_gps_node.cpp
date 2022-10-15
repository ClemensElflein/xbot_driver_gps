//
// Created by Clemens Elflein on 14.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "ros/ros.h"
#include "ublox_gps_interface.h"
#include "xbot_msgs/WheelTick.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "xbot_msgs/AbsolutePose.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

void wheel_tick_received(const xbot_msgs::WheelTick::ConstPtr &msg) {
//    gpsInterface.send_raw(nullptr, 0);
}

void convert_gps_result(const GpsInterface::GpsState &state, xbot_msgs::AbsolutePose &result) {
    result.source = xbot_msgs::AbsolutePose::SOURCE_GPS;
    result.flags = 0;
    switch (state.rtk_type) {
        case GpsInterface::GpsState::RTK_FLOAT:
            result.flags = xbot_msgs::AbsolutePose::FLAG_GPS_RTK | xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT;
            break;
        case GpsInterface::GpsState::RTK_FIX:
            result.flags = xbot_msgs::AbsolutePose::FLAG_GPS_RTK | xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED;
            break;
        default:
            result.flags = 0;
    }


    result.orientation_valid = state.vehicle_heading_valid;
    result.motion_vector_valid = true;
    result.position_accuracy = state.position_accuracy;
    result.orientation_accuracy = state.vehicle_heading_accuracy;



    double heading = state.vehicle_heading_valid ? state.vehicle_heading : state.motion_heading;
    double headingAcc = state.vehicle_heading_valid ? state.vehicle_heading_accuracy : state.motion_heading_accuracy;

    result.pose.pose.position.x = state.pos_e;
    result.pose.pose.position.y = state.pos_n;
    result.pose.pose.position.z = state.pos_u;

    tf2::Quaternion q_mag(0.0, 0.0, heading);
    result.pose.pose.orientation = tf2::toMsg(q_mag);

    result.pose.covariance = {
            pow(state.position_accuracy, 2), 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, pow(state.position_accuracy, 2), 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, pow(state.position_accuracy, 2), 0.0, 0.0,
            0.0, 0.0, 0.0, 10000.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 10000.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, pow(headingAcc, 2)
    };


    result.motion_vector.x = state.vel_e;
    result.motion_vector.y = state.vel_n;
    result.motion_vector.z = state.vel_u;

    result.vehicle_heading = state.vehicle_heading;
    result.motion_heading = state.motion_heading;

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
    ros::Subscriber wheel_tick_sub = paramNh.subscribe("wheel_ticks", 0, wheel_tick_received, ros::TransportHints().tcpNoDelay(true));

    ros::Publisher pose_pub = paramNh.advertise<geometry_msgs::PoseWithCovariance>("pose", 1);
    ros::Publisher xbot_pose_pub = paramNh.advertise<xbot_msgs::AbsolutePose>("xb_pose", 1);

    GpsInterface::GpsState state = {0};
    xbot_msgs::AbsolutePose pose_result;

    while(ros::ok()) {
        ros::spinOnce();
        if(gpsInterface.get_gps_result(&state)) {
            // new state received, publish
            convert_gps_result(state, pose_result);
            xbot_pose_pub.publish(pose_result);
            pose_pub.publish(pose_result.pose);
        }
    }

    gpsInterface.stop();
    return 0;
}
