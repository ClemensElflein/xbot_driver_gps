//
// Created by Clemens Elflein on 14.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "ros/ros.h"
#include "ublox_gps_interface.h"
#include "nmea_gps_interface.h"
#include "xbot_msgs/WheelTick.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "xbot_msgs/AbsolutePose.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/UInt32.h"
#include "sensor_msgs/Imu.h"
#include "rtcm_msgs/Message.h"
#include <nmeaparse/nmea.h>
#include "GeographicLib/DMS.hpp"
#include <boost/algorithm/string.hpp>
#include "nmea_msgs/Sentence.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

using namespace xbot::driver::gps;
using namespace nmea;

ros::Publisher pose_pub;
ros::Publisher xbot_pose_pub;
ros::Publisher latency_pub1;
ros::Publisher latency_pub2;
ros::Publisher latency_pub3;
ros::Publisher imu_pub;
ros::Publisher vrs_nmea_pub;

bool isUbxInterface = false;
GpsInterface *gpsInterface;

bool allow_verbose_logging = false;
xbot_msgs::AbsolutePose pose_result;

std_msgs::UInt32 latency_msg1, latency_msg2, latency_msg3;
sensor_msgs::Imu imu_msg;

ros::Time last_wheel_tick_time(0.0);
ros::Time last_vrs_feedback(0.0);
nmea_msgs::Sentence vrs_msg;

void generate_nmea(double lat_in, double lon_in) {
    // only send every 10 seconds, this will be more than needed
    if ((ros::Time::now() - last_vrs_feedback).toSec() < 10.0) {
        return;
    }
    last_vrs_feedback = ros::Time::now();
    NMEACommand cmd1;

    auto lat = GeographicLib::DMS::Encode(lat_in, GeographicLib::DMS::component::MINUTE, 4,
                                          GeographicLib::DMS::flag::LATITUDE, ';');
    auto lon = GeographicLib::DMS::Encode(lon_in, GeographicLib::DMS::component::MINUTE, 4,
                                          GeographicLib::DMS::flag::LONGITUDE, ';');

    // remove separator char
    boost::erase_all(lat, ";");
    boost::erase_all(lon, ";");

    auto lat_hemisphere = lat.substr(lat.length() - 1, 1);
    auto lon_hemisphere = lon.substr(lon.length() - 1, 1);


    std::stringstream message_ss;
    auto time_facet = new boost::posix_time::time_facet("%H%M%s");

    message_ss.imbue(std::locale(message_ss.getloc(), time_facet));
    message_ss << ros::Time::now().toBoost() << "," <<
               lat.substr(0, lat.length() - 1) << "," <<
               lat_hemisphere << "," <<
               lon.substr(0, lon.length() - 1) << "," <<
               lon_hemisphere << ",1,0,0,0,M,0,M,0000,";

    //build message
    cmd1.name = "GPGGA";
    cmd1.message = message_ss.str();

    vrs_msg.header.frame_id = "gps";
    vrs_msg.header.seq++;
    vrs_msg.header.stamp = ros::Time::now();
    vrs_msg.sentence = cmd1.toString();
    boost::erase_all(vrs_msg.sentence, "\r\n");
    vrs_nmea_pub.publish(vrs_msg);
}

void gps_log(std::string text, GpsInterface::Level level) {
    switch (level) {
        case GpsInterface::Level::VERBOSE:
            if (!allow_verbose_logging) {
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
    // Limit frequency
    if (msg->stamp - last_wheel_tick_time < ros::Duration(0.09))
        return;
    // drop if not ubx
    if(!isUbxInterface)
        return;
    ((UbxGpsInterface*)gpsInterface)->send_wheel_ticks(static_cast<uint32_t>(msg->stamp.toNSec() / 1000000), msg->wheel_direction_rl,
                                  msg->wheel_ticks_rl / 10,
                                  msg->wheel_direction_rr, msg->wheel_ticks_rr / 10);
    last_wheel_tick_time = msg->stamp;
}

void rtcm_received(const rtcm_msgs::Message::ConstPtr &rtcm) {
    gpsInterface->send_rtcm(rtcm->message.data(), rtcm->message.size());
}

void convert_gps_result(const GpsInterface::GpsState &state, xbot_msgs::AbsolutePose &result) {
    result.header.seq++;
    result.header.frame_id = "gps";
    result.header.stamp = ros::Time::now();

    result.source = xbot_msgs::AbsolutePose::SOURCE_GPS;
    result.flags = 0;
    result.sensor_stamp = state.sensor_time;
    result.received_stamp = state.received_time;
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

    if (state.fix_type == GpsInterface::GpsState::FixType::DR_ONLY ||
        state.fix_type == GpsInterface::GpsState::FixType::GNSS_DR_COMBINED) {
        result.flags |= xbot_msgs::AbsolutePose::FLAG_GPS_DEAD_RECKONING;
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

void gps_state_received(const GpsInterface::GpsState &state) {
    // new state received, publish
    convert_gps_result(state, pose_result);
    xbot_pose_pub.publish(pose_result);
    pose_pub.publish(pose_result.pose);

    // send feedback to VRS
    generate_nmea(state.pos_lat, state.pos_lon);
}

void
wheel_latency_received(uint32_t wheel_tick_stamp, uint32_t wheel_tick_stamp_ublox,
                       uint32_t wheel_tick_round_trip_stamp) {
    latency_msg1.data = wheel_tick_stamp;
    latency_msg2.data = wheel_tick_stamp_ublox;
    latency_msg3.data = wheel_tick_round_trip_stamp;
    latency_pub1.publish(latency_msg1);
    latency_pub2.publish(latency_msg2);
    latency_pub3.publish(latency_msg3);
}

void
imu_received(const GpsInterface::ImuState &state) {
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "gps";
    imu_msg.header.seq++;
    imu_msg.angular_velocity.x = state.gx;
    imu_msg.angular_velocity.y = state.gy;
    imu_msg.angular_velocity.z = state.gz;
    imu_msg.linear_acceleration.x = state.ax;
    imu_msg.linear_acceleration.y = state.ay;
    imu_msg.linear_acceleration.z = state.az;
    imu_pub.publish(imu_msg);
}

int main(int argc, char **argv) {


    ros::init(argc, argv, "xbot_driver_gps");

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    isUbxInterface = paramNh.param("ubx_mode", true);
    if(isUbxInterface) {
        ROS_INFO_STREAM("Using UBX mode for GPS");
        gpsInterface = new UbxGpsInterface();
    } else {
        ROS_INFO_STREAM("Using NMEA mode for GPS");
        gpsInterface = new NmeaGpsInterface();
    }

    gpsInterface->set_log_function(gps_log);

    allow_verbose_logging = paramNh.param("verbose", false);
    if (allow_verbose_logging) {
        ROS_WARN("GPS node has verbose logging enabled");
    }

    if(paramNh.param("read_from_file", false)) {
        ROS_INFO_STREAM("Reading GPS data from file!");
        gpsInterface->set_file_name(paramNh.param("filename", std::string("/dev/null")));
    } else {
        gpsInterface->set_baudrate(paramNh.param("baudrate", 38400));
        gpsInterface->set_serial_port(paramNh.param("serial_port", std::string("/dev/ttyACM0")));
    }

    std::string mode = paramNh.param("mode", std::string("absolute"));
    if (mode == "absolute") {
        ROS_INFO_STREAM("Using absolute mode for GPS");
        gpsInterface->set_mode(xbot::driver::gps::GpsInterface::ABSOLUTE);
        double datum_lat, datum_long, datum_height;
        bool has_datum = true;
        has_datum &= paramNh.getParam("datum_lat", datum_lat);
        has_datum &= paramNh.getParam("datum_long", datum_long);
        has_datum &= paramNh.getParam("datum_height", datum_height);
        if (!has_datum) {
            ROS_ERROR_STREAM(
                    "You need to provide datum_lat and datum_long and datum_height in order to use the absolute mode");
            return 2;
        }
        gpsInterface->set_datum(datum_lat, datum_long, datum_height);
    } else if (mode == "relative") {
        ROS_INFO_STREAM("Using relative mode for GPS");
        gpsInterface->set_mode(xbot::driver::gps::GpsInterface::RELATIVE);
    }


    // Subscribe to wheel ticks
    ros::Subscriber wheel_tick_sub = paramNh.subscribe("wheel_ticks", 0, wheel_tick_received,
                                                       ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber rtcm_sub = n.subscribe("rtcm", 0, rtcm_received,
                                           ros::TransportHints().tcpNoDelay(true));


    vrs_nmea_pub = n.advertise<nmea_msgs::Sentence>("/nmea", 10);
    pose_pub = paramNh.advertise<geometry_msgs::PoseWithCovariance>("pose", 10);
    xbot_pose_pub = paramNh.advertise<xbot_msgs::AbsolutePose>("xb_pose", 10);
    imu_pub = paramNh.advertise<sensor_msgs::Imu>("imu", 10);


    gpsInterface->set_state_callback(gps_state_received);

    if (paramNh.param("publish_latency", true) && isUbxInterface) {
        latency_pub1 = paramNh.advertise<std_msgs::UInt32>("wheel_tick_stamp_esc", 100);
        latency_pub2 = paramNh.advertise<std_msgs::UInt32>("wheel_tick_ublox_rx", 100);
        latency_pub3 = paramNh.advertise<std_msgs::UInt32>("wheel_tick_round_trip_host", 100);
        dynamic_cast<UbxGpsInterface*>(gpsInterface)->set_wheel_latency_callback(wheel_latency_received);
    }
    gpsInterface->set_imu_callback(imu_received);

    if (!gpsInterface->start()) {
        return 1;
    }


    while (ros::ok()) {
        ros::spin();
    }

    gpsInterface->stop();
    delete gpsInterface;
    return 0;
}
