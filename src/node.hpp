#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <rtcm_msgs/msg/message.hpp>

#include "ublox.hpp"

#define UBLOX_F9P_NODE_NAME "ublox_f9p"

class UbloxF9PNode final : public rclcpp::Node {
public:
//    const float diagnosticPeriod = 0.2;

    explicit UbloxF9PNode(const rclcpp::NodeOptions &options);

    ~UbloxF9PNode() override;

private:
    bool debug;

    UBlox *ublox_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_fix_publisher_;

    rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_subscriber_;

    void gpsLogCallback(const std::string &msg, UBlox::LogLevel level);

    /**
     * @brief Callback for u-blox F9P GPS state
     */
    void gpsStateCallback(const UBlox::GPSState &state);

    /**
     * @brief Callback for u-blox F9P IMU state
     */
//    void imuCallback(const UBlox::IMUState &state);

    /**
     * @brief Callback for '/ntrip_client/rtcm' subscription to handle RTCM correction data
     */
    void rtcmCallback(const rtcm_msgs::msg::Message::SharedPtr msg);

};
