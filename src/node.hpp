#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <rtcm_msgs/msg/message.hpp>

#include "ublox_gps_interface.hpp"

#define UBLOX_F9P_NODE_NAME "ublox_f9p"

class UbloxF9PNode final : public rclcpp::Node {
public:
    const float diagnosticPeriod = 0.2;

    explicit UbloxF9PNode(const rclcpp::NodeOptions &options);

    ~UbloxF9PNode() override;

private:
    bool debug;

    GpsInterface *gpsInterface;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navSatFixPublisher;

    rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcmSubscription;

    void gpsLogCallback(const std::string &msg, GpsInterface::Level level);

    /**
     * @brief Callback for u-blox F9P GPS state
     */
    void gpsStateCallback(const GpsInterface::GpsState &state);

    /**
     * @brief Callback for u-blox F9P IMU state
     */
    void imuCallback(const GpsInterface::ImuState &state);

    /**
     * @brief Callback for '/ntrip_client/rtcm' subscription to handle RTCM correction data
     */
    void rtcmCallback(const rtcm_msgs::msg::Message::SharedPtr msg);
};
