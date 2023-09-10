#include "node.hpp"

UbloxF9PNode::UbloxF9PNode(const rclcpp::NodeOptions &options) : rclcpp::Node(UBLOX_F9P_NODE_NAME, options) {
    debug = this->declare_parameter("debug", false);
    if (debug) {
        if (rcutils_logging_set_logger_level(UBLOX_F9P_NODE_NAME, RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK) {
            RCLCPP_WARN(this->get_logger(), "Failed to set the debugging level");
        }
    }

    navSatFixPublisher = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);

    if (this->declare_parameter("publish_imu", false)) {
        imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    }

    this->rtcmSubscription = this->create_subscription<rtcm_msgs::msg::Message>("/rtcm", 10, std::bind(&UbloxF9PNode::rtcmCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "UbloxF9PNode started");
    gpsInterface = new UbxGpsInterface();
    gpsInterface->set_log_function(std::bind(&UbloxF9PNode::gpsLogCallback, this, std::placeholders::_1, std::placeholders::_2));
    gpsInterface->set_state_callback(std::bind(&UbloxF9PNode::gpsStateCallback, this, std::placeholders::_1));
    gpsInterface->set_imu_callback(std::bind(&UbloxF9PNode::imuCallback, this, std::placeholders::_1));
}

void UbloxF9PNode::gpsLogCallback(const std::string &msg, GpsInterface::Level level) {
    switch (level) {
        case GpsInterface::Level::VERBOSE:
            if (!debug) {
                return;
            }
            ROS_INFO_STREAM("[driver_gps] " << msg);
            break;
        case GpsInterface::Level::INFO:
            ROS_INFO_STREAM("[driver_gps] " << msg);
            break;
        case GpsInterface::Level::WARN:
            ROS_WARN_STREAM("[driver_gps] " << msg);
            break;
        default:
            ROS_ERROR_STREAM("[driver_gps] " << msg);
            break;
    }
}

void UbloxF9PNode::gpsStateCallback(const GpsInterface::GpsState &state) {
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "gps";
    msg.latitude = state.pos_lat;
    msg.longitude = state.pos_lon;
    msg.position_covariance = {
            pow(state.position_accuracy, 2), 0.0, 0.0,
            0.0, pow(state.position_accuracy, 2), 0.0,
            0.0, 0.0, pow(state.position_accuracy, 2)
    };
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    navSatFixPublisher->publish(msg);

    // todo: handle the rest of GpsState values
}

void UbloxF9PNode::imuCallback(const GpsInterface::ImuState &state) {
    if (imuPublisher == nullptr) {
        return;
    }

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "gps";
    msg.header.seq++;

    msg.angular_velocity.x = state.gx;
    msg.angular_velocity.y = state.gy;
    msg.angular_velocity.z = state.gz;
    msg.linear_acceleration.x = state.ax;
    msg.linear_acceleration.y = state.ay;
    msg.linear_acceleration.z = state.az;

    imuPublisher->publish(msg);
}

void UbloxF9PNode::rtcmCallback(const rtcm_msgs::msg::Message::SharedPtr msg) {
    gpsInterface->send_rtcm(rtcm->message.data(), rtcm->message.size());
}
