#include "node.hpp"

UbloxF9PNode::UbloxF9PNode(const rclcpp::NodeOptions &options) : rclcpp::Node(UBLOX_F9P_NODE_NAME, options) {
    debug = this->declare_parameter("debug", false);
    if (debug) {
        if (rcutils_logging_set_logger_level(UBLOX_F9P_NODE_NAME, RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK) {
            RCLCPP_WARN(this->get_logger(), "Failed to set the debugging level");
        }
    }

    navsat_fix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);

    if (this->declare_parameter("publish_imu", false)) {
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    }

    this->rtcm_subscriber_ = this->create_subscription<rtcm_msgs::msg::Message>("/rtcm", 10,
                                                                                std::bind(&UbloxF9PNode::rtcmCallback,
                                                                                          this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "UbloxF9PNode started");
    const std::string port = this->declare_parameter("port", "/dev/ttyACM0");
    const int baudrate = this->declare_parameter("baudrate", 115200);
    ublox_ = new UBlox();
    ublox_->setLogCallback(
            std::bind(&UbloxF9PNode::gpsLogCallback, this, std::placeholders::_1, std::placeholders::_2));
    ublox_->setGPSStateCallback(std::bind(&UbloxF9PNode::gpsStateCallback, this, std::placeholders::_1));

    ublox_->connect(port, baudrate);
//    ublox_->setIMUCallback(std::bind(&UbloxF9PNode::imuCallback, this, std::placeholders::_1));
}

void UbloxF9PNode::gpsLogCallback(const std::string &msg, UBlox::LogLevel level) {
    switch (level) {
        case UBlox::LogLevel::DEBUG:
            if (!debug) {
                return;
            }
            RCLCPP_DEBUG_STREAM(this->get_logger(), msg);
            break;
        case UBlox::LogLevel::INFO:
            RCLCPP_INFO_STREAM(this->get_logger(), msg);
            break;
        case UBlox::LogLevel::WARN:
            RCLCPP_WARN_STREAM(this->get_logger(), msg);
            break;
        default:
            RCLCPP_ERROR_STREAM(this->get_logger(), msg);
            break;
    }
}

void UbloxF9PNode::gpsStateCallback(const UBlox::GPSState &state) {
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "gps";
    msg.latitude = state.pos_lat;
    msg.longitude = state.pos_lon;
    msg.altitude = state.pos_altitude;
    msg.position_covariance = {
            pow(state.position_accuracy, 2), 0.0, 0.0,
            0.0, pow(state.position_accuracy, 2), 0.0,
            0.0, 0.0, pow(state.position_accuracy, 2)
    };
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    navsat_fix_publisher_->publish(msg);

    // todo: handle the rest of GpsState values
}

//void UbloxF9PNode::imuCallback(const UBlox::IMUState &state) {
//    if (imu_publisher_ == nullptr) {
//        return;
//    }
//
//    sensor_msgs::msg::Imu msg;
//    msg.header.stamp = this->now();
//    msg.header.frame_id = "gps";
//    msg.header.seq++;
//
//    msg.angular_velocity.x = state.gx;
//    msg.angular_velocity.y = state.gy;
//    msg.angular_velocity.z = state.gz;
//    msg.linear_acceleration.x = state.ax;
//    msg.linear_acceleration.y = state.ay;
//    msg.linear_acceleration.z = state.az;
//
//    imu_publisher_->publish(msg);
//}

void UbloxF9PNode::rtcmCallback(const rtcm_msgs::msg::Message::SharedPtr msg) {
    std::vector<uint8_t> data(&msg->message.data()[0], &msg->message.data()[msg->message.size()]);

    ublox_->sendRTCM(data);
}

UbloxF9PNode::~UbloxF9PNode() {
    delete ublox_;
}
