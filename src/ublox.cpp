#include <serial_driver/serial_driver.hpp>
#include <pthread.h>
#include <robot_localization/navsat_conversions.hpp>

#include "ublox.hpp"

class UBlox::Serial {
public:
    typedef std::vector<uint8_t> Buffer;

    Serial() : owned_ctx(new IoContext(2)), serial_driver_(new drivers::serial_driver::SerialDriver(*owned_ctx)) {
        data_updated_ = false;
    }

    void *rxThread(void);

    void write(const Buffer &buffer) {
        serial_driver_->port()->send(buffer);
    }

    static void *rxThreadHelper(void *context) {
        return ((UBlox::Serial *) context)->rxThread();
    }

    bool validateChecksum(const Buffer &packet);

    pthread_t rx_thread_;
    bool rx_thread_run_;
    NavPacketHandlerFunction packet_handler_;
    LogFunction log_function_;
    std::unique_ptr<IoContext> owned_ctx{};
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
    bool data_updated_;

    void receiveUbxPacket(std::chrono::time_point<std::chrono::steady_clock> &point, Buffer data);
};

void *UBlox::Serial::rxThread(void) {
    auto headerAlreadyFound = false;
    std::chrono::time_point<std::chrono::steady_clock> packetReceivedTime;

    int bytesNeeded = 6; // we need at least 6 bytes to parse the header

    Buffer buffer;
    auto tempBuffer = Buffer(4096);

    while (rx_thread_run_) {
        auto bytesRead = serial_driver_->port()->receive(tempBuffer);
        buffer.reserve(buffer.size() + bytesRead);
        buffer.insert(buffer.end(), tempBuffer.begin(), tempBuffer.begin() + bytesRead);

        if (bytesNeeded > 0 && 0 == bytesRead && !buffer.empty()) {
            log_function_("Possibly out-of-sync with u-blox. Read timeout in the middle of a frame.", WARN);
            continue;
        }

        if (buffer.empty()) {
            continue;
        }

        while (buffer.size() >= 6) {
            if (buffer[0] != 0x05 && buffer[1] != 0x62) {
                buffer.erase(buffer.begin()); // remove first byte
                continue;
            }

            if (!headerAlreadyFound) {
                headerAlreadyFound = true;
                packetReceivedTime = std::chrono::steady_clock::now();
            }

            auto totalLength = (buffer[5] << 8 | buffer[4]) + 8;

            if (totalLength > buffer.size()) {
                log_function_(
                        "Not enough data in buffer to parse packetData. Need " + std::to_string(totalLength) + " bytes, have " +
                        std::to_string(buffer.size()), DEBUG);
                bytesNeeded = totalLength - buffer.size();
                break;
            }

            bytesNeeded = totalLength;

            Buffer packetData;
            packetData.reserve(totalLength);
            packetData.insert(packetData.begin(), buffer.begin(), buffer.begin() + totalLength);

            buffer.erase(buffer.begin(), buffer.begin() + totalLength);

            if (!validateChecksum(packetData)) {
                log_function_("Got ubx packet with invalid checksum", WARN);

                continue;
            }

            receiveUbxPacket(packetReceivedTime, Buffer(packetData.begin() + 2, packetData.end() - 2));
        }
    }
}

bool UBlox::Serial::validateChecksum(const Buffer &packet) {
    uint8_t ck_a = 0, ck_b = 0;
    for (auto iter = packet.begin() + 2; iter != packet.end() - 2; ++iter) {
        ck_a += *iter;
        ck_b += ck_a;
    }
    return ck_a == packet[packet.size() - 2] && ck_b == packet[packet.size() - 1];
}

void UBlox::Serial::receiveUbxPacket(std::chrono::time_point<std::chrono::steady_clock> &point, Buffer data) {
    uint16_t packetID = data[0] << 8 | data[1];

    switch (packetID) {
        case UbxNavPvt::CLASS_ID << 8 | UbxNavPvt::MESSAGE_ID: {
            // substract class, id and length
            if (data.size() - 4 == sizeof(struct UbxNavPvt)) {
                const UbxNavPvt *msg = reinterpret_cast<UbxNavPvt *>(data.data() + 4);
                const auto s = std::make_shared<UbxNavPvt const>(*msg);
                packet_handler_(point, s);
            } else {
                log_function_("size mismatch for PVT message!", WARN);
            }
        }
            break;

        case 0x10 << 8 | 0x02: {
            log_function_("received unsupported esf measurement message", DEBUG);
        }
            break;
        default:
            log_function_("received unsupported ubx message: " + std::to_string(packetID), DEBUG);
            break;
    }
}

UBlox::UBlox() : serial_(new Serial()) {
    serial_->packet_handler_ = std::bind(&UBlox::navPacketHandler, this, std::placeholders::_1, std::placeholders::_2);
}

void UBlox::connect(const std::string &port, const uint32_t baudRate) {
    serial_->log_function_ = log_function_;

    if (isConnected()) {
        throw SerialException("Already connected to serial port.");
    }

    try {
        const auto fc = drivers::serial_driver::FlowControl::NONE;
        const auto pt = drivers::serial_driver::Parity::NONE;
        const auto sb = drivers::serial_driver::StopBits::ONE;
        serial_->device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baudRate, fc, pt, sb);
        serial_->serial_driver_->init_port(port, *serial_->device_config_);
        if (!serial_->serial_driver_->port()->is_open())
        {
            serial_->serial_driver_->port()->open();
        }
    } catch (const std::exception &e) {
        std::stringstream ss;
        ss << "Failed to open the serial port " << port << " to the u-blox. " << e.what();
        throw SerialException(ss.str().c_str());
    }

    // start up a monitoring thread
    serial_->rx_thread_run_ = true;
    int result = pthread_create(&serial_->rx_thread_, NULL, &UBlox::Serial::rxThreadHelper, serial_.get());

    if (result != 0) {
        throw SerialException("Failed to start serial rx thread.");
    }
}

void UBlox::disconnect() {
    if (isConnected()) {
        serial_->rx_thread_run_ = false;
        pthread_join(serial_->rx_thread_, nullptr);
        serial_->serial_driver_->port()->close();
    }
}

bool UBlox::isConnected() const {
    auto port = serial_->serial_driver_->port();
    return port && port->is_open();
}

UBlox::~UBlox() {
    disconnect();
}

void UBlox::navPacketHandler(const std::chrono::time_point<std::chrono::steady_clock> &time, const UbxNavPvtConstPtr &packet) {
    bool gnssFixOK = packet->flags & 0b0000001;

    if (!(packet->flags & 0b0000001)) {
        log_function_("No GNSS fix, skipping message", WARN);
        return;
    }

    if (packet->flags3 & 0b1) {
        log_function_("invalid coordinates, skipping message", DEBUG);
        return;
    }

//    if (gps_state_valid_) {
//        auto time_diff = duration_cast<milliseconds>(header_stamp - last_gps_message).count();
//        uint32_t pvt_diff = msg->iTOW - gps_state_iTOW_;
//
//        double diff = (double) time_diff - (double) pvt_diff;
//
//        // Check, if time was spent since the last packet. If not, the data was already in some buffer somewhere
//        if (time_diff == 0) {
//            log("gps time diff was: " + std::to_string(pvt_diff) + ", host time diff was: " +
//                std::to_string(time_diff), ERROR);
//        } else if (abs(diff) > 100.0) {
//            log("gps time diff was: " + std::to_string(pvt_diff) + ", host time diff was: " +
//                std::to_string(time_diff), ERROR);
//        }
//    }

    auto gpsState = GPSState();

    switch(packet->fixType) {
        case 1:
            gpsState.fix_type = GPSState::FixType::DR_ONLY;
            break;
        case 2:
            gpsState.fix_type = GPSState::FixType::FIX_2D;
            break;
        case 3:
            gpsState.fix_type = GPSState::FixType::FIX_3D;
            break;
        case 4:
            gpsState.fix_type = GPSState::FixType::GNSS_DR_COMBINED;
            break;
        default:
            gpsState.fix_type = GPSState::FixType::NO_FIX;
            break;
    }

    // Calculate the position
    double lat = (double) packet->lat / 10000000.0;
    double lon = (double) packet->lon / 10000000.0;
    double altitude = (double) packet->height / 1000.0;
    double e, n;
    std::string zone;
    robot_localization::navsat_conversions::LLtoUTM(lat, lon, n, e, zone);
    gpsState.pos_lat = lat;
    gpsState.pos_lon = lon;
    gpsState.pos_altitude = altitude;
    gpsState.position_valid = true;
    gpsState.pos_e = e;
    gpsState.pos_n = n;
    gpsState.position_accuracy = (double) sqrt(
            pow((double) packet->hAcc / 1000.0, 2) + pow((double) packet->vAcc / 1000.0, 2));

    gpsState.vel_e = packet->velE / 1000.0;
    gpsState.vel_n = packet->velN / 1000.0;
    gpsState.vel_u = -packet->velD / 1000.0;


    double headAcc = (packet->headAcc / 100000.0) * (M_PI / 180.0);

    double hedVeh = packet->headVeh / 100000.0;
    hedVeh = -hedVeh * (M_PI / 180.0);
    hedVeh = fmod(hedVeh + (M_PI_2), 2.0 * M_PI);
    while (hedVeh < 0) {
        hedVeh += M_PI * 2.0;
    }

    double headMotion = packet->headMot / 100000.0;
    headMotion = -headMotion * (M_PI / 180.0);
    headMotion = fmod(headMotion + (M_PI_2), 2.0 * M_PI);
    while (headMotion < 0) {
        headMotion += M_PI * 2.0;
    }

    // There's no flag for that. Assume it's good
    gpsState.motion_heading_valid = true;
    gpsState.motion_heading = headMotion;
    gpsState.motion_heading_accuracy = headAcc;

    // headAcc is the same for both
    gpsState.vehicle_heading_valid = (packet->flags & 0b100000) >> 5;
    gpsState.vehicle_heading_accuracy = headAcc;
    gpsState.vehicle_heading = hedVeh;

    gpsState.sensor_time = packet->iTOW;
    gpsState.received_time = std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count();

    // Latency tracking
//    auto last_gps_message = time;
//    gps_state_valid_ = true;
//    gps_state_iTOW_ = msg->iTOW;

    std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
    if (gps_state_handler_)
        gps_state_handler_(gpsState);
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
    if (millis > 10) {
        log_function_("Slow GPS state handler: " + std::to_string(millis) + " ms", ERROR);
    }
}

void UBlox::sendRTCM(const std::vector<uint8_t> &data) {
    serial_->write(data);
}

void UBlox::setGPSStateCallback(const UBlox::GPSStateHandlerFunction &handler) {
    gps_state_handler_ = handler;
}

void UBlox::setLogCallback(const UBlox::LogFunction &handler) {
    log_function_ = handler;
}
