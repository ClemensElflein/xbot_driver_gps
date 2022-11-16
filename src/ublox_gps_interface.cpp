//
// Created by Clemens Elflein on 14.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "ublox_gps_interface.h"

using namespace std::chrono;

namespace xbot {
    namespace driver {
        namespace gps {
            void GpsInterface::set_serial_port(std::string port) {
                port_ = port;
            }

            bool GpsInterface::start() {

                gps_state_ = {0};

                if (mode_ != ABSOLUTE && mode_ != RELATIVE) {
                    log("no mode set, can't start", ERROR);
                    return false;
                }

                if (mode_ == ABSOLUTE && (std::isnan(datum_n_) || std::isnan(datum_e_) || std::isnan(datum_u_))) {
                    log("absolute positioning with invalid datum, can't start", ERROR);
                    return false;
                }

                if (baudrate_ == 0) {
                    log("no baudrate set, can't start", ERROR);
                    return false;
                }
                if (port_.empty()) {
                    log("no serial port set, can't start", ERROR);
                    return false;
                }

                stopped_ = false;
                // clear tx buffer
                {
                    std::unique_lock<std::mutex> lk(tx_mutex_);
                    tx_buffer_.clear();
                }

                pthread_create(&rx_thread_handle_, NULL, &GpsInterface::rx_thread_helper, this);
                pthread_create(&tx_thread_handle_, NULL, &GpsInterface::tx_thread_helper, this);

                /*
                 TODO: test how much impact that has in real life application
                 TODO: only enable in sensor-fusion mode
                struct sched_param param;
                param.sched_priority = sched_get_priority_max(SCHED_FIFO);
                log("setting prio to: " + std::to_string(param.sched_priority), INFO);
                bool success = true;
                success &= pthread_setschedparam(rx_thread_handle_, SCHED_FIFO, &param) == 0;
                success &= pthread_setschedparam(rx_thread_handle_, SCHED_FIFO, &param) == 0;
                if (!success) {
                    log("error setting realtime prio. Are you root?", ERROR);
                }
                 */

                return true;
            }

            void GpsInterface::stop() {
                stopped_ = true;
                log("waiting for serial rx thread to stop", INFO);
                pthread_join(rx_thread_handle_, nullptr);
                log("waiting for serial tx thread to stop", INFO);
                pthread_join(tx_thread_handle_, nullptr);
            }

            void *GpsInterface::tx_thread() {
                while (!stopped_) {
                    std::unique_lock<std::mutex> lk(tx_mutex_);
                    // wait for new data in the tx buffer. We set a timeout so that the thread will stop even if no data arrives
                    if (std::cv_status::timeout == tx_cv_.wait_for(lk, std::chrono::milliseconds(1000))) {
                        // tx timeout - no data
                        continue;
                    }

                    if (!lk.owns_lock()) {
                        log("we don't have the tx lock!", ERROR);
                    }

                    log(std::string("writing ") + std::to_string(tx_buffer_.size()) + " bytes of data", VERBOSE);
                    if (!serial_.isOpen()) {
                        log("serial is closed, dropping data", WARN);
                        continue;
                    }

                    while (!tx_buffer_.empty()) {
                        try {
                            size_t written = serial_.write(tx_buffer_.data(), tx_buffer_.size());
                            if (written != tx_buffer_.size()) {
                                log("not all data has been written to serial. tx_buffer size: " +
                                    std::to_string(tx_buffer_.size()) + ", written: " + std::to_string(written), WARN);
                                tx_buffer_.erase(tx_buffer_.begin(), tx_buffer_.begin() + written);
                            } else {
                                tx_buffer_.clear();
                            }
                        } catch (std::exception &e) {
                            log("error writing to serial port!", ERROR);
                        }
                    }
                }

                return nullptr;
            }

            void *GpsInterface::rx_thread() {
                std::vector<uint8_t> buf;

                gps_state_valid_ = false;
                found_header_ = false;
                imu_fields_valid_ = 0;

                // stores the amount of bytes to read. At first we have this small until the parse_rx_buffer is able to know
                // how many more bytes it needs. then we read that amount of data, before parsing again.
                // This reduces latency, because we don't fill a buffer before reading and we don't parse on every byte as soon as we know how many
                // we actually need
                size_t bytes_to_read = 6;

                while (!stopped_) {
                    // Check, if the serial port is connected. If not, connect to it.
                    if (!serial_.isOpen()) {
                        // start small
                        bytes_to_read = 6;
                        found_header_ = false;
                        gps_state_valid_ = false;
                        imu_fields_valid_ = 0;
                        log("opening serial port: " + port_ + " with baudrate: " + std::to_string(baudrate_), INFO);
                        try {
                            serial_.setPort(port_);
                            serial_.setBaudrate(baudrate_);
                            auto to = serial::Timeout::simpleTimeout(100);
                            serial_.setTimeout(to);
                            serial_.open();
                        } catch (std::exception &e) {
                            // retry later
                            log("error opening serial port.", ERROR);
                            sleep(1);
                            continue;
                        }
                        // clear tx buffer
                        {
                            std::unique_lock<std::mutex> lk(tx_mutex_);
                            tx_buffer_.clear();
                        }
                    }

                    // Serial port connected, read data
                    try {
                        int bytes_read = serial_.read(buf, bytes_to_read);
                        if (bytes_read) {
                            rx_buffer_.insert(rx_buffer_.end(), buf.begin(), buf.end());
                            buf.clear();
                            bytes_to_read = parse_rx_buffer();
                        }
                    } catch (std::exception &e) {
                        log("error during serial read. reconnecting.", ERROR);
                        serial_.close();
                    }
                }

                serial_.close();

                return nullptr;
            }

            void GpsInterface::set_baudrate(uint32_t baudrate) {
                baudrate_ = baudrate;
            }

            bool GpsInterface::send_raw(const void *data, size_t size) {
                time_point<std::chrono::steady_clock> start = steady_clock::now();
                std::unique_lock<std::mutex> lk(tx_mutex_);
                auto millis = duration_cast<milliseconds>(steady_clock::now() - start).count();

                if (millis > 10) {
                    log("waited " + std::to_string(millis) +
                        " ms to write to the tx buffer, serial port is probably congested!", ERROR);
                }

                // extend the buffer
                tx_buffer_.reserve(size + tx_buffer_.size());
                // insert the data
                tx_buffer_.insert(tx_buffer_.end(), (uint8_t *) data, (uint8_t *) data + size);


                if (tx_buffer_.size() > 5000) {
                    log("high tx buffer size: " + std::to_string(tx_buffer_.size()), ERROR);
                }

                tx_cv_.notify_all();
                return true;
            }

            bool GpsInterface::send_packet(uint8_t *frame, size_t size) {
                frame[0] = 0xb5;
                frame[1] = 0x62;
                auto *length_ptr = reinterpret_cast<uint16_t *>(frame + 4);
                *length_ptr = size - 8;

                uint8_t ck_a, ck_b;
                calculate_checksum(frame + 2, size - 4, ck_a, ck_b);

                frame[size - 2] = ck_a;
                frame[size - 1] = ck_b;

                return send_raw(frame, size);
            }

            void GpsInterface::set_log_function(const GpsInterface::LogFunction &function) {
                log = function;
            }

            /**
             * parses the buffer and returns how many more bytes to read
             */
            size_t GpsInterface::parse_rx_buffer() {
                while (rx_buffer_.size() >= 6) {
                    // skip to the 0xb5
                    if (rx_buffer_[0] != 0x05 && rx_buffer_[1] != 0x62) {
                        rx_buffer_.pop_front();
                        log("skipping rx byte", WARN);
                        found_header_ = false;
                        // check again
                        continue;
                    }

                    // we're here the first time for this packet, note the time
                    if (!found_header_) {
                        current_gps_header_time_ = steady_clock::now();
                        found_header_ = true;
                    }

                    // get the length first to check, if we already got enough bytes
                    uint16_t payload_length = rx_buffer_[5] << 8 | rx_buffer_[4];
                    uint16_t total_length = payload_length + 8;

                    if (total_length > rx_buffer_.size()) {
                        // fetch more data. bonus: we know exactly how many bytes
                        return total_length - rx_buffer_.size();
                    }

                    // we have all the data, copy it to an array and remove it from the deque
                    std::vector<uint8_t> packet;
                    packet.reserve(total_length);

                    packet.insert(packet.begin(), rx_buffer_.begin(), rx_buffer_.begin() + total_length);
                    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + total_length);


                    if (!validate_checksum(packet.data(), packet.size())) {
                        // invalid packet, reset header
                        found_header_ = false;
                        continue;
                    }

                    process_ubx_packet(current_gps_header_time_, packet.data() + 2, packet.size() - 4);
                    found_header_ = false;
                }

                // we need at least 6 bytes to process anything
                return 6;
            }

            bool GpsInterface::validate_checksum(const uint8_t *packet, size_t size) {
                uint8_t ck_a, ck_b;
                calculate_checksum(packet + 2, size - 4, ck_a, ck_b);

                bool valid = packet[size - 2] == ck_a && packet[size - 1] == ck_b;

                if (!valid) {
                    log("got ubx packet with invalid checksum", WARN);
                    log("expected: a = " + std::to_string(packet[size - 2]) + ", b = " +
                        std::to_string(packet[size - 1]),
                        VERBOSE);
                    log("real: a = " + std::to_string(ck_a) + ", b = " + std::to_string(ck_b), VERBOSE);
                }

                return valid;
            }

            void GpsInterface::process_ubx_packet(const time_point<steady_clock> &header_stamp, const uint8_t *data,
                                                  const size_t &size) {
                // data = no header bytes (starts with class) and stops before checksum

                uint16_t packet_id = data[0] << 8 | data[1];
                switch (packet_id) {
                    case xbot::driver::gps::UbxNavPvt::CLASS_ID << 8 | xbot::driver::gps::UbxNavPvt::MESSAGE_ID: {
                        // substract class, id and length
                        if (size - 4 == sizeof(struct UbxNavPvt)) {
                            const auto *msg = reinterpret_cast<const UbxNavPvt *>(data + 4);
                            handle_nav_pvt(header_stamp, msg);
                        } else {
                            log("size mismatch for PVT message!", WARN);
                        }
                    }
                        break;
/*                    case 0x10 << 8 | 0x03: {
                        log("got esf-raw", INFO);
                    }
                        break;*/
                    case 0x10 << 8 | 0x02: {
                        handle_esf_meas(header_stamp, data + 4, size - 4);
                    }
                        break;
                    default:
                        // unknown message, ignore it
                        break;
                }
            }

            void GpsInterface::handle_nav_pvt(const time_point<steady_clock> &header_stamp, const UbxNavPvt *msg) {
                // We have received a nav pvt message, copy to GPS state
                // check, if message is even roughly valid. If not - ignore it.
                bool gnssFixOK = (msg->flags & 0b0000001);
                bool invalidLlh = (msg->flags3 & 0b1);


                if (!gnssFixOK) {
                    gps_state_valid_ = false;
                    log("invalid gnssFix - dropping message", WARN);
                    return;
                }
                if (invalidLlh) {
                    gps_state_valid_ = false;
                    log("invalid lat, lon, height - dropping message", WARN);
                    return;
                }

                if (gps_state_valid_) {
                    auto time_diff = duration_cast<milliseconds>(header_stamp - last_gps_message).count();
                    uint32_t pvt_diff = msg->iTOW - gps_state_iTOW_;

                    double diff = (double) time_diff - (double) pvt_diff;

                    // Check, if time was spent since the last packet. If not, the data was already in some buffer somewhere
                    if (time_diff == 0) {
                        log("gps time diff was: " + std::to_string(pvt_diff) + ", host time diff was: " +
                            std::to_string(time_diff), ERROR);
                    } else if (abs(diff) > 100.0) {
                        log("gps time diff was: " + std::to_string(pvt_diff) + ", host time diff was: " +
                            std::to_string(time_diff), ERROR);
                    }
                }

                switch (msg->fixType) {
                    case 1:
                        gps_state_.fix_type = GpsState::FixType::DR_ONLY;
                        break;
                    case 2:
                        gps_state_.fix_type = GpsState::FixType::FIX_2D;
                        break;
                    case 3:
                        gps_state_.fix_type = GpsState::FixType::FIX_3D;
                        break;
                    case 4:
                        gps_state_.fix_type = GpsState::FixType::GNSS_DR_COMBINED;
                        break;
                    default:
                        gps_state_.fix_type = GpsState::FixType::NO_FIX;
                        break;
                }


                bool diffSoln = (msg->flags & 0b0000010) >> 1;
                auto carrSoln = (uint8_t) ((msg->flags & 0b11000000) >> 6);
                if (diffSoln) {
                    switch (carrSoln) {
                        case 1:
                            gps_state_.rtk_type = GpsState::RTK_FLOAT;
                            break;
                        case 2:
                            gps_state_.rtk_type = GpsState::RTK_FIX;
                            break;
                        default:
                            gps_state_.rtk_type = GpsState::RTK_NONE;
                            break;
                    }
                } else {
                    gps_state_.rtk_type = GpsState::RTK_NONE;
                }



                // Calculate the position
                double lat = (double) msg->lat / 10000000.0;
                double lon = (double) msg->lon / 10000000.0;
                double u = (double) msg->hMSL / 1000.0;
                double e, n;
                std::string zone;
                RobotLocalization::NavsatConversions::LLtoUTM(lat, lon, n, e, zone);
                gps_state_.pos_lat = lat;
                gps_state_.pos_lon = lon;
                gps_state_.position_valid = true;
                gps_state_.pos_e = e - datum_e_;
                gps_state_.pos_n = n - datum_n_;
                gps_state_.pos_u = u - datum_u_;
                gps_state_.position_accuracy = (double) sqrt(
                        pow((double) msg->hAcc / 1000.0, 2) + pow((double) msg->vAcc / 1000.0, 2));;

                gps_state_.vel_e = msg->velE / 1000.0;
                gps_state_.vel_n = msg->velN / 1000.0;
                gps_state_.vel_u = -msg->velD / 1000.0;


                double headAcc = (msg->headAcc / 100000.0) * (M_PI / 180.0);

                double hedVeh = msg->headVeh / 100000.0;
                hedVeh = -hedVeh * (M_PI / 180.0);
                hedVeh = fmod(hedVeh + (M_PI_2), 2.0 * M_PI);
                while (hedVeh < 0) {
                    hedVeh += M_PI * 2.0;
                }

                double headMotion = msg->headMot / 100000.0;
                headMotion = -headMotion * (M_PI / 180.0);
                headMotion = fmod(headMotion + (M_PI_2), 2.0 * M_PI);
                while (headMotion < 0) {
                    headMotion += M_PI * 2.0;
                }

                // There's no flag for that. Assume it's good
                gps_state_.motion_heading_valid = true;
                gps_state_.motion_heading = headMotion;
                gps_state_.motion_heading_accuracy = headAcc;

                // headAcc is the same for both
                gps_state_.vehicle_heading_valid = (msg->flags & 0b100000) >> 5;
                gps_state_.vehicle_heading_accuracy = headAcc;
                gps_state_.vehicle_heading = hedVeh;

                gps_state_.sensor_time = msg->iTOW;
                gps_state_.received_time = duration_cast<milliseconds>(header_stamp.time_since_epoch()).count();

                // Latency tracking
                last_gps_message = header_stamp;
                gps_state_valid_ = true;
                gps_state_iTOW_ = msg->iTOW;

                time_point<std::chrono::steady_clock> start = steady_clock::now();
                if (state_callback)
                    state_callback(gps_state_);
                auto millis = duration_cast<milliseconds>(steady_clock::now() - start).count();
                if (millis > 10) {
                    log("slow ros publisher: " + std::to_string(millis) + " ms", ERROR);
                }
            }

            void GpsInterface::set_datum(double datum_lat, double datum_long, double datum_height) {
                datum_u_ = datum_height;
                RobotLocalization::NavsatConversions::LLtoUTM(datum_lat, datum_long, datum_n_, datum_e_, datum_zone_);
            }

            void GpsInterface::set_mode(GpsInterface::Mode mode) {
                mode_ = mode;
            }

            GpsInterface::GpsInterface() {
                datum_n_ = datum_e_ = datum_u_ = NAN;
                state_callback = nullptr;
                wheel_latency_callback = nullptr;
                imu_callback = nullptr;
            }

            void GpsInterface::send_rtcm(const uint8_t *data, size_t size) {
                send_raw(data,size);
            }

            void GpsInterface::send_wheel_ticks(uint32_t timestamp, bool direction_left, uint32_t ticks_left,
                                                bool direction_right, uint32_t ticks_right) {
                uint8_t frame[8 + 2 * 4 + 8] = {0};
                // Set the message class and ID
                frame[2] = 0x10;
                frame[3] = 0x02;

                auto *payload = reinterpret_cast<uint32_t *>(frame + 6);
                payload[0] = timestamp;
                // flags etc, it's all 0
                payload[1] = 0;

                uint32_t data_left = ticks_left & 0x7FFFFF;
                if (direction_left) {
                    data_left |= 1 << 23;
                }
                data_left |= 8 << 24;
                payload[2] = data_left;

                uint32_t data_right = ticks_right & 0x7FFFFF;
                if (direction_right) {
                    data_right |= 1 << 23;
                }
                data_right |= 9 << 24;
                payload[3] = data_right;

                send_packet(frame, sizeof(frame));
            }

            void GpsInterface::calculate_checksum(const uint8_t *packet, size_t size, uint8_t &ck_a, uint8_t &ck_b) {
                ck_a = 0;
                ck_b = 0;

                for (size_t i = 0; i < size; i++) {
                    ck_a += packet[i];
                    ck_b += ck_a;
                }
            }

            void GpsInterface::handle_esf_meas(const std::chrono::time_point<std::chrono::steady_clock> &header_stamp,
                                               const uint8_t *payload, size_t payload_sie) {
                uint32_t time_tag = *reinterpret_cast<const uint32_t *>(payload);
                uint16_t flags = *reinterpret_cast<const uint16_t *>(payload + 4);
                uint16_t id = *reinterpret_cast<const uint16_t *>(payload + 6);
                bool has_calib_ttag = flags & 0b1000;

                uint32_t calib_ttag = 0;

                size_t measurement_size = payload_sie - 8;
                if (has_calib_ttag)
                    measurement_size -= 4;
                int n = measurement_size / 4;

                // get the measurements
                auto *measurement_ptr = reinterpret_cast<const uint32_t *>(payload + 8);

                if (has_calib_ttag) {
                    calib_ttag = measurement_ptr[n];
                }

                for (int i = 0; i < n; i++) {
                    int32_t data = measurement_ptr[i] & 0xFFFFFF;

                    // sign extend the data (https://stackoverflow.com/questions/42534749/signed-extension-from-24-bit-to-32-bit-in-c)
                    int32_t m = 1u << 23;
                    data = (data ^ m) - m;

                    uint8_t data_type = measurement_ptr[i] >> 24;
                    switch (data_type) {
                        case 8:
                            // it's a wheel tick we sent earlier, track the round trip latency
                            if (wheel_latency_callback) {
                                wheel_latency_callback(time_tag, calib_ttag,
                                                       duration_cast<milliseconds>(
                                                               header_stamp.time_since_epoch()).count());
                            }
                            continue;
                        case 5:
                            // gyro z
                            if (imu_state_.sensor_time != time_tag) {
                                // a new frame started, clear state
                                imu_fields_valid_ = 0;
                                imu_state_.sensor_time = time_tag;
                                imu_state_.received_time = duration_cast<milliseconds>(
                                        header_stamp.time_since_epoch()).count();
                            }
                            imu_state_.gz = -(double) data / 4096.0 * (M_PI / 180.0);
                            imu_fields_valid_ |= 0b1;
                            break;
                        case 13:
                            // gyro y
                            if (imu_state_.sensor_time != time_tag) {
                                // a new frame started, clear state
                                imu_fields_valid_ = 0;
                                imu_state_.sensor_time = time_tag;
                                imu_state_.received_time = duration_cast<milliseconds>(
                                        header_stamp.time_since_epoch()).count();
                            }
                            imu_state_.gy = (double) data / 4096.0 * (M_PI / 180.0);
                            imu_fields_valid_ |= 0b10;
                            break;
                        case 14:
                            // gyro x
                            if (imu_state_.sensor_time != time_tag) {
                                // a new frame started, clear state
                                imu_fields_valid_ = 0;
                                imu_state_.sensor_time = time_tag;
                                imu_state_.received_time = duration_cast<milliseconds>(
                                        header_stamp.time_since_epoch()).count();
                            }
                            imu_state_.gx = -(double) data / 4096.0 * (M_PI / 180.0);
                            imu_fields_valid_ |= 0b100;
                            break;
                        case 16:
                            // acc x
                            imu_state_.ax = (double) data / 1024.0;
                            imu_fields_valid_ |= 0b1000;
                            break;
                        case 17:
                            // acc y
                            imu_state_.ay = (double) data / 1024.0;
                            imu_fields_valid_ |= 0b10000;
                            break;
                        case 18:
                            // acc z
                            imu_state_.az = (double) data / 1024.0;
                            imu_fields_valid_ |= 0b100000;
                            break;
                        default:
                            // some other measurement
                            continue;
                    }

                    // check, if imu frame is done
                    if(imu_fields_valid_ == 0b111111) {
                        imu_callback(imu_state_);
                        // prevent duplicate send in case other data arrives before imu
                        imu_fields_valid_ = 0b1111111;
                    }
                }
            }

            void GpsInterface::set_state_callback(const GpsInterface::StateCallback &function) {
                state_callback = function;
            }

            void GpsInterface::set_wheel_latency_callback(const GpsInterface::LatencyCallback &function) {
                wheel_latency_callback = function;
            }

            void GpsInterface::set_imu_callback(const GpsInterface::ImuCallback &function) {
                imu_callback = function;
            }

        }
    }
}

