//
// Created by Clemens Elflein on 14.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "gps_interface.h"

namespace xbot {
    namespace driver_gps_ublox {
        void GpsInterface::set_serial_port(std::string port) {
            port_ = port;
        }

        bool GpsInterface::start() {

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
                    log("tx timeout - no data", VERBOSE);
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
                }

                // Serial port connected, read data
                try {
                    int bytes_read = serial_.read(buf, bytes_to_read);
                    if (bytes_read) {
                        log("read " + std::to_string(bytes_read) + " bytes", VERBOSE);
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
            using namespace std::chrono;
            time_point<std::chrono::steady_clock> start = steady_clock::now();
            std::unique_lock<std::mutex> lk(tx_mutex_);
            auto millis = duration_cast<milliseconds>(steady_clock::now() - start).count();

            if (millis > 10) {
                log("waited " + std::to_string(millis) +
                    " ms to write to the tx buffer, serial port is probably congested!", ERROR);
            } else {
                log("waited " + std::to_string(millis) + " ms to write to the tx buffer", VERBOSE);
            }

            // extend the buffer
            tx_buffer_.reserve(size + tx_buffer_.size());
            // insert the data
            tx_buffer_.insert(tx_buffer_.end(), (uint8_t *) data, (uint8_t *) data + size);


            if (tx_buffer_.size() > 1000) {
                log("high tx buffer size: " + std::to_string(tx_buffer_.size()), ERROR);
            }

            tx_cv_.notify_all();
            return true;
        }

        bool GpsInterface::send_packet(const void *data, size_t size) {
            return false;
        }

        void GpsInterface::set_log_function(const GpsInterface::LogFunction &function) {
            log = function;
        }

        /**
         * parses the buffer and returns how many more bytes to read
         */
        size_t GpsInterface::parse_rx_buffer() {
            while (rx_buffer_.size() >= 6) {
                log("rx buffer size: " + std::to_string(rx_buffer_.size()), VERBOSE);

                // skip to the 0xb5
                if (rx_buffer_[0] != 0x05 && rx_buffer_[1] != 0x62) {
                    rx_buffer_.pop_front();
                    log("skipping rx byte", VERBOSE);

                    // check again
                    continue;
                }

                // get the length first to check, if we already got enough bytes
                uint16_t payload_length = rx_buffer_[5] << 8 | rx_buffer_[4];
                log("expecting " + std::to_string(payload_length) + " bytes of data", VERBOSE);
                uint16_t total_length = payload_length + 8;

                if (total_length > rx_buffer_.size()) {
                    log("not enough data in the buffer yet", VERBOSE);

                    // fetch more data. bonus: we know exactly how many bytes
                    return total_length - rx_buffer_.size();
                }

                // we have all the data, copy it to an array and remove it from the deque
                std::vector<uint8_t> packet;
                packet.reserve(total_length);

                packet.insert(packet.begin(), rx_buffer_.begin(), rx_buffer_.begin() + total_length);
                rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + total_length);


                if (!validate_checksum(packet.data(), packet.size())) {
                    log("got ubx packet with invalid checksum", WARN);
                    continue;
                }

                log("got valid packet!", VERBOSE);
            }

            // we need at least 6 bytes to process anything
            return 6;
        }

        bool GpsInterface::validate_checksum(const uint8_t *packet, size_t size) {
            uint8_t ck_a = 0;
            uint8_t ck_b = 0;

            for (size_t i = 2; i < size - 2; i++) {
                ck_a += packet[i];
                ck_b += ck_a;
            }

            log("expected: a = " + std::to_string(packet[size - 2]) + ", b = " + std::to_string(packet[size - 1]), VERBOSE);
            log("real: a = " + std::to_string(ck_a) + ", b = " + std::to_string(ck_b), VERBOSE);

            return packet[size - 2] == ck_a && packet[size - 1] == ck_b;
        }


    }
}

