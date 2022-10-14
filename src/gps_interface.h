//
// Created by Clemens Elflein on 14.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#ifndef XBOT_WORKSPACE_GPS_INTERFACE_H
#define XBOT_WORKSPACE_GPS_INTERFACE_H

#include <pthread.h>
#include <mutex>
#include <serial/serial.h>
#include <functional>
#include <condition_variable>
#include <unistd.h>
#include <deque>

namespace xbot {
    namespace driver_gps_ublox {
        class GpsInterface {

        public:
            enum Level {
                VERBOSE,
                INFO,
                WARN,
                ERROR
            };
            typedef std::function<void(const std::string &, Level level)> LogFunction;

            void set_serial_port(std::string port);

            void set_baudrate(uint32_t baudrate);

            void set_log_function(const GpsInterface::LogFunction &function);

            bool start();

            void stop();
            /**
             * Send a message to the GPS. This will just output to the serial port directly
             */
            bool send_raw(const void *data, size_t size);
        private:


            /**
             * Send a packet to the GPS. This will add a header and a checksum
             */
            bool send_packet(const void *data, size_t size);

            // Helpers to call the threads, since these are member functions
            static void *rx_thread_helper(void *context) {
                return ((GpsInterface *) context)->rx_thread();
            }

            static void *tx_thread_helper(void *context) {
                return ((GpsInterface *) context)->tx_thread();
            }


            // The actual thread functions
            void *rx_thread();

            void *tx_thread();

            size_t parse_rx_buffer();

            bool validate_checksum(const uint8_t *packet, size_t size);

            pthread_t rx_thread_handle_;
            pthread_t tx_thread_handle_;

            // Mutex for tx_buffer_
            std::mutex tx_mutex_;
            // Condition variable for tx_buffer_
            std::condition_variable tx_cv_;
            std::vector<uint8_t> tx_buffer_;


            // Set to true to stop the threads
            bool stopped_;


            LogFunction log;
            serial::Serial serial_;
            std::string port_;
            uint32_t baudrate_;


            // buffer to store ubx messages before parsing
            std::deque<uint8_t> rx_buffer_;
        };
    }
}

#endif //XBOT_WORKSPACE_GPS_INTERFACE_H
