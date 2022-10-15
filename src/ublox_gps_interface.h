//
// Created by Clemens Elflein on 14.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#ifndef XBOT_WORKSPACE_UBLOX_GPS_INTERFACE_H
#define XBOT_WORKSPACE_UBLOX_GPS_INTERFACE_H

#include <pthread.h>
#include <mutex>
#include <serial/serial.h>
#include <functional>
#include <condition_variable>
#include <unistd.h>
#include <deque>
#include "ubx_datatypes.h"

namespace xbot {
    namespace driver_gps {
        class GpsInterface {

        public:
            /*
             * The final GPS state we're interested in. Positions
             */
            struct GpsState {
                enum FixType {
                    NO_FIX = 0,
                    DR_ONLY = 1,
                    FIX_2D = 2,
                    FIX_3D = 3,
                    GNSS_DR_COMBINED = 4,
                    TIME_ONLY = 5
                };

                enum RTKType {
                    RTK_NONE = 0,
                    RTK_FLOAT = 1,
                    RTK_FIX = 2
                };

                // Position and velocity
                double posE, posN, posU, velE, velN, velU;

                // Position accuracy in m
                double positionAccuracy;

                // Heading in rad
                double heading;

                bool position_valid;
                bool heading_valid;

                FixType fix_type;
                RTKType rtk_type;
            };

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

            bool get_gps_result(GpsState * result);


        private:

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

            /**
             * Send a message to the GPS. This will just output to the serial port directly
             */
            bool send_raw(const void *data, size_t size);

            /**
             * Send a packet to the GPS. This will add a header and a checksum
             */
            bool send_packet(const void *data, size_t size);


            /**
             * Parses the rx buffer and looks for valid ubx messages
             */
            size_t parse_rx_buffer();

            /**
             * Gets called with a valid ubx frame and switches to the handle_xxx functions
             */
            void process_ubx_packet(const uint8_t *data, const size_t &size);

            /**
             * Validate a frame, return true on valid checksum
             */
            bool validate_checksum(const uint8_t *packet, size_t size);

            void handle_nav_pvt(const UbxNavPvt *msg);

            // Current state
            // Mutex for gps_state_
            std::mutex gps_state_mutex_;
            // Condition variable for gps_state_
            std::condition_variable gps_state_cv_;
            GpsState gps_state_;

            // Set to true to stop the threads
            bool stopped_;

            pthread_t rx_thread_handle_;
            pthread_t tx_thread_handle_;

            // Mutex for tx_buffer_
            std::mutex tx_mutex_;
            // Condition variable for tx_buffer_
            std::condition_variable tx_cv_;
            std::vector<uint8_t> tx_buffer_;

            serial::Serial serial_;
            std::string port_;
            uint32_t baudrate_;
            // buffer to store ubx messages before parsing
            std::deque<uint8_t> rx_buffer_;

            LogFunction log;
        };
    }
}

#endif //XBOT_WORKSPACE_UBLOX_GPS_INTERFACE_H
