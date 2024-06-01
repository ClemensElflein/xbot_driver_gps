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
#include "robot_localization/navsat_conversions.h"
#include "gps_interface.h"

namespace xbot {
    namespace driver {
        namespace gps {
            class UbxGpsInterface : public GpsInterface {

            public:
                UbxGpsInterface();

                typedef std::function<void(uint32_t wheel_tick_stamp, uint32_t wheel_tick_stamp_ublox,
                                           uint32_t wheel_tick_round_trip_stamp)> LatencyCallback;

                void set_wheel_latency_callback(const UbxGpsInterface::LatencyCallback &function);

                void
                send_wheel_ticks(uint32_t timestamp, bool direction_left, uint32_t ticks_left, bool direction_right,
                                 uint32_t ticks_right);

            private:


            protected:
                void reset_parser_state() override;

            private:

                /**
                 * Send a packet to the GPS. This will add a header and a checksum, but the space is assumed to already be allocated
                 */
                bool send_packet(uint8_t *data, size_t size);


                /**
                 * Parses the rx buffer and looks for valid ubx messages
                 */
                size_t parse_rx_buffer() override;

                /**
                 * Gets called with a valid ubx frame and switches to the handle_xxx functions
                 */
                void process_ubx_packet(const std::chrono::time_point<std::chrono::steady_clock> &header_stamp,
                                        const uint8_t *data, const size_t &size);

                /**
                 * Validate a frame, return true on valid checksum
                 */
                bool validate_checksum(const uint8_t *packet, size_t size);

                /**
                 * calculates the checksum for a packet
                 */
                void calculate_checksum(const uint8_t *packet, size_t size, uint8_t &ck_a, uint8_t &ck_b);

                void handle_nav_pvt(const std::chrono::time_point<std::chrono::steady_clock> &header_stamp,
                                    const UbxNavPvt *msg);

                void handle_esf_meas(const std::chrono::time_point<std::chrono::steady_clock> &header_stamp,
                                     const uint8_t *payload, size_t payload_sie);



                // track if we have filled all values
                uint8_t imu_fields_valid_;
                uint32_t gps_state_iTOW_;


                // flag if we found the header for time tracking only
                bool found_header_;
                std::chrono::time_point<std::chrono::steady_clock> current_gps_header_time_;

                LatencyCallback wheel_latency_callback = nullptr;
            };
        }
    }
}

#endif //XBOT_WORKSPACE_UBLOX_GPS_INTERFACE_H
