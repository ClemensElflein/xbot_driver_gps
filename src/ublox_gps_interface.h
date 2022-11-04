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

namespace xbot {
    namespace driver {
        namespace gps {
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
                        GNSS_DR_COMBINED = 4
                    };

                    enum RTKType {
                        RTK_NONE = 0,
                        RTK_FLOAT = 1,
                        RTK_FIX = 2
                    };

                    uint32_t sensor_time;
                    uint32_t received_time;

                    // Position
                    bool position_valid;
                    // Position accuracy in m
                    double position_accuracy;
                    double pos_e, pos_n, pos_u;

                    // Pos in lat/lon for VRS
                    double pos_lat, pos_lon;

                    // Motion
                    bool motion_heading_valid;
                    double vel_e, vel_n, vel_u;
                    double motion_heading_accuracy;
                    double motion_heading;

                    // Heading
                    bool vehicle_heading_valid;
                    double vehicle_heading_accuracy;
                    // Vehicle heading in rad.
                    double vehicle_heading;


                    FixType fix_type;
                    RTKType rtk_type;
                };

                struct ImuState {
                    uint32_t sensor_time;
                    uint32_t received_time;

                    // acceleration in m/s
                    double ax,ay,az;
                    // rotation in rad/s
                    double gx,gy,gz;
                };

            public:
                GpsInterface();

                enum Level {
                    VERBOSE,
                    INFO,
                    WARN,
                    ERROR
                };

                enum Mode {
                    ABSOLUTE = 1,
                    RELATIVE = 2
                };

                typedef std::function<void(const std::string &, Level level)> LogFunction;
                typedef std::function<void(const GpsState &new_state)> StateCallback;
                typedef std::function<void(const ImuState &new_state)> ImuCallback;
                typedef std::function<void(uint32_t wheel_tick_stamp, uint32_t wheel_tick_stamp_ublox,
                                           uint32_t wheel_tick_round_trip_stamp)> LatencyCallback;

                void set_imu_callback(const GpsInterface::ImuCallback &function);
                void set_state_callback(const GpsInterface::StateCallback &function);

                void set_wheel_latency_callback(const GpsInterface::LatencyCallback &function);

                void set_serial_port(std::string port);

                void set_baudrate(uint32_t baudrate);

                void set_log_function(const GpsInterface::LogFunction &function);

                bool start();

                void stop();

                void set_mode(Mode mode);

                void set_datum(double datum_lat, double datum_long, double datum_height);

                void
                send_wheel_ticks(uint32_t timestamp, bool direction_left, uint32_t ticks_left, bool direction_right,
                                 uint32_t ticks_right);

                void send_rtcm(const uint8_t *data, size_t size);

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
                 * Send a packet to the GPS. This will add a header and a checksum, but the space is assumed to already be allocated
                 */
                bool send_packet(uint8_t *data, size_t size);


                /**
                 * Parses the rx buffer and looks for valid ubx messages
                 */
                size_t parse_rx_buffer();

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


                // Track the time of the last navigation solution
                std::chrono::time_point<std::chrono::steady_clock> last_gps_message;
                bool gps_state_valid_;
                GpsState gps_state_;
                ImuState imu_state_;
                // track if we have filled all values
                uint8_t imu_fields_valid_;
                uint32_t gps_state_iTOW_;

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

                // Operation mode
                Mode mode_;

                double datum_e_, datum_n_, datum_u_;
                std::string datum_zone_;


                // flag if we found the header for time tracking only
                bool found_header_;
                std::chrono::time_point<std::chrono::steady_clock> current_gps_header_time_;

                StateCallback state_callback = nullptr;
                LatencyCallback wheel_latency_callback = nullptr;
                ImuCallback imu_callback = nullptr;
            };
        }
    }
}

#endif //XBOT_WORKSPACE_UBLOX_GPS_INTERFACE_H
