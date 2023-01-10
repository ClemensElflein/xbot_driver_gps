//
// Created by clemens on 10.01.23.
//

#ifndef OPEN_MOWER_ROS_GPS_INTERFACE_H
#define OPEN_MOWER_ROS_GPS_INTERFACE_H

#include <serial/serial.h>
#include <functional>
#include <chrono>
#include <pthread.h>
#include <mutex>
#include <unistd.h>
#include "robot_localization/navsat_conversions.h"
#include <condition_variable>
#include "deque"
#include <filesystem>
#include <fstream>
namespace xbot {
    namespace driver {
        namespace gps {
            class GpsInterface {
            public:
                GpsInterface();

                virtual ~GpsInterface() = default;

                /*
                 * The final GPS state we're interested in.
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



            public:
                void set_imu_callback(const GpsInterface::ImuCallback &function);
                void set_state_callback(const GpsInterface::StateCallback &function);
                void set_log_function(const GpsInterface::LogFunction &function);

                void set_serial_port(std::string port);
                void set_baudrate(uint32_t baudrate);

                void set_datum(double datum_lat, double datum_long, double datum_height);
                void set_mode(Mode mode);

                bool start();
                void stop();

                void set_file_name(std::string filename);

                void send_rtcm(const uint8_t *data, size_t size);


            protected:
                StateCallback state_callback = nullptr;
                ImuCallback imu_callback = nullptr;
                LogFunction log = nullptr;

                // Operation mode
                Mode mode_;

                double datum_e_, datum_n_, datum_u_;
                std::string datum_zone_;



                // Track the time of the last navigation solution
                std::chrono::time_point<std::chrono::steady_clock> last_gps_message;
                bool gps_state_valid_;
                GpsState gps_state_;
                ImuState imu_state_;


                // buffer to store ubx messages before parsing
                std::deque<uint8_t> rx_buffer_;

                /**
                 * Send a message to the GPS. This will just output to the serial port directly
                 */
                bool send_raw(const void *data, size_t size);

                // Called on serial reconnect
                virtual void reset_parser_state() = 0;

                virtual size_t parse_rx_buffer() = 0;

            private:

                // Helpers to call the threads, since these are member functions
                static void *rx_thread_helper(void *context) {
                    return ((GpsInterface *) context)->rx_thread();
                }

                static void *tx_thread_helper(void *context) {
                    return ((GpsInterface *) context)->tx_thread();
                }
                // Helpers to call the threads, since these are member functions
                static void *rx_thread_helper_file(void *context) {
                    return ((GpsInterface *) context)->rx_thread_file();
                }

                static void *tx_thread_helper_file(void *context) {
                    return ((GpsInterface *) context)->tx_thread_file();
                }

                // The actual thread functions
                void *rx_thread();
                void *rx_thread_file();

                void *tx_thread();
                void *tx_thread_file();

                // Set to true to stop the threads
                bool stopped_;

                // Mutex for tx_buffer_
                std::mutex tx_mutex_;
                // Condition variable for tx_buffer_
                std::condition_variable tx_cv_;
                std::vector<uint8_t> tx_buffer_;

                pthread_t rx_thread_handle_;
                pthread_t tx_thread_handle_;

                serial::Serial serial_;
                std::string port_;
                uint32_t baudrate_;

                bool read_from_file_;
                std::string filename;
            };
        }
    }
}
#endif //OPEN_MOWER_ROS_GPS_INTERFACE_H
