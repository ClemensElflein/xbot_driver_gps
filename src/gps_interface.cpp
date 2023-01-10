//
// Created by clemens on 10.01.23.
//

#include "gps_interface.h"

using namespace std::chrono;

namespace xbot {
    namespace driver {
        namespace gps {
            void GpsInterface::set_serial_port(std::string port) {
                port_ = port;
            }

            void GpsInterface::set_baudrate(uint32_t baudrate) {
                baudrate_ = baudrate;
            }



            void GpsInterface::set_log_function(const GpsInterface::LogFunction &function) {
                log = function;
            }

            void GpsInterface::set_state_callback(const GpsInterface::StateCallback &function) {
                state_callback = function;
            }


            void GpsInterface::set_imu_callback(const GpsInterface::ImuCallback &function) {
                imu_callback = function;
            }

            GpsInterface::GpsInterface() {
                datum_n_ = datum_e_ = datum_u_ = NAN;
                state_callback = nullptr;
                imu_callback = nullptr;
                read_from_file_ = false;
            }

            void GpsInterface::set_datum(double datum_lat, double datum_long, double datum_height) {
                datum_u_ = datum_height;
                RobotLocalization::NavsatConversions::LLtoUTM(datum_lat, datum_long, datum_n_, datum_e_, datum_zone_);
            }

            void GpsInterface::set_mode(GpsInterface::Mode mode) {
                mode_ = mode;
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

            void *GpsInterface::rx_thread_file() {
                gps_state_valid_ = false;
                reset_parser_state();

                std::ifstream file(filename, std::ios::in | std::ios::binary);
                if (!file.is_open()) {
                    if(log) {
                        log("error opening file.", Level::ERROR);
                    }
                    return nullptr;
                }

                std::istreambuf_iterator<char> iter(file);

                while(true) {
                    if(!file.eof()) {
                        char i = *iter;
                        rx_buffer_.push_back(i);
                        parse_rx_buffer();
                        iter++;
                        usleep(100);
                    } else {
                        break;
                    }
                }

                log("end of file", Level::INFO);

                return nullptr;
            }

            void *GpsInterface::tx_thread_file() {
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

                    tx_buffer_.clear();
                }

                return nullptr;
            }

            void *GpsInterface::rx_thread() {
                std::vector<uint8_t> buf;

                gps_state_valid_ = false;
                reset_parser_state();

                // stores the amount of bytes to read. At first we have this small until the parse_rx_buffer is able to know
                // how many more bytes it needs. then we read that amount of data, before parsing again.
                // This reduces latency, because we don't fill a buffer before reading and we don't parse on every byte as soon as we know how many
                // we actually need
                size_t bytes_to_read = parse_rx_buffer();

                while (!stopped_) {
                    // Check, if the serial port is connected. If not, connect to it.
                    if (!serial_.isOpen()) {
                        bytes_to_read = parse_rx_buffer();
                        reset_parser_state();
                        gps_state_valid_ = false;
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


            void GpsInterface::stop() {
                stopped_ = true;
                log("waiting for serial rx thread to stop", INFO);
                pthread_join(rx_thread_handle_, nullptr);
                log("waiting for serial tx thread to stop", INFO);
                pthread_join(tx_thread_handle_, nullptr);
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

                if (baudrate_ == 0 && !read_from_file_) {
                    log("no baudrate set, can't start", ERROR);
                    return false;
                }
                if (port_.empty() && !read_from_file_) {
                    log("no serial port set, can't start", ERROR);
                    return false;
                }

                stopped_ = false;
                // clear tx buffer
                {
                    std::unique_lock<std::mutex> lk(tx_mutex_);
                    tx_buffer_.clear();
                }

                if(!read_from_file_) {
                    pthread_create(&rx_thread_handle_, NULL, &GpsInterface::rx_thread_helper, this);
                    pthread_create(&tx_thread_handle_, NULL, &GpsInterface::tx_thread_helper, this);
                } else {
                    if(log) {
                        log("reading from file: " + filename, Level::WARN);
                    }
                    pthread_create(&rx_thread_handle_, NULL, &GpsInterface::rx_thread_helper_file, this);
                    pthread_create(&tx_thread_handle_, NULL, &GpsInterface::tx_thread_helper_file, this);
                }

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

            void GpsInterface::set_file_name(std::string filename) {
                read_from_file_ = true;
                this->filename = filename;
            }


            void GpsInterface::send_rtcm(const uint8_t *data, size_t size) {
                send_raw(data,size);
            }
        }
    }
}