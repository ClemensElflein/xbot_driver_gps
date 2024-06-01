#include "serial_gps_device.h"

namespace xbot {
    namespace driver {
        namespace gps {
            void SerialGpsDevice::set_serial_port(std::string port) {
                port_ = port;
            }

            void SerialGpsDevice::set_baudrate(uint32_t baudrate) {
                baudrate_ = baudrate;
            }

            bool SerialGpsDevice::check_parameters() {
                if (baudrate_ == 0) {
                    log("no baudrate set, can't start", ERROR);
                    return false;
                }
                if (port_.empty()) {
                    log("no serial port set, can't start", ERROR);
                    return false;
                }
                return true;
            }

            bool SerialGpsDevice::is_open() {
                return serial_.isOpen();
            }

            bool SerialGpsDevice::open() {
                log("opening serial port: " + port_ + " with baudrate: " + std::to_string(baudrate_), INFO);
                try {
                    serial_.setPort(port_);
                    serial_.setBaudrate(baudrate_);
                    auto to = serial::Timeout::simpleTimeout(100);
                    serial_.setTimeout(to);
                    serial_.open();
                    return true;
                } catch (std::exception &e) {
                    log("error opening serial port.", ERROR);
                    return false;
                }
            }

            void SerialGpsDevice::close() {
                serial_.close();
            }

            size_t SerialGpsDevice::read(std::vector<uint8_t> &buffer, size_t size) {
                return serial_.read(buffer, size);
            }

            size_t SerialGpsDevice::write(const uint8_t *data, size_t size) {
                return serial_.write(data, size);
            }
        }
    }
}
