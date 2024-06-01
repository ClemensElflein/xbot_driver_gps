#ifndef OPEN_MOWER_ROS_SERIAL_GPS_DEVICE_H
#define OPEN_MOWER_ROS_SERIAL_GPS_DEVICE_H

#include "gps_device.h"
#include <serial/serial.h>

namespace xbot {
    namespace driver {
        namespace gps {
            class SerialGpsDevice : public GpsDevice {
                public:
                    void set_serial_port(std::string port);
                    void set_baudrate(uint32_t baudrate);

                    bool check_parameters() override;
                    bool is_open() override;
                    bool open() override;
                    void close() override;
                    size_t read(std::vector<uint8_t> &buffer, size_t size) override;
                    size_t write(const uint8_t *data, size_t size) override;

                private:
                    serial::Serial serial_;
                    std::string port_;
                    uint32_t baudrate_;
            };
        }
    }
}

#endif //OPEN_MOWER_ROS_SERIAL_GPS_DEVICE_H
