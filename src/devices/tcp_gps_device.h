#ifndef OPEN_MOWER_ROS_TCP_GPS_DEVICE_H
#define OPEN_MOWER_ROS_TCP_GPS_DEVICE_H

#include "gps_device.h"

namespace xbot {
    namespace driver {
        namespace gps {
            class TcpGpsDevice : public GpsDevice {
                public:
                    void set_host(std::string host);
                    void set_port(std::string port);

                    bool check_parameters() override;
                    bool is_open() override;
                    bool open() override;
                    void close() override;
                    size_t read(std::vector<uint8_t> &buffer, size_t size) override;
                    size_t write(const uint8_t *data, size_t size) override;

                private:
                    std::string host_;
                    std::string port_;
                    int sockfd_ = -1;
            };
        }
    }
}

#endif //OPEN_MOWER_ROS_TCP_GPS_DEVICE_H
