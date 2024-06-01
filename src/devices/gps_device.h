#ifndef OPEN_MOWER_ROS_GPS_DEVICE_H
#define OPEN_MOWER_ROS_GPS_DEVICE_H

#include "../gps_log.h"
#include <vector>

namespace xbot {
    namespace driver {
        namespace gps {
            class GpsDevice {
                public:
                    GpsDevice();
                    virtual ~GpsDevice() = default;

                    void set_log_function(const LogFunction &function);

                    virtual bool check_parameters() = 0;
                    virtual bool is_open() = 0;
                    virtual bool open() = 0;
                    virtual void close() = 0;
                    virtual size_t read(std::vector<uint8_t> &buffer, size_t size) = 0;
                    virtual size_t write(const uint8_t *data, size_t size) = 0;

                protected:
                    LogFunction log = nullptr;
            };
        }
    }
}

#endif //OPEN_MOWER_ROS_GPS_DEVICE_H
