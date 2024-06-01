#ifndef OPEN_MOWER_ROS_GPS_LOG_H
#define OPEN_MOWER_ROS_GPS_LOG_H

#include <functional>
#include <string>

namespace xbot {
    namespace driver {
        namespace gps {

          enum LogLevel {
              VERBOSE,
              INFO,
              WARN,
              ERROR
          };

          typedef std::function<void(const std::string &, LogLevel level)> LogFunction;

        }
    }
}

#endif // OPEN_MOWER_ROS_GPS_LOG_H
