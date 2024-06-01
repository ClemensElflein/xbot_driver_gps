//
// Created by clemens on 10.01.23.
//

#ifndef OPEN_MOWER_ROS_NMEA_GPS_INTERFACE_H
#define OPEN_MOWER_ROS_NMEA_GPS_INTERFACE_H
#include "gps_interface.h"
#include <nmeaparse/nmea.h>

#include <nmeaparse/NMEAParser.h>
#include <nmeaparse/NumberConversion.h>

using namespace nmea;

namespace xbot {
    namespace driver {
        namespace gps {
            class NmeaGpsInterface : public GpsInterface {
            public:
                NmeaGpsInterface();
            protected:
                void reset_parser_state() override;

                size_t parse_rx_buffer() override;

            private:
                NMEAParser parser;
                GPSService gps;
            };
        }
    }
}


#endif //OPEN_MOWER_ROS_NMEA_GPS_INTERFACE_H
