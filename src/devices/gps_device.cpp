#include "gps_device.h"

namespace xbot {
    namespace driver {
        namespace gps {
            GpsDevice::GpsDevice() {
            }

            void GpsDevice::set_log_function(const LogFunction &function) {
                log = function;
            }
        }
    }
}
