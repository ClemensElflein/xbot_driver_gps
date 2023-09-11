#pragma once

#include <functional>
#include <string>
#include <memory>
#include <chrono>
#include <sstream>

#include "ubx_datatypes.hpp"

class UBlox {
public:
    enum LogLevel {
        DEBUG,
        INFO,
        WARN,
        ERROR,
        FATAL
    };

    struct GPSState {
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
        double pos_lat, pos_lon, pos_altitude;

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

//    struct IMUState {
//        uint32_t sensor_time;
//        uint32_t received_time;
//
//        // acceleration in m/s
//        double ax,ay,az;
//        // rotation in rad/s
//        double gx,gy,gz;
//    };

    typedef std::function<void(const GPSState&)> GPSStateHandlerFunction;
//    typedef std::function<void(const IMUState&)> IMUStateHandlerFunction;
    typedef std::function<void(const std::string&, LogLevel level)> LogFunction;

    UBlox();

    ~UBlox();

    void setGPSStateCallback(const GPSStateHandlerFunction& handler);
    void setLogCallback(const LogFunction& handler);

    void connect(const std::string& port, const uint32_t baudRate);

    void disconnect();

    bool isConnected() const;

    void sendRTCM(const std::vector<uint8_t>& data);

private:
    typedef std::function<void(const std::chrono::time_point<std::chrono::steady_clock>&, const UbxNavPvtConstPtr&)> NavPacketHandlerFunction;

    void navPacketHandler(const std::chrono::time_point<std::chrono::steady_clock>& time, const UbxNavPvtConstPtr& packet);

    // Pimpl - hide serial port members from class users
    class Serial;
    std::unique_ptr<Serial> serial_;

    GPSStateHandlerFunction gps_state_handler_;
    LogFunction log_function_;
};

class SerialException : public std::exception
{
    // Disable copy constructors
    SerialException& operator=(const SerialException&);
    std::string e_what_;

public:
    explicit SerialException(const char* description)
    {
        std::stringstream ss;
        ss << "SerialException " << description << " failed.";
        e_what_ = ss.str();
    }
    SerialException(const SerialException& other) : e_what_(other.e_what_)
    {
    }
    virtual ~SerialException() throw()
    {
    }
    virtual const char* what() const throw()
    {
        return e_what_.c_str();
    }
};
