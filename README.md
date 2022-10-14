# Xbot Ublox Driver
This repository contains the ublox driver for the Xbot platform.

This implementation differs from other popular ublox ROS drivers in the following points:
- **Low latency, high throughput:** I want this to support the F9R with its internal sensor fusion. No point in having the data after some seconds
- **Use the current configuration protocol:** since we're only supporting the newer generations of ublox chips (9+), we can use the new configuration protocol instead of the deprecated one.
- **Simple code base:** With less code, there are hopefully fewer errors


## Operation Modes:
The driver can be configured in multiple operation modes:
- **Relative Positioning:** In this mode, the driver is using the NAV-RELPOSNED message and outputs that as position. There is no need to set an origin, since the position is relative to a local base. **F9R Sensor Fusion is NOT possible in this mode!**
- **Absolute Positioning:** In this mode, you need to specify a reference point near the robot. The position output will be relative to that reference point. The driver uses the NAVPVT message in this case.

## Configuration:
The driver supports the following configurations:
TODO
- use_sensor_fusion: True to subscribe to wheel_ticks and send to F9R.
- position_mode: Relative vs Absolute mode
- datum_lat, datum_lon, datum_height: Datum position for pose generation. Only used in absolute mode.

## Subscribed Topics:
- **/rtcm (rtcm_msgs/Message)** RTCM which will be sent to the GPS
- **/wheel_ticks (xbot_msgs/WheelTick)** WheelTick messages which will be sent to the GPS for sensor fusion (e.g. F9R)

## Published Topics:
- **/pose (geometry_msgs/Pose):** The current pose of the robot for use with legacy systems
- **/pose_info (xbot_msgs/AbsolutePoseInfo):** The current pose with some additional information about the quality and source of the pose.

