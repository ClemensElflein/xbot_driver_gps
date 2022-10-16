# xBot u-blox Driver

This repository contains the u-blox driver for the xBot platform.

This is a no-nonsense, high performance driver for the latest u-blox devices. It will eventually also get NMEA support,
but for now ubx only.

This implementation differs from other popular u-blox ROS drivers in the following points:

- **Low latency:** I want this to support the F9R with its internal sensor fusion. No point in having the data after
  some seconds.
    - :heavy_check_mark:The driver reads the header first and then the exact amount of bytes needed to process the next
      packet. These bytes are read and the packet is immediately processed and sent to ROS. This way, the latency is
      kept to a minimum.
    - The driver will notice congestion on the port in both directions:
        - :heavy_check_mark: For RX it looks at the message timestamps and compares them with the ROS timestamps. If
          there's a large jump, it prints an error.
        - :heavy_check_mark: For TX it looks at the serial write buffer and checks how long it takes for it to clear. If
          the buffer fills or we need too much time to get the write lock, it prints an error.
        - :heavy_check_mark: For wheel ticks it is able to read back the ESF-RAW messages to calculate the wheel-tick timings on
          the u-blox chip. This way we can be 100% sure that the timing was as expected.
- **:heavy_check_mark: IMU support:** Receives ESF-MEAS messages and builds sensor_msgs/Imu messages
- **:heavy_check_mark: RTCM support:** Sends RTCM from ROS to the u-blox
- **:wrench: Scheduling Priority:** If in sensor fusion mode, the driver tries to get scheduled faster than other
  processes. If in "normal" GPS mode, it's just a normal process.
- **:heavy_check_mark: Wheel Tick Support:** The driver is able to send wheel ticks to the F9R for the internal sensor fusion to
  work.
- **:wrench: Use the latest configuration protocol:** since we're only supporting the newer generations of u-blox
  chips (9+), we can use the new configuration protocol instead of the deprecated one.
- **:heavy_check_mark: Simple code base:** With less code, there are hopefully fewer errors
- **:heavy_check_mark: Robust:** The driver recovers quickly from lost bytes or invalid data

## Operation Modes:

The driver can be configured in multiple operation modes:

- **Relative Positioning:** In this mode, the driver is using the NAV-RELPOSNED message and outputs that as position.
  There is no need to set an origin, since the position is relative to a local base. **F9R Sensor Fusion is NOT possible
  in this mode!**
- **Absolute Positioning:** In this mode, you need to specify a reference point near the robot. The position output will
  be relative to that reference point. The driver uses the NAVPVT message in this case.

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
- **/pose_info (xbot_msgs/AbsolutePoseInfo):** The current pose with some additional information about the quality and
  source of the pose.


## Improvements
- Replace the deque - vector copy. Since we won't buffer anything anyways, we can just write into a vactor directly and reset it after each packet.