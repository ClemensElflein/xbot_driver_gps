# u-blox F9P driver for ROS2

Driver focused on supporting single u-blox chip with purpose of using the internal sensor fusion.

The work is based on the [xbot_driver_gps](https://github.com/ClemensElflein/xbot_driver_gps/) originally used in the [OpenMower](https://openmower.de/) project.

## Features:

- **Low latency:** I want this to support the F9R with its internal sensor fusion. No point in having the data after
  some seconds.
    - :heavy_check_mark:The driver reads the header first and then the exact amount of bytes needed to process the next
      packet. These bytes are read and the packet is immediately processed and sent to ROS. This way, the latency is
      kept to a minimum.
- **:heavy_check_mark: RTCM support:** Sends RTCM from ROS to the u-blox
- **:wrench: Use the latest configuration protocol:** since we're only supporting the newer generations of u-blox
  chips (9+), we can use the new configuration protocol instead of the deprecated one.
- **:heavy_check_mark: Simple code base:** With less code, there are hopefully fewer errors
- **:heavy_check_mark: Robust:** The driver recovers quickly from lost bytes or invalid data

## TODO

- **:wrench: node diagnostic:** Update node diagnostic with more information about node state
- **:wrench: publish raw data:** Publish raw data from the GPS
- **:wrench: port IMU support:** Receives ESF-MEAS messages and builds sensor_msgs/Imu messages
- **:wrench: port odometry feedback:** The driver is able to send odometry feedback to the F9R for the internal sensor fusion to
  work.

## Parameters

- **/port (string):** The serial port to use, defaults to /dev/ttyACM0
- **/baudrate (int):** The baudrate to use, defaults to 115200

## Subscribed Topics:

- **/rtcm (rtcm_msgs/Message)** RTCM which will be sent to the GPS
- TODO: **/odometry (nav_msgs/Odometry) ** odometry messages which will be sent to the GPS for sensor fusion (e.g. F9R)

## Published Topics:

- **/gps/fix (sensor_msgs/NavSatFix):** The GPS fix
