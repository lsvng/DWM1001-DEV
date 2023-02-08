**Introduction**
====
DWM1001C is a radio technology that utilizes the DW1000 Ultra-Wideband (UWB) transceiver IC, which is compliant with the IEEE 802.15.4a standard for UWB implementation. This module integrates a UWB (Channel 5) and Bluetooth® antenna, a Nordic Semiconductor nRF52832 microcontroller, and a motion sensor.

It is known for its ability to transmit high-bandwidth data over short distances, using minimal amounts of energy. It operates across a broad range of the radio spectrum, making it highly versatile and ideal for various applications. This technology is a cost-effective and efficient solution for communication needs within close proximity.

The DWM1001-DEV is a development board specifically designed for the DWM1001C module. It features an integrated Jlink, providing a convenient solution for development and debugging.

Applications that require precise positioning include, but are not limited to:

- Hospitals for emergency response and patient care.
- Peer-to-peer fine ranging for precise distance measurement.
- Real-time location systems for navigation and asset tracking.

**About**
=========
This project contains C++ examples that demonstrate how to interface with the DWM1001-DEV board through a UART protocol with a Linux-based OS using the [Standard C POSIX library](https://en.wikipedia.org/wiki/C_POSIX_library). The [Eigen3 Library](https://eigen.tuxfamily.org/index.php?title=Main_Page) is used to ensure smooth geolocation data acquisition by implementing a Kalman Filter for noise reduction. The examples within this project leverage [ROS](https://www.ros.org/) for potential use in state estimation and navigation applications.

**Docker Usage**
====
1. Pulling the Docker container.

    ```
    docker pull duckstarr/dwm1001:arm32v7
    ```

2. Deploying (executing) the Docker container.

    ```
    docker run -it --rm --privileged --name dwm1001 \
        duckstarr/dwm1001:arm32v7 \
        roslaunch dwm1001_ros main.launch
    ```

**DWM1001-DEV API**
====
**Published Topics**

~< name >/Point ([geometry_msgs/PointStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html))

* The position of the DWM1001 Module in cartesian coordinate.

**Parameters**

~< name >/device (string, default: /dev/ttyACM0)
* The aboslute path to DWM1001 device.

~< name >/nominal_update_rate (int, default: 100)
* The nominal update rate of the DMW1001 module.
