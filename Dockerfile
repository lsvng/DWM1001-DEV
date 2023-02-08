FROM arm32v7/ros:melodic-ros-core-bionic
COPY qemu-arm-static /usr/bin

# Install navigation stack.
RUN apt update && apt install -y \
    g++ \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

COPY . ./root/catkin_ws/src/dwm1001
WORKDIR /root/catkin_ws

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/${ROS_DISTRO}"]

COPY entrypoint.sh /.
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
