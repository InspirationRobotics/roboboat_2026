# Use Humble base (GPU if needed)
ARG ROS_DISTRO=humble
FROM humble-gpu-ubuntu22.04:latest

ARG USE_RVIZ
ARG BUILD_SEQUENTIAL=0

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        software-properties-common git libusb-1.0-0-dev wget zsh python3-colcon-common-extensions build-essential cmake

# Oh-my-zsh
RUN sh -c "$(wget https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" || true

# Setup workspace environment
ENV WS=/ws
RUN mkdir -p $WS/src

# ----------------------------
# DepthAI ROS
# ----------------------------
COPY ./depthai-ros $WS/src/depthai-ros
RUN cd $WS && \
    rosdep install --from-paths src --ignore-src -y && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    ./src/depthai-ros/build.sh -s $BUILD_SEQUENTIAL -r 1 -m 1

# Optional RViz
RUN if [ "$USE_RVIZ" = "1" ] ; then \
        apt install -y ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-rviz-imu-plugin ; \
    fi

# ----------------------------
# Livox SDK + Driver
# ----------------------------
RUN mkdir -p /livox_ws && cd /livox_ws && \
    git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install

RUN mkdir -p /livox_ws/livox_ros2_ws/src && cd /livox_ws/livox_ros2_ws/src && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git

COPY config/MID360_config.json /livox_ws/livox_ros2_ws/src/livox_ros_driver2/config/MID360_config.json

# Build Livox ROS driver
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    cd /livox_ws/livox_ros2_ws/src/livox_ros_driver2 && \
    ./build.sh humble && \
    source /livox_ws/livox_ros2_ws/install/setup.bash

# ----------------------------
# Cleanup / Environment
# ----------------------------
# Remove old bashrc entries
RUN sed -i '/source \/opt\/ros\/iron\/setup.bash/d' /etc/bash.bashrc || true
RUN sed -i '/source \/workspace\/oak_ws\/install\/setup.bash/d' ~/.bashrc || true

# Source both DepthAI and Livox
RUN echo "source $WS/install/setup.bash" >> ~/.bashrc && \
    echo "source /livox_ws/livox_ros2_ws/install/setup.bash" >> ~/.bashrc

# Set working directory
WORKDIR /workspace

# Entrypoint
COPY ./depthai-ros/entrypoint.sh /workspace/entrypoint.sh
ENTRYPOINT ["/workspace/entrypoint.sh"]
CMD ["bash"]
