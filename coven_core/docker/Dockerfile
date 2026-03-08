# COVEN-CORE ROS2 Jazzy Development Container
# Dock-side only: ROS2 Jazzy, SLAM Toolbox, TF2
# No Gazebo - rovers run Rust firmware, not simulation

FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2 packages needed for dock operation
RUN apt-get update && apt-get install -y \
    # SLAM (dock processes sensor batches offline)
    ros-jazzy-slam-toolbox \
    # Map server (for saving SLAM output)
    ros-jazzy-nav2-map-server \
    # TF and transforms
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf-transformations \
    # Visualization (for development/debugging)
    ros-jazzy-rviz2 \
    ros-jazzy-rqt* \
    # Development tools
    ros-jazzy-ament-cmake \
    ros-jazzy-ament-cmake-python \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    vim \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --break-system-packages \
    numpy \
    pyserial

# Create workspace
RUN mkdir -p /ros2_ws/src

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /root/.bashrc

WORKDIR /ros2_ws

CMD ["/bin/bash"]
