# COVEN-CORE ROS2 Jazzy Development Container
# Production rovers run Rust firmware on Pi Zero 2W. This container hosts:
#   - The dock-side Python/ROS2 stack (rover_bridge, frontier_dispatcher,
#     SLAM Toolbox, Nav2)
#   - Gazebo Harmonic + ros_gz bridges (for sim rover orchestration)
#   - The sim_rover_proxy node, which bridges Gazebo-simulated rovers to
#     the dock over a virtual UART (PTY pair). This lets the dock run
#     unmodified production code against simulated rovers.

FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2 packages needed for dock operation + simulation
RUN apt-get update && apt-get install -y \
    # SLAM (dock processes sensor batches offline)
    ros-jazzy-slam-toolbox \
    # Nav2 (used by sim rovers when navigation is wired in)
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-map-server \
    # TF and transforms
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf-transformations \
    # Gazebo Harmonic + ROS2 bridges (for rover simulation)
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-interfaces \
    # PTY / virtual-serial helpers for the sim rover proxy
    socat \
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
