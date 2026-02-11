# COVEN-CORE ROS2 Jazzy Development Container
# Includes: ROS2 Jazzy, Gazebo Harmonic, Nav2, SLAM Toolbox

FROM osrf/ros:jazzy-desktop-full

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install additional ROS2 packages and tools
RUN apt-get update && apt-get install -y \
    # Navigation and SLAM
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    # Gazebo integration
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-interfaces \
    # TF and transforms
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf-transformations \
    # Robot state and description
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    # Visualization
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
    # Clean up
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --break-system-packages \
    transforms3d \
    numpy

# Create workspace
RUN mkdir -p /ros2_ws/src

# Set up environment
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Source ROS2 automatically
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /root/.bashrc

# Set working directory
WORKDIR /ros2_ws

# Default command
CMD ["/bin/bash"]
