cmake_minimum_required(VERSION 3.5)
project(coven_core)

# Required by colcon/ament to install Python packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# --- Install model files so Gazebo can find them ---
install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# --- Install config files (nav2, slam, coven params) ---
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# --- Install world files ---
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()
