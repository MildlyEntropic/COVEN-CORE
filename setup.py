"""
setup.py — COVEN Phase 1

Setuptools build config for coven_core ROS2 Python package.
Provides shared message definitions, FSM logic, and runtime nodes
for both dock and module behavior.

Responsibilities:
- Install Python modules under coven_core
- Register ROS2 console entry points
- Support hybrid ROS2 + Python toolchain (colcon + pip)

Author: Alexander Shultis
Date: September 2025
"""

# ------------------------
# --- Imports ---
# ------------------------
from setuptools import setup, find_packages
import os
from glob import glob


# ------------------------
# --- Package Metadata ---
# ------------------------
package_name = 'coven_core'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
        (os.path.join('share', package_name, 'models', 'coven_rover'),
         glob('models/coven_rover/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='Alexander Shultis',
    maintainer_email='shultisa@hawaii.edu',
    description='COVEN Phase 1: Modular FSM framework for dock + module simulation',
    license='MIT',
    tests_require=['pytest'],

    # ------------------------
    # --- ROS2 Entry Points ---
    # ------------------------
    entry_points={
        'console_scripts': [
            # Legacy smart-rover architecture
            'dock = coven_core.dock_node:main',
            'module = coven_core.module_node:main',
            'coven_dock = coven_core.dock_node:main',
            'coven_module = coven_core.module_node:main',
            # Dock-centric architecture
            'simplified_module = coven_core.simplified_module_node:main',
            'coven_simple_module = coven_core.simplified_module_node:main',
            'simplified_dock = coven_core.simplified_dock_node:main',
            'coven_simple_dock = coven_core.simplified_dock_node:main',
            # Dock-centric architecture (offline SLAM, sensor batch processing)
            'offline_slam_processor = coven_core.offline_slam_processor:main',
            'frontier_dispatcher = coven_core.frontier_dispatcher:main',
            # Hardware drivers (for physical CubeRover deployment)
            'motor_driver = coven_core.motor_driver:main',
            'encoder_odom = coven_core.encoder_odom:main',
            # Utilities
            'spawn_module = coven_core.spawn_module:main',
            'odom_tf_broadcaster = coven_core.odom_tf_broadcaster:main',
            'scan_frame_republisher = coven_core.scan_frame_republisher:main',
            'topic_relay = coven_core.topic_relay:main',
            # Lightweight rover bridge (TCP to ROS2)
            'rover_bridge = coven_core.rover_bridge:main',
        ],
    },
)