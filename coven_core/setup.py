"""
setup.py — COVEN Core

Setuptools build config for coven_core ROS2 Python package.
Dock-side coordination: UART bridge, offline SLAM, frontier dispatch,
and task auctioning. Rovers run Rust firmware (rover/).

Author: Alexander Shultis
Date: September 2025
"""

from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'coven_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        # Simulation assets: Gazebo SDF worlds, models, and the recovered
        # December launch files. Installed under share/<pkg>/sim/* so the
        # sim launch files can resolve them via get_package_share_directory.
        (os.path.join('share', package_name, 'sim', 'worlds'),
         glob('sim/worlds/*.sdf')),
        (os.path.join('share', package_name, 'sim', 'models'),
         glob('sim/models/*.sdf')),
        (os.path.join('share', package_name, 'sim', 'models', 'coven_rover'),
         glob('sim/models/coven_rover/*.sdf')),
    ],
    install_requires=['setuptools', 'numpy', 'pyserial'],
    zip_safe=True,
    maintainer='Alexander Shultis',
    maintainer_email='shultisa@hawaii.edu',
    description='COVEN: dock-side coordination for autonomous rover swarms (UART, no wireless)',
    license='MIT',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            # === Production dock-side nodes (run continuously) ===
            # UART bridge: Rust rover <-> ROS2 ecosystem
            'rover_bridge = coven_core.rover_bridge:main',
            # Offline SLAM: processes recorded sensor batches
            'offline_slam_processor = coven_core.offline_slam_processor:main',
            # Frontier dispatcher: sends rovers to explore
            'frontier_dispatcher = coven_core.frontier_dispatcher:main',

            # === Simulation infrastructure ===
            # Simulation rover proxy: bridges a Gazebo-simulated rover into
            # the COVEN protocol over a virtual UART (PTY pair) so the dock
            # can talk to it as if it were a Rust-firmware Pi Zero rover.
            'sim_rover_proxy = coven_core.sim_rover_proxy:main',

            # === Operator CLIs (one-shot tools) ===
            # Manually issue a mission to the dock auctioneer; useful for
            # demos, smoke tests, and triggering tasks during sim runs.
            'coven-dispatch-task = coven_core.dispatch_task_cli:main',
            # Open a serial port (or PTY symlink), send SYSTEM_PING, await
            # response. Verifies the COVEN link is alive without launching
            # the full ROS2 stack.
            'coven-ping = coven_core.ping_cli:main',
        ],
    },
)
