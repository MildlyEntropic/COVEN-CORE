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
            # UART bridge: Rust rover <-> ROS2 ecosystem
            'rover_bridge = coven_core.rover_bridge:main',
            # Offline SLAM: processes recorded sensor batches
            'offline_slam_processor = coven_core.offline_slam_processor:main',
            # Frontier dispatcher: sends rovers to explore
            'frontier_dispatcher = coven_core.frontier_dispatcher:main',
        ],
    },
)
