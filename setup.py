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
    ],
    install_requires=['setuptools'],
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
            'dock = coven_core.dock_node:main',
            'dock_multi = coven_core.dock_node_multi:main',
            'module = coven_core.module_node:main',
            'module_flaky = coven_core.module_flaky:main',
        ],
    },
)