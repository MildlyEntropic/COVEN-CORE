"""
dock_map.py — Map persistence and SLAM coverage tracking.

Saves map data (PGM + YAML) received from rovers, and calculates
global coverage from the SLAM occupancy grid. Instantiated by the
Dock node and accesses ROS2 via self._node.

Author: Alexander Shultis
Date: December 2025
"""

from __future__ import annotations

import base64
import gzip
import json
import os
from datetime import datetime
from typing import TYPE_CHECKING

import numpy as np
from nav_msgs.msg import OccupancyGrid

import coven_core.common as common
from coven_core.common import COLOR_GREEN, COLOR_RESET

if TYPE_CHECKING:
    from coven_core.dock_node import Dock


class MapManager:
    """Handles map persistence and SLAM-based coverage calculation."""

    def __init__(self, node: 'Dock', storage_dir: str):
        self._node = node
        self.storage_dir = os.path.expanduser(storage_dir)
        os.makedirs(self.storage_dir, exist_ok=True)
        self.current_map: OccupancyGrid | None = None

    def map_callback(self, msg: OccupancyGrid):
        """Update current map from SLAM and recalculate global coverage."""
        self.current_map = msg
        self._update_global_coverage()

    def _update_global_coverage(self):
        """Calculate global coverage from SLAM occupancy grid.

        Coverage = known cells / total cells, where known means free (0) or
        occupied (100), not unknown (-1).
        """
        if self.current_map is None:
            return

        data = np.array(self.current_map.data)
        known = np.sum(data >= 0)
        total = len(data)

        coverage = self._node.coverage
        with coverage._lock:
            old_coverage = coverage.global_coverage
            coverage.global_coverage = known / total if total > 0 else 0.0

            if abs(coverage.global_coverage - old_coverage) > 0.05:
                self._node.get_logger().info(
                    f"SLAM map coverage: {coverage.global_coverage:.1%} "
                    f"({known}/{total} cells known)"
                )

    def save_map_data(self, tc: common.TaskComplete):
        """Deserialize and save map data received from module.

        Args:
            tc: TaskComplete message containing map data
        """
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            mission_dir = os.path.join(
                self.storage_dir,
                tc.module_id,
                f"{tc.task}_{timestamp}"
            )
            os.makedirs(mission_dir, exist_ok=True)

            if tc.map_data:
                pgm_compressed = base64.b64decode(tc.map_data)
                pgm_data = gzip.decompress(pgm_compressed)

                pgm_file = os.path.join(mission_dir, "exploration_map.pgm")
                with open(pgm_file, 'wb') as f:
                    f.write(pgm_data)

                self._node.get_logger().info(f"Saved map PGM: {pgm_file} ({len(pgm_data)} bytes)")

            if tc.map_yaml:
                yaml_compressed = base64.b64decode(tc.map_yaml)
                yaml_data = gzip.decompress(yaml_compressed)

                yaml_file = os.path.join(mission_dir, "exploration_map.yaml")
                with open(yaml_file, 'wb') as f:
                    f.write(yaml_data)

                self._node.get_logger().info(f"Saved map YAML: {yaml_file} ({len(yaml_data)} bytes)")

            if tc.exploration_metrics:
                metrics_file = os.path.join(mission_dir, "metrics.json")
                with open(metrics_file, 'w') as f:
                    json.dump({
                        "module_id": tc.module_id,
                        "task": tc.task,
                        "timestamp": timestamp,
                        "success": tc.success,
                        "note": tc.note,
                        "metrics": tc.exploration_metrics
                    }, f, indent=2)

                self._node.get_logger().info(f"Saved metrics: {metrics_file}")

            self._node.get_logger().info(
                f"{COLOR_GREEN}Map data from {tc.module_id} saved to {mission_dir}{COLOR_RESET}"
            )

        except Exception as e:
            self._node.get_logger().error(f"Failed to save map data: {e}")
