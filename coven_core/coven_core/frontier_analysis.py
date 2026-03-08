# SPDX-License-Identifier: MIT
"""
frontier_analysis.py — Pure frontier detection and scoring functions.

Extracts frontier regions from an OccupancyGrid and scores them by
size, distance from dock, and directional novelty. All functions are
stateless — the FrontierDispatcher node calls them with the current map.

Author: Alexander Shultis
Date: December 2025
"""

import math
import numpy as np
from typing import List, Tuple
from dataclasses import dataclass


@dataclass
class Frontier:
    """An unexplored frontier region."""
    centroid: Tuple[float, float]  # World coordinates
    size: int                       # Number of frontier cells
    distance: float                 # Distance from dock
    direction: float                # Angle from dock (radians)
    score: float = 0.0              # Exploration priority


def analyze_frontiers(
    grid_data: np.ndarray,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    dock_pos: Tuple[float, float],
    explored_directions: List[float],
    min_frontier_size: int,
) -> List[Frontier]:
    """Find and score frontier regions in an occupancy grid.

    Args:
        grid_data: Flat occupancy grid array (-1=unknown, 0=free, 100=occupied).
        width: Grid width in cells.
        height: Grid height in cells.
        resolution: Meters per cell.
        origin_x: World X of grid origin.
        origin_y: World Y of grid origin.
        dock_pos: Dock world coordinates (x, y).
        explored_directions: Previously explored directions (radians).
        min_frontier_size: Minimum cells for a cluster to count.

    Returns:
        List of Frontier objects sorted by score (best first).
    """
    grid = grid_data.reshape((height, width))

    # Find frontier cells (free cells adjacent to unknown)
    frontier_cells: List[Tuple[float, float]] = []

    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if grid[y, x] == 0:  # Free cell
                neighbors = [
                    grid[y-1, x], grid[y+1, x],
                    grid[y, x-1], grid[y, x+1]
                ]
                if -1 in neighbors:
                    world_x = origin_x + x * resolution
                    world_y = origin_y + y * resolution
                    frontier_cells.append((world_x, world_y))

    if not frontier_cells:
        return []

    # Cluster frontier cells into regions
    clusters = cluster_frontiers(frontier_cells)

    frontiers: List[Frontier] = []
    for cluster in clusters:
        if len(cluster) < min_frontier_size:
            continue

        # Centroid
        cx = sum(p[0] for p in cluster) / len(cluster)
        cy = sum(p[1] for p in cluster) / len(cluster)

        # Distance and direction from dock
        dx = cx - dock_pos[0]
        dy = cy - dock_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        direction = math.atan2(dy, dx)

        # Score: prefer larger frontiers, reasonable distance, new directions
        novelty = direction_novelty(direction, explored_directions)
        score = len(cluster) * novelty / (1.0 + distance * 0.1)

        frontiers.append(Frontier(
            centroid=(cx, cy),
            size=len(cluster),
            distance=distance,
            direction=direction,
            score=score,
        ))

    frontiers.sort(key=lambda f: f.score, reverse=True)
    return frontiers


def _bfs_expand(cells, seed, used, threshold_sq):
    """Expand a cluster from seed via BFS, marking visited cells."""
    cluster = [cells[seed]]
    used[seed] = True
    queue = [seed]

    while queue:
        cx, cy = cells[queue.pop(0)]
        for j in range(len(cells)):
            if used[j]:
                continue
            dx = cx - cells[j][0]
            dy = cy - cells[j][1]
            if dx * dx + dy * dy < threshold_sq:
                cluster.append(cells[j])
                used[j] = True
                queue.append(j)

    return cluster


def cluster_frontiers(
    cells: List[Tuple[float, float]],
    threshold: float = 0.5,
) -> List[List[Tuple[float, float]]]:
    """Cluster frontier cells by proximity using BFS flood-fill.

    Transitive: if A is near B and B is near C, all three are in the same
    cluster even if A is not near C.
    """
    if not cells:
        return []

    clusters: List[List[Tuple[float, float]]] = []
    used = [False] * len(cells)
    threshold_sq = threshold * threshold

    for i in range(len(cells)):
        if not used[i]:
            clusters.append(_bfs_expand(cells, i, used, threshold_sq))

    return clusters


def direction_novelty(direction: float, explored_directions: List[float]) -> float:
    """Score how novel a direction is (prefer unexplored directions).

    Returns a value in [0.5, 1.0] — 1.0 means completely new direction.
    """
    if not explored_directions:
        return 1.0

    min_diff = float('inf')
    for explored in explored_directions:
        diff = abs(normalize_angle(direction - explored))
        min_diff = min(min_diff, diff)

    # Normalize: pi = completely new, 0 = same direction
    novelty = min_diff / math.pi
    return 0.5 + 0.5 * novelty


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
