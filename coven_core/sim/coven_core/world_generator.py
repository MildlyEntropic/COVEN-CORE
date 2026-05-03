#!/usr/bin/env python3
"""
world_generator.py — Dynamic COVEN world generation

Generates random obstacle arrangements and finds valid spawn positions
for docks that are at least 2m away from any obstacles.

Usage:
    from coven_core.world_generator import generate_world, find_dock_spawn

Author: Alexander Shultis
Date: December 2025
"""

import random
import math
import json
from dataclasses import dataclass
from typing import List, Tuple, Optional


@dataclass
class Obstacle:
    """Represents an obstacle in the world."""
    name: str
    x: float
    y: float
    z: float  # Height above ground (half-height for SDF)
    size_x: float
    size_y: float
    size_z: float
    shape: str = "box"  # "box" or "cylinder"
    radius: float = 0.0  # For cylinders


# World boundaries (simulation area)
WORLD_MIN_X = -10.0
WORLD_MAX_X = 10.0
WORLD_MIN_Y = -10.0
WORLD_MAX_Y = 10.0

# Spacing requirements
MIN_DOCK_CLEARANCE = 2.0  # Dock must be at least 2m from any obstacle
MIN_OBSTACLE_SPACING = 1.0  # Obstacles should be at least 1m apart


def generate_random_obstacles(
    num_walls: int = 3,
    num_boxes: int = 3,
    num_cylinders: int = 2,
    exclusion_zone: Optional[Tuple[float, float, float]] = None
) -> List[Obstacle]:
    """
    Generate random obstacles for the world.

    Args:
        num_walls: Number of wall segments to create
        num_boxes: Number of box obstacles to create
        num_cylinders: Number of cylinder obstacles to create
        exclusion_zone: Optional (x, y, radius) to keep clear for dock

    Returns:
        List of Obstacle objects
    """
    obstacles = []
    positions_used = []

    def is_valid_position(x: float, y: float, size: float) -> bool:
        """Check if position is valid (not overlapping existing obstacles)."""
        # Check exclusion zone
        if exclusion_zone:
            ex, ey, er = exclusion_zone
            dist = math.sqrt((x - ex)**2 + (y - ey)**2)
            if dist < er + size:
                return False

        # Check existing obstacles
        for px, py, ps in positions_used:
            dist = math.sqrt((x - px)**2 + (y - py)**2)
            if dist < ps + size + MIN_OBSTACLE_SPACING:
                return False

        return True

    def find_valid_position(size: float, max_attempts: int = 50) -> Optional[Tuple[float, float]]:
        """Find a valid position for an obstacle of given size."""
        for _ in range(max_attempts):
            x = random.uniform(WORLD_MIN_X + size, WORLD_MAX_X - size)
            y = random.uniform(WORLD_MIN_Y + size, WORLD_MAX_Y - size)
            if is_valid_position(x, y, size):
                return x, y
        return None

    # Generate walls (long thin boxes)
    for i in range(num_walls):
        # Walls are 0.2m thick, 2-5m long, 1m tall
        length = random.uniform(2.0, 5.0)
        width = 0.2
        height = 1.0
        size = max(length, width) / 2

        pos = find_valid_position(size)
        if pos:
            x, y = pos
            # Random rotation (horizontal or vertical)
            if random.random() > 0.5:
                sx, sy = length, width
            else:
                sx, sy = width, length

            obstacles.append(Obstacle(
                name=f"wall_{i+1}",
                x=x,
                y=y,
                z=height / 2,
                size_x=sx,
                size_y=sy,
                size_z=height,
                shape="box"
            ))
            positions_used.append((x, y, size))

    # Generate box obstacles
    for i in range(num_boxes):
        # Boxes are 0.3-0.8m cubes, 0.4-0.8m tall
        size_xy = random.uniform(0.3, 0.8)
        height = random.uniform(0.4, 0.8)
        size = size_xy / 2

        pos = find_valid_position(size)
        if pos:
            x, y = pos
            obstacles.append(Obstacle(
                name=f"obstacle_box_{i+1}",
                x=x,
                y=y,
                z=height / 2,
                size_x=size_xy,
                size_y=size_xy,
                size_z=height,
                shape="box"
            ))
            positions_used.append((x, y, size))

    # Generate cylinder obstacles
    for i in range(num_cylinders):
        # Cylinders are 0.2-0.5m radius, 0.3-0.7m tall
        radius = random.uniform(0.2, 0.5)
        height = random.uniform(0.3, 0.7)

        pos = find_valid_position(radius)
        if pos:
            x, y = pos
            obstacles.append(Obstacle(
                name=f"obstacle_cyl_{i+1}",
                x=x,
                y=y,
                z=height / 2,
                size_x=0,
                size_y=0,
                size_z=height,
                shape="cylinder",
                radius=radius
            ))
            positions_used.append((x, y, radius))

    return obstacles


def find_dock_spawn(obstacles: List[Obstacle], clearance: float = MIN_DOCK_CLEARANCE) -> Tuple[float, float]:
    """
    Find a valid spawn position for the dock.

    Args:
        obstacles: List of existing obstacles
        clearance: Minimum distance from any obstacle (default 2m)

    Returns:
        Tuple of (x, y) coordinates for dock spawn
    """
    # Try random positions
    for _ in range(100):
        x = random.uniform(WORLD_MIN_X + clearance, WORLD_MAX_X - clearance)
        y = random.uniform(WORLD_MIN_Y + clearance, WORLD_MAX_Y - clearance)

        # Check distance from all obstacles
        valid = True
        for obs in obstacles:
            if obs.shape == "cylinder":
                obs_size = obs.radius
            else:
                obs_size = max(obs.size_x, obs.size_y) / 2

            dist = math.sqrt((x - obs.x)**2 + (y - obs.y)**2)
            if dist < clearance + obs_size:
                valid = False
                break

        if valid:
            return x, y

    # Fallback to origin if no valid position found
    return 0.0, 0.0


def obstacle_to_sdf(obs: Obstacle) -> str:
    """Convert an Obstacle to SDF XML string."""
    # Random muted color
    r = random.uniform(0.3, 0.6)
    g = random.uniform(0.3, 0.6)
    b = random.uniform(0.3, 0.6)

    if obs.shape == "cylinder":
        geometry = f"""<cylinder><radius>{obs.radius}</radius><length>{obs.size_z}</length></cylinder>"""
    else:
        geometry = f"""<box><size>{obs.size_x} {obs.size_y} {obs.size_z}</size></box>"""

    return f"""
    <model name="{obs.name}">
      <static>true</static>
      <pose>{obs.x} {obs.y} {obs.z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>{geometry}</geometry>
        </collision>
        <visual name="visual">
          <geometry>{geometry}</geometry>
          <material>
            <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
            <diffuse>{r+0.1:.2f} {g+0.1:.2f} {b+0.1:.2f} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def generate_world_sdf(
    num_walls: int = 3,
    num_boxes: int = 3,
    num_cylinders: int = 2
) -> Tuple[str, float, float]:
    """
    Generate a complete world SDF with random obstacles.

    Args:
        num_walls: Number of walls
        num_boxes: Number of box obstacles
        num_cylinders: Number of cylinder obstacles

    Returns:
        Tuple of (sdf_content, dock_x, dock_y)
    """
    # First find a dock position, then generate obstacles around it
    dock_x = random.uniform(-5.0, 5.0)
    dock_y = random.uniform(-5.0, 5.0)

    # Generate obstacles with exclusion zone around dock
    obstacles = generate_random_obstacles(
        num_walls=num_walls,
        num_boxes=num_boxes,
        num_cylinders=num_cylinders,
        exclusion_zone=(dock_x, dock_y, MIN_DOCK_CLEARANCE)
    )

    # Generate obstacle SDF
    obstacles_sdf = "\n".join(obstacle_to_sdf(obs) for obs in obstacles)

    # Complete world SDF
    sdf = f"""<?xml version="1.0"?>
<!--
COVEN Dynamic World
Auto-generated with random obstacles
Dock spawn: ({dock_x:.2f}, {dock_y:.2f})
-->
<sdf version="1.9">
  <world name="coven_world">

    <!-- Physics -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>50 50</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>50 50</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Rovers are spawned dynamically via ./coven Runtime options -->
    <!-- Suggested dock spawn: ({dock_x:.2f}, {dock_y:.2f}) -->

    <!-- Random obstacles -->
{obstacles_sdf}

  </world>
</sdf>
"""

    return sdf, dock_x, dock_y


def main():
    """Command-line interface for world generation."""
    import argparse

    parser = argparse.ArgumentParser(description="Generate COVEN world with random obstacles")
    parser.add_argument("--walls", type=int, default=3, help="Number of walls")
    parser.add_argument("--boxes", type=int, default=3, help="Number of box obstacles")
    parser.add_argument("--cylinders", type=int, default=2, help="Number of cylinder obstacles")
    parser.add_argument("--output", type=str, help="Output SDF file path")
    parser.add_argument("--dock-position", action="store_true", help="Only output dock position as JSON")

    args = parser.parse_args()

    if args.dock_position:
        # Just generate obstacles and find dock position
        obstacles = generate_random_obstacles(args.walls, args.boxes, args.cylinders)
        dock_x, dock_y = find_dock_spawn(obstacles)
        print(json.dumps({"dock_x": dock_x, "dock_y": dock_y}))
    else:
        # Generate full world
        sdf, dock_x, dock_y = generate_world_sdf(args.walls, args.boxes, args.cylinders)

        if args.output:
            with open(args.output, 'w') as f:
                f.write(sdf)
            print(f"World saved to {args.output}")
            print(f"Dock spawn position: ({dock_x:.2f}, {dock_y:.2f})")
        else:
            print(sdf)


if __name__ == "__main__":
    main()
