# Coverage-Based Autonomous Exploration Architecture

**Status**: Design Document
**Author**: Claude Code
**Date**: December 2025

## Overview

This document describes the architectural refactor from waypoint-based missions to coverage-based autonomous exploration. The key changes:

1. **Dock sends high-level goals** ("map 95% of zone") instead of specific waypoints
2. **Rovers explore autonomously** using frontier-based exploration
3. **Simulated battery** requires rovers to return periodically
4. **Map data uploaded on return** - dock only gets updates when rovers physically return
5. **Re-dispatch cycle** continues until target coverage achieved

## Current Architecture (Waypoint-Based)

```
User → MissionRequest{waypoints[]} → Dock → BidNotice → Rover
                                                           ↓
                                                    Execute waypoints
                                                    Return to dock
                                                           ↓
                                           TaskComplete{map_data} → Dock
```

**Limitations**:
- Requires pre-planned waypoints
- No online adaptation to discoveries
- Fixed paths don't optimize for coverage

## Proposed Architecture (Coverage-Based)

```
                    ┌──────────────────────────────────────────┐
                    │           DOCK (Coverage Coordinator)     │
                    │                                           │
                    │  ┌─────────────────────────────────────┐ │
                    │  │ CoverageState:                      │ │
                    │  │   - global_map (merged from rovers) │ │
                    │  │   - explored_cells / total_cells    │ │
                    │  │   - target_coverage (e.g., 0.95)    │ │
                    │  │   - assigned_sectors per rover      │ │
                    │  └─────────────────────────────────────┘ │
                    │                                           │
                    │  [Receive TaskComplete] → merge_map()     │
                    │  [Check coverage] → dispatch_next_rover() │
                    └───────────────┬───────────────────────────┘
                                    │
               ┌────────────────────┴────────────────────┐
               ▼                                         ▼
    ┌─────────────────────┐                   ┌─────────────────────┐
    │   ROVER (Akko)      │                   │   ROVER (Baba_Yaga) │
    │                     │                   │                     │
    │  CoverageGoal:      │                   │  CoverageGoal:      │
    │    target: 0.95     │                   │    target: 0.95     │
    │    sector: NE       │                   │    sector: SW       │
    │    max_time: 300s   │                   │    max_time: 300s   │
    │                     │                   │                     │
    │  Battery: 85%       │                   │  Battery: 72%       │
    │  drain: 0.5%/meter  │                   │  drain: 0.5%/meter  │
    │  return_threshold:  │                   │  return_threshold:  │
    │    20%              │                   │    20%              │
    └─────────────────────┘                   └─────────────────────┘
```

## New Message Types

### 1. CoverageGoal (replaces waypoints in some missions)

```python
@dataclass
class CoverageGoal:
    """High-level coverage exploration goal."""

    target_coverage: float = 0.95      # 0.0-1.0 target coverage
    sector: Optional[str] = None       # "NE", "NW", "SE", "SW", or None for all
    sector_bounds: Optional[Tuple[float, float, float, float]] = None  # (x_min, y_min, x_max, y_max)
    max_exploration_time: float = 300.0  # seconds before mandatory return
    return_on_low_battery: bool = True   # honor battery threshold
    battery_return_threshold: float = 0.20  # return when battery < 20%
```

### 2. Updated MissionRequest

```python
@dataclass
class MissionRequest:
    """Top-level mission request from user to dock."""

    task: str  # "explore", "coverage", or other task types

    # Waypoint mode (existing)
    waypoints: Optional[List[Waypoint]] = None

    # Coverage mode (new)
    coverage_goal: Optional[CoverageGoal] = None

    return_to_dock: bool = True
```

### 3. CoverageStatus (new feedback type)

```python
@dataclass
class CoverageStatus:
    """Periodic status update during coverage exploration."""

    module_id: str
    current_coverage: float      # 0.0-1.0 cells explored / total
    battery_remaining: float     # 0.0-1.0
    distance_traveled: float     # meters
    frontiers_found: int
    returning_to_dock: bool      # True if heading back
    reason: str = ""             # Why returning (battery, coverage, timeout)
```

## Component Changes

### 1. Dock Node Changes (`dock_node.py`)

**New State**:
```python
class Dock(Node):
    def __init__(self):
        # ... existing init ...

        # Coverage mission state
        self.coverage_mission_active = False
        self.coverage_target = 0.0
        self.global_coverage_map = None  # Merged OccupancyGrid
        self.rover_maps = {}  # module_id → latest map
        self.rover_coverage = {}  # module_id → coverage contribution

        # Subscribe to coverage status updates
        self.sub_coverage_status = self.create_subscription(
            String, '/coven/coverage_status', self.on_coverage_status, 10
        )
```

**New Methods**:
```python
def start_coverage_mission(self, target: float = 0.95):
    """Start a multi-rover coverage exploration mission."""
    self.coverage_mission_active = True
    self.coverage_target = target
    self.global_coverage_map = None
    self._assign_sectors_to_available_rovers()

def _assign_sectors_to_available_rovers(self):
    """Divide arena into sectors and assign to available rovers."""
    available = self._get_available_modules()
    sectors = self._divide_into_sectors(len(available))
    for rover, sector in zip(available, sectors):
        self._dispatch_coverage_task(rover, sector)

def _dispatch_coverage_task(self, module_id: str, sector: dict):
    """Send a coverage exploration task to a specific rover."""
    goal = CoverageGoal(
        target_coverage=self.coverage_target,
        sector=sector['name'],
        sector_bounds=sector['bounds'],
        max_exploration_time=300.0
    )
    mission = MissionRequest(task="coverage", coverage_goal=goal)
    # ... start auction or direct assignment ...

def on_task_complete(self, msg: String):
    """Handle task completion - merge map and check global coverage."""
    tc = common.task_complete_decode(msg)
    # ... existing handling ...

    if self.coverage_mission_active:
        self._merge_rover_map(tc.module_id, tc.map_data)
        global_coverage = self._calculate_global_coverage()

        if global_coverage >= self.coverage_target:
            self._complete_coverage_mission()
        else:
            # Re-dispatch this rover to unexplored area
            self._redispatch_rover(tc.module_id)

def _merge_rover_map(self, module_id: str, map_data: str):
    """Merge rover's map into global coverage map."""
    # Decode map, merge with global map using cell-wise max confidence
    pass

def _calculate_global_coverage(self) -> float:
    """Calculate total explored coverage from merged map."""
    if self.global_coverage_map is None:
        return 0.0
    explored = np.sum(self.global_coverage_map >= 0)  # known cells
    total = self.global_coverage_map.size
    return explored / total
```

### 2. Module Node Changes (`module_node.py`)

**Battery Simulation**:
```python
class ModuleNode(Node):
    def __init__(self):
        # ... existing init ...

        # Battery simulation
        self.battery_level = 1.0  # Start full
        self.battery_drain_per_meter = 0.005  # 0.5% per meter
        self.battery_return_threshold = 0.20
        self.last_pose = None

    def _update_battery(self, distance_moved: float):
        """Drain battery based on distance traveled."""
        self.battery_level -= distance_moved * self.battery_drain_per_meter
        self.battery_level = max(0.0, self.battery_level)

    def _should_return_for_battery(self) -> bool:
        """Check if battery is low enough to require return."""
        return self.battery_level < self.battery_return_threshold
```

**Coverage Task Execution**:
```python
def _execute_coverage_mission(self, goal: CoverageGoal):
    """Execute autonomous frontier exploration for coverage goal."""

    start_time = time.time()
    total_distance = 0.0

    while True:
        # Check termination conditions
        elapsed = time.time() - start_time

        if elapsed >= goal.max_exploration_time:
            reason = "timeout"
            break

        if self._should_return_for_battery():
            reason = "low_battery"
            break

        local_coverage = self.explorer.get_coverage()
        if local_coverage >= goal.target_coverage:
            reason = "coverage_achieved"
            break

        # Find and navigate to next frontier
        frontier = self.explorer.find_best_frontier(goal.sector_bounds)
        if frontier is None:
            reason = "no_frontiers"
            break

        # Navigate with battery tracking
        nav_result = self._navigate_with_battery_tracking(frontier)
        total_distance += nav_result.distance

        # Publish status update
        self._publish_coverage_status(local_coverage, reason="exploring")

    # Return to dock
    self._publish_coverage_status(local_coverage, reason=reason, returning=True)
    self._return_to_dock()

    # Upload map data on arrival
    return self._create_task_complete(reason, total_distance, local_coverage)
```

### 3. Explorer Changes (`exploration.py`)

**Enhanced Frontier Selection**:
```python
class Explorer:
    def find_best_frontier(self, sector_bounds: Optional[Tuple] = None) -> Optional[Point]:
        """Find the best frontier point, optionally within sector bounds."""
        frontiers = self.find_frontiers()

        if sector_bounds:
            # Filter to frontiers within assigned sector
            x_min, y_min, x_max, y_max = sector_bounds
            frontiers = [f for f in frontiers
                        if x_min <= f.x <= x_max and y_min <= f.y <= y_max]

        if not frontiers:
            return None

        # Score by: information gain / navigation cost
        robot_pos = self.get_robot_position()
        scored = [(f, self._score_frontier(f, robot_pos)) for f in frontiers]
        scored.sort(key=lambda x: x[1], reverse=True)

        return scored[0][0]

    def _score_frontier(self, frontier: Point, robot_pos: Point) -> float:
        """Score frontier by expected information gain vs travel cost."""
        distance = self._euclidean_distance(frontier, robot_pos)
        unknown_cells = self._count_unknown_cells_near(frontier)

        # Prefer high info gain, low travel cost
        if distance < 0.1:
            distance = 0.1  # Avoid division by zero
        return unknown_cells / distance
```

### 4. New Config Options (`config.py`)

```python
@dataclass
class BatteryConfig:
    """Battery simulation configuration."""

    initial_level: float = 1.0           # Start at 100%
    drain_per_meter: float = 0.005       # 0.5% per meter traveled
    drain_per_second_idle: float = 0.0001  # 0.01% per second when idle
    return_threshold: float = 0.20       # Return when below 20%
    critical_threshold: float = 0.05     # Emergency stop below 5%
    recharge_rate: float = 0.10          # 10% per second at dock

@dataclass
class CoverageConfig:
    """Coverage exploration configuration."""

    default_target: float = 0.95         # 95% coverage target
    max_mission_time: float = 600.0      # 10 minutes max
    status_update_interval: float = 5.0  # Update every 5 seconds
    sector_overlap: float = 0.10         # 10% overlap between sectors
    min_sector_size: float = 25.0        # Minimum 25 sq meters per sector
```

## Message Flow Diagram

```
User                 Dock                          Rovers
  │                    │                              │
  │ MissionRequest     │                              │
  │ task="coverage"    │                              │
  │ target=0.95        │                              │
  ├───────────────────>│                              │
  │                    │                              │
  │                    │ Divide arena into sectors    │
  │                    │ (NE, NW, SE, SW)             │
  │                    │                              │
  │                    │ BidNotice (sector NE)        │
  │                    ├─────────────────────────────>│ Akko
  │                    │                              │
  │                    │ BidNotice (sector SW)        │
  │                    ├─────────────────────────────>│ Baba_Yaga
  │                    │                              │
  │                    │<─────────────────────────────┤ BidProposal
  │                    │<─────────────────────────────┤ BidProposal
  │                    │                              │
  │                    │ TaskReq (sector NE)          │
  │                    ├─────────────────────────────>│ Akko
  │                    │                              │
  │                    │ TaskReq (sector SW)          │
  │                    ├─────────────────────────────>│ Baba_Yaga
  │                    │                              │
  │                    │          ┌───────────────────┤ Autonomous
  │                    │          │ Frontier explore  │ exploration
  │                    │          │ Battery drains    │ (parallel)
  │                    │          │ Status updates    │
  │                    │          └───────────────────┤
  │                    │                              │
  │                    │ CoverageStatus (Akko)        │
  │                    │<─────────────────────────────┤ coverage=23%
  │                    │                              │ battery=72%
  │                    │                              │
  │                    │ CoverageStatus (Baba_Yaga)   │
  │                    │<─────────────────────────────┤ coverage=31%
  │                    │                              │ battery=65%
  │                    │                              │
  │                    │          [Battery low!]      │
  │                    │          [Return to dock]    │
  │                    │                              │
  │                    │ TaskComplete (Akko)          │
  │                    │<─────────────────────────────┤ map_data
  │                    │                              │ coverage=45%
  │                    │                              │
  │                    │ Merge map                    │
  │                    │ Global coverage = 45%        │
  │                    │ Target = 95% (not met)       │
  │                    │                              │
  │                    │ Re-dispatch Akko to          │
  │                    │ unexplored area              │
  │                    │ TaskReq (new sector)         │
  │                    ├─────────────────────────────>│ Akko
  │                    │                              │
  │                    │ ... cycle continues ...      │
  │                    │                              │
  │                    │ [Global coverage = 96%]      │
  │                    │ Mission Complete!            │
  │<───────────────────┤                              │
  │ MissionComplete    │                              │
  │ coverage=96%       │                              │
```

## Implementation Phases

### Phase 1: Battery Simulation (Minimal Changes)
1. Add battery tracking to `module_node.py`
2. Add `BatteryConfig` to `config.py`
3. Rover returns when battery < threshold
4. Battery "recharges" when at dock

### Phase 2: Coverage Goal Type
1. Add `CoverageGoal` dataclass to `common.py`
2. Update `MissionRequest` to include optional `CoverageGoal`
3. Add `_execute_coverage_mission()` to module_node
4. Explorer uses frontier exploration (already exists!)

### Phase 3: Dock Coverage Coordinator
1. Add coverage state tracking to dock_node
2. Implement map merging logic
3. Add sector assignment algorithm
4. Implement re-dispatch cycle

### Phase 4: Status Updates & UI
1. Add `CoverageStatus` message type
2. Periodic status publishing from rovers
3. Dock logs global coverage progress
4. Optional: RViz visualization of coverage

## Testing Plan

1. **Unit Tests**:
   - Battery drain calculation
   - Sector division algorithm
   - Map merging logic
   - Coverage calculation

2. **Integration Tests**:
   - Single rover coverage mission
   - Two rover parallel coverage
   - Battery return and re-dispatch
   - Coverage target achievement

3. **Simulation Tests**:
   - Full 2-rover mission in Gazebo
   - Verify map quality
   - Measure time-to-coverage

## Backwards Compatibility

- Existing waypoint missions continue to work unchanged
- `task="explore"` with waypoints → waypoint mode
- `task="coverage"` with coverage_goal → coverage mode
- `task="explore"` without waypoints → existing frontier exploration (single rover)

## Open Questions

1. **Sector Assignment Strategy**:
   - Static division (NE/NW/SE/SW)?
   - Dynamic based on current coverage gaps?
   - Voronoi-based on rover positions?

2. **Map Merging Algorithm**:
   - Simple overlay (latest wins)?
   - Confidence-weighted merge?
   - Full SLAM graph optimization?

3. **Re-dispatch Strategy**:
   - Return to same sector?
   - Assign to least-covered sector?
   - Information-gain optimization?
