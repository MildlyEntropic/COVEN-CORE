# COVEN Production Readiness Roadmap

**Generated:** 2025-11-05
**Status:** Phase 1 Complete (Simulation), Phase 2 Planning (Hardware)

---

## Executive Summary

COVEN is **2,089 lines** of well-architected ROS2 code, currently **~70% production-ready**. The protocol design is solid, multi-robot coordination works (tested 10+ modules), and frontier exploration is sophisticated. **Main gaps:** hardcoded configuration, no physical docking, limited error recovery, and no hardware abstraction.

**Timeline to "Autobots Roll Out" on CubeRovers:** 2-3 weeks of focused development.

---

## Current Architecture Assessment

### ✅ Strengths

1. **Clean FSM Protocol**
   - BOOT → IDENTIFY → VERIFY → NORMAL → FIELD_OPS
   - JSON messages (easy debugging)
   - Extensible message format

2. **Robust Multi-Module Coordination**
   - Tested up to 10 modules
   - Heartbeat monitoring with 3-miss dropout
   - Graceful recovery (1-2 missed heartbeats)

3. **Sophisticated Exploration**
   - Frontier-based navigation
   - Coverage tracking (80% threshold)
   - Map serialization (gzip + base64)
   - Transfer to dock via TaskComplete

4. **Excellent Documentation**
   - README, QUICK_START, TROUBLESHOOTING, SIMULATION_GUIDE
   - Clear examples and command reference

5. **Smart Launcher**
   - Waits for /clock, /scan, controller_manager
   - No blind sleeps
   - Graceful cleanup (SIGTERM → SIGKILL)

### ❌ Critical Gaps

1. **No ROS2 Parameters**
   - All constants hardcoded (timing, paths, thresholds)
   - Requires code rebuild to tune

2. **Topic-Based Tasks**
   - Should use Action servers for feedback/cancellation
   - No progress updates during execution

3. **No Physical Docking**
   - IR beacon tracking not implemented
   - No auto-alignment behavior
   - No mechanical latch control

4. **No Hardware Abstraction**
   - Assumes Gazebo simulation
   - No GPIO control for power rails
   - No physical sensor interfaces

5. **Limited Error Recovery**
   - Modules can get stuck in FIELD_OPS
   - No watchdog timers
   - Silent failures (return None, return "")

6. **No Testing Infrastructure**
   - No unit tests
   - No integration tests
   - Manual testing only

---

## Implementation Tiers (Prioritized)

### TIER 1: Quick Wins (1-2 days) 🎯 **DO THESE FIRST**

These provide maximum value with minimal effort.

#### 1.1 ROS2 Parameter System

**Why:** Eliminate hardcoded constants, enable tuning without rebuilding.

**Files to Modify:**
- `coven_core/module_node.py`
- `coven_core/dock_node_multi.py`
- `coven_core/exploration.py`

**Files to Create:**
- `config/coven_params.yaml`
- `config/dock_params.yaml`
- `config/exploration_params.yaml`

**Implementation:**
```python
# In module_node.py __init__:
self.declare_parameter('heartbeat_period', 0.8)
self.declare_parameter('task_timeout', 300.0)
self.declare_parameter('map_storage_dir', '~/coven_maps')

self.hb_period = self.get_parameter('heartbeat_period').value
self.task_timeout = self.get_parameter('task_timeout').value
```

**YAML Structure:**
```yaml
# config/coven_params.yaml
coven_module:
  ros__parameters:
    heartbeat_period: 0.8
    task_timeout: 300.0
    map_storage_dir: '~/coven_maps'

    exploration:
      frontier_radius: 3.0
      min_frontier_size: 10
      coverage_threshold: 0.80
      timeout: 300.0
      no_frontier_limit: 3

coven_dock:
  ros__parameters:
    identify_period: 5.0
    heartbeat_timeout: 1.0
    max_heartbeat_misses: 3
    map_storage_dir: '~/coven_maps'
```

**Launch File Update:**
```python
# In coven.launch.py
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

config_file = PathJoinSubstitution([
    get_package_share_directory('coven_core'),
    'config',
    'coven_params.yaml'
])

Node(
    package='coven_core',
    executable='module',
    parameters=[config_file]
)
```

**Time Estimate:** 2-3 hours
**Impact:** HIGH - Enables tuning, follows ROS2 best practices

---

#### 1.2 Watchdog Timers

**Why:** Prevent modules from getting stuck in FIELD_OPS if task hangs.

**Files to Modify:**
- `coven_core/module_node.py`

**Implementation:**
```python
# In module_node.py

def __init__(self):
    # ... existing code ...
    self.task_watchdog_timer = None

def _start_task_watchdog(self):
    """Start watchdog timer when entering FIELD_OPS"""
    if self.task_watchdog_timer:
        self.task_watchdog_timer.cancel()

    self.task_watchdog_timer = self.create_timer(
        self.task_timeout,  # From ROS parameter
        self._on_task_timeout
    )

def _stop_task_watchdog(self):
    """Stop watchdog timer when task completes"""
    if self.task_watchdog_timer:
        self.task_watchdog_timer.cancel()
        self.task_watchdog_timer = None

def _on_task_timeout(self):
    """Called if task exceeds timeout"""
    if self.state == ModuleState.FIELD_OPS:
        self.get_logger().error(
            f"{COLOR_RED}Task timeout after {self.task_timeout}s - aborting{COLOR_RESET}"
        )

        # Cancel navigation if active
        if self.explorer and self.navigator:
            self.navigator.cancelTask()

        # Return to NORMAL state
        self.state = ModuleState.NORMAL
        self._stop_task_watchdog()

        # Send failure result
        self._send_task_complete(success=False, note="Task timeout")

def execute_task(self, task: str):
    """Modified to start watchdog"""
    self._start_task_watchdog()  # START WATCHDOG

    try:
        # ... existing task execution ...

        self._send_task_complete(success=True, ...)
    finally:
        self._stop_task_watchdog()  # STOP WATCHDOG
```

**Time Estimate:** 1 hour
**Impact:** HIGH - Prevents stuck states, critical for reliability

---

#### 1.3 TF2 for Pose Tracking

**Why:** Currently assumes dock at origin (0,0,0). TF2 provides accurate transforms.

**Community Pattern:** Standard ROS2 practice for all coordinate transforms.

**Files to Modify:**
- `coven_core/module_node.py`
- `coven_core/exploration.py`

**Implementation:**
```python
# In module_node.py
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Pose2D
from tf2_ros.transform_listener import TransformException

def __init__(self):
    # ... existing code ...

    # TF2 setup
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

def get_dock_pose(self) -> Pose2D:
    """Get dock pose in map frame via TF2"""
    try:
        # Look up transform from map to dock_frame
        transform = self.tf_buffer.lookup_transform(
            'map',           # target frame
            'dock_frame',    # source frame
            rclpy.time.Time(),  # latest available
            timeout=rclpy.duration.Duration(seconds=1.0)
        )

        return Pose2D(
            x=transform.transform.translation.x,
            y=transform.transform.translation.y,
            theta=0.0  # Extract from quaternion if needed
        )

    except TransformException as e:
        self.get_logger().warn(f"TF2 lookup failed: {e}, using fallback (0,0)")
        return Pose2D(x=0.0, y=0.0, theta=0.0)

def return_to_dock(self):
    """Modified to use TF2"""
    dock_pose = self.get_dock_pose()  # Use TF2 instead of (0,0)

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = self.get_clock().now().to_msg()
    goal.pose.position.x = dock_pose.x
    goal.pose.position.y = dock_pose.y
    # ... set orientation ...

    self.navigator.goToPose(goal)
```

**Dock Frame Publishing:**
```python
# In dock_node_multi.py
from tf2_ros import StaticTransformBroadcaster

def __init__(self):
    # ... existing code ...

    # Publish static transform for dock location
    self.tf_static_broadcaster = StaticTransformBroadcaster(self)
    self._publish_dock_transform()

def _publish_dock_transform(self):
    """Publish dock position as static TF"""
    from geometry_msgs.msg import TransformStamped

    t = TransformStamped()
    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'map'
    t.child_frame_id = 'dock_frame'

    # Dock at origin for now (configurable via param later)
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.w = 1.0

    self.tf_static_broadcaster.sendTransform(t)
```

**Time Estimate:** 2 hours
**Impact:** MEDIUM - More robust, enables future multi-dock scenarios

---

#### 1.4 Unit Tests for common.py

**Why:** Catch regressions, enable CI/CD, validate message encode/decode.

**Files to Create:**
- `test/test_common.py`
- `test/test_fsm_transitions.py`

**Implementation:**
```python
# test/test_common.py
import pytest
from std_msgs.msg import String
from coven_core.common import (
    IdentifyReq, IdentifyRep, VerifyReq, VerifyRep,
    Heartbeat, TaskReq, TaskAck, TaskStart, TaskComplete,
    ident_req_encode, ident_req_decode,
    ident_rep_encode, ident_rep_decode,
    # ... other encode/decode functions
)

class TestIdentifyMessages:
    def test_identify_req_roundtrip(self):
        original = IdentifyReq(dock_id="DOCK-001", broadcast="all")
        msg = ident_req_encode(original)
        decoded = ident_req_decode(msg)

        assert decoded is not None
        assert decoded.dock_id == "DOCK-001"
        assert decoded.broadcast == "all"

    def test_identify_req_malformed_json(self):
        bad_msg = String(data="{not valid json")
        decoded = ident_req_decode(bad_msg)
        assert decoded is None  # Should handle gracefully

    def test_identify_rep_roundtrip(self):
        original = IdentifyRep(
            module_id="RR-a3b4c5",
            module_type="ReconRover",
            firmware_version="v1.0.0"
        )
        msg = ident_rep_encode(original)
        decoded = ident_rep_decode(msg)

        assert decoded is not None
        assert decoded.module_id == "RR-a3b4c5"
        assert decoded.module_type == "ReconRover"
        assert decoded.firmware_version == "v1.0.0"

class TestVerifyMessages:
    def test_verify_req_roundtrip(self):
        original = VerifyReq(module_id="RR-a3b4c5")
        msg = verify_req_encode(original)
        decoded = verify_req_decode(msg)

        assert decoded is not None
        assert decoded.module_id == "RR-a3b4c5"

    def test_verify_rep_success(self):
        original = VerifyRep(
            module_id="RR-a3b4c5",
            ok=True,
            reason="All sensors operational"
        )
        msg = verify_rep_encode(original)
        decoded = verify_rep_decode(msg)

        assert decoded is not None
        assert decoded.ok is True
        assert decoded.reason == "All sensors operational"

    def test_verify_rep_failure(self):
        original = VerifyRep(
            module_id="RR-a3b4c5",
            ok=False,
            reason="Lidar offline"
        )
        msg = verify_rep_encode(original)
        decoded = verify_rep_decode(msg)

        assert decoded is not None
        assert decoded.ok is False
        assert decoded.reason == "Lidar offline"

class TestHeartbeat:
    def test_heartbeat_roundtrip(self):
        original = Heartbeat(module_id="RR-a3b4c5", seq=42)
        msg = heartbeat_encode(original)
        decoded = heartbeat_decode(msg)

        assert decoded is not None
        assert decoded.module_id == "RR-a3b4c5"
        assert decoded.seq == 42

class TestTaskMessages:
    def test_task_req_roundtrip(self):
        original = TaskReq(
            module_id="RR-a3b4c5",
            task="explore_warehouse"
        )
        msg = task_req_encode(original)
        decoded = task_req_decode(msg)

        assert decoded is not None
        assert decoded.module_id == "RR-a3b4c5"
        assert decoded.task == "explore_warehouse"

    def test_task_complete_with_map_data(self):
        original = TaskComplete(
            module_id="RR-a3b4c5",
            success=True,
            note="Explored 85% of warehouse",
            map_data="base64encodedmapdata",
            map_yaml="resolution: 0.05\norigin: [0,0,0]",
            exploration_metrics={"coverage": 0.85, "frontiers": 12}
        )
        msg = task_complete_encode(original)
        decoded = task_complete_decode(msg)

        assert decoded is not None
        assert decoded.success is True
        assert decoded.exploration_metrics["coverage"] == 0.85
```

**Run Tests:**
```bash
cd ~/ros2_ws
colcon test --packages-select coven_core
colcon test-result --verbose
```

**Time Estimate:** 3 hours
**Impact:** MEDIUM - Foundation for CI/CD, prevents regressions

---

### TIER 2: Hardware Readiness (3-5 days)

Critical for deploying to physical CubeRovers.

#### 2.1 Action Server for Tasks

**Why:** Actions provide feedback, cancellation, and better error handling than topics.

**Community Best Practice:** "Use Actions for long-running tasks (>1s) that need feedback/cancellation"

**Files to Create:**
- `coven_interfaces/action/ExecuteTask.action`
- `coven_interfaces/CMakeLists.txt`
- `coven_interfaces/package.xml`

**Action Definition:**
```
# coven_interfaces/action/ExecuteTask.action

# Goal
string task_name
---
# Result
bool success
string note
string map_data
string map_yaml
string exploration_metrics_json
---
# Feedback
float32 progress           # 0.0 to 1.0
string current_status      # "Navigating to frontier 3/10"
int32 frontiers_explored
int32 frontiers_remaining
```

**Module Implementation:**
```python
# In module_node.py
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from coven_interfaces.action import ExecuteTask

class Module(Node):
    def __init__(self):
        # ... existing code ...

        self._action_server = ActionServer(
            self,
            ExecuteTask,
            'coven/execute_task',
            execute_callback=self._execute_task_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback
        )

    def _goal_callback(self, goal_request):
        """Accept or reject goal"""
        if self.state != ModuleState.NORMAL:
            self.get_logger().warn(f"Rejecting task - not in NORMAL state")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Accept cancellation requests"""
        self.get_logger().info("Task cancellation requested")
        return CancelResponse.ACCEPT

    async def _execute_task_callback(self, goal_handle):
        """Execute task with feedback"""
        task_name = goal_handle.request.task_name

        self.get_logger().info(f"Executing task: {task_name}")
        self.state = ModuleState.FIELD_OPS

        # Send periodic feedback
        feedback = ExecuteTask.Feedback()

        if task_name == "explore_warehouse":
            # Initialize exploration
            if not self._initialize_navigation():
                goal_handle.abort()
                return ExecuteTask.Result(success=False, note="Nav2 init failed")

            # Explore with feedback
            total_frontiers = 10  # Estimate
            for i, frontier in enumerate(self._find_frontiers()):
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.state = ModuleState.NORMAL
                    return ExecuteTask.Result(success=False, note="Cancelled by user")

                # Send feedback
                feedback.progress = (i + 1) / total_frontiers
                feedback.current_status = f"Navigating to frontier {i+1}/{total_frontiers}"
                feedback.frontiers_explored = i + 1
                feedback.frontiers_remaining = total_frontiers - (i + 1)
                goal_handle.publish_feedback(feedback)

                # Navigate to frontier
                self._navigate_to_frontier(frontier)

            # Save map
            map_data, map_yaml = self._save_and_serialize_map()

            # Success
            goal_handle.succeed()
            result = ExecuteTask.Result()
            result.success = True
            result.note = "Exploration complete"
            result.map_data = map_data
            result.map_yaml = map_yaml
            result.exploration_metrics_json = json.dumps({"coverage": 0.85})

            self.state = ModuleState.NORMAL
            return result
```

**Dock Client:**
```python
# In dock_node_multi.py
from rclpy.action import ActionClient
from coven_interfaces.action import ExecuteTask

class DockMulti(Node):
    def __init__(self):
        # ... existing code ...

        self._action_client = ActionClient(
            self,
            ExecuteTask,
            'coven/execute_task'
        )

    def assign_task(self, module_id: str, task: str):
        """Send task via action instead of topic"""
        goal = ExecuteTask.Goal()
        goal.task_name = task

        # Send goal
        send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(
            lambda future: self._goal_response_callback(future, module_id)
        )

    def _feedback_callback(self, feedback_msg):
        """Receive progress updates"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Task progress: {feedback.progress*100:.1f}% - {feedback.current_status}"
        )

    def _goal_response_callback(self, future, module_id: str):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"Task rejected by {module_id}")
            return

        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self._result_callback(future, module_id)
        )

    def _result_callback(self, future, module_id: str):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"Task completed by {module_id}")
            self._store_map(module_id, result.map_data, result.map_yaml)
        else:
            self.get_logger().warn(f"Task failed: {result.note}")
```

**Time Estimate:** 6 hours
**Impact:** HIGH - Better UX, follows ROS2 best practices, enables cancellation

---

#### 2.2 Hardware Abstraction Layer (HAL)

**Why:** Need to control physical power rails, IR beacons, docking latch.

**Community Pattern:** Abstract hardware interfaces, similar to ros2_control.

**Files to Create:**
- `coven_core/hardware_interface.py`
- `coven_core/simulated_hardware.py`
- `coven_core/cuberover_hardware.py`

**Implementation:**
```python
# coven_core/hardware_interface.py
from abc import ABC, abstractmethod
from typing import Tuple

class HardwareInterface(ABC):
    """Abstract interface for COVEN hardware control"""

    @abstractmethod
    def enable_power_rail(self, module_id: str, voltage: int) -> bool:
        """
        Enable power rail for module.

        Args:
            module_id: Module identifier
            voltage: 5 or 12

        Returns:
            True if successful
        """
        pass

    @abstractmethod
    def disable_power_rail(self, module_id: str, voltage: int) -> bool:
        """Disable power rail for module"""
        pass

    @abstractmethod
    def get_battery_voltage(self) -> float:
        """Read battery voltage in volts"""
        pass

    @abstractmethod
    def get_battery_current(self) -> float:
        """Read battery current in amps"""
        pass

    @abstractmethod
    def detect_ir_beacon(self) -> Tuple[bool, float, float]:
        """
        Detect IR docking beacon.

        Returns:
            (detected, distance_m, angle_rad)
        """
        pass

    @abstractmethod
    def engage_mechanical_latch(self) -> bool:
        """Lock module to dock mechanically"""
        pass

    @abstractmethod
    def release_mechanical_latch(self) -> bool:
        """Unlock module from dock"""
        pass

    @abstractmethod
    def get_module_presence(self, slot: int) -> bool:
        """Detect if module is physically docked in slot"""
        pass
```

```python
# coven_core/simulated_hardware.py
from coven_core.hardware_interface import HardwareInterface
from typing import Tuple
import rclpy

class SimulatedHardware(HardwareInterface):
    """Simulated hardware for Gazebo testing"""

    def __init__(self, node: rclpy.node.Node):
        self.node = node
        self.powered_modules = {}

    def enable_power_rail(self, module_id: str, voltage: int) -> bool:
        self.node.get_logger().info(f"[SIM] Enabling {voltage}V rail for {module_id}")
        self.powered_modules[module_id] = voltage

        # In simulation, publish message instead
        # (existing topic-based mechanism)
        return True

    def disable_power_rail(self, module_id: str, voltage: int) -> bool:
        self.node.get_logger().info(f"[SIM] Disabling {voltage}V rail for {module_id}")
        if module_id in self.powered_modules:
            del self.powered_modules[module_id]
        return True

    def get_battery_voltage(self) -> float:
        return 12.6  # Simulated full charge

    def get_battery_current(self) -> float:
        return 0.5  # Simulated 0.5A draw

    def detect_ir_beacon(self) -> Tuple[bool, float, float]:
        # Simulated: always "detect" beacon at origin
        return (True, 1.0, 0.0)  # 1m away, 0 rad angle

    def engage_mechanical_latch(self) -> bool:
        self.node.get_logger().info("[SIM] Mechanical latch engaged")
        return True

    def release_mechanical_latch(self) -> bool:
        self.node.get_logger().info("[SIM] Mechanical latch released")
        return True

    def get_module_presence(self, slot: int) -> bool:
        # In sim, assume presence if powered
        return len(self.powered_modules) > 0
```

```python
# coven_core/cuberover_hardware.py
from coven_core.hardware_interface import HardwareInterface
from typing import Tuple
import rclpy

class CubeRoverHardware(HardwareInterface):
    """Physical CubeRover hardware implementation"""

    def __init__(self, node: rclpy.node.Node):
        self.node = node

        # TODO: Initialize GPIO, I2C, etc.
        # import RPi.GPIO as GPIO
        # import smbus2

        self._setup_gpio()
        self._setup_i2c()

    def _setup_gpio(self):
        """Initialize GPIO pins for power control"""
        # Example using Raspberry Pi GPIO
        # import RPi.GPIO as GPIO
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(POWER_ENABLE_5V_PIN, GPIO.OUT)
        # GPIO.setup(POWER_ENABLE_12V_PIN, GPIO.OUT)
        # GPIO.setup(LATCH_SERVO_PIN, GPIO.OUT)
        pass

    def _setup_i2c(self):
        """Initialize I2C for IR sensor communication"""
        # Example using smbus2
        # self.i2c_bus = smbus2.SMBus(1)
        # self.ir_sensor_addr = 0x60
        pass

    def enable_power_rail(self, module_id: str, voltage: int) -> bool:
        """Enable GPIO-controlled power rail"""
        # import RPi.GPIO as GPIO
        #
        # if voltage == 5:
        #     GPIO.output(POWER_ENABLE_5V_PIN, GPIO.HIGH)
        # elif voltage == 12:
        #     GPIO.output(POWER_ENABLE_12V_PIN, GPIO.HIGH)
        # else:
        #     return False
        #
        # return True

        self.node.get_logger().info(f"[HW] Enabling {voltage}V rail for {module_id}")
        return True  # Stub

    def disable_power_rail(self, module_id: str, voltage: int) -> bool:
        # GPIO.output(pin, GPIO.LOW)
        return True  # Stub

    def get_battery_voltage(self) -> float:
        """Read battery voltage via I2C ADC"""
        # Read from I2C ADC (e.g., ADS1115)
        # voltage = (adc_value / 32768.0) * 4.096 * voltage_divider_ratio
        return 12.0  # Stub

    def get_battery_current(self) -> float:
        """Read battery current via I2C current sensor"""
        # Read from INA219 or similar
        return 0.0  # Stub

    def detect_ir_beacon(self) -> Tuple[bool, float, float]:
        """Read IR beacon sensor via I2C"""
        # Example: Read from IR sensor array
        # data = self.i2c_bus.read_i2c_block_data(self.ir_sensor_addr, 0, 6)
        # Parse distance and angle from sensor data
        return (False, 0.0, 0.0)  # Stub

    def engage_mechanical_latch(self) -> bool:
        """Actuate servo to engage latch"""
        # import RPi.GPIO as GPIO
        # pwm = GPIO.PWM(LATCH_SERVO_PIN, 50)  # 50Hz
        # pwm.start(7.5)  # Engage position
        # time.sleep(0.5)
        # pwm.stop()
        return True  # Stub

    def release_mechanical_latch(self) -> bool:
        # pwm.start(2.5)  # Release position
        return True  # Stub

    def get_module_presence(self, slot: int) -> bool:
        """Read magnetic reed switch or hall sensor"""
        # GPIO.input(PRESENCE_DETECT_PIN)
        return False  # Stub
```

**Dock Integration:**
```python
# In dock_node_multi.py
from coven_core.hardware_interface import HardwareInterface
from coven_core.simulated_hardware import SimulatedHardware
from coven_core.cuberover_hardware import CubeRoverHardware

class DockMulti(Node):
    def __init__(self):
        # ... existing code ...

        # Select hardware implementation
        use_simulation = self.declare_parameter('use_sim_time', False).value

        if use_simulation:
            self.hardware = SimulatedHardware(self)
        else:
            self.hardware = CubeRoverHardware(self)

    def on_verify_rep(self, msg: String):
        # ... existing verification logic ...

        if ok:
            # Enable 12V rail via hardware abstraction
            success = self.hardware.enable_power_rail(module_id, voltage=12)

            if not success:
                self.get_logger().error(f"Failed to enable 12V for {module_id}")
                return
```

**Time Estimate:** 4 hours (stub implementations), +8 hours (full physical impl)
**Impact:** CRITICAL - Required for physical CubeRover deployment

---

#### 2.3 Physical Docking Behavior

**Why:** CubeRovers need to auto-align and mechanically latch to dock.

**Community Pattern:** Sensor-based docking similar to Create3 dock behavior.

**Files to Create:**
- `coven_core/docking_behavior.py`

**Implementation:**
```python
# coven_core/docking_behavior.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
from coven_core.hardware_interface import HardwareInterface
from typing import Tuple
import math

class DockingBehavior:
    """Physical docking behavior for CubeRover"""

    def __init__(
        self,
        node: Node,
        hardware: HardwareInterface,
        navigator: BasicNavigator
    ):
        self.node = node
        self.hw = hardware
        self.nav = navigator

        # Velocity publisher for fine control
        self.vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)

        # Parameters
        self.approach_speed = 0.1  # m/s
        self.alignment_tolerance = 0.05  # rad
        self.distance_tolerance = 0.05  # m

    def dock(self) -> bool:
        """
        Execute full docking sequence.

        Returns:
            True if docked successfully
        """
        self.node.get_logger().info("Starting docking sequence...")

        # Phase 1: Navigate to dock approach position
        if not self._navigate_to_approach():
            self.node.get_logger().error("Failed to reach dock approach position")
            return False

        # Phase 2: IR beacon fine alignment
        if not self._align_with_ir_beacon():
            self.node.get_logger().error("Failed IR beacon alignment")
            return False

        # Phase 3: Final approach
        if not self._final_approach():
            self.node.get_logger().error("Failed final approach")
            return False

        # Phase 4: Engage mechanical latch
        if not self._engage_latch():
            self.node.get_logger().error("Failed to engage latch")
            return False

        self.node.get_logger().info("Docking complete!")
        return True

    def _navigate_to_approach(self) -> bool:
        """Phase 1: Navigate to dock vicinity using Nav2"""
        # Get dock pose from TF2
        from tf2_ros import Buffer, TransformListener
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, self.node)

        try:
            transform = tf_buffer.lookup_transform(
                'map',
                'dock_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
        except Exception as e:
            self.node.get_logger().error(f"Failed to lookup dock pose: {e}")
            return False

        # Create approach goal (1m in front of dock)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = transform.transform.translation.x - 1.0  # 1m offset
        goal.pose.position.y = transform.transform.translation.y
        goal.pose.orientation = transform.transform.rotation  # Face dock

        # Navigate using Nav2
        self.nav.goToPose(goal)

        while not self.nav.isTaskComplete():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        return self.nav.getResult() == BasicNavigator.TaskResult.SUCCEEDED

    def _align_with_ir_beacon(self) -> bool:
        """Phase 2: Use IR beacon for fine alignment"""
        self.node.get_logger().info("Aligning with IR beacon...")

        max_iterations = 50
        for i in range(max_iterations):
            # Read IR beacon sensor
            detected, distance, angle = self.hw.detect_ir_beacon()

            if not detected:
                self.node.get_logger().warn("IR beacon not detected")
                continue

            # Check if aligned
            if abs(angle) < self.alignment_tolerance:
                self.node.get_logger().info("Alignment complete")
                return True

            # Send corrective twist
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = -angle * 0.5  # Proportional control
            self.vel_pub.publish(twist)

            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Stop
        self.vel_pub.publish(Twist())
        return False

    def _final_approach(self) -> bool:
        """Phase 3: Slow approach to contact"""
        self.node.get_logger().info("Final approach...")

        max_iterations = 100
        for i in range(max_iterations):
            detected, distance, angle = self.hw.detect_ir_beacon()

            if not detected:
                self.node.get_logger().warn("Lost IR beacon during approach")
                break

            # Check if close enough
            if distance < self.distance_tolerance:
                self.node.get_logger().info("Reached dock")
                self.vel_pub.publish(Twist())  # Stop
                return True

            # Send slow approach velocity
            twist = Twist()
            twist.linear.x = self.approach_speed
            twist.angular.z = -angle * 0.3  # Minor angle correction
            self.vel_pub.publish(twist)

            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Stop
        self.vel_pub.publish(Twist())
        return False

    def _engage_latch(self) -> bool:
        """Phase 4: Mechanical latch engagement"""
        self.node.get_logger().info("Engaging mechanical latch...")
        return self.hw.engage_mechanical_latch()

    def undock(self) -> bool:
        """
        Release latch and back away from dock.

        Returns:
            True if undocked successfully
        """
        self.node.get_logger().info("Undocking...")

        # Release latch
        if not self.hw.release_mechanical_latch():
            self.node.get_logger().error("Failed to release latch")
            return False

        # Back away slowly
        twist = Twist()
        twist.linear.x = -0.1  # Reverse

        for _ in range(20):  # 2 seconds at 10Hz
            self.vel_pub.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Stop
        self.vel_pub.publish(Twist())

        self.node.get_logger().info("Undocking complete")
        return True
```

**Module Integration:**
```python
# In module_node.py
from coven_core.docking_behavior import DockingBehavior

class Module(Node):
    def __init__(self):
        # ... existing code ...

        # Create docking behavior
        self.docking = DockingBehavior(
            node=self,
            hardware=self.hardware,  # From HAL
            navigator=self.navigator  # From Nav2
        )

    def return_to_dock(self):
        """Modified to use physical docking"""
        self.get_logger().info("Returning to dock...")

        # Use physical docking behavior
        success = self.docking.dock()

        if success:
            self.get_logger().info("Successfully docked")
            self.state = ModuleState.NORMAL
        else:
            self.get_logger().error("Docking failed")
            # Fallback: return to NORMAL anyway
            self.state = ModuleState.NORMAL
```

**Time Estimate:** 4 hours (implementation), +4 hours (tuning/testing)
**Impact:** CRITICAL - Required for autonomous CubeRover operation

---

### TIER 3: Robustness & Testing (2-3 days)

#### 3.1 Comprehensive Unit Tests

(See TIER 1.4 for test_common.py)

Additional test files:

```python
# test/test_fsm_transitions.py
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from coven_core.module_node import Module
from coven_core.common import ModuleState, IdentifyReq, VerifyReq
from std_msgs.msg import String
import json

@pytest.fixture
def module_node():
    rclpy.init()
    node = Module(module_id="TEST-001")
    yield node
    node.destroy_node()
    rclpy.shutdown()

def test_boot_to_wait_verify(module_node):
    """Test BOOT → WAIT_VERIFY transition"""
    assert module_node.state == ModuleState.BOOT

    # Simulate IDENTIFY_REQ
    ident_req = IdentifyReq(dock_id="DOCK-TEST", broadcast="all")
    msg = String(data=json.dumps({
        "dock_id": ident_req.dock_id,
        "broadcast": ident_req.broadcast
    }))

    module_node.on_ident_req(msg)

    # Should transition to WAIT_VERIFY
    assert module_node.state == ModuleState.WAIT_VERIFY

def test_wait_verify_to_normal(module_node):
    """Test WAIT_VERIFY → NORMAL transition"""
    # Setup: module in WAIT_VERIFY
    module_node.state = ModuleState.WAIT_VERIFY
    module_node.module_id = "TEST-001"

    # Simulate 12V enable
    enable_msg = String(data=json.dumps({
        "module_id": "TEST-001",
        "data": True
    }))

    module_node.on_enable_12v(enable_msg)

    # Should transition to NORMAL
    assert module_node.state == ModuleState.NORMAL

def test_heartbeat_timeout_recovery(module_node):
    """Test heartbeat recovery after 1-2 missed beats"""
    # TODO: Implement heartbeat testing
    pass
```

**Time Estimate:** 3 hours
**Impact:** MEDIUM - Catch bugs early, enable refactoring

---

#### 3.2 Improved Error Handling

**Files to Modify:**
- `coven_core/common.py`
- `coven_core/module_node.py`
- `coven_core/dock_node_multi.py`

**Implementation:**
```python
# In common.py - add logging
import logging

# Module-level logger
logger = logging.getLogger(__name__)

def ident_rep_decode(msg: String) -> Optional[IdentifyRep]:
    """Decode IdentifyRep message with error logging"""
    try:
        d = json.loads(msg.data)
        return IdentifyRep(
            module_id=d["module_id"],
            module_type=d["module_type"],
            firmware_version=d["firmware_version"]
        )
    except json.JSONDecodeError as e:
        logger.error(f"Failed to decode IdentifyRep - invalid JSON: {e}")
        logger.debug(f"Raw message: {msg.data}")
        return None
    except KeyError as e:
        logger.error(f"Failed to decode IdentifyRep - missing field: {e}")
        logger.debug(f"Parsed data: {d}")
        return None
    except Exception as e:
        logger.error(f"Unexpected error decoding IdentifyRep: {e}")
        return None
```

```python
# In module_node.py - better error handling
def execute_task(self, task: str):
    """Execute task with comprehensive error handling"""
    try:
        self._start_task_watchdog()

        if task == "explore_warehouse":
            # Initialize navigation with timeout
            if not self._initialize_navigation_with_retry(max_retries=3):
                raise RuntimeError("Failed to initialize Nav2 after 3 attempts")

            # Execute exploration
            success, metrics = self.explorer.explore(duration=self.task_timeout)

            if not success:
                raise RuntimeError(f"Exploration failed: {metrics.get('error', 'Unknown')}")

            # Save map with error checking
            map_data, map_yaml = self._save_and_serialize_map()
            if not map_data or not map_yaml:
                raise RuntimeError("Failed to save/serialize map")

            # Send completion
            self._send_task_complete(
                success=True,
                note=f"Explored {metrics.get('coverage', 0)*100:.1f}%",
                map_data=map_data,
                map_yaml=map_yaml,
                exploration_metrics=metrics
            )

        else:
            raise ValueError(f"Unknown task: {task}")

    except RuntimeError as e:
        self.get_logger().error(f"Task execution error: {e}")
        self._send_task_complete(success=False, note=str(e))

    except Exception as e:
        self.get_logger().error(f"Unexpected error during task: {e}", exc_info=True)
        self._send_task_complete(success=False, note=f"Internal error: {type(e).__name__}")

    finally:
        self._stop_task_watchdog()
        self.state = ModuleState.NORMAL

def _initialize_navigation_with_retry(self, max_retries: int = 3) -> bool:
    """Initialize Nav2 with retry logic"""
    for attempt in range(max_retries):
        try:
            self._initialize_navigation()
            return True
        except RuntimeError as e:
            self.get_logger().warn(
                f"Nav2 init attempt {attempt+1}/{max_retries} failed: {e}"
            )
            if attempt < max_retries - 1:
                time.sleep(2.0)  # Wait before retry
            else:
                return False

    return False
```

**Time Estimate:** 4 hours
**Impact:** HIGH - Prevents silent failures, easier debugging

---

### TIER 4: Community Integration (3-4 days)

#### 4.1 m-explore-ros2 Integration

**Why:** Battle-tested exploration (5+ years in ROS1, ported to ROS2).

**Package:** https://github.com/robo-friends/m-explore-ros2

**Installation:**
```bash
cd ~/ros2_ws/src
git clone https://github.com/robo-friends/m-explore-ros2.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select explore_lite
```

**Configuration:**
```yaml
# config/m_explore.yaml
explore:
  ros__parameters:
    robot_base_frame: base_link
    costmap_topic: /map
    costmap_updates_topic: /map_updates
    visualize: true
    planner_frequency: 0.33
    progress_timeout: 30.0
    potential_scale: 3.0
    orientation_scale: 0.0
    gain_scale: 1.0
    transform_tolerance: 0.3
    min_frontier_size: 10
```

**Module Integration:**
```python
# In module_node.py
def execute_task(self, task: str):
    if task == "explore_warehouse":
        # Launch m-explore instead of custom Explorer
        self._launch_m_explore()

        # Monitor for completion
        timeout = self.task_timeout
        start_time = time.time()

        while time.time() - start_time < timeout:
            # Check coverage via /map topic
            if self._get_coverage() > 0.80:
                break
            time.sleep(1.0)

        # Save map
        map_data, map_yaml = self._save_and_serialize_map()

        self._send_task_complete(...)
```

**Time Estimate:** 3 hours
**Impact:** MEDIUM - More robust exploration, community-maintained

---

#### 4.2 robot_localization for Sensor Fusion

**Why:** Fuse IMU + wheel odometry for better pose estimates.

**Package:** ros-humble-robot-localization

**Installation:**
```bash
sudo apt install ros-humble-robot-localization
```

**Configuration:**
```yaml
# config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: true
    transform_timeout: 0.0
    print_diagnostics: true
    debug: false

    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    odom0: /odom
    odom0_config: [true,  true,  false,  # x, y, z
                   false, false, true,   # roll, pitch, yaw
                   true,  true,  false,  # vx, vy, vz
                   false, false, true,   # vroll, vpitch, vyaw
                   false, false, false]  # ax, ay, az

    imu0: /imu
    imu0_config: [false, false, false,  # x, y, z
                  false, false, true,   # roll, pitch, yaw
                  false, false, false,  # vx, vy, vz
                  false, false, true,   # vroll, vpitch, vyaw
                  true,  true,  true]   # ax, ay, az

    imu0_differential: false
    imu0_relative: true
```

**Launch File:**
```python
# In coven.launch.py
Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[ekf_config_file],
    remappings=[('odometry/filtered', '/odom_filtered')]
)
```

**Time Estimate:** 2 hours
**Impact:** MEDIUM - Better localization, reduced drift

---

### TIER 5: Advanced Features (Future)

These are for later phases, not critical for initial deployment.

#### 5.1 Open-RMF Integration

**When:** Multi-dock scenarios, warehouse-scale deployments.

**Package:** https://github.com/open-rmf

---

#### 5.2 BehaviorTree.CPP

**When:** Complex mission logic beyond simple FSM.

**Package:** https://github.com/BehaviorTree/BehaviorTree.CPP

---

## Key Research Findings

### ROS2 Best Practices (2025)

1. **Multi-Robot Coordination:**
   - Open-RMF is the industry standard for fleet management
   - Use DDS for communication (default in ROS2)
   - Zenoh considered for large-scale WiFi deployments
   - Per-robot namespaces recommended for distributed systems

2. **Topic vs Service vs Action:**
   - **Topics:** Continuous data streams (sensors, state)
   - **Services:** Quick request/response (< 1s)
   - **Actions:** Long-running tasks with feedback/cancellation (> 1s)

3. **Parameters:**
   - YAML files in `config/` folder
   - Use `/**` wildcard for multi-namespace parameters
   - Must declare parameters in node before use
   - Load via `--params-file` argument

4. **Exploration:**
   - m-explore-ros2 is the standard (explore_lite port)
   - Frontier-based is still dominant approach
   - Integrate with Nav2 + SLAM Toolbox

5. **Sensor Fusion:**
   - robot_localization (EKF) for IMU + odometry fusion
   - Significantly reduces drift

---

## File Structure Summary

### New Directories
```
coven_core/
├── config/                    # NEW - ROS2 parameter files
│   ├── coven_params.yaml
│   ├── dock_params.yaml
│   ├── exploration_params.yaml
│   ├── ekf.yaml
│   └── m_explore.yaml
```

### New Python Modules
```
coven_core/coven_core/
├── hardware_interface.py      # NEW - HAL abstraction
├── simulated_hardware.py      # NEW - Gazebo HAL impl
├── cuberover_hardware.py      # NEW - Physical HAL impl
├── docking_behavior.py        # NEW - Physical docking
└── safety_monitor.py          # NEW - Watchdogs (future)
```

### New Interfaces Package
```
coven_interfaces/              # NEW PACKAGE
├── action/
│   └── ExecuteTask.action
├── msg/
│   └── CovenMetrics.msg       # Future: telemetry
├── CMakeLists.txt
└── package.xml
```

### New Tests
```
coven_core/test/
├── test_common.py             # NEW - Message encode/decode
├── test_fsm.py                # NEW - State transitions
└── test_integration.py        # NEW - Full protocol (future)
```

---

## Implementation Priority

### Week 1: Foundation (Quick Wins)
- [ ] Add ROS2 parameters (2h)
- [ ] Add watchdog timers (1h)
- [ ] Use TF2 for poses (2h)
- [ ] Unit tests for common.py (3h)
- [ ] Improve error handling (4h)

**Total:** ~12 hours (1.5 days)

### Week 2: Hardware Readiness
- [ ] Create HAL interface (4h)
- [ ] Implement SimulatedHardware (2h)
- [ ] Stub CubeRoverHardware (2h)
- [ ] Physical docking behavior (4h)
- [ ] Convert to Action servers (6h)

**Total:** ~18 hours (2.5 days)

### Week 3: Polish & Testing
- [ ] FSM integration tests (4h)
- [ ] End-to-end testing (4h)
- [ ] Documentation updates (2h)
- [ ] Performance tuning (4h)
- [ ] Code review & cleanup (2h)

**Total:** ~16 hours (2 days)

---

## Success Criteria

### Phase 1 Complete (Simulation) ✅
- [x] Multi-robot coordination working (10+ modules)
- [x] Frontier exploration functional
- [x] Map serialization and transfer
- [x] Heartbeat monitoring with recovery
- [x] Comprehensive documentation

### Phase 2 Complete (Hardware Ready) 🎯
- [ ] ROS2 parameters implemented
- [ ] Action servers replace topics
- [ ] HAL abstraction in place
- [ ] Physical docking behavior implemented
- [ ] Watchdog timers active
- [ ] Unit test coverage > 80%
- [ ] TF2 for all pose tracking

### Phase 3 Complete (Field Deployable) 🚀
- [ ] Physical hardware tested
- [ ] IR beacon docking validated
- [ ] Battery monitoring active
- [ ] Safety systems operational
- [ ] Multi-CubeRover field test passed
- [ ] Integration tests passing

---

## References

### Community Packages
- **Open-RMF:** https://github.com/open-rmf
- **m-explore-ros2:** https://github.com/robo-friends/m-explore-ros2
- **robot_localization:** https://github.com/cra-ros-pkg/robot_localization
- **BehaviorTree.CPP:** https://github.com/BehaviorTree/BehaviorTree.CPP

### Documentation
- **ROS2 Multi-Robot Book:** https://osrf.github.io/ros2multirobotbook/
- **Nav2 Documentation:** https://navigation.ros.org/
- **SLAM Toolbox:** https://github.com/SteveMacenski/slam_toolbox

### Best Practices
- **Topics vs Services vs Actions:** https://docs.ros.org/en/foxy/How-To-Guides/Topics-Services-Actions.html
- **ROS2 Parameters:** https://roboticsbackend.com/ros2-yaml-params/
- **Launch Files:** https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html

---

## Contact & Support

**Thesis Advisor:** Frankie (Zhu)
**Institution:** [University Name]
**Date:** November 2025
**Repository:** [GitHub URL]

---

*This roadmap will be updated as implementation progresses.*
