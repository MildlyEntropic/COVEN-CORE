// SPDX-License-Identifier: MIT
//! subsumption.rs — COVEN Subsumption Architecture
//!
//! Layered reactive behavioral architecture per Brooks (1986), adapted for
//! COVEN rover autonomy. Higher-numbered layers subsume (override) lower
//! layers when active; lower layers serve as automatic fallbacks.
//!
//! ## Layer Hierarchy (per thesis Section 2.2.3)
//!
//! | Priority | Layer              | Role                                      |
//! |----------|--------------------|-------------------------------------------|
//! | L0       | Obstacle Avoidance | LiDAR safety — always active, constrains  |
//! | L1       | Return to Dock     | Default fallback — come home               |
//! | L2       | Wander & Collect   | Opportunistic data — scenic return         |
//! | L3       | Navigate to Goal   | Potential field to dock-assigned frontier   |
//! | L4       | Execute Task       | Polymorphic — varies by rover type          |
//!
//! ## Degradation Cascade
//!
//! Task fail → hold position (L3), nav fail → wander (L2),
//! resource depletion → return to dock (L1), with obstacle avoidance (L0)
//! active throughout. Every failure mode resolves to a safe behavior.
//!
//! ## Design Notes
//!
//! - L0 is always-on: if an obstacle is within `d_safe`, ALL motion stops.
//! - L4–L1 are evaluated highest-priority first; first active layer wins.
//! - L3 and L4 pass through the pre-computed WaypointFollower velocity.
//! - L2 computes gap-following velocity from LiDAR for opportunistic wandering.
//! - L1 owns a LyapunovNavigator that drives toward the dock.
//! - L4 is the polymorphic layer: for a mapping rover it's a pass-through;
//!   future rover types (drill, spectrometer) override with task-specific behavior.
//!
//! Author: Alexander Shultis
//! Date: March 2026

use crate::navigation::{LyapunovNavigator, NavParams, VelocityCmd};

// ------------------------
// --- Data Structures ---
// ------------------------

/// Output from a behavior layer evaluation.
#[derive(Debug, Clone)]
pub struct LayerOutput {
    /// Velocity command (None = this layer has no opinion).
    pub cmd: Option<VelocityCmd>,
    /// Whether this layer is actively asserting control.
    pub active: bool,
}

impl LayerOutput {
    /// Layer is inactive — no command, doesn't suppress lower layers.
    pub fn inactive() -> Self {
        Self {
            cmd: None,
            active: false,
        }
    }

    /// Layer is active and emitting a command — suppresses all lower layers.
    pub fn suppress(cmd: VelocityCmd) -> Self {
        Self {
            cmd: Some(cmd),
            active: true,
        }
    }
}

/// Sensor and state context passed to each behavior layer for evaluation.
///
/// Built by the state machine each control loop iteration from odometry,
/// LiDAR, battery, and navigation state.
#[derive(Debug, Clone)]
pub struct LayerContext {
    /// Robot X position in meters (from odometry).
    pub robot_x: f64,
    /// Robot Y position in meters (from odometry).
    pub robot_y: f64,
    /// Robot heading in radians (from odometry).
    pub robot_theta: f64,

    /// LiDAR range readings in meters (f64 to match navigator API).
    pub lidar_ranges: Vec<f64>,
    /// Minimum valid LiDAR range in current scan (meters).
    pub min_range: f64,
    /// Start angle of LiDAR scan (radians).
    pub lidar_angle_min: f64,
    /// Angle between consecutive LiDAR rays (radians).
    pub lidar_angle_increment: f64,

    /// Minimum safe distance threshold (meters).
    pub d_safe: f64,

    /// Battery level percentage (0–100).
    pub battery_pct: f64,
    /// Battery threshold below which rover should return to dock.
    pub low_battery_threshold: f64,

    /// Pre-computed velocity from WaypointFollower (None if no scan or no goal).
    pub nav_cmd: Option<VelocityCmd>,
    /// Whether WaypointFollower is actively navigating toward a goal.
    pub has_goal: bool,

    /// Whether a dock-assigned mission is active.
    pub has_mission: bool,

    /// Dock X position in meters (for L1 return-to-dock).
    pub dock_x: f64,
    /// Dock Y position in meters (for L1 return-to-dock).
    pub dock_y: f64,
}

// ------------------------
// --- Behavior Trait ---
// ------------------------

/// A behavior layer in the subsumption hierarchy.
pub trait BehaviorLayer {
    /// Human-readable name for logging/debugging.
    #[allow(dead_code)]
    fn name(&self) -> &'static str;

    /// Priority (lower number = evaluated first by arbiter).
    fn priority(&self) -> u8;

    /// Evaluate this layer given current sensor context.
    fn evaluate(&mut self, ctx: &LayerContext) -> LayerOutput;
}

// ------------------------
// --- L0: Obstacle Avoidance ---
// ------------------------

/// L0 — Obstacle avoidance (always active, constrains all layers).
///
/// The foundational layer. When the closest LiDAR reading drops below
/// `d_safe`, all motion is suppressed with a hard stop. This layer
/// cannot be overridden — every other behavior is constrained by
/// collision safety.
pub struct ObstacleAvoidanceLayer;

impl BehaviorLayer for ObstacleAvoidanceLayer {
    fn name(&self) -> &'static str {
        "L0:ObstacleAvoidance"
    }

    fn priority(&self) -> u8 {
        0
    }

    fn evaluate(&mut self, ctx: &LayerContext) -> LayerOutput {
        if ctx.min_range > 0.0 && ctx.min_range < ctx.d_safe {
            LayerOutput::suppress(VelocityCmd::stop())
        } else {
            LayerOutput::inactive()
        }
    }
}

// ------------------------
// --- L4: Execute Task ---
// ------------------------

/// L4 — Execute task (highest behavioral priority, polymorphic).
///
/// For a mapping/reconnaissance rover, task execution IS navigation with
/// continuous LiDAR scanning — the sensor batch pipeline runs regardless
/// of which layer controls velocity. L4 passes through the WaypointFollower
/// velocity during an active mission.
///
/// Future rover types (drill, spectrometer) would override this layer with
/// task-specific behavior at the goal coordinate. This is where OOP
/// polymorphism manifests at the behavioral level.
pub struct ExecuteTaskLayer;

impl BehaviorLayer for ExecuteTaskLayer {
    fn name(&self) -> &'static str {
        "L4:ExecuteTask"
    }

    fn priority(&self) -> u8 {
        1
    }

    fn evaluate(&mut self, ctx: &LayerContext) -> LayerOutput {
        if ctx.has_mission && ctx.has_goal {
            if let Some(cmd) = ctx.nav_cmd {
                return LayerOutput::suppress(cmd);
            }
        }
        LayerOutput::inactive()
    }
}

// ------------------------
// --- L3: Navigate to Goal ---
// ------------------------

/// L3 — Navigate to dock-assigned frontier via potential field.
///
/// Active when a goal exists (even without an active mission — e.g.,
/// a rover directed to a coordinate for any reason). Passes through
/// the pre-computed WaypointFollower velocity.
pub struct NavigateToGoalLayer;

impl BehaviorLayer for NavigateToGoalLayer {
    fn name(&self) -> &'static str {
        "L3:NavigateToGoal"
    }

    fn priority(&self) -> u8 {
        2
    }

    fn evaluate(&mut self, ctx: &LayerContext) -> LayerOutput {
        if ctx.has_goal {
            if let Some(cmd) = ctx.nav_cmd {
                return LayerOutput::suppress(cmd);
            }
        }
        LayerOutput::inactive()
    }
}

// ------------------------
// --- L2: Wander & Collect ---
// ------------------------

/// L2 — Wander and collect opportunistic sensor data.
///
/// When no goal is assigned but the rover has adequate battery and
/// functional sensors, it wanders by following the widest gap in
/// LiDAR — exploring unvisited space while collecting scans that
/// contribute to the global map. This transforms dead-heading into
/// productive data collection.
pub struct WanderCollectLayer {
    /// Maximum linear velocity for wandering (m/s).
    max_linear: f64,
    /// Maximum angular velocity for wandering (rad/s).
    max_angular: f64,
}

impl WanderCollectLayer {
    /// Create a new wander layer with velocity limits from NavParams.
    pub fn new(params: &NavParams) -> Self {
        Self {
            // Wander at 60% of max speed — cautious exploration
            max_linear: params.max_linear * 0.6,
            max_angular: params.max_angular,
        }
    }

    /// Find the widest gap in LiDAR and compute a velocity toward it.
    fn gap_follow(&self, ctx: &LayerContext) -> VelocityCmd {
        let ranges = &ctx.lidar_ranges;
        if ranges.is_empty() {
            return VelocityCmd::stop();
        }

        // Find the widest contiguous gap (ranges > d_safe)
        let mut best_start = 0;
        let mut best_len = 0;
        let mut cur_start = 0;
        let mut cur_len = 0;

        for (i, &r) in ranges.iter().enumerate() {
            if r.is_finite() && r > ctx.d_safe {
                if cur_len == 0 {
                    cur_start = i;
                }
                cur_len += 1;
                if cur_len > best_len {
                    best_start = cur_start;
                    best_len = cur_len;
                }
            } else {
                cur_len = 0;
            }
        }

        // No safe gap found — stop
        if best_len == 0 {
            return VelocityCmd::stop();
        }

        // Compute bearing to gap center
        let gap_center_idx = best_start + best_len / 2;
        let gap_angle = ctx.lidar_angle_min
            + (gap_center_idx as f64) * ctx.lidar_angle_increment;

        // Mean depth of gap (for speed modulation)
        let mean_depth: f64 = ranges[best_start..best_start + best_len]
            .iter()
            .filter(|r| r.is_finite())
            .sum::<f64>()
            / best_len as f64;

        // Angular: steer toward gap center
        let angular = (gap_angle * self.max_angular).clamp(-self.max_angular, self.max_angular);

        // Linear: proportional to clearance, reduced when turning hard
        let turn_factor = 1.0 - (angular.abs() / self.max_angular).min(1.0);
        let depth_factor = (mean_depth / 2.0).min(1.0); // Full speed at 2m+ clearance
        let linear = self.max_linear * turn_factor * depth_factor;

        VelocityCmd { linear, angular }
    }
}

impl BehaviorLayer for WanderCollectLayer {
    fn name(&self) -> &'static str {
        "L2:WanderCollect"
    }

    fn priority(&self) -> u8 {
        3
    }

    fn evaluate(&mut self, ctx: &LayerContext) -> LayerOutput {
        // Active when: no goal, adequate battery, have LiDAR data
        if !ctx.has_goal
            && ctx.battery_pct > ctx.low_battery_threshold
            && !ctx.lidar_ranges.is_empty()
        {
            let cmd = self.gap_follow(ctx);
            LayerOutput::suppress(cmd)
        } else {
            LayerOutput::inactive()
        }
    }
}

// ------------------------
// --- L1: Return to Dock ---
// ------------------------

/// L1 — Return to dock (lowest priority, always-on fallback).
///
/// A rover with no active task, exhausted sensors, or insufficient
/// battery navigates toward the dock. This is the rover's baseline
/// state: when all higher-level purposes are exhausted, it returns
/// home with whatever data it has collected.
pub struct ReturnToDockLayer {
    /// Navigator for computing velocity toward dock.
    navigator: LyapunovNavigator,
}

impl ReturnToDockLayer {
    /// Create a new return-to-dock layer with navigation parameters.
    pub fn new(params: NavParams) -> Self {
        Self {
            navigator: LyapunovNavigator::with_params(params),
        }
    }
}

impl BehaviorLayer for ReturnToDockLayer {
    fn name(&self) -> &'static str {
        "L1:ReturnToDock"
    }

    fn priority(&self) -> u8 {
        4
    }

    fn evaluate(&mut self, ctx: &LayerContext) -> LayerOutput {
        // Need LiDAR to navigate safely
        if ctx.lidar_ranges.is_empty() {
            return LayerOutput::inactive();
        }

        // Set dock as goal
        self.navigator.set_goal(ctx.dock_x, ctx.dock_y);

        // Already at dock — no need to move
        if self.navigator.goal_reached(ctx.robot_x, ctx.robot_y) {
            return LayerOutput::inactive();
        }

        // Navigate toward dock
        let cmd = self.navigator.compute_velocity(
            ctx.robot_x,
            ctx.robot_y,
            ctx.robot_theta,
            &ctx.lidar_ranges,
            ctx.lidar_angle_min,
            ctx.lidar_angle_increment,
        );

        LayerOutput::suppress(cmd)
    }
}

// ------------------------
// --- Arbiter ---
// ------------------------

/// Subsumption arbiter — evaluates layers by priority, first active wins.
///
/// Layers are evaluated in priority order (L0 first, then L4, L3, L2, L1).
/// L0 (obstacle avoidance) is the emergency override — if active, it
/// suppresses everything. Otherwise, the first active layer among L4–L1
/// wins and its velocity command is used.
///
/// If no layer is active (e.g., no LiDAR data), the arbiter returns
/// `VelocityCmd::stop()` — the safest default.
pub struct SubsumptionArbiter {
    layers: Vec<Box<dyn BehaviorLayer>>,
}

impl SubsumptionArbiter {
    /// Create a new arbiter with the COVEN layer stack.
    pub fn new(params: NavParams) -> Self {
        let mut layers: Vec<Box<dyn BehaviorLayer>> = vec![
            Box::new(ObstacleAvoidanceLayer),            // L0: priority 0
            Box::new(ExecuteTaskLayer),                  // L4: priority 1
            Box::new(NavigateToGoalLayer),               // L3: priority 2
            Box::new(WanderCollectLayer::new(&params)),  // L2: priority 3
            Box::new(ReturnToDockLayer::new(params)),    // L1: priority 4
        ];
        // Sort by priority (should already be in order, but enforce it)
        layers.sort_by_key(|l| l.priority());
        Self { layers }
    }

    /// Evaluate all layers and return the winning velocity command.
    ///
    /// Returns the command from the highest-priority active layer,
    /// or `VelocityCmd::stop()` if no layer is active.
    pub fn evaluate(&mut self, ctx: &LayerContext) -> VelocityCmd {
        for layer in &mut self.layers {
            let output = layer.evaluate(ctx);
            if output.active {
                if let Some(cmd) = output.cmd {
                    return cmd;
                }
            }
        }
        // No layer active — stop (safe default)
        VelocityCmd::stop()
    }
}
