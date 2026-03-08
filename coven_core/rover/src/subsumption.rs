// SPDX-License-Identifier: MIT
//! subsumption.rs — COVEN Subsumption Architecture
//!
//! Layered reactive behavioral architecture per Brooks (1986), adapted for
//! COVEN rover autonomy. Layers compose — each builds on the layers below it.
//! Lower layers are always running; higher layers selectively override or
//! absorb parts of lower layers' output.
//!
//! ## Layer Hierarchy (per thesis Section 2.2.3)
//!
//! | Layer | Role                   | Composition                              |
//! |-------|------------------------|------------------------------------------|
//! | L0    | Obstacle Avoidance     | Repulsive potential field — always active |
//! | L1    | Return to Dock         | Attractive field toward dock (baseline)   |
//! | L2    | Wander & Collect       | Gap-following, overrides L1 when active   |
//! | L3    | Navigate to Goal       | Attractive field toward assigned frontier  |
//! | L4    | Execute Task           | Polymorphic — varies by rover type         |
//!
//! ## Thesis Decomposition
//!
//! The Lyapunov potential field is decomposed across layers:
//! - L0 = repulsive component (always active, constrains all)
//! - L1/L2/L3 = which attractive target to use (priority arbitrated)
//! - L4 = polymorphic task behavior on top
//!
//! Per thesis: "the attractive component implements L3 (navigate to goal)
//! while the repulsive component implements L0 (obstacle avoidance)."
//!
//! ## Arbiter Model
//!
//! 1. Always compute L0 repulsive velocity from LiDAR
//! 2. Evaluate L4→L3→L2→L1 for behavioral velocity (first active wins)
//! 3. Combine: final = behavioral + L0 repulsive, clamped to limits
//! 4. Emergency: if within d_safe, L0 overrides everything with escape velocity
//!
//! Author: Alexander Shultis
//! Date: March 2026

use crate::navigation::{LyapunovNavigator, NavParams, VelocityCmd};

// ------------------------
// --- Data Structures ---
// ------------------------

/// Output from a behavioral layer evaluation (L1–L4).
#[derive(Debug, Clone)]
pub struct LayerOutput {
    /// Velocity command (None = this layer has no opinion).
    pub cmd: Option<VelocityCmd>,
    /// Whether this layer is actively asserting control.
    pub active: bool,
}

impl LayerOutput {
    /// Layer is inactive — no command, doesn't affect output.
    pub fn inactive() -> Self {
        Self {
            cmd: None,
            active: false,
        }
    }

    /// Layer is active and emitting a command.
    pub fn active(cmd: VelocityCmd) -> Self {
        Self {
            cmd: Some(cmd),
            active: true,
        }
    }
}

/// Output from L0 obstacle avoidance — always computed.
#[derive(Debug, Clone, Copy)]
pub enum ObstacleOutput {
    /// No obstacles within influence distance — no constraint.
    Clear,
    /// Obstacles within d_influence — repulsive velocity to sum with behavioral.
    Constrain(VelocityCmd),
    /// Obstacle within d_safe — EMERGENCY override, replaces everything.
    Emergency(VelocityCmd),
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

    /// Pre-computed attractive velocity from WaypointFollower (goal-seeking only).
    pub nav_cmd: VelocityCmd,
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

/// A behavioral layer in the subsumption hierarchy (L1–L4).
///
/// L0 is handled separately by the arbiter since it composes with
/// (rather than competes against) the behavioral layers.
pub trait BehaviorLayer {
    /// Human-readable name for logging/debugging.
    #[allow(dead_code)]
    fn name(&self) -> &'static str;

    /// Priority (lower number = evaluated first by arbiter).
    fn priority(&self) -> u8;

    /// Evaluate this layer given current sensor context.
    fn evaluate(&mut self, ctx: &LayerContext) -> LayerOutput;
}

// ----------------------------------------
// --- L0: Obstacle Avoidance (Repulsive) ---
// ----------------------------------------

/// L0 — Obstacle avoidance via Lyapunov repulsive potential field.
///
/// Per thesis: "The foundational layer. This layer processes LiDAR readings
/// and generates repulsive responses to nearby surfaces. It is always active
/// and cannot be overridden — every other behavior is constrained by
/// collision safety."
///
/// L0 IS the repulsive component of the Lyapunov potential field. It is
/// always computed and always applied. The arbiter sums L0's repulsive
/// velocity with the winning behavioral layer's attractive velocity.
///
/// Two modes:
/// - **Constrain** (d_safe < obstacle < d_influence): repulsive velocity
///   summed with behavioral velocity — smoothly deflects the rover.
/// - **Emergency** (obstacle < d_safe): overrides everything with escape
///   velocity — turns toward clearest direction and backs away.
pub struct ObstacleAvoidanceLayer {
    /// Navigator instance for repulsive force computation.
    /// Uses the same potential field math as the Lyapunov navigator.
    navigator: LyapunovNavigator,
}

impl ObstacleAvoidanceLayer {
    /// Create L0 with navigation parameters.
    fn new(params: NavParams) -> Self {
        Self {
            navigator: LyapunovNavigator::with_params(params),
        }
    }

    /// Compute L0 repulsive output from current LiDAR data.
    ///
    /// Always called by the arbiter — L0 is never "inactive."
    fn evaluate(&self, ctx: &LayerContext) -> ObstacleOutput {
        if ctx.lidar_ranges.is_empty() {
            return ObstacleOutput::Clear;
        }

        // Emergency: obstacle within d_safe — override everything
        if ctx.min_range > 0.0 && ctx.min_range < ctx.d_safe {
            let escape = self.navigator.compute_escape_velocity(
                ctx.robot_theta,
                &ctx.lidar_ranges,
                ctx.lidar_angle_min,
                ctx.lidar_angle_increment,
            );
            return ObstacleOutput::Emergency(escape);
        }

        // Compute repulsive force from all obstacles within d_influence
        let (f_rep_x, f_rep_y) = self.navigator.compute_repulsive_force(
            ctx.robot_theta,
            &ctx.lidar_ranges,
            ctx.lidar_angle_min,
            ctx.lidar_angle_increment,
        );

        // If repulsive force is negligible, no constraint needed
        let force_mag = (f_rep_x * f_rep_x + f_rep_y * f_rep_y).sqrt();
        if force_mag < 0.01 {
            return ObstacleOutput::Clear;
        }

        // Convert repulsive force (world frame) to velocity (robot frame)
        let cos_theta = ctx.robot_theta.cos();
        let sin_theta = ctx.robot_theta.sin();
        let f_robot_x = f_rep_x * cos_theta + f_rep_y * sin_theta;
        let f_robot_y = -f_rep_x * sin_theta + f_rep_y * cos_theta;

        let desired_heading = f_robot_y.atan2(f_robot_x);

        // Linear: forward component of repulsive force
        // Can be negative (backing away from obstacle ahead)
        let params = self.navigator.params();
        let linear = f_robot_x.clamp(-params.max_linear * 0.5, params.max_linear);

        // Angular: steer away from obstacles
        let angular = (params.k_heading * desired_heading)
            .clamp(-params.max_angular, params.max_angular);

        ObstacleOutput::Constrain(VelocityCmd { linear, angular })
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
/// attractive velocity during an active mission.
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
            return LayerOutput::active(ctx.nav_cmd);
        }
        LayerOutput::inactive()
    }
}

// ------------------------
// --- L3: Navigate to Goal ---
// ------------------------

/// L3 — Navigate to dock-assigned frontier via attractive potential.
///
/// Per thesis: "the attractive component implements L3 (navigate to goal)."
/// Active when a goal exists (even without an active mission). Passes
/// through the WaypointFollower's attractive-only velocity.
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
            return LayerOutput::active(ctx.nav_cmd);
        }
        LayerOutput::inactive()
    }
}

// ------------------------
// --- L2: Wander & Collect ---
// ------------------------

/// L2 — Wander and collect opportunistic sensor data.
///
/// Per thesis: "this layer replaces direct return-to-dock with undirected
/// exploration. The rover is still broadly returning to the dock, but it
/// takes the scenic route — and every meter of that route produces LiDAR
/// scans that contribute to the global map."
///
/// Overrides L1's direct dock-return with gap-following exploration.
/// L0's repulsive field still constrains motion (applied by arbiter).
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
            LayerOutput::active(cmd)
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
/// Per thesis: "L1 sets the dock as the attractor." Provides the attractive
/// component toward the dock position. L0 handles obstacle avoidance
/// (repulsive component) via the arbiter's composition step.
pub struct ReturnToDockLayer {
    /// Navigator for computing attractive velocity toward dock.
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
        // Need LiDAR to navigate safely (L0 needs it for obstacle avoidance)
        if ctx.lidar_ranges.is_empty() {
            return LayerOutput::inactive();
        }

        // Set dock as goal
        self.navigator.set_goal(ctx.dock_x, ctx.dock_y);

        // Already at dock — no need to move
        if self.navigator.goal_reached(ctx.robot_x, ctx.robot_y) {
            return LayerOutput::inactive();
        }

        // Compute attractive-only velocity toward dock
        let cmd = self.navigator.compute_attractive_velocity(
            ctx.robot_x,
            ctx.robot_y,
            ctx.robot_theta,
        );

        LayerOutput::active(cmd)
    }
}

// ------------------------
// --- Arbiter ---
// ------------------------

/// Subsumption arbiter — composes layers per Brooks (1986).
///
/// Per thesis: the Lyapunov potential field is decomposed across layers.
/// L0 (repulsive) is always active and constrains all behavioral layers.
/// L4→L3→L2→L1 are evaluated by priority for the attractive/behavioral
/// component. The arbiter combines them:
///
/// - Normal: final = behavioral + L0_repulsive, clamped to limits
/// - Emergency (d_safe violation): L0 overrides everything with escape velocity
/// - No layers active: stop (safe default)
pub struct SubsumptionArbiter {
    /// L0: obstacle avoidance — always active, constrains all layers.
    obstacle_avoidance: ObstacleAvoidanceLayer,
    /// Behavioral layers L4→L3→L2→L1 evaluated by priority.
    layers: Vec<Box<dyn BehaviorLayer>>,
    /// Maximum linear velocity for clamping (m/s).
    max_linear: f64,
    /// Maximum angular velocity for clamping (rad/s).
    max_angular: f64,
}

impl SubsumptionArbiter {
    /// Create a new arbiter with the COVEN layer stack.
    pub fn new(params: NavParams) -> Self {
        let max_linear = params.max_linear;
        let max_angular = params.max_angular;

        let obstacle_avoidance = ObstacleAvoidanceLayer::new(params.clone());

        let mut layers: Vec<Box<dyn BehaviorLayer>> = vec![
            Box::new(ExecuteTaskLayer),                  // L4: priority 1
            Box::new(NavigateToGoalLayer),               // L3: priority 2
            Box::new(WanderCollectLayer::new(&params)),  // L2: priority 3
            Box::new(ReturnToDockLayer::new(params)),    // L1: priority 4
        ];
        // Sort by priority (should already be in order, but enforce it)
        layers.sort_by_key(|l| l.priority());

        Self {
            obstacle_avoidance,
            layers,
            max_linear,
            max_angular,
        }
    }

    /// Evaluate all layers and return the composed velocity command.
    ///
    /// Two-phase composition per thesis:
    /// 1. L0 repulsive field (always computed)
    /// 2. Behavioral layer arbitration (L4→L3→L2→L1, first active wins)
    /// 3. Combine: behavioral + repulsive, clamped
    pub fn evaluate(&mut self, ctx: &LayerContext) -> VelocityCmd {
        // Phase 1: L0 — always compute repulsive constraint
        let l0_output = self.obstacle_avoidance.evaluate(ctx);

        // If L0 emergency (within d_safe) — override everything
        if let ObstacleOutput::Emergency(escape) = l0_output {
            return escape;
        }

        // Phase 2: Behavioral layer arbitration (L4→L3→L2→L1)
        let mut behavioral = VelocityCmd::stop();
        for layer in &mut self.layers {
            let output = layer.evaluate(ctx);
            if output.active {
                if let Some(cmd) = output.cmd {
                    behavioral = cmd;
                    break;
                }
            }
        }

        // Phase 3: Compose — behavioral + L0 repulsive, clamped
        match l0_output {
            ObstacleOutput::Constrain(repulsive) => {
                let linear = (behavioral.linear + repulsive.linear)
                    .clamp(-self.max_linear, self.max_linear);
                let angular = (behavioral.angular + repulsive.angular)
                    .clamp(-self.max_angular, self.max_angular);
                VelocityCmd { linear, angular }
            }
            ObstacleOutput::Clear => behavioral,
            ObstacleOutput::Emergency(_) => unreachable!(),
        }
    }
}
