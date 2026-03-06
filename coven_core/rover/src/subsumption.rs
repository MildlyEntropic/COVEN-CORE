//! subsumption.rs — COVEN Subsumption Architecture
//!
//! Layered behavioral architecture where higher-priority layers can suppress
//! lower-priority outputs. This provides robust, composable behavior blending
//! for the rover's autonomous navigation.
//!
//! ## Layer Hierarchy (highest priority first)
//!
//! | Priority | Layer    | Role                                    |
//! |----------|----------|-----------------------------------------|
//! | L0       | Safety   | Emergency stop if obstacle too close    |
//! | L1       | Escape   | Reverse/turn when physically stuck      |
//! | L2       | Navigate | Lyapunov potential-field goal following  |
//! | L3       | Explore  | Frontier-based wandering (no goal)      |
//! | L4       | Mission  | High-level mission state coordination   |
//!
//! ## Design Notes
//!
//! - Each layer independently evaluates sensor context and optionally emits a
//!   velocity command. The arbiter picks the first active layer (highest priority).
//! - Layers are stubs for now; integration with the existing WaypointFollower and
//!   LyapunovNavigator happens in a future pass.
//! - The Safety layer is the only one with real logic — it suppresses all motion
//!   when the closest LiDAR range is below `d_safe`.
//!
//! Author: Alexander Shultis
//! Date: March 2026

// Subsumption stubs — not yet wired into the state machine.
#![allow(dead_code)]

use crate::navigation::VelocityCmd;

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

/// Sensor context passed to each behavior layer for evaluation.
#[derive(Debug, Clone)]
pub struct LayerContext {
    /// Robot X position in meters (from odometry).
    pub robot_x: f64,
    /// Robot Y position in meters (from odometry).
    pub robot_y: f64,
    /// Robot heading in radians (from odometry).
    pub robot_theta: f64,
    /// LiDAR range readings (meters, 0.0 = invalid).
    pub lidar_ranges: Vec<f32>,
    /// Battery level (0.0–1.0).
    pub battery_pct: f64,
    /// Minimum valid LiDAR range in current scan (meters).
    pub min_range: f32,
    /// Minimum safe distance threshold (from NavParams.d_safe).
    pub d_safe: f64,
}

// ------------------------
// --- Behavior Trait ---
// ------------------------

/// A behavior layer in the subsumption hierarchy.
pub trait BehaviorLayer {
    /// Human-readable name for logging.
    fn name(&self) -> &'static str;

    /// Priority (lower number = higher priority, evaluated first).
    fn priority(&self) -> u8;

    /// Evaluate this layer given current sensor context.
    fn evaluate(&mut self, ctx: &LayerContext) -> LayerOutput;
}

// ------------------------
// --- L0: Safety ---
// ------------------------

/// L0 — Emergency stop when an obstacle is dangerously close.
///
/// This is the highest-priority layer. When the closest LiDAR reading
/// drops below `d_safe`, all motion is suppressed with a hard stop.
pub struct SafetyLayer;

impl BehaviorLayer for SafetyLayer {
    fn name(&self) -> &'static str {
        "L0:Safety"
    }

    fn priority(&self) -> u8 {
        0
    }

    fn evaluate(&mut self, ctx: &LayerContext) -> LayerOutput {
        if ctx.min_range > 0.0 && (ctx.min_range as f64) < ctx.d_safe {
            LayerOutput::suppress(VelocityCmd::stop())
        } else {
            LayerOutput::inactive()
        }
    }
}

// ------------------------
// --- L1: Escape ---
// ------------------------

/// L1 — Escape behavior when physically stuck.
///
/// Stub: will integrate with StuckDetector in a future pass.
/// When stuck, reverses briefly then rotates to clear the obstacle.
pub struct EscapeLayer;

impl BehaviorLayer for EscapeLayer {
    fn name(&self) -> &'static str {
        "L1:Escape"
    }

    fn priority(&self) -> u8 {
        1
    }

    fn evaluate(&mut self, _ctx: &LayerContext) -> LayerOutput {
        // TODO: Integrate StuckDetector — detect lack of odometry progress
        // and emit reverse + turn commands to escape.
        LayerOutput::inactive()
    }
}

// ------------------------
// --- L2: Navigate ---
// ------------------------

/// L2 — Goal-directed navigation via Lyapunov potential fields.
///
/// Stub: will wrap LyapunovNavigator.compute_velocity() in a future pass.
/// Active only when a waypoint goal is set.
pub struct NavigateLayer;

impl BehaviorLayer for NavigateLayer {
    fn name(&self) -> &'static str {
        "L2:Navigate"
    }

    fn priority(&self) -> u8 {
        2
    }

    fn evaluate(&mut self, _ctx: &LayerContext) -> LayerOutput {
        // TODO: Delegate to LyapunovNavigator for goal-seeking with
        // obstacle avoidance via repulsive potential fields.
        LayerOutput::inactive()
    }
}

// ------------------------
// --- L3: Explore ---
// ------------------------

/// L3 — Frontier-based exploration when no explicit goal is set.
///
/// Stub: will implement frontier selection from local LiDAR data.
/// Lowest behavioral priority — only active when no goal is assigned.
pub struct ExploreLayer;

impl BehaviorLayer for ExploreLayer {
    fn name(&self) -> &'static str {
        "L3:Explore"
    }

    fn priority(&self) -> u8 {
        3
    }

    fn evaluate(&mut self, _ctx: &LayerContext) -> LayerOutput {
        // TODO: Pick frontiers from LiDAR gaps and drive toward unexplored space.
        LayerOutput::inactive()
    }
}

// ------------------------
// --- L4: Mission ---
// ------------------------

/// L4 — Mission-level coordination.
///
/// Stub: will manage high-level mission state (waypoint sequencing,
/// return-to-dock decisions, battery-aware task switching).
pub struct MissionLayer;

impl BehaviorLayer for MissionLayer {
    fn name(&self) -> &'static str {
        "L4:Mission"
    }

    fn priority(&self) -> u8 {
        4
    }

    fn evaluate(&mut self, _ctx: &LayerContext) -> LayerOutput {
        // TODO: Coordinate waypoint sequencing from dock-assigned mission,
        // trigger return-to-dock when battery is low.
        LayerOutput::inactive()
    }
}

// ------------------------
// --- Arbiter ---
// ------------------------

/// Subsumption arbiter — evaluates layers by priority, first active wins.
///
/// The arbiter holds all behavior layers and evaluates them in priority order
/// (L0 first). The first layer that returns `active: true` wins and its
/// velocity command is used. If no layer is active, a stop command is returned.
pub struct SubsumptionArbiter {
    layers: Vec<Box<dyn BehaviorLayer>>,
}

impl SubsumptionArbiter {
    /// Create a new arbiter with the default COVEN layer stack.
    pub fn new() -> Self {
        let mut layers: Vec<Box<dyn BehaviorLayer>> = vec![
            Box::new(SafetyLayer),
            Box::new(EscapeLayer),
            Box::new(NavigateLayer),
            Box::new(ExploreLayer),
            Box::new(MissionLayer),
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
        // No layer active — stop
        VelocityCmd::stop()
    }
}
