// SPDX-License-Identifier: MIT
//! navigation.rs — COVEN Lyapunov Navigation
//!
//! Lyapunov-based potential field navigation for reactive obstacle avoidance.
//!
//! The rover is "dumb" - it doesn't build maps or do SLAM. It just follows
//! potential gradients: attracted to goals, repelled by obstacles.
//!
//! This is computationally trivial and mathematically guaranteed to be stable
//! (no oscillations, smooth trajectories). The dock does all the smart stuff.
//!
//! Responsibilities:
//! - Compute attractive forces toward goal
//! - Compute repulsive forces from LiDAR obstacles
//! - Generate velocity commands from potential field gradients
//! - Follow sequences of waypoints
//! - Detect stuck conditions
//!
//! ## Potential Field Approach
//!
//! Total potential: U(x) = U_att(x) + U_rep(x)
//!
//! - U_att: Attractive potential pulling toward goal (parabolic)
//! - U_rep: Repulsive potential pushing away from obstacles
//!
//! The velocity command is proportional to -∇U (negative gradient = downhill).
//!
//! Author: Alexander Shultis
//! Date: January 2026

// ------------------------
// --- Imports ---
// ------------------------

// --- Standard Library ---
use std::f64::consts::PI;

// --- Local ---
use crate::config::NavigationConfig;

// ------------------------
// --- Data Structures ---
// ------------------------

/// Navigation parameters for potential field control.
#[derive(Debug, Clone)]
pub struct NavParams {
    /// Attractive gain (how strongly to pull toward goal).
    pub k_att: f64,
    /// Repulsive gain (how strongly to push from obstacles).
    pub k_rep: f64,
    /// Influence distance for obstacles (meters).
    pub d_influence: f64,
    /// Minimum safe distance to obstacles (meters).
    pub d_safe: f64,
    /// Goal reached threshold (meters).
    pub goal_tolerance: f64,
    /// Maximum linear velocity (m/s).
    pub max_linear: f64,
    /// Maximum angular velocity (rad/s).
    pub max_angular: f64,
    /// Heading alignment gain (turn toward movement direction).
    pub k_heading: f64,
}

impl Default for NavParams {
    fn default() -> Self {
        Self {
            k_att: 1.0,
            k_rep: 0.8,
            d_influence: 1.5, // Start repelling at 1.5m
            d_safe: 0.25,     // Emergency stop at 25cm
            goal_tolerance: 0.1,
            max_linear: 0.3, // Conservative max speed
            max_angular: 1.5,
            k_heading: 2.0, // Heading correction gain
        }
    }
}

impl From<&NavigationConfig> for NavParams {
    fn from(config: &NavigationConfig) -> Self {
        Self {
            k_att: config.k_att,
            k_rep: config.k_rep,
            d_influence: config.d_influence,
            d_safe: config.d_safe,
            goal_tolerance: config.goal_tolerance,
            max_linear: config.max_linear,
            max_angular: config.max_angular,
            k_heading: config.k_heading,
        }
    }
}

/// A 2D goal position.
#[derive(Debug, Clone, Copy)]
pub struct Goal {
    /// X position in meters.
    pub x: f64,
    /// Y position in meters.
    pub y: f64,
}

/// Navigation output - velocity command for differential drive.
#[derive(Debug, Clone, Copy)]
pub struct VelocityCmd {
    /// Linear velocity in m/s.
    pub linear: f64,
    /// Angular velocity in rad/s.
    pub angular: f64,
}

impl VelocityCmd {
    /// Create a stop command (zero velocity).
    pub fn stop() -> Self {
        Self {
            linear: 0.0,
            angular: 0.0,
        }
    }
}

/// Lyapunov-based navigator using potential fields for reactive obstacle avoidance.
pub struct LyapunovNavigator {
    /// Navigation parameters.
    params: NavParams,
    /// Current goal position.
    goal: Option<Goal>,
}

// ------------------------
// --- Implementation ---
// ------------------------

impl LyapunovNavigator {
    /// Create a new navigator with default parameters.
    pub fn new() -> Self {
        Self {
            params: NavParams::default(),
            goal: None,
        }
    }

    /// Create with custom parameters.
    pub fn with_params(params: NavParams) -> Self {
        Self { params, goal: None }
    }

    /// Set the current goal.
    pub fn set_goal(&mut self, x: f64, y: f64) {
        self.goal = Some(Goal { x, y });
    }

    /// Clear the current goal (stop navigating).
    pub fn clear_goal(&mut self) {
        self.goal = None;
    }

    /// Check if we have an active goal.
    #[allow(dead_code)]
    pub fn has_goal(&self) -> bool {
        self.goal.is_some()
    }

    /// Check if goal is reached.
    pub fn goal_reached(&self, robot_x: f64, robot_y: f64) -> bool {
        if let Some(goal) = &self.goal {
            let dx = goal.x - robot_x;
            let dy = goal.y - robot_y;
            let dist = (dx * dx + dy * dy).sqrt();
            dist < self.params.goal_tolerance
        } else {
            false
        }
    }

    /// Compute combined velocity command (attractive + repulsive).
    ///
    /// Note: In the thesis subsumption model, attractive and repulsive
    /// components are decomposed into separate layers (L0 vs L1-L4).
    /// This combined method is retained for testing and direct use.
    #[allow(dead_code)]
    pub fn compute_velocity(
        &self,
        robot_x: f64,
        robot_y: f64,
        robot_theta: f64,
        lidar_ranges: &[f64],
        lidar_angle_min: f64,
        lidar_angle_increment: f64,
    ) -> VelocityCmd {
        // No goal = no movement
        let goal = match &self.goal {
            Some(g) => g,
            None => return VelocityCmd::stop(),
        };

        // Check if goal reached
        let dx_goal = goal.x - robot_x;
        let dy_goal = goal.y - robot_y;
        let dist_to_goal = (dx_goal * dx_goal + dy_goal * dy_goal).sqrt();

        if dist_to_goal < self.params.goal_tolerance {
            return VelocityCmd::stop();
        }

        // === Attractive Force ===
        // F_att = -k_att * (robot - goal) = k_att * (goal - robot)
        // Negative gradient of U_att = 0.5 * k_att * ||robot - goal||^2
        let f_att_x = self.params.k_att * dx_goal;
        let f_att_y = self.params.k_att * dy_goal;

        // === Repulsive Force ===
        // Sum repulsive forces from all obstacles within influence distance
        let (f_rep_x, f_rep_y) = self.compute_repulsive_force(
            robot_theta,
            lidar_ranges,
            lidar_angle_min,
            lidar_angle_increment,
        );

        // === Total Force ===
        let f_total_x = f_att_x + f_rep_x;
        let f_total_y = f_att_y + f_rep_y;

        // Emergency stop if obstacle too close
        let min_range = self.find_min_range(lidar_ranges);
        if min_range < self.params.d_safe {
            // Only allow backward motion or turning
            return self.compute_escape_velocity(
                robot_theta,
                lidar_ranges,
                lidar_angle_min,
                lidar_angle_increment,
            );
        }

        // === Convert Force to Velocity ===
        // Transform force from world frame to robot frame
        let cos_theta = robot_theta.cos();
        let sin_theta = robot_theta.sin();

        // Force in robot frame (x = forward, y = left)
        let f_robot_x = f_total_x * cos_theta + f_total_y * sin_theta;
        let f_robot_y = -f_total_x * sin_theta + f_total_y * cos_theta;

        // Desired heading in robot frame
        let desired_heading = f_robot_y.atan2(f_robot_x);

        // Linear velocity: proportional to forward component of force
        // Reduce speed when not facing the right direction
        let heading_factor = (1.0 + desired_heading.cos()) / 2.0; // 1 when aligned, 0 when opposite
        let linear = (f_robot_x.max(0.0) * heading_factor).min(self.params.max_linear);

        // Angular velocity: proportional to heading error
        let angular = (self.params.k_heading * desired_heading)
            .clamp(-self.params.max_angular, self.params.max_angular);

        // Scale down linear velocity if we need to turn a lot
        let turn_factor = 1.0 - (angular.abs() / self.params.max_angular).powi(2);
        let linear = linear * turn_factor.max(0.3);

        VelocityCmd { linear, angular }
    }

    /// Compute attractive-only velocity toward goal (no repulsive component).
    ///
    /// Per thesis Section 2.2.3: the attractive potential implements L3
    /// (navigate to goal), while the repulsive potential implements L0
    /// (obstacle avoidance). This method provides the pure attractive
    /// component for layers L1–L4; L0 handles obstacle avoidance separately.
    pub fn compute_attractive_velocity(
        &self,
        robot_x: f64,
        robot_y: f64,
        robot_theta: f64,
    ) -> VelocityCmd {
        let goal = match &self.goal {
            Some(g) => g,
            None => return VelocityCmd::stop(),
        };

        let dx_goal = goal.x - robot_x;
        let dy_goal = goal.y - robot_y;
        let dist_to_goal = (dx_goal * dx_goal + dy_goal * dy_goal).sqrt();

        if dist_to_goal < self.params.goal_tolerance {
            return VelocityCmd::stop();
        }

        // Attractive force: F_att = k_att * (goal - robot)
        let f_att_x = self.params.k_att * dx_goal;
        let f_att_y = self.params.k_att * dy_goal;

        // Convert to robot frame
        let cos_theta = robot_theta.cos();
        let sin_theta = robot_theta.sin();
        let f_robot_x = f_att_x * cos_theta + f_att_y * sin_theta;
        let f_robot_y = -f_att_x * sin_theta + f_att_y * cos_theta;

        // Desired heading in robot frame
        let desired_heading = f_robot_y.atan2(f_robot_x);

        // Linear velocity: proportional to forward component
        let heading_factor = (1.0 + desired_heading.cos()) / 2.0;
        let linear = (f_robot_x.max(0.0) * heading_factor).min(self.params.max_linear);

        // Angular velocity: proportional to heading error
        let angular = (self.params.k_heading * desired_heading)
            .clamp(-self.params.max_angular, self.params.max_angular);

        // Reduce linear when turning hard
        let turn_factor = 1.0 - (angular.abs() / self.params.max_angular).powi(2);
        let linear = linear * turn_factor.max(0.3);

        VelocityCmd { linear, angular }
    }

    /// Compute repulsive force from LiDAR obstacles (world frame).
    ///
    /// Returns the summed repulsive force vector (fx, fy) from all obstacles
    /// within influence distance. Per thesis: this is the gradient of
    /// U_rep = 0.5 * k_rep * (1/d - 1/d_influence)^2.
    pub fn compute_repulsive_force(
        &self,
        robot_theta: f64,
        ranges: &[f64],
        angle_min: f64,
        angle_increment: f64,
    ) -> (f64, f64) {
        let mut f_rep_x = 0.0;
        let mut f_rep_y = 0.0;

        for (i, &range) in ranges.iter().enumerate() {
            // Skip invalid readings
            if !range.is_finite() || range <= 0.0 || range > self.params.d_influence {
                continue;
            }

            // Angle in robot frame
            let angle_robot = angle_min + i as f64 * angle_increment;

            // Position of obstacle in robot frame
            let obs_robot_x = range * angle_robot.cos();
            let obs_robot_y = range * angle_robot.sin();

            // Transform to world frame
            let cos_theta = robot_theta.cos();
            let sin_theta = robot_theta.sin();
            let obs_world_dx = obs_robot_x * cos_theta - obs_robot_y * sin_theta;
            let obs_world_dy = obs_robot_x * sin_theta + obs_robot_y * cos_theta;

            // Repulsive force magnitude (gradient of repulsive potential)
            // U_rep = 0.5 * k_rep * (1/d - 1/d_influence)^2 for d < d_influence
            // F_rep = -∇U_rep = k_rep * (1/d - 1/d_influence) * (1/d^2) * n
            // where n is unit vector pointing away from obstacle
            let d = range.max(0.01); // Avoid division by zero
            let inv_d = 1.0 / d;
            let inv_d_inf = 1.0 / self.params.d_influence;

            if d < self.params.d_influence {
                let magnitude = self.params.k_rep * (inv_d - inv_d_inf) * inv_d * inv_d;

                // Unit vector pointing from obstacle to robot (in world frame)
                let dist = (obs_world_dx * obs_world_dx + obs_world_dy * obs_world_dy).sqrt();
                if dist > 0.01 {
                    // Repulsive force points away from obstacle
                    f_rep_x -= magnitude * obs_world_dx / dist;
                    f_rep_y -= magnitude * obs_world_dy / dist;
                }
            }
        }

        (f_rep_x, f_rep_y)
    }

    /// Find minimum range in scan.
    /// Returns f64::INFINITY if no valid ranges (allows operation to continue).
    #[allow(dead_code)]
    pub fn find_min_range(&self, ranges: &[f64]) -> f64 {
        ranges
            .iter()
            .filter(|r| r.is_finite() && **r > 0.0)
            .cloned()
            .reduce(f64::min)
            .unwrap_or(f64::INFINITY)
    }

    /// Compute escape velocity when too close to obstacle.
    pub fn compute_escape_velocity(
        &self,
        _robot_theta: f64, // Not used - escape works in robot frame
        ranges: &[f64],
        angle_min: f64,
        angle_increment: f64,
    ) -> VelocityCmd {
        // Find the direction with most clearance
        let mut best_angle = 0.0;
        let mut best_range = 0.0;

        for (i, &range) in ranges.iter().enumerate() {
            if range.is_finite() && range > best_range {
                best_range = range;
                best_angle = angle_min + i as f64 * angle_increment;
            }
        }

        // Turn toward the clearest direction
        // Don't move forward, just rotate
        let angular = (self.params.k_heading * best_angle * 0.5)
            .clamp(-self.params.max_angular, self.params.max_angular);

        // Allow small backward motion if front is blocked
        let front_blocked = self.is_front_blocked(ranges, angle_min, angle_increment);
        let linear = if front_blocked { -0.05 } else { 0.0 };

        VelocityCmd { linear, angular }
    }

    /// Check if front of robot is blocked.
    pub fn is_front_blocked(&self, ranges: &[f64], angle_min: f64, angle_increment: f64) -> bool {
        // Check rays within ±30° of forward
        let front_angle_limit = PI / 6.0;

        for (i, &range) in ranges.iter().enumerate() {
            let angle = angle_min + i as f64 * angle_increment;
            if angle.abs() < front_angle_limit
                && range.is_finite()
                && range < self.params.d_safe * 1.5
            {
                return true;
            }
        }

        false
    }

    /// Get navigation parameters (for tuning).
    #[allow(dead_code)]
    pub fn params(&self) -> &NavParams {
        &self.params
    }

    /// Set navigation parameters.
    #[allow(dead_code)]
    pub fn set_params(&mut self, params: NavParams) {
        self.params = params;
    }
}

impl Default for LyapunovNavigator {
    fn default() -> Self {
        Self::new()
    }
}

// ------------------------
// --- Waypoint Following ---
// ------------------------

/// Navigation state for waypoint following.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NavState {
    /// No active navigation.
    Idle,
    /// Navigating to waypoint.
    Navigating,
    /// Waypoint reached.
    Arrived,
    /// Stuck (oscillating or no progress).
    Stuck,
}

/// Waypoint follower - executes a sequence of waypoints.
pub struct WaypointFollower {
    /// Underlying navigator for computing velocities.
    navigator: LyapunovNavigator,
    /// List of waypoints to visit.
    waypoints: Vec<Goal>,
    /// Current waypoint index.
    current_idx: usize,
    /// Current navigation state.
    state: NavState,
    /// Stuck condition detector.
    stuck_detector: StuckDetector,
}

impl WaypointFollower {
    /// Create a new waypoint follower with default parameters.
    pub fn new() -> Self {
        Self {
            navigator: LyapunovNavigator::new(),
            waypoints: Vec::new(),
            current_idx: 0,
            state: NavState::Idle,
            stuck_detector: StuckDetector::new(),
        }
    }

    /// Create a new waypoint follower with custom navigation parameters.
    pub fn with_config(config: &NavigationConfig) -> Self {
        Self {
            navigator: LyapunovNavigator::with_params(NavParams::from(config)),
            waypoints: Vec::new(),
            current_idx: 0,
            state: NavState::Idle,
            stuck_detector: StuckDetector::new(),
        }
    }

    /// Set waypoints to follow.
    pub fn set_waypoints(&mut self, waypoints: Vec<(f64, f64)>) {
        self.waypoints = waypoints.into_iter().map(|(x, y)| Goal { x, y }).collect();
        self.current_idx = 0;
        if !self.waypoints.is_empty() {
            let wp = self.waypoints[0];
            self.navigator.set_goal(wp.x, wp.y);
            self.state = NavState::Navigating;
        } else {
            self.navigator.clear_goal();
            self.state = NavState::Idle;
        }
        self.stuck_detector.reset();
    }

    /// Set dock return position (final destination).
    pub fn set_dock_position(&mut self, x: f64, y: f64) {
        self.waypoints.push(Goal { x, y });
    }

    /// Get current navigation state.
    pub fn state(&self) -> NavState {
        self.state
    }

    /// Update navigation and get attractive-only velocity command.
    ///
    /// Per thesis: WaypointFollower provides the attractive component (L3/L4).
    /// Obstacle avoidance (repulsive component) is handled by L0 in the
    /// subsumption arbiter, which combines attractive + repulsive velocities.
    pub fn update(
        &mut self,
        robot_x: f64,
        robot_y: f64,
        robot_theta: f64,
    ) -> VelocityCmd {
        match self.state {
            NavState::Idle | NavState::Arrived | NavState::Stuck => {
                return VelocityCmd::stop();
            }
            NavState::Navigating => {}
        }

        // Check if current waypoint reached
        if self.navigator.goal_reached(robot_x, robot_y) {
            self.current_idx += 1;

            if self.current_idx >= self.waypoints.len() {
                // All waypoints completed
                self.navigator.clear_goal();
                self.state = NavState::Arrived;
                return VelocityCmd::stop();
            }

            // Move to next waypoint
            let wp = self.waypoints[self.current_idx];
            self.navigator.set_goal(wp.x, wp.y);
            self.stuck_detector.reset();
        }

        // Compute attractive velocity (goal-seeking, no obstacle avoidance)
        let cmd = self.navigator.compute_attractive_velocity(
            robot_x,
            robot_y,
            robot_theta,
        );

        // Check for stuck condition
        if self.stuck_detector.update(robot_x, robot_y) {
            self.state = NavState::Stuck;
            return VelocityCmd::stop();
        }

        cmd
    }

    /// Get progress (current waypoint index, total waypoints).
    #[allow(dead_code)]
    pub fn progress(&self) -> (usize, usize) {
        (self.current_idx, self.waypoints.len())
    }

    /// Check if actively navigating toward a goal.
    pub fn has_goal(&self) -> bool {
        self.state == NavState::Navigating
    }

    /// Clear all waypoints and stop.
    pub fn clear(&mut self) {
        self.waypoints.clear();
        self.current_idx = 0;
        self.navigator.clear_goal();
        self.state = NavState::Idle;
    }
}

impl Default for WaypointFollower {
    fn default() -> Self {
        Self::new()
    }
}

/// Detects when robot is stuck (not making progress).
struct StuckDetector {
    /// Last recorded X position (None until first measurement).
    last_x: Option<f64>,
    /// Last recorded Y position (None until first measurement).
    last_y: Option<f64>,
    /// Counter for consecutive stuck detections.
    stuck_count: u32,
    /// Total update calls.
    update_count: u32,
}

impl StuckDetector {
    /// Create a new stuck detector.
    fn new() -> Self {
        Self {
            last_x: None,
            last_y: None,
            stuck_count: 0,
            update_count: 0,
        }
    }

    /// Reset the stuck detector.
    fn reset(&mut self) {
        self.last_x = None;
        self.last_y = None;
        self.stuck_count = 0;
        self.update_count = 0;
    }

    /// Update detector and return true if stuck.
    ///
    /// Checks position progress only — does not inspect commanded velocity.
    /// With the subsumption decomposition, the attractive velocity (from L3/L4)
    /// is always non-zero during navigation, even when L0 legitimately stops
    /// the rover near an obstacle. Checking position progress alone is the
    /// correct criterion: if the rover hasn't moved in 5+ seconds during
    /// active navigation, it's genuinely stuck regardless of commands.
    fn update(&mut self, x: f64, y: f64) -> bool {
        self.update_count += 1;

        // Check every ~1 second (at 20Hz control rate)
        if self.update_count % 20 != 0 {
            return false;
        }

        // Calculate distance moved (first measurement assumes moving)
        let dist_moved = match (self.last_x, self.last_y) {
            (Some(lx), Some(ly)) => {
                let dx = x - lx;
                let dy = y - ly;
                (dx * dx + dy * dy).sqrt()
            }
            _ => f64::INFINITY, // First measurement - assume moving
        };

        self.last_x = Some(x);
        self.last_y = Some(y);

        // Not making progress — might be stuck
        if dist_moved < 0.02 {
            self.stuck_count += 1;
        } else {
            self.stuck_count = 0;
        }

        // Stuck for 5 seconds = give up
        self.stuck_count > 5
    }
}

// ------------------------
// --- Tests ---
// ------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_goal_reached() {
        let mut nav = LyapunovNavigator::new();
        nav.set_goal(1.0, 0.0);

        assert!(!nav.goal_reached(0.0, 0.0));
        assert!(nav.goal_reached(1.0, 0.0));
        assert!(nav.goal_reached(1.05, 0.05)); // Within tolerance
    }

    #[test]
    fn test_no_goal_stops() {
        let nav = LyapunovNavigator::new();
        let ranges = vec![5.0; 360];
        let cmd = nav.compute_velocity(0.0, 0.0, 0.0, &ranges, -PI, 2.0 * PI / 360.0);

        assert_eq!(cmd.linear, 0.0);
        assert_eq!(cmd.angular, 0.0);
    }

    #[test]
    fn test_attractive_force() {
        let mut nav = LyapunovNavigator::new();
        nav.set_goal(2.0, 0.0); // Goal directly ahead

        let ranges = vec![10.0; 360]; // No obstacles
        let cmd = nav.compute_velocity(0.0, 0.0, 0.0, &ranges, -PI, 2.0 * PI / 360.0);

        // Should move forward
        assert!(cmd.linear > 0.0);
        // Should have minimal turning
        assert!(cmd.angular.abs() < 0.5);
    }

    #[test]
    fn test_obstacle_avoidance() {
        let mut nav = LyapunovNavigator::new();
        nav.set_goal(5.0, 0.0);

        // Obstacle directly ahead at 0.5m
        let mut ranges = vec![10.0; 360];
        ranges[180] = 0.5; // Front ray

        let cmd = nav.compute_velocity(0.0, 0.0, 0.0, &ranges, -PI, 2.0 * PI / 360.0);

        // Should have significant angular component to avoid obstacle
        // Linear might be reduced due to obstacle proximity
        println!(
            "With obstacle: linear={}, angular={}",
            cmd.linear, cmd.angular
        );
    }
}
