// SPDX-License-Identifier: MIT
//! mock.rs — Mock Hardware Simulation
//!
//! Provides simulated motors, encoders, and LiDAR for desktop testing.
//!
//! In mock mode, we still simulate the "dumb rover" behavior - collecting
//! raw encoder ticks and LiDAR ranges during a mission.
//!
//! Responsibilities:
//! - Simulate differential drive kinematics
//! - Generate simulated encoder ticks
//! - Generate simulated LiDAR scans
//! - Provide simple environment with walls and obstacles
//! - Support testing without physical hardware
//!
//! Author: Alexander Shultis
//! Date: January 2026

// ------------------------
// --- Imports ---
// ------------------------

// --- Standard Library ---
use std::f64::consts::PI;
use std::time::Instant;

// --- Local ---
use crate::hardware::encoders::Odometry;
use crate::lidar::LaserScan;
use crate::utils::{normalize_angle, now_secs};

// ------------------------
// --- Data Structures ---
// ------------------------

/// Simulated differential drive robot.
pub struct MockHardware {
    /// Robot X position in meters.
    x: f64,
    /// Robot Y position in meters.
    y: f64,
    /// Robot heading in radians.
    theta: f64,
    /// Current linear velocity in m/s.
    v_linear: f64,
    /// Current angular velocity in rad/s.
    v_angular: f64,
    /// Wheel base (track width) in meters.
    wheel_base: f64,
    /// Wheel radius in meters.
    wheel_radius: f64,
    /// Encoder ticks per wheel revolution.
    ticks_per_rev: u16,
    /// Accumulated left wheel ticks (fractional).
    left_ticks_accum: f64,
    /// Accumulated right wheel ticks (fractional).
    right_ticks_accum: f64,
    /// Last reported left ticks (integer).
    last_left_ticks: i64,
    /// Last reported right ticks (integer).
    last_right_ticks: i64,
    /// Last update timestamp.
    last_update: Instant,
    /// Simulated room width in meters.
    room_width: f64,
    /// Simulated room height in meters.
    room_height: f64,
    /// Obstacles as (x, y, radius) tuples.
    obstacles: Vec<(f64, f64, f64)>,
}

// ------------------------
// --- Implementation ---
// ------------------------

impl MockHardware {
    /// Create new mock hardware with default room.
    pub fn new() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            v_linear: 0.0,
            v_angular: 0.0,
            wheel_base: 0.298,
            wheel_radius: 0.1,
            ticks_per_rev: 816, // JGA25-371: 408 PPR * X2 quadrature
            left_ticks_accum: 0.0,
            right_ticks_accum: 0.0,
            last_left_ticks: 0,
            last_right_ticks: 0,
            last_update: Instant::now(),
            room_width: 10.0,
            room_height: 10.0,
            obstacles: vec![
                // Some obstacles in the room
                (3.0, 2.0, 0.5),
                (-2.0, 3.0, 0.3),
                (1.0, -2.0, 0.4),
                (-3.0, -1.0, 0.6),
            ],
        }
    }

    /// Set velocity command.
    pub fn set_velocity(&mut self, linear: f64, angular: f64) {
        self.v_linear = linear.clamp(-0.5, 0.5);
        self.v_angular = angular.clamp(-2.0, 2.0);
    }

    /// Stop the robot.
    pub fn stop(&mut self) {
        self.v_linear = 0.0;
        self.v_angular = 0.0;
    }

    /// Update simulation state.
    pub fn update(&mut self) -> Odometry {
        let now = Instant::now();
        let dt = now.duration_since(self.last_update).as_secs_f64();
        self.last_update = now;

        if dt < 0.001 {
            return self.get_odometry();
        }

        // Calculate wheel velocities from differential drive kinematics
        // v_left = v_linear - (wheel_base/2) * v_angular
        // v_right = v_linear + (wheel_base/2) * v_angular
        let v_left = self.v_linear - (self.wheel_base / 2.0) * self.v_angular;
        let v_right = self.v_linear + (self.wheel_base / 2.0) * self.v_angular;

        // Calculate wheel distances traveled
        let dist_left = v_left * dt;
        let dist_right = v_right * dt;

        // Convert to encoder ticks (accumulate fractional ticks)
        let wheel_circumference = 2.0 * PI * self.wheel_radius;
        let ticks_per_meter = self.ticks_per_rev as f64 / wheel_circumference;

        self.left_ticks_accum += dist_left * ticks_per_meter;
        self.right_ticks_accum += dist_right * ticks_per_meter;

        // Simple differential drive kinematics for pose update
        let delta_theta = self.v_angular * dt;
        let delta_dist = self.v_linear * dt;

        // Update pose using midpoint integration
        let theta_mid = self.theta + delta_theta / 2.0;
        let new_x = self.x + delta_dist * theta_mid.cos();
        let new_y = self.y + delta_dist * theta_mid.sin();

        // Check for wall collisions (keep inside room)
        let margin = 0.15; // Robot radius
        self.x = new_x.clamp(
            -self.room_width / 2.0 + margin,
            self.room_width / 2.0 - margin,
        );
        self.y = new_y.clamp(
            -self.room_height / 2.0 + margin,
            self.room_height / 2.0 - margin,
        );

        // Update heading
        self.theta = normalize_angle(self.theta + delta_theta);

        self.get_odometry()
    }

    /// Get current odometry.
    pub fn get_odometry(&self) -> Odometry {
        Odometry {
            x: self.x,
            y: self.y,
            theta: self.theta,
            v_linear: self.v_linear,
            v_angular: self.v_angular,
            timestamp: now_secs(),
        }
    }

    /// Generate a simulated LiDAR scan.
    pub fn get_scan(&self) -> LaserScan {
        let num_rays = 360;
        let angle_min = -PI;
        let angle_max = PI;
        let angle_increment = (angle_max - angle_min) / num_rays as f64;
        let range_min = 0.12;
        let range_max = 10.0;

        let mut ranges = Vec::with_capacity(num_rays);

        for i in 0..num_rays {
            let angle = angle_min + i as f64 * angle_increment;
            let world_angle = self.theta + angle;

            // Ray cast to find distance
            let range = self.ray_cast(world_angle, range_max);
            ranges.push(range);
        }

        LaserScan {
            timestamp: now_secs(),
            angle_min,
            angle_max,
            angle_increment,
            time_increment: 0.0,
            scan_time: 1.0 / 6.0,
            range_min,
            range_max,
            ranges,
            intensities: Vec::new(),
        }
    }

    /// Ray cast from robot position in world coordinates.
    fn ray_cast(&self, world_angle: f64, max_range: f64) -> f64 {
        let dx = world_angle.cos();
        let dy = world_angle.sin();

        let mut min_dist = max_range;

        // Check walls
        // Right wall (x = room_width/2)
        if dx > 0.001 {
            let t = (self.room_width / 2.0 - self.x) / dx;
            if t > 0.0 && t < min_dist {
                let hit_y = self.y + t * dy;
                if hit_y.abs() <= self.room_height / 2.0 {
                    min_dist = t;
                }
            }
        }

        // Left wall (x = -room_width/2)
        if dx < -0.001 {
            let t = (-self.room_width / 2.0 - self.x) / dx;
            if t > 0.0 && t < min_dist {
                let hit_y = self.y + t * dy;
                if hit_y.abs() <= self.room_height / 2.0 {
                    min_dist = t;
                }
            }
        }

        // Top wall (y = room_height/2)
        if dy > 0.001 {
            let t = (self.room_height / 2.0 - self.y) / dy;
            if t > 0.0 && t < min_dist {
                let hit_x = self.x + t * dx;
                if hit_x.abs() <= self.room_width / 2.0 {
                    min_dist = t;
                }
            }
        }

        // Bottom wall (y = -room_height/2)
        if dy < -0.001 {
            let t = (-self.room_height / 2.0 - self.y) / dy;
            if t > 0.0 && t < min_dist {
                let hit_x = self.x + t * dx;
                if hit_x.abs() <= self.room_width / 2.0 {
                    min_dist = t;
                }
            }
        }

        // Check obstacles (circles)
        for &(ox, oy, radius) in &self.obstacles {
            if let Some(dist) = self.ray_circle_intersect(ox, oy, radius, dx, dy) {
                if dist < min_dist {
                    min_dist = dist;
                }
            }
        }

        min_dist
    }

    /// Ray-circle intersection.
    fn ray_circle_intersect(&self, cx: f64, cy: f64, r: f64, dx: f64, dy: f64) -> Option<f64> {
        // Vector from ray origin to circle center
        let ocx = cx - self.x;
        let ocy = cy - self.y;

        // Project onto ray direction
        let proj = ocx * dx + ocy * dy;

        // Closest point on ray to circle center
        let closest_x = proj * dx;
        let closest_y = proj * dy;

        // Distance from closest point to circle center
        let dist_to_center_sq = (ocx - closest_x).powi(2) + (ocy - closest_y).powi(2);

        if dist_to_center_sq > r * r {
            return None; // Ray misses circle
        }

        // Distance from closest point to intersection
        let half_chord = (r * r - dist_to_center_sq).sqrt();

        // Two intersection points
        let t1 = proj - half_chord;
        let t2 = proj + half_chord;

        // Return closest positive intersection
        if t1 > 0.01 {
            Some(t1)
        } else if t2 > 0.01 {
            Some(t2)
        } else {
            None
        }
    }

    /// Get delta encoder ticks since last call.
    pub fn get_delta_ticks(&mut self) -> (i32, i32) {
        let left_ticks = self.left_ticks_accum as i64;
        let right_ticks = self.right_ticks_accum as i64;

        let delta_left = (left_ticks - self.last_left_ticks) as i32;
        let delta_right = (right_ticks - self.last_right_ticks) as i32;

        self.last_left_ticks = left_ticks;
        self.last_right_ticks = right_ticks;

        (delta_left, delta_right)
    }

    /// Get wheel radius in mm.
    pub fn wheel_radius_mm(&self) -> u16 {
        (self.wheel_radius * 1000.0) as u16
    }

    /// Get wheel base in mm.
    pub fn wheel_base_mm(&self) -> u16 {
        (self.wheel_base * 1000.0) as u16
    }

    /// Get ticks per revolution.
    pub fn ticks_per_rev(&self) -> u16 {
        self.ticks_per_rev
    }

    /// Emergency stop.
    #[allow(dead_code)]
    pub fn emergency_stop(&mut self) {
        self.stop();
    }

    /// Shutdown.
    #[allow(dead_code)]
    pub fn shutdown(&mut self) {
        self.stop();
    }
}

impl Default for MockHardware {
    fn default() -> Self {
        Self::new()
    }
}
