//! diagnostics.rs — Hardware Diagnostic Logging
//!
//! Centralized error logging and diagnostic output for hardware initialization.
//!
//! Responsibilities:
//! - Provide consistent error message formatting
//! - Log troubleshooting hints for hardware failures
//! - Reduce duplication of error handling code
//!
//! Author: Alexander Shultis
//! Date: January 2026

// ------------------------
// --- Imports ---
// ------------------------

// --- Standard Library ---
use std::fmt::Display;

// --- Third-party ---
use tracing::error;

// ------------------------
// --- Hardware Components ---
// ------------------------

/// Known hardware components with predefined troubleshooting hints.
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub enum HardwareComponent {
    /// I2C bus for ADC communication.
    I2cBus,
    /// ADS1015/ADS1115 ADC for battery monitoring.
    Adc,
    /// GPIO subsystem.
    Gpio,
    /// PWM channels for motor control.
    Pwm,
    /// Serial port for LiDAR.
    Serial,
    /// Motor driver (TB6612FNG).
    Motor,
    /// Quadrature encoder.
    Encoder,
    /// YDLiDAR X4.
    Lidar,
    /// TCP network connection.
    Network,
}

impl HardwareComponent {
    /// Get display name for the component.
    pub fn name(&self) -> &'static str {
        match self {
            Self::I2cBus => "I2C BUS",
            Self::Adc => "ADC",
            Self::Gpio => "GPIO",
            Self::Pwm => "PWM",
            Self::Serial => "SERIAL PORT",
            Self::Motor => "MOTOR DRIVER",
            Self::Encoder => "ENCODER",
            Self::Lidar => "LIDAR",
            Self::Network => "NETWORK",
        }
    }

    /// Get troubleshooting hints for the component.
    pub fn hints(&self) -> &'static [&'static str] {
        match self {
            Self::I2cBus => &[
                "Check I2C is enabled: sudo raspi-config -> Interfaces -> I2C",
                "Check /dev/i2c-1 exists: ls -la /dev/i2c*",
                "Check user is in i2c group: groups $USER",
            ],
            Self::Adc => &[
                "Check ADC wiring: SDA->GPIO2, SCL->GPIO3",
                "Scan I2C bus: i2cdetect -y 1",
                "Check ADC power supply (3.3V or 5V)",
                "Verify ADC address matches config (common: 0x48, 0x49)",
            ],
            Self::Gpio => &[
                "Check if another process is using GPIO: lsof /dev/gpiomem",
                "Verify user is in gpio group: groups $USER",
                "Check pin is not reserved: cat /sys/kernel/debug/gpio",
            ],
            Self::Pwm => &[
                "Check PWM is enabled in /boot/config.txt: dtoverlay=pwm-2chan",
                "Verify /sys/class/pwm/pwmchip0 exists",
                "Check user has permission: ls -la /sys/class/pwm/",
                "Try: sudo chmod -R 777 /sys/class/pwm/pwmchip0/",
            ],
            Self::Serial => &[
                "Check serial port exists: ls -la /dev/ttyUSB* /dev/ttyAMA*",
                "Verify user is in dialout group: groups $USER",
                "Check baud rate matches device specification",
                "Ensure no other process has the port open: lsof /dev/ttyUSB0",
            ],
            Self::Motor => &[
                "Check TB6612FNG wiring matches config pin assignments",
                "Verify motor power supply is connected",
                "Check standby pin is correctly wired",
            ],
            Self::Encoder => &[
                "Check encoder wiring: A/B channels to correct GPIO pins",
                "Verify pull-up resistors are enabled or external",
                "Check encoder power supply (usually 3.3V or 5V)",
            ],
            Self::Lidar => &[
                "Check LiDAR USB connection",
                "Verify LiDAR has power (should spin on startup)",
                "Check serial port permissions",
                "Try unplugging and reconnecting USB",
            ],
            Self::Network => &[
                "Check network connectivity: ping dock_ip",
                "Verify firewall allows connection: sudo ufw status",
                "Check dock service is running",
                "Verify port is correct in config",
            ],
        }
    }
}

// ------------------------
// --- Logging Functions ---
// ------------------------

/// Log a hardware initialization failure with troubleshooting hints.
///
/// Produces consistent error output:
/// ```text
/// !!! COMPONENT INIT FAILED !!!
///   Error: <error message>
///   Troubleshooting:
///     1. First hint
///     2. Second hint
/// ```
pub fn log_init_failure<E: Display>(component: HardwareComponent, error: &E) {
    error!("!!! {} INIT FAILED !!!", component.name());
    error!("  Error: {}", error);
    log_hints(component);
}

/// Log a hardware initialization failure with custom context.
#[allow(dead_code)]
pub fn log_init_failure_with_context<E: Display>(
    component: HardwareComponent,
    error: &E,
    context: &str,
) {
    error!("!!! {} INIT FAILED !!!", component.name());
    error!("  Context: {}", context);
    error!("  Error: {}", error);
    log_hints(component);
}

/// Log a GPIO pin access failure.
pub fn log_gpio_failure<E: Display>(pin: u8, purpose: &str, error: &E) {
    error!("Cannot access GPIO{} for {}: {}", pin, purpose, error);
    error!("  Pin may be in use by another process or reserved by system");
    error!("  Troubleshooting:");
    error!("    - Check if pin is available: cat /sys/kernel/debug/gpio");
    error!("    - Check for conflicts: lsof /dev/gpiomem");
    error!("    - Verify pin number matches hardware");
}

/// Log troubleshooting hints for a component.
fn log_hints(component: HardwareComponent) {
    let hints = component.hints();
    if !hints.is_empty() {
        error!("  Troubleshooting:");
        for (i, hint) in hints.iter().enumerate() {
            error!("    {}. {}", i + 1, hint);
        }
    }
}

/// Log a connection failure with retry information.
#[allow(dead_code)]
pub fn log_connection_failure<E: Display>(
    target: &str,
    port: u16,
    error: &E,
    attempt: u32,
    retry_delay_secs: f64,
) {
    error!("!!! DOCK CONNECTION FAILED !!!");
    error!("  Error: {}", error);
    error!("  Target: {}:{}", target, port);
    error!("  Attempt: #{}", attempt);
    log_hints(HardwareComponent::Network);
    error!("  Retrying in {:.1}s...", retry_delay_secs);
}
