//! battery.rs — Battery Voltage Reader
//!
//! Reads battery voltage using I2C ADC for power monitoring.
//!
//! Responsibilities:
//! - Initialize I2C communication with ADS1015/ADS1115 ADC
//! - Configure ADC for single-shot voltage readings
//! - Apply voltage divider ratio to convert ADC voltage to battery voltage
//! - Calculate battery percentage from voltage
//! - Report diagnostic information on I2C/ADC failures
//!
//! Author: Alexander Shultis
//! Date: January 2026

// ------------------------
// --- Imports ---
// ------------------------

// --- Third-party ---
use anyhow::{Context, Result};
use rppal::i2c::I2c;
use tracing::{debug, error, info, trace, warn};

// --- Local ---
use crate::config::BatteryConfig;

// ------------------------
// --- Constants ---
// ------------------------

/// ADS1015/ADS1115 conversion register address.
const ADS_CONVERSION_REG: u8 = 0x00;
/// ADS1015/ADS1115 config register address.
const ADS_CONFIG_REG: u8 = 0x01;

/// Start single conversion.
const ADS_CONFIG_OS_SINGLE: u16 = 0x8000;
/// AIN0 to GND multiplexer setting.
const ADS_CONFIG_MUX_SINGLE_0: u16 = 0x4000;
/// +/- 4.096V PGA range.
const ADS_CONFIG_PGA_4V: u16 = 0x0200;
/// Single-shot mode.
const ADS_CONFIG_MODE_SINGLE: u16 = 0x0100;
/// 1600 samples per second data rate.
const ADS_CONFIG_DR_1600: u16 = 0x0080;
/// Disable comparator.
const ADS_CONFIG_COMP_DISABLE: u16 = 0x0003;

// ------------------------
// --- Data Structures ---
// ------------------------

/// Battery voltage reader using I2C ADC.
pub struct BatteryReader {
    /// I2C interface for ADC communication.
    i2c: I2c,
    /// ADC I2C address (stored for diagnostics).
    #[allow(dead_code)]
    address: u8,
    /// Voltage divider ratio (actual_voltage = adc_voltage * divider_ratio).
    divider_ratio: f64,
    /// Full battery voltage threshold.
    full_voltage: f64,
    /// Empty battery voltage threshold.
    empty_voltage: f64,
}

// ------------------------
// --- Implementation ---
// ------------------------

impl BatteryReader {
    /// Create a new battery reader.
    pub fn new(config: &BatteryConfig) -> Result<Self> {
        info!("--- Battery Monitor ---");
        info!("  ADC address: 0x{:02X}", config.adc_address);
        info!("  Voltage divider ratio: {:.2}", config.divider_ratio);
        info!("  Full voltage: {:.2}V", config.full_voltage);
        info!("  Empty voltage: {:.2}V", config.empty_voltage);

        trace!("Opening I2C bus...");
        let mut i2c = I2c::new()
            .map_err(|e| {
                error!("!!! I2C BUS INIT FAILED !!!");
                error!("  Error: {}", e);
                error!("  Troubleshooting:");
                error!("    1. Check I2C is enabled: sudo raspi-config -> Interfaces -> I2C");
                error!("    2. Check /dev/i2c-1 exists: ls -la /dev/i2c*");
                error!("    3. Check user is in i2c group: groups $USER");
                e
            })
            .context("Failed to open I2C bus")?;
        trace!("I2C bus opened");

        trace!(
            "Setting I2C slave address to 0x{:02X}...",
            config.adc_address
        );
        i2c.set_slave_address(config.adc_address as u16)
            .map_err(|e| {
                error!(
                    "Failed to set I2C address 0x{:02X}: {}",
                    config.adc_address, e
                );
                e
            })
            .context("Failed to set I2C address")?;

        // Verify ADC is present by reading config register
        trace!("Probing ADC at address 0x{:02X}...", config.adc_address);
        let mut buf = [0u8; 2];
        i2c.write_read(&[ADS_CONFIG_REG], &mut buf)
            .map_err(|e| {
                error!("!!! ADC NOT RESPONDING !!!");
                error!("  Error: {}", e);
                error!("  Address: 0x{:02X}", config.adc_address);
                error!("  Troubleshooting:");
                error!("    1. Check ADC wiring: SDA->GPIO2, SCL->GPIO3");
                error!("    2. Scan I2C bus: i2cdetect -y 1");
                error!("    3. Check ADC power supply (3.3V or 5V)");
                error!("    4. Verify ADC address matches config (common: 0x48, 0x49)");
                e
            })
            .context("ADC not responding - check I2C connection")?;

        let config_val = u16::from_be_bytes(buf);
        trace!("ADC config register: 0x{:04X}", config_val);

        // Try to identify ADC type
        let adc_type = if config_val & 0x8000 != 0 {
            "ADS1x15 (idle)"
        } else {
            "ADS1x15 (busy/unknown)"
        };
        debug!("ADC type detected: {}", adc_type);

        // Do a test read to verify full functionality
        trace!("Performing test voltage read...");
        let reader = Self {
            i2c,
            address: config.adc_address,
            divider_ratio: config.divider_ratio,
            full_voltage: config.full_voltage,
            empty_voltage: config.empty_voltage,
        };

        match reader.read_voltage() {
            Ok(voltage) => {
                let percent = reader.voltage_to_percent(voltage);
                info!("  [OK] Battery ADC initialized");
                info!("  Initial reading: {:.2}V ({:.0}%)", voltage, percent);

                if voltage < config.empty_voltage {
                    warn!("  [WARN] Battery voltage below empty threshold!");
                } else if voltage > config.full_voltage * 1.1 {
                    warn!("  [WARN] Battery voltage above expected maximum - check divider ratio");
                }
            }
            Err(e) => {
                warn!("  [WARN] Test read failed: {}", e);
                warn!("  ADC detected but reading failed - may work after warmup");
            }
        }

        Ok(reader)
    }

    /// Convert voltage to percentage.
    fn voltage_to_percent(&self, voltage: f64) -> f64 {
        let range = self.full_voltage - self.empty_voltage;
        if range > 0.0 {
            ((voltage - self.empty_voltage) / range * 100.0).clamp(0.0, 100.0)
        } else {
            100.0
        }
    }

    /// Read raw ADC voltage.
    fn read_voltage(&self) -> Result<f64> {
        // Configure ADC for single-shot reading
        let config = ADS_CONFIG_OS_SINGLE
            | ADS_CONFIG_MUX_SINGLE_0
            | ADS_CONFIG_PGA_4V
            | ADS_CONFIG_MODE_SINGLE
            | ADS_CONFIG_DR_1600
            | ADS_CONFIG_COMP_DISABLE;

        // Write config to start conversion
        let config_bytes = [(config >> 8) as u8, config as u8];
        // Workaround: rppal I2C doesn't have write_block_data, use smbus
        self.i2c
            .block_write(ADS_CONFIG_REG, &config_bytes)
            .context("Failed to write ADC config")?;

        // Poll for conversion complete (OS bit = 1) with timeout
        let timeout = std::time::Instant::now();
        loop {
            let mut config_buf = [0u8; 2];
            self.i2c
                .block_read(ADS_CONFIG_REG, &mut config_buf)
                .context("Failed to read ADC config during conversion")?;

            // Check OS bit (bit 15) - 1 means conversion complete
            if (config_buf[0] & 0x80) != 0 {
                break;
            }

            // Timeout after 100ms (more than enough for ADS1115)
            if timeout.elapsed() > std::time::Duration::from_millis(100) {
                return Err(anyhow::anyhow!("ADC conversion timeout"));
            }

            std::thread::sleep(std::time::Duration::from_millis(1));
        }

        // Read conversion result
        let mut buf = [0u8; 2];
        self.i2c
            .block_read(ADS_CONVERSION_REG, &mut buf)
            .context("Failed to read ADC conversion")?;

        // Convert to voltage (ADS1115 is 16-bit, ADS1015 is 12-bit left-aligned)
        let raw = i16::from_be_bytes(buf);
        // For PGA = 4.096V, LSB = 4.096 / 32768 = 0.000125V
        let adc_voltage = raw as f64 * 0.000125;

        // Apply voltage divider ratio to get actual battery voltage
        let battery_voltage = adc_voltage * self.divider_ratio;

        debug!(
            raw = raw,
            adc_v = adc_voltage,
            battery_v = battery_voltage,
            "Battery voltage read"
        );

        Ok(battery_voltage)
    }

    /// Read battery percentage (0-100).
    pub fn read_percent(&self) -> Result<f64> {
        let voltage = self.read_voltage()?;

        // Linear interpolation between empty and full voltage
        let range = self.full_voltage - self.empty_voltage;
        let percent = if range > 0.0 {
            ((voltage - self.empty_voltage) / range * 100.0).clamp(0.0, 100.0)
        } else {
            warn!("Invalid battery voltage range configuration");
            100.0
        };

        debug!(voltage = voltage, percent = percent, "Battery level");

        Ok(percent)
    }
}
