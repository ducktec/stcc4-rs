//! STCC4 CO2 sensor driver (no-std).
//!
//! This crate provides a platform agnostic driver for the Sensirion STCC4 CO2 sensor.
//! It supports both blocking and async modes via `embedded-hal` and `embedded-hal-async`.
//!
//! ## Features
//! - `async` *(default)*: Enables async support via `embedded-hal-async`.
//! - `defmt`: Enables `defmt::Format` derives and adds `defmt` logging statements.
//! - `serde`: Enables `serde` support for public data types (no std features).
//!
//! ## Usage (blocking)
//! ```no_run
//! use stcc4_rs::blocking::Stcc4;
//!
//! # fn example<I2C, D>(i2c: I2C, delay: D)
//! # where
//! #     D: embedded_hal::delay::DelayNs,
//! #     I2C: embedded_hal::i2c::I2c,
//! # {
//! let mut stcc4 = Stcc4::new(delay, i2c);
//! stcc4.start_continuous_measurement().ok();
//! let measurement = stcc4.read_measurement().ok();
//! stcc4.stop_continuous_measurement().ok();
//! # }
//! ```
//!
//! ## Usage (async)
//! ```no_run
//! use stcc4_rs::asynchronous::Stcc4;
//!
//! # async fn example<I2C, D>(i2c: I2C, delay: D)
//! # where
//! #     D: embedded_hal_async::delay::DelayNs,
//! #     I2C: embedded_hal_async::i2c::I2c,
//! # {
//! let mut stcc4 = Stcc4::new(delay, i2c);
//! stcc4.start_continuous_measurement().await.ok();
//! let measurement = stcc4.read_measurement().await.ok();
//! stcc4.stop_continuous_measurement().await.ok();
//! # }
//! ```
#![cfg_attr(not(test), no_std)]

use crc_internal::CrcError;

#[cfg(feature = "async")]
pub mod asynchronous;

pub mod blocking;

mod crc_internal;

/// Default I2C address.
pub const STCC4_ADDR_DEFAULT: u8 = 0x64;

/// Alternative I2C address.
pub const STCC4_ADDR_ALT: u8 = 0x65;

/// General call I2C address.
pub const I2C_GENERAL_CALL_ADDR: u8 = 0x00;

/// Exit sleep payload byte (not acknowledged by the sensor).
pub const EXIT_SLEEP_PAYLOAD: u8 = 0x00;

/// Soft reset command (general call, not acknowledged by the sensor).
pub const SOFT_RESET_CMD: u8 = 0x06;

/// The maximum number of bytes that the driver has to read for any command.
const MAX_RX_BYTES: usize = 18;

/// The maximum number of bytes that the driver has to write for any command.
const MAX_TX_BYTES: usize = 8;

/// Command ID enum (16-bit commands).
#[repr(u16)]
#[derive(Copy, Clone, Debug)]
enum CommandId {
    StartContinuousMeasurement = 0x218B,
    StopContinuousMeasurement = 0x3F86,
    ReadMeasurement = 0xEC05,
    SetRhtCompensation = 0xE000,
    SetPressureCompensation = 0xE016,
    MeasureSingleShot = 0x219D,
    EnterSleepMode = 0x3650,
    PerformConditioning = 0x29BC,
    PerformFactoryReset = 0x3632,
    PerformSelfTest = 0x278C,
    EnableTestingMode = 0x3FBC,
    DisableTestingMode = 0x3F3D,
    PerformForcedRecalibration = 0x362F,
    GetProductId = 0x365B,
}

/// Get execution time per command id (milliseconds).
fn get_execution_time(command: CommandId) -> u32 {
    match command {
        CommandId::StopContinuousMeasurement => 1_200,
        CommandId::ReadMeasurement => 1,
        CommandId::SetRhtCompensation => 1,
        CommandId::SetPressureCompensation => 1,
        CommandId::MeasureSingleShot => 500,
        CommandId::EnterSleepMode => 1,
        CommandId::PerformConditioning => 22_000,
        CommandId::PerformFactoryReset => 90,
        CommandId::PerformSelfTest => 360,
        CommandId::PerformForcedRecalibration => 90,
        CommandId::GetProductId => 1,
        _ => 0,
    }
}

/// Representing sensor measurement state.
#[derive(Copy, Clone, Debug, PartialEq)]
enum ModuleState {
    Idle,
    Measuring,
    Sleep,
}

/// Shorthand for all functions returning an error in this module.
type Result<T, E> = core::result::Result<T, Stcc4Error<E>>;

/// Represents any error that may happen during communication.
#[derive(Copy, Clone, Debug, Ord, PartialOrd, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Stcc4Error<E> {
    /// An error occurred while reading from the sensor.
    ReadI2cError(E),
    /// An error occurred while writing to the sensor.
    WriteI2cError(E),
    /// The sensor is in a state that does not permit this command.
    InvalidState,
    /// The sensor returned data which could not be parsed.
    InvalidData,
    /// CRC related error.
    CrcError(CrcError),
}

impl<E> From<CrcError> for Stcc4Error<E> {
    fn from(e: CrcError) -> Self {
        Stcc4Error::CrcError(e)
    }
}

/// Represents a measured sample from the sensor.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Measurement {
    /// CO2 concentration in ppm.
    pub co2_ppm: u16,
    /// Temperature in degrees Celsius.
    pub temperature_c: f32,
    /// Relative humidity in percent.
    pub humidity_percent: f32,
    /// Sensor status word.
    pub status: SensorStatus,
}

/// Sensor status (16-bit word).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SensorStatus {
    /// Raw status word.
    pub raw: u16,
}

impl SensorStatus {
    /// Whether the sensor is in testing mode.
    pub fn testing_mode(&self) -> bool {
        (self.raw & 0x0040) != 0
    }
}

/// Self test result (raw word + helpers).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SelfTestResult {
    /// Raw self-test word.
    pub raw: u16,
}

impl SelfTestResult {
    /// Whether the self test is considered a success.
    pub fn is_success(&self) -> bool {
        self.raw == 0x0000 || self.raw == 0x0010
    }

    /// Supply voltage out of range.
    pub fn supply_voltage_out_of_range(&self) -> bool {
        (self.raw & 0x0001) != 0
    }

    /// SHT not connected through STCC4 I2C controller interface pad.
    pub fn sht_missing(&self) -> bool {
        (self.raw & 0x0010) != 0
    }

    /// Memory error (bits 6:5).
    pub fn memory_error(&self) -> bool {
        (self.raw & 0x0060) != 0
    }

    /// Debug bits (3:1) for manufacturer diagnostics.
    pub fn debug_bits(&self) -> u8 {
        ((self.raw >> 1) & 0x0007) as u8
    }
}

/// Forced recalibration correction (ppm).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FrcCorrection(pub i16);

/// Product ID and serial number.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ProductId {
    /// 32-bit product ID.
    pub product_id: u32,
    /// 64-bit unique serial number.
    pub serial_number: u64,
}

#[inline]
fn raw_to_humidity_percent(raw: u16) -> f32 {
    125.0 * (raw as f32) / 65_535.0 - 6.0
}

#[inline]
fn raw_to_temperature_c(raw: u16) -> f32 {
    175.0 * (raw as f32) / 65_535.0 - 45.0
}

#[inline]
fn clamp_u16(value: f32) -> u16 {
    if value <= 0.0 {
        0
    } else if value >= u16::MAX as f32 {
        u16::MAX
    } else {
        (value + 0.5) as u16
    }
}

#[inline]
fn humidity_percent_to_raw(rh_percent: f32) -> u16 {
    clamp_u16(((rh_percent + 6.0) * 65_535.0) / 125.0)
}

#[inline]
fn temperature_c_to_raw(temperature_c: f32) -> u16 {
    clamp_u16(((temperature_c + 45.0) * 65_535.0) / 175.0)
}

#[inline]
fn pressure_pa_to_raw(pressure_pa: u32) -> u16 {
    let raw = (pressure_pa / 2) as f32;
    clamp_u16(raw)
}

#[inline]
fn frc_correction_from_raw(raw: u16) -> FrcCorrection {
    FrcCorrection((raw as i32 - 32_768) as i16)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    /// Verifies temperature conversion round-trip accuracy.
    fn conversion_roundtrip_temperature() {
        let t_c = 25.0_f32;
        let raw = temperature_c_to_raw(t_c);
        let out = raw_to_temperature_c(raw);
        assert!((out - t_c).abs() < 0.5);
    }

    #[test]
    /// Verifies humidity conversion round-trip accuracy.
    fn conversion_roundtrip_humidity() {
        let rh = 50.0_f32;
        let raw = humidity_percent_to_raw(rh);
        let out = raw_to_humidity_percent(raw);
        assert!((out - rh).abs() < 0.5);
    }

    #[test]
    /// Verifies pressure input conversion to raw ticks.
    fn pressure_conversion() {
        let pressure_pa = 101_300_u32;
        let raw = pressure_pa_to_raw(pressure_pa);
        assert_eq!(raw, 50_650);
    }

    #[test]
    /// Verifies forced recalibration correction conversion.
    fn frc_correction_conversion() {
        let raw = 32_668_u16;
        let correction = frc_correction_from_raw(raw);
        assert_eq!(correction.0, -100);
    }

    #[test]
    /// Verifies testing mode bit decoding.
    fn sensor_status_testing_mode() {
        let status = SensorStatus { raw: 0x0040 };
        assert!(status.testing_mode());

        let status = SensorStatus { raw: 0x0000 };
        assert!(!status.testing_mode());
    }

    #[test]
    /// Verifies self-test helper flag decoding.
    fn self_test_helpers() {
        let result = SelfTestResult { raw: 0x0001 };
        assert!(!result.is_success());
        assert!(result.supply_voltage_out_of_range());

        let result = SelfTestResult { raw: 0x0010 };
        assert!(result.is_success());
        assert!(result.sht_missing());

        let result = SelfTestResult { raw: 0x0060 };
        assert!(result.memory_error());

        let result = SelfTestResult { raw: 0x000E };
        assert_eq!(result.debug_bits(), 0x07);
    }

    #[test]
    /// Verifies clamp bounds at low and high inputs.
    fn clamp_u16_bounds() {
        assert_eq!(clamp_u16(-10.0), 0);
        assert_eq!(clamp_u16(100_000.0), u16::MAX);
    }

    #[test]
    /// Verifies selected execution time mappings.
    fn execution_time_mapping() {
        assert_eq!(
            get_execution_time(CommandId::StopContinuousMeasurement),
            1_200
        );
        assert_eq!(get_execution_time(CommandId::PerformConditioning), 22_000);
        assert_eq!(get_execution_time(CommandId::StartContinuousMeasurement), 0);
    }
}
