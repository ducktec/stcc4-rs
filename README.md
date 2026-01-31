# STCC4 Rust Driver

A Rust `no-std` driver for the Sensirion STCC4 CO2 sensor with blocking and async APIs.

[![Crates.io](https://img.shields.io/crates/v/stcc4.svg)](https://crates.io/crates/stcc4)
[![Docs.rs](https://docs.rs/stcc4/badge.svg)](https://docs.rs/stcc4)

## Description

This library provides a platform-agnostic Rust interface for the STCC4 CO2 sensor over I2C. It is designed for embedded, `no-std` environments and supports both blocking and async access patterns based on `embedded-hal` and `embedded-hal-async`.

## Supported features

All supported features are available in both blocking and async mode.

- Start/stop continuous measurement
- Single-shot measurement
- Read measurement data (CO2, temperature, humidity, status)
- Set RHT compensation values (external sensor)
- Set pressure compensation
- Sleep / wake
- Conditioning
- Soft reset (general call)
- Factory reset
- Self test
- Enable/disable testing mode
- Forced recalibration (FRC)
- Read product ID and serial number

## Rust features

- `async` *(default)*: Enables async support using `embedded-hal-async`
- `defmt`: Enables `defmt::Format` derives and adds `defmt` logging statements
- `serde`: Enables `serde` support for public data types (no std features)

## Dependencies

- `embedded-hal` for blocking I2C and delays
- `embedded-hal-async` for async I2C and delays (gated by `async` feature)
- `defmt` and `serde` are optional and controlled by features

For tests, the crate uses `embedded-hal-mock` and a minimal async mock implementation.

## Usage

Add the crate to your `Cargo.toml`:

```toml
[dependencies]
stcc4 = "0.1.0"
```

### Blocking example

```rust
use stcc4::blocking::Stcc4;

# fn example<I2C, D>(i2c: I2C, delay: D)
# where
#     D: embedded_hal::delay::DelayNs,
#     I2C: embedded_hal::i2c::I2c,
# {
let mut stcc4 = Stcc4::new(delay, i2c);

stcc4.start_continuous_measurement().ok();
let measurement = stcc4.read_measurement().ok();
stcc4.stop_continuous_measurement().ok();
# }
```

### Async example

```rust
use stcc4::asynchronous::Stcc4;

# async fn example<I2C, D>(i2c: I2C, delay: D)
# where
#     D: embedded_hal_async::delay::DelayNs,
#     I2C: embedded_hal_async::i2c::I2c,
# {
let mut stcc4 = Stcc4::new(delay, i2c);

stcc4.start_continuous_measurement().await.ok();
let measurement = stcc4.read_measurement().await.ok();
stcc4.stop_continuous_measurement().await.ok();
# }
```

## Testing

Run unit tests:

```bash
cargo test
```

Generate coverage (HTML):

```bash
cargo llvm-cov --html --open
```

## Notes

- Default I2C address is `0x64`, alternate is `0x65`.
- The `exit_sleep_mode` and `perform_soft_reset` commands are not acknowledged by the sensor, and the driver handles this accordingly.
- Temperature and humidity values in measurements are reported by an external SHT4x sensor; use `set_rht_compensation` if not using the STCC4’s dedicated controller interface for SHT4x.

## Links

- Datasheet: https://sensirion.com/resource/datasheet/STCC4 (developed on datasheet version 1)

## License

This library is licensed under either of:

- MIT license (LICENSE-MIT or http://opensource.org/licenses/MIT)
- Apache License, Version 2.0 (LICENSE-APACHE or http://www.apache.org/licenses/LICENSE-2.0)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this work is dual licensed as above, without any additional terms or conditions.

## Change Log
- Version 0.1.0 - Initial release