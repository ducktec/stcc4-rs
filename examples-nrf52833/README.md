# STCC4 nRF52833 Example (Embassy)

This example shows how to use the STCC4 driver on an nRF52833 without a SoftDevice.

Targeted to https://github.com/ducktec/usense_v2, but should also work with some slight modifications on other nRF52833 boards (change SDA/SCL pins).

## Wiring

- SDA: P0.06
- SCL: P0.26

## Build and flash

```bash
cargo run -p stcc4-nrf52833-example
```

The runner is configured for `probe-run` in [.cargo/config.toml](.cargo/config.toml). Adjust as needed.

## Notes

- Memory layout is defined in `memory.x` (no SoftDevice).
- Logging uses `defmt` + `defmt-rtt`.
- Make sure to configure the defmt log level as needed with `export DEFMT_LOG=debug` (or `info`, `trace`, etc).
