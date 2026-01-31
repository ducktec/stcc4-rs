//! # Async API
//!
//! This module contains the async API for the STCC4 sensor.
//! It is based on the `embedded-hal-async` traits and is intended to be used
//! with asynchronous code.

use crate::{
    CommandId, EXIT_SLEEP_PAYLOAD, FrcCorrection, I2C_GENERAL_CALL_ADDR, MAX_RX_BYTES,
    MAX_TX_BYTES, Measurement, ModuleState, ProductId, Result, SOFT_RESET_CMD, STCC4_ADDR_DEFAULT,
    SelfTestResult, SensorStatus, Stcc4Error, crc_internal, frc_correction_from_raw,
    get_execution_time, humidity_percent_to_raw, pressure_pa_to_raw, raw_to_humidity_percent,
    raw_to_temperature_c, temperature_c_to_raw,
};

/// Represents an I2C-connected STCC4 sensor (async).
pub struct Stcc4<I2C, D> {
    delay: D,
    i2c: I2C,
    address: u8,
    state: ModuleState,
}

impl<I2C, D> Stcc4<I2C, D>
where
    D: embedded_hal_async::delay::DelayNs,
    I2C: embedded_hal_async::i2c::I2c,
{
    /// Create a new STCC4 instance with the default I2C address.
    pub fn new(delay: D, i2c: I2C) -> Self {
        Self {
            delay,
            i2c,
            address: STCC4_ADDR_DEFAULT,
            state: ModuleState::Idle,
        }
    }

    /// Create a new STCC4 instance with a custom I2C address.
    pub fn with_address(delay: D, i2c: I2C, address: u8) -> Self {
        Self {
            delay,
            i2c,
            address,
            state: ModuleState::Idle,
        }
    }

    /// Release the I2C bus and delay provider.
    pub fn release(self) -> (I2C, D) {
        (self.i2c, self.delay)
    }

    /// Start continuous measurement (1/second).
    pub async fn start_continuous_measurement(&mut self) -> Result<(), I2C::Error> {
        self.check_state(ModuleState::Idle)?;
        #[cfg(feature = "defmt")]
        defmt::debug!("STCC4_driver: start continuous measurement");
        let result = self.send_wait(CommandId::StartContinuousMeasurement).await;
        if result.is_ok() {
            self.state = ModuleState::Measuring;
        }
        result
    }

    /// Stop continuous measurement and return to idle.
    pub async fn stop_continuous_measurement(&mut self) -> Result<(), I2C::Error> {
        self.check_state(ModuleState::Measuring)?;
        #[cfg(feature = "defmt")]
        defmt::debug!("STCC4_driver: stop continuous measurement");
        let result = self.send_wait(CommandId::StopContinuousMeasurement).await;
        if result.is_ok() {
            self.state = ModuleState::Idle;
        }
        result
    }

    /// Read the latest measurement values.
    pub async fn read_measurement(&mut self) -> Result<Measurement, I2C::Error> {
        if self.state == ModuleState::Sleep {
            return Err(Stcc4Error::InvalidState);
        }

        let mut data = [0u16; 4];
        self.send_wait_read(CommandId::ReadMeasurement, &mut data)
            .await?;

        #[cfg(feature = "defmt")]
        defmt::debug!(
            "STCC4_driver: measurement raw co2={} t_raw={} rh_raw={} status=0x{=u16:04X}",
            data[0],
            data[1],
            data[2],
            data[3]
        );

        Ok(Measurement {
            co2_ppm: data[0],
            temperature_c: raw_to_temperature_c(data[1]),
            humidity_percent: raw_to_humidity_percent(data[2]),
            status: SensorStatus { raw: data[3] },
        })
    }

    /// Set external temperature and humidity compensation.
    pub async fn set_rht_compensation(
        &mut self,
        temperature_c: f32,
        humidity_percent: f32,
    ) -> Result<(), I2C::Error> {
        self.check_state(ModuleState::Idle)?;

        let temp_raw = temperature_c_to_raw(temperature_c);
        let rh_raw = humidity_percent_to_raw(humidity_percent);
        let data = [temp_raw, rh_raw];

        #[cfg(feature = "defmt")]
        defmt::debug!(
            "STCC4_driver: set rht compensation t_raw={} rh_raw={}",
            temp_raw,
            rh_raw
        );

        self.send_write_wait(CommandId::SetRhtCompensation, &data)
            .await
    }

    /// Set external pressure compensation.
    pub async fn set_pressure_compensation(&mut self, pressure_pa: u32) -> Result<(), I2C::Error> {
        self.check_state(ModuleState::Idle)?;
        let pressure_raw = pressure_pa_to_raw(pressure_pa);

        #[cfg(feature = "defmt")]
        defmt::debug!(
            "STCC4_driver: set pressure compensation p_raw={}",
            pressure_raw
        );

        self.send_write_wait(CommandId::SetPressureCompensation, &[pressure_raw])
            .await
    }

    /// Perform a single shot measurement (does not read the result).
    pub async fn measure_single_shot(&mut self) -> Result<(), I2C::Error> {
        self.check_state(ModuleState::Idle)?;
        #[cfg(feature = "defmt")]
        defmt::debug!("STCC4_driver: single shot measurement");
        self.send_wait(CommandId::MeasureSingleShot).await
    }

    /// Enter sleep mode.
    pub async fn enter_sleep_mode(&mut self) -> Result<(), I2C::Error> {
        self.check_state(ModuleState::Idle)?;
        #[cfg(feature = "defmt")]
        defmt::debug!("STCC4_driver: enter sleep mode");
        let result = self.send_wait(CommandId::EnterSleepMode).await;
        if result.is_ok() {
            self.state = ModuleState::Sleep;
        }
        result
    }

    /// Exit sleep mode (not acknowledged by the sensor).
    pub async fn exit_sleep_mode(&mut self) -> Result<(), I2C::Error> {
        self.check_state(ModuleState::Sleep)?;
        #[cfg(feature = "defmt")]
        defmt::debug!("STCC4_driver: exit sleep mode");

        let write_result = self.i2c.write(self.address, &[EXIT_SLEEP_PAYLOAD]).await;

        if let Err(_e) = write_result {
            #[cfg(feature = "defmt")]
            defmt::warn!("STCC4_driver: exit sleep mode not acknowledged");
        }

        self.delay.delay_ms(5).await;
        self.state = ModuleState::Idle;
        Ok(())
    }

    /// Perform conditioning sequence.
    pub async fn perform_conditioning(&mut self) -> Result<(), I2C::Error> {
        self.check_state(ModuleState::Idle)?;
        #[cfg(feature = "defmt")]
        defmt::debug!("STCC4_driver: perform conditioning");
        self.send_wait(CommandId::PerformConditioning).await
    }

    /// Perform a soft reset (I2C general call, not acknowledged).
    pub async fn perform_soft_reset(&mut self) -> Result<(), I2C::Error> {
        #[cfg(feature = "defmt")]
        defmt::debug!("STCC4_driver: perform soft reset");
        let write_result = self
            .i2c
            .write(I2C_GENERAL_CALL_ADDR, &[SOFT_RESET_CMD])
            .await;

        if let Err(_e) = write_result {
            #[cfg(feature = "defmt")]
            defmt::warn!("STCC4_driver: soft reset not acknowledged");
        }

        self.delay.delay_ms(10).await;
        self.state = ModuleState::Idle;
        Ok(())
    }

    /// Perform a factory reset.
    pub async fn perform_factory_reset(&mut self) -> Result<(), I2C::Error> {
        self.check_state(ModuleState::Idle)?;
        #[cfg(feature = "defmt")]
        defmt::debug!("STCC4_driver: perform factory reset");
        self.send_wait(CommandId::PerformFactoryReset).await
    }

    /// Perform a self test and return the result.
    pub async fn perform_self_test(&mut self) -> Result<SelfTestResult, I2C::Error> {
        self.check_state(ModuleState::Idle)?;
        #[cfg(feature = "defmt")]
        defmt::debug!("STCC4_driver: perform self test");
        let mut data = [0u16; 1];
        self.send_wait_read(CommandId::PerformSelfTest, &mut data)
            .await?;
        Ok(SelfTestResult { raw: data[0] })
    }

    /// Enable testing mode.
    pub async fn enable_testing_mode(&mut self) -> Result<(), I2C::Error> {
        self.check_state(ModuleState::Idle)?;
        #[cfg(feature = "defmt")]
        defmt::debug!("STCC4_driver: enable testing mode");
        self.send_wait(CommandId::EnableTestingMode).await
    }

    /// Disable testing mode.
    pub async fn disable_testing_mode(&mut self) -> Result<(), I2C::Error> {
        self.check_state(ModuleState::Idle)?;
        #[cfg(feature = "defmt")]
        defmt::debug!("STCC4_driver: disable testing mode");
        self.send_wait(CommandId::DisableTestingMode).await
    }

    /// Perform forced recalibration and return the applied correction.
    pub async fn perform_forced_recalibration(
        &mut self,
        target_co2_ppm: u16,
    ) -> Result<FrcCorrection, I2C::Error> {
        self.check_state(ModuleState::Idle)?;
        #[cfg(feature = "defmt")]
        defmt::debug!("STCC4_driver: perform forced recalibration");

        let data = [target_co2_ppm];
        self.send_write_wait_read(CommandId::PerformForcedRecalibration, &data, 1)
            .await
            .map(frc_correction_from_raw)
    }

    /// Get product ID and serial number.
    pub async fn get_product_id(&mut self) -> Result<ProductId, I2C::Error> {
        self.check_state(ModuleState::Idle)?;
        #[cfg(feature = "defmt")]
        defmt::debug!("STCC4_driver: get product id");

        let mut data = [0u16; 6];
        self.send_wait_read(CommandId::GetProductId, &mut data)
            .await?;

        let product_id = ((data[0] as u32) << 16) | (data[1] as u32);
        let serial_number = ((data[2] as u64) << 48)
            | ((data[3] as u64) << 32)
            | ((data[4] as u64) << 16)
            | (data[5] as u64);

        Ok(ProductId {
            product_id,
            serial_number,
        })
    }

    fn check_state(&self, required: ModuleState) -> Result<(), I2C::Error> {
        if self.state != required {
            return Err(Stcc4Error::InvalidState);
        }
        Ok(())
    }

    async fn send_wait(&mut self, command: CommandId) -> Result<(), I2C::Error> {
        self.write_command(command).await?;
        let delay_ms = get_execution_time(command);
        if delay_ms > 0 {
            #[cfg(feature = "defmt")]
            defmt::trace!("STCC4_driver: wait {} ms", delay_ms);
            self.delay.delay_ms(delay_ms).await;
        }
        Ok(())
    }

    async fn send_wait_read(
        &mut self,
        command: CommandId,
        data: &mut [u16],
    ) -> Result<(), I2C::Error> {
        self.write_command(command).await?;
        let delay_ms = get_execution_time(command);
        if delay_ms > 0 {
            #[cfg(feature = "defmt")]
            defmt::trace!("STCC4_driver: wait {} ms", delay_ms);
            self.delay.delay_ms(delay_ms).await;
        }
        self.read_words(data).await
    }

    async fn send_write_wait(
        &mut self,
        command: CommandId,
        data: &[u16],
    ) -> Result<(), I2C::Error> {
        self.write_command_with_words(command, data).await?;
        let delay_ms = get_execution_time(command);
        if delay_ms > 0 {
            #[cfg(feature = "defmt")]
            defmt::trace!("STCC4_driver: wait {} ms", delay_ms);
            self.delay.delay_ms(delay_ms).await;
        }
        Ok(())
    }

    async fn send_write_wait_read(
        &mut self,
        command: CommandId,
        data: &[u16],
        words_to_read: usize,
    ) -> Result<u16, I2C::Error> {
        self.write_command_with_words(command, data).await?;
        let delay_ms = get_execution_time(command);
        if delay_ms > 0 {
            #[cfg(feature = "defmt")]
            defmt::trace!("STCC4_driver: wait {} ms", delay_ms);
            self.delay.delay_ms(delay_ms).await;
        }

        let mut out = [0u16; 1];
        if words_to_read != 1 {
            return Err(Stcc4Error::InvalidData);
        }
        self.read_words(&mut out).await?;
        Ok(out[0])
    }

    async fn write_command(&mut self, command: CommandId) -> Result<(), I2C::Error> {
        #[cfg(feature = "defmt")]
        defmt::trace!("STCC4_driver: write cmd 0x{=u16:04X}", command as u16);

        self.i2c
            .write(self.address, &(command as u16).to_be_bytes())
            .await
            .map_err(Stcc4Error::WriteI2cError)
    }

    async fn write_command_with_words(
        &mut self,
        command: CommandId,
        data: &[u16],
    ) -> Result<(), I2C::Error> {
        let required_len = 2 + data.len() * 3;
        if required_len > MAX_TX_BYTES {
            return Err(Stcc4Error::InvalidData);
        }

        let mut buf = [0u8; MAX_TX_BYTES];
        let cmd_bytes = (command as u16).to_be_bytes();
        buf[0] = cmd_bytes[0];
        buf[1] = cmd_bytes[1];

        let mut offset = 2;
        for word in data {
            let bytes = word.to_be_bytes();
            buf[offset] = bytes[0];
            buf[offset + 1] = bytes[1];
            buf[offset + 2] = crc_internal::generate_crc(&bytes);
            offset += 3;
        }

        #[cfg(feature = "defmt")]
        defmt::trace!(
            "STCC4_driver: write cmd 0x{=u16:04X} ({} words)",
            command as u16,
            data.len()
        );

        self.i2c
            .write(self.address, &buf[..required_len])
            .await
            .map_err(Stcc4Error::WriteI2cError)
    }

    async fn read_words(&mut self, data: &mut [u16]) -> Result<(), I2C::Error> {
        let required_len = data.len() * 3;
        if required_len > MAX_RX_BYTES {
            return Err(Stcc4Error::InvalidData);
        }

        let mut raw = [0u8; MAX_RX_BYTES];
        self.i2c
            .read(self.address, &mut raw[..required_len])
            .await
            .map_err(Stcc4Error::ReadI2cError)?;

        crc_internal::validate_and_extract_data(&raw[..required_len], data)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::i2c::{ErrorKind, ErrorType, Operation};
    use futures::executor::block_on;

    #[derive(Debug)]
    struct MockError;

    impl embedded_hal::i2c::Error for MockError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    enum Transaction {
        Write(u8, Vec<u8>),
        Read(u8, Vec<u8>),
        WriteError(u8, Vec<u8>),
        ReadError(u8, usize),
    }

    struct I2cMock {
        transactions: Vec<Transaction>,
        index: usize,
    }

    impl I2cMock {
        fn new(transactions: Vec<Transaction>) -> Self {
            Self {
                transactions,
                index: 0,
            }
        }

        fn next(&mut self) -> &Transaction {
            let tx = self
                .transactions
                .get(self.index)
                .expect("missing transaction");
            self.index += 1;
            tx
        }

        fn done(self) {
            assert_eq!(self.index, self.transactions.len());
        }
    }

    impl ErrorType for I2cMock {
        type Error = MockError;
    }

    impl embedded_hal_async::i2c::I2c for I2cMock {
        async fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> core::result::Result<(), Self::Error> {
            for op in operations.iter_mut() {
                match (self.next(), op) {
                    (Transaction::Write(exp_addr, exp_data), Operation::Write(data)) => {
                        assert_eq!(*exp_addr, address);
                        assert_eq!(exp_data.as_slice(), *data);
                    }
                    (Transaction::Read(exp_addr, exp_data), Operation::Read(data)) => {
                        assert_eq!(*exp_addr, address);
                        assert_eq!(exp_data.len(), data.len());
                        data.copy_from_slice(exp_data);
                    }
                    (Transaction::WriteError(exp_addr, exp_data), Operation::Write(data)) => {
                        assert_eq!(*exp_addr, address);
                        assert_eq!(exp_data.as_slice(), *data);
                        return Err(MockError);
                    }
                    (Transaction::ReadError(exp_addr, exp_len), Operation::Read(data)) => {
                        assert_eq!(*exp_addr, address);
                        assert_eq!(*exp_len, data.len());
                        return Err(MockError);
                    }
                    _ => panic!("transaction mismatch"),
                }
            }

            Ok(())
        }
    }

    struct NoopDelay;

    impl embedded_hal_async::delay::DelayNs for NoopDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
        async fn delay_us(&mut self, _us: u32) {}
        async fn delay_ms(&mut self, _ms: u32) {}
    }

    fn word_with_crc(word: u16) -> [u8; 3] {
        let bytes = word.to_be_bytes();
        let crc = crate::crc_internal::generate_crc(&bytes);
        [bytes[0], bytes[1], crc]
    }

    #[test]
    /// Verifies frame encoding for RHT compensation (async).
    fn test_set_rht_compensation_frame_async() {
        let temp_raw = crate::temperature_c_to_raw(25.0);
        let rh_raw = crate::humidity_percent_to_raw(50.0);

        let mut expected = vec![0xE0, 0x00];
        expected.extend_from_slice(&word_with_crc(temp_raw));
        expected.extend_from_slice(&word_with_crc(rh_raw));

        let transactions = vec![Transaction::Write(STCC4_ADDR_DEFAULT, expected)];
        let i2c = I2cMock::new(transactions);
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);

        block_on(async {
            stcc4.set_rht_compensation(25.0, 50.0).await.unwrap();
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }

    #[test]
    /// Verifies measurement read parsing and conversions (async).
    fn test_read_measurement_frame_and_parse_async() {
        let co2 = 500_u16;
        let temp_raw = crate::temperature_c_to_raw(25.0);
        let rh_raw = crate::humidity_percent_to_raw(50.0);
        let status = 0x0040_u16;

        let mut read_bytes = Vec::new();
        read_bytes.extend_from_slice(&word_with_crc(co2));
        read_bytes.extend_from_slice(&word_with_crc(temp_raw));
        read_bytes.extend_from_slice(&word_with_crc(rh_raw));
        read_bytes.extend_from_slice(&word_with_crc(status));

        let transactions = vec![
            Transaction::Write(STCC4_ADDR_DEFAULT, vec![0xEC, 0x05]),
            Transaction::Read(STCC4_ADDR_DEFAULT, read_bytes),
        ];

        let i2c = I2cMock::new(transactions);
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);

        block_on(async {
            let measurement = stcc4.read_measurement().await.unwrap();
            assert_eq!(measurement.co2_ppm, co2);
            assert!(measurement.temperature_c > 24.0 && measurement.temperature_c < 26.0);
            assert!(measurement.humidity_percent > 49.0 && measurement.humidity_percent < 51.0);
            assert!(measurement.status.testing_mode());
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }

    #[test]
    /// Verifies sleep entry/exit state transitions (async).
    fn test_enter_exit_sleep_mode_async() {
        let transactions = vec![
            Transaction::Write(STCC4_ADDR_DEFAULT, vec![0x36, 0x50]),
            Transaction::Write(STCC4_ADDR_DEFAULT, vec![0x00]),
        ];

        let i2c = I2cMock::new(transactions);
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);

        block_on(async {
            stcc4.enter_sleep_mode().await.unwrap();
            assert_eq!(stcc4.state, ModuleState::Sleep);
            stcc4.exit_sleep_mode().await.unwrap();
            assert_eq!(stcc4.state, ModuleState::Idle);
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }

    #[test]
    /// Verifies factory reset command handling (async).
    fn test_perform_factory_reset_async() {
        let transactions = vec![Transaction::Write(STCC4_ADDR_DEFAULT, vec![0x36, 0x32])];
        let i2c = I2cMock::new(transactions);
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);

        block_on(async {
            stcc4.perform_factory_reset().await.unwrap();
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }

    #[test]
    /// Verifies pressure compensation command encoding (async).
    fn test_set_pressure_compensation_async() {
        let mut expected = vec![0xE0, 0x16];
        expected.extend_from_slice(&word_with_crc(50_650));

        let transactions = vec![Transaction::Write(STCC4_ADDR_DEFAULT, expected)];
        let i2c = I2cMock::new(transactions);
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);

        block_on(async {
            stcc4.set_pressure_compensation(101_300).await.unwrap();
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }

    #[test]
    /// Verifies product ID parsing (async).
    fn test_get_product_id_async() {
        let mut read_bytes = Vec::new();
        read_bytes.extend_from_slice(&word_with_crc(0x0901));
        read_bytes.extend_from_slice(&word_with_crc(0x018A));
        read_bytes.extend_from_slice(&word_with_crc(0x1122));
        read_bytes.extend_from_slice(&word_with_crc(0x3344));
        read_bytes.extend_from_slice(&word_with_crc(0x5566));
        read_bytes.extend_from_slice(&word_with_crc(0x7788));

        let transactions = vec![
            Transaction::Write(STCC4_ADDR_DEFAULT, vec![0x36, 0x5B]),
            Transaction::Read(STCC4_ADDR_DEFAULT, read_bytes),
        ];

        let i2c = I2cMock::new(transactions);
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);

        block_on(async {
            let product = stcc4.get_product_id().await.unwrap();
            assert_eq!(product.product_id, 0x0901_018A);
            assert_eq!(product.serial_number, 0x1122_3344_5566_7788);
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }

    #[test]
    /// Verifies self-test response handling (async).
    fn test_perform_self_test_async() {
        let mut read_bytes = Vec::new();
        read_bytes.extend_from_slice(&word_with_crc(0x0010));

        let transactions = vec![
            Transaction::Write(STCC4_ADDR_DEFAULT, vec![0x27, 0x8C]),
            Transaction::Read(STCC4_ADDR_DEFAULT, read_bytes),
        ];

        let i2c = I2cMock::new(transactions);
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);

        block_on(async {
            let result = stcc4.perform_self_test().await.unwrap();
            assert!(result.is_success());
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }

    #[test]
    /// Verifies forced recalibration flow and correction parsing (async).
    fn test_perform_forced_recalibration_async() {
        let mut write_bytes = vec![0x36, 0x2F];
        write_bytes.extend_from_slice(&word_with_crc(400));

        let mut read_bytes = Vec::new();
        read_bytes.extend_from_slice(&word_with_crc(32_668));

        let transactions = vec![
            Transaction::Write(STCC4_ADDR_DEFAULT, write_bytes),
            Transaction::Read(STCC4_ADDR_DEFAULT, read_bytes),
        ];

        let i2c = I2cMock::new(transactions);
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);

        block_on(async {
            let correction = stcc4.perform_forced_recalibration(400).await.unwrap();
            assert_eq!(correction.0, -100);
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }

    #[test]
    /// Verifies invalid state errors are raised (async).
    fn test_invalid_state_errors_async() {
        let i2c = I2cMock::new(Vec::new());
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);
        stcc4.state = ModuleState::Measuring;

        block_on(async {
            assert!(matches!(
                stcc4.enter_sleep_mode().await,
                Err(Stcc4Error::InvalidState)
            ));
            assert!(matches!(
                stcc4.set_pressure_compensation(101_300).await,
                Err(Stcc4Error::InvalidState)
            ));
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }

    #[test]
    /// Verifies start/stop measurement state transitions (async).
    fn test_start_stop_measurement_async() {
        let transactions = vec![
            Transaction::Write(STCC4_ADDR_DEFAULT, vec![0x21, 0x8B]),
            Transaction::Write(STCC4_ADDR_DEFAULT, vec![0x3F, 0x86]),
        ];

        let i2c = I2cMock::new(transactions);
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);

        block_on(async {
            stcc4.start_continuous_measurement().await.unwrap();
            assert_eq!(stcc4.state, ModuleState::Measuring);
            stcc4.stop_continuous_measurement().await.unwrap();
            assert_eq!(stcc4.state, ModuleState::Idle);
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }

    #[test]
    /// Verifies single-shot and conditioning commands (async).
    fn test_measure_single_shot_and_conditioning_async() {
        let transactions = vec![
            Transaction::Write(STCC4_ADDR_DEFAULT, vec![0x21, 0x9D]),
            Transaction::Write(STCC4_ADDR_DEFAULT, vec![0x29, 0xBC]),
        ];

        let i2c = I2cMock::new(transactions);
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);

        block_on(async {
            stcc4.measure_single_shot().await.unwrap();
            stcc4.perform_conditioning().await.unwrap();
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }

    #[test]
    /// Verifies enable/disable testing mode commands (async).
    fn test_enable_disable_testing_mode_async() {
        let transactions = vec![
            Transaction::Write(STCC4_ADDR_DEFAULT, vec![0x3F, 0xBC]),
            Transaction::Write(STCC4_ADDR_DEFAULT, vec![0x3F, 0x3D]),
        ];

        let i2c = I2cMock::new(transactions);
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);

        block_on(async {
            stcc4.enable_testing_mode().await.unwrap();
            stcc4.disable_testing_mode().await.unwrap();
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }

    #[test]
    /// Verifies read_measurement fails in sleep mode (async).
    fn test_read_measurement_invalid_state_async() {
        let i2c = I2cMock::new(Vec::new());
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);
        stcc4.state = ModuleState::Sleep;

        block_on(async {
            assert!(matches!(
                stcc4.read_measurement().await,
                Err(Stcc4Error::InvalidState)
            ));
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }

    #[test]
    /// Verifies custom I2C address usage (async).
    fn test_with_address_async() {
        let transactions = vec![Transaction::Write(crate::STCC4_ADDR_ALT, vec![0x21, 0x8B])];
        let i2c = I2cMock::new(transactions);
        let mut stcc4 = Stcc4::with_address(NoopDelay, i2c, crate::STCC4_ADDR_ALT);

        block_on(async {
            stcc4.start_continuous_measurement().await.unwrap();
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }

    #[test]
    /// Verifies read/write I2C error mapping (async).
    fn test_error_mapping_async() {
        let transactions = vec![Transaction::WriteError(
            STCC4_ADDR_DEFAULT,
            vec![0x21, 0x8B],
        )];
        let i2c = I2cMock::new(transactions);
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);

        block_on(async {
            assert!(matches!(
                stcc4.start_continuous_measurement().await,
                Err(Stcc4Error::WriteI2cError(_))
            ));
        });

        let (i2c, _) = stcc4.release();
        i2c.done();

        let transactions = vec![
            Transaction::Write(STCC4_ADDR_DEFAULT, vec![0xEC, 0x05]),
            Transaction::ReadError(STCC4_ADDR_DEFAULT, 12),
        ];
        let i2c = I2cMock::new(transactions);
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);

        block_on(async {
            assert!(matches!(
                stcc4.read_measurement().await,
                Err(Stcc4Error::ReadI2cError(_))
            ));
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }

    #[test]
    /// Verifies exit sleep ignores missing ack (async).
    fn test_exit_sleep_mode_no_ack_async() {
        let transactions = vec![Transaction::WriteError(STCC4_ADDR_DEFAULT, vec![0x00])];
        let i2c = I2cMock::new(transactions);
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);
        stcc4.state = ModuleState::Sleep;

        block_on(async {
            stcc4.exit_sleep_mode().await.unwrap();
            assert_eq!(stcc4.state, ModuleState::Idle);
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }

    #[test]
    /// Verifies soft reset ignores missing ack (async).
    fn test_perform_soft_reset_no_ack_async() {
        let transactions = vec![Transaction::WriteError(I2C_GENERAL_CALL_ADDR, vec![0x06])];
        let i2c = I2cMock::new(transactions);
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);

        block_on(async {
            stcc4.perform_soft_reset().await.unwrap();
            assert_eq!(stcc4.state, ModuleState::Idle);
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }

    #[test]
    /// Verifies internal length validation guards (async).
    fn test_internal_invalid_lengths_async() {
        let i2c = I2cMock::new(Vec::new());
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);

        block_on(async {
            let mut big_buf = [0u16; 7];
            assert!(matches!(
                stcc4.read_words(&mut big_buf).await,
                Err(Stcc4Error::InvalidData)
            ));

            assert!(matches!(
                stcc4
                    .write_command_with_words(CommandId::SetRhtCompensation, &[0u16; 3])
                    .await,
                Err(Stcc4Error::InvalidData)
            ));
        });

        let (i2c, _) = stcc4.release();
        i2c.done();

        let mut write_bytes = vec![0x36, 0x2F];
        write_bytes.extend_from_slice(&word_with_crc(400));
        let i2c = I2cMock::new(vec![Transaction::Write(STCC4_ADDR_DEFAULT, write_bytes)]);
        let mut stcc4 = Stcc4::new(NoopDelay, i2c);

        block_on(async {
            assert!(matches!(
                stcc4
                    .send_write_wait_read(CommandId::PerformForcedRecalibration, &[400], 2)
                    .await,
                Err(Stcc4Error::InvalidData)
            ));
        });

        let (i2c, _) = stcc4.release();
        i2c.done();
    }
}
