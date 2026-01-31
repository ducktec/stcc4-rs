#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_nrf::twim::{self, Twim};
use embassy_time::{Delay, Timer};
use panic_probe as _;
use stcc4_rs::asynchronous::Stcc4;

bind_interrupts!(struct Irqs {
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 =>
        embassy_nrf::twim::InterruptHandler<embassy_nrf::peripherals::TWISPI0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    let mut config = twim::Config::default();
    config.frequency = twim::Frequency::K100;
    config.sda_pullup = true;
    config.scl_pullup = true;

    let i2c = Twim::new(p.TWISPI0, Irqs, p.P0_06, p.P0_26, config);
    let delay = Delay;

    let mut stcc4 = Stcc4::new(delay, i2c);

    info!("STCC4: reset");
    stcc4.perform_soft_reset().await.ok();

    info!("STCC4: read product id");
    match stcc4.get_product_id().await {
        Ok(p) => info!(
            "STCC4: product_id=0x{=u32:08X} serial=0x{=u64:016X}",
            p.product_id, p.serial_number
        ),
        Err(_) => info!("STCC4: get_product_id failed"),
    }

    // Recommended after >3h idle/power-off to improve CO2 sensing performance.
    info!("STCC4: conditioning (22s)...");
    match stcc4.perform_conditioning().await {
        Ok(_) => info!("STCC4: conditioning complete"),
        Err(_) => info!("STCC4: conditioning failed"),
    }

    info!("STCC4: start continuous measurement");
    stcc4.start_continuous_measurement().await.ok();

    loop {
        match stcc4.read_measurement().await {
            Ok(m) => {
                info!(
                    "CO2={} ppm T={} C RH={} % status=0x{=u16:04X}",
                    m.co2_ppm, m.temperature_c, m.humidity_percent, m.status.raw
                );
            }
            Err(_e) => {
                info!("STCC4: read_measurement failed");
            }
        }

        Timer::after_millis(1000).await;
    }
}
