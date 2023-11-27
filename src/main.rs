//! Simple i2c protocol experiments

// This is needed by RTIC.
#![feature(type_alias_impl_trait)]
#![no_std]
#![no_main]

use adafruit_feather_rp2040 as bsp;

use panic_probe as _;
use defmt_rtt as _;

#[rtic::app(
    device = crate::bsp::pac,
)]
mod app {
    use crate::bsp;
    use bsp::{
        hal::{
            self,
            clocks::init_clocks_and_plls,
            Sio,
            I2C,
            Clock,
            gpio::{
                Pin,
                bank0::{Gpio2, Gpio3, Gpio13},
                FunctionI2c,
                PullUp, FunctionSio, PullDown, SioOutput,
            }, i2c::peripheral::{I2CPeripheralEventIterator, I2CEvent},
        },
        XOSC_CRYSTAL_FREQ,
        pac::I2C1,
    };
    use defmt::info;
    use embedded_hal::digital::v2::PinState;
    use embedded_hal::digital::v2::OutputPin;
    use embedded_hal::digital::v2::InputPin;
    // use embedded_hal::blocking::i2c::Write;
    // use embedded_hal::blocking::i2c::Read;
    use fugit::RateExtU32;
    use rtic_monotonics::rp2040::Timer;
    use rtic_monotonics::Monotonic;
    use rtic_monotonics::rp2040::ExtU64;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        i2c: Option<I2C<I2C1, (
            Pin<Gpio2, FunctionI2c, PullUp>,
            Pin<Gpio3, FunctionI2c, PullUp>,
        )>>,
        i2c_target: Option<I2CPeripheralEventIterator<
                I2C1, (Pin<Gpio2, FunctionI2c, PullUp>,
                       Pin<Gpio3, FunctionI2c, PullUp>)>>,
        blink: Pin<Gpio13, FunctionSio<SioOutput>, PullDown>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        // RP2040 workaround. Not really necessary with a probe that does proper
        // resets.
        unsafe {
            bsp::hal::sio::spinlock_reset();
        }

        info!("Init");

        let rp2040_timer_token = rtic_monotonics::create_rp2040_monotonic_token!();
        Timer::start(ctx.device.TIMER, &mut ctx.device.RESETS, rp2040_timer_token);

        let mut watchdog = hal::Watchdog::new(ctx.device.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog
        )
        .ok()
        .unwrap();

        let sio = Sio::new(ctx.device.SIO);

        let pins = bsp::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );

        // Distinguish between the controller and the peripheral based on
        // whether gpio 7 is pulled to ground.
        let mode_select = pins.d5.into_pull_up_input();
        // Delay just a bit to allow this to settle.
        cortex_m::asm::delay(5_000);
        let mode = mode_select.is_low().unwrap();
        if mode {
            info!("Pulldown present, we are peripheral");
        } else {
            info!("Pulldown absent, we are controller");
        }

        // Turn on the LED so that we can tell this is the controller.
        let blink = pins.d13.into_push_pull_output_in_state(PinState::High);

        #[cfg(feature = "controller")]
        let i2c = Some(I2C::i2c1(
            ctx.device.I2C1,
            pins.sda.reconfigure(),
            pins.scl.reconfigure(),
            400.kHz(),
            &mut ctx.device.RESETS,
            clocks.peripheral_clock.freq()));
        #[cfg(not(feature = "controller"))]
        let i2c = None;
        #[cfg(feature = "target")]
        let i2c_target = Some(I2C::new_peripheral_event_iterator(
            ctx.device.I2C1,
            pins.sda.reconfigure(),
            pins.scl.reconfigure(),
            &mut ctx.device.RESETS,
            0x2c));
        #[cfg(not(feature = "target"))]
        let i2c_target = None;

        // Just write a small pattern out.
        /*
        use embedded_hal::blocking::i2c::Write;
        i2c.write(0x2c, &[1, 2, 3]).unwrap();
         */

        info!("End of init");

        i2c_thing::spawn().unwrap();
        i2c_target::spawn().unwrap();
        blinky::spawn().unwrap();

        (
            Shared {},
            Local {
                blink,
                i2c,
                i2c_target,
            },
        )
    }

    #[task(local = [i2c])]
    async fn i2c_thing(ctx: i2c_thing::Context) {
        if let Some(dev) = ctx.local.i2c {
            let mut next = Timer::now();
            loop {
                next += 1000.millis();
                Timer::delay_until(next).await;

                use embedded_hal::blocking::i2c::Write;
                use embedded_hal::blocking::i2c::Read;

                // First is a "command" to set the LEDs.
                info!("Write to i2c");
                // match dev.write(0x2c, &[0x00, 0x01, 0x03, 0x04]) {
                match dev.write(0x3c, &[0x00, 0xae]) {
                    Ok(()) => (),
                    Err(_) => {
                        info!("No i2c peripheral acked");
                        continue;
                    }
                }

                // Then, we want a write/read To get the status.
                let mut buf = [0x00; 1];
                info!("Read from i2c");
                match dev.read(0x3c, &mut buf) {
                    Ok(count) => {
                        info!("Scan result: {} bytes", count);
                    }
                    Err(_) => {
                        info!("No i2c response for scan");
                    }
                }
            }
        }
    }

    #[task(local = [i2c_target])]
    async fn i2c_target(ctx: i2c_target::Context) {
        if let Some(ref mut dev) = ctx.local.i2c_target {
            info!("Target mode");
            loop {
                if let Some(evt) = dev.next() {
                    match evt {
                        I2CEvent::Start => info!("evt: Start"),
                        I2CEvent::Restart => info!("evt: Restart"),
                        I2CEvent::TransferRead => {
                            info!("TransferRead");
                            // They are requesting some data from us.
                            let count = dev.write(&[8, 7, 6, 5]);
                            info!("Sent them {} bytes", count);
                        }
                        I2CEvent::TransferWrite => {
                            // They have sent data, so accept it.
                            let mut buf = [0u8; 8];
                            let count = dev.read(&mut buf);
                            info!("Sent: {} bytes", count);
                            for b in &buf[..count] {
                                info!("  byte: {:x}", b);
                            }
                        }
                        I2CEvent::Stop => info!("evt: Stop"),
                    }
                } else {
                    // info!("Some other event");
                }
            }
        }
    }

    #[task(local = [blink])]
    async fn blinky(ctx: blinky::Context) {
        let mut next = Timer::now();
        let mut state = PinState::High;
        loop {
            next += 1200.millis();
            Timer::delay_until(next).await;

            ctx.local.blink.set_state(state).unwrap();
            state = !state;
        }
    }
}
