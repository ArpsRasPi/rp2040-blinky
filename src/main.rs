//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
//! This example uses PIO to blink the LED instead of turning it on and off directly.
#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::{
    entry,
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::{FunctionPio0, Pin},
        pac,
        pio::{PIOBuilder, PIOExt, PinDir},
        sio::Sio,
        watchdog::Watchdog,
    },
    Pins,
};

use pio_proc::pio_file;

use cortex_m::delay::Delay;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.
    let led_pin: Pin<_, FunctionPio0, _> = pins.led.into_function();

    let program_with_defines = pio_file!(
        "./src/blinky.pio",
        select_program("blinky"),
        options(max_program_size = 32)
    );
    let program = program_with_defines.program;

    // Initialise and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program).unwrap();
    let (int, frac) = (0, 0); // as slow as possible (0 is interpreted as 65536)
    let (mut sm, mut _rx, mut tx) = PIOBuilder::from_program(installed)
        .out_pins(led_pin.id().num, 1)
        .clock_divisor_fixed_point(int, frac)
        .build(sm0);

    sm.set_pindirs([(led_pin.id().num, PinDir::Output)]);
    sm.start();

    loop {
        info!("on!");
        // Push all 1s to the FIFO queue and wait.
        tx.write(u32::MAX);
        delay.delay_ms(100);
        info!("off!");
        // Push all 0s to the FIFI queue and wait
        tx.write(0);
        delay.delay_ms(100);
    }
}

// End of file
