#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rp_pico as bsp;

use embedded_hal::can::{ExtendedId, Frame, Id};
use mcp2515::{error::Error, frame::CanFrame, regs::OpMode, CanSpeed, McpSpeed, Settings, MCP2515};

use defmt::{panic, *};
use fugit::RateExtU32;

use bsp::{
    entry,
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::FunctionSpi,
        pac,
        sio::Sio,
        watchdog::Watchdog,
        Spi,
    },
};

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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let _spi_sclk = pins.gpio2.into_mode::<FunctionSpi>();
    let _spi_mosi = pins.gpio3.into_mode::<FunctionSpi>();
    let _spi_miso = pins.gpio4.into_mode::<FunctionSpi>();
    let spi_cs = pins.gpio5.into_push_pull_output();

    let spi = Spi::<_, _, 8>::new(pac.SPI0).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        200_000.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let mut can = MCP2515::new(spi, spi_cs);
    can.init(
        &mut delay,
        Settings {
            mode: OpMode::Loopback,       // Loopback for testing and example
            can_speed: CanSpeed::Kbps100, // Many options supported.
            mcp_speed: McpSpeed::MHz8,    // Currently 16MHz and 8MHz chips are supported.
            clkout_en: false,
        },
    )
    .unwrap();

    loop {
        // Send a message
        let frame = CanFrame::new(
            Id::Extended(ExtendedId::MAX),
            &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
        )
        .unwrap();
        can.send_message(frame).unwrap();
        info!("Sent message!");

        // Read the message back (we are in loopback mode)
        match can.read_message() {
            Ok(frame) => info!("Received frame {:?}", frame),
            Err(Error::NoMessage) => info!("No message to read!"),
            Err(_) => panic!("Oh no!"),
        }

        delay.delay_ms(500);
    }
}
