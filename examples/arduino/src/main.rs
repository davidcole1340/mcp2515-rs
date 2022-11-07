#![no_std]
#![no_main]

use panic_halt as _;

use arduino_hal::{spi::Settings, Delay, Spi};
use embedded_hal::can::{ExtendedId, Frame, Id};
use mcp2515::{error::Error, frame::CanFrame, regs::OpMode, CanSpeed, McpSpeed, MCP2515};

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let mut delay = Delay::new();

    let mut serial = arduino_hal::default_serial!(dp, pins, 115200);
    let (spi, cs) = Spi::new(
        dp.SPI,
        pins.d13.into_output(),
        pins.d11.into_output(),
        pins.d12.into_pull_up_input(),
        pins.d10.into_output(),
        Settings {
            data_order: arduino_hal::spi::DataOrder::MostSignificantFirst,
            clock: arduino_hal::spi::SerialClockRate::OscfOver128,
            mode: embedded_hal::spi::MODE_0,
        },
    );
    let mut can = MCP2515::new(spi, cs);
    can.init(
        &mut delay,
        mcp2515::Settings {
            mode: OpMode::Loopback,       // Loopback for testing and example
            can_speed: CanSpeed::Kbps100, // Many options supported.
            mcp_speed: McpSpeed::MHz8,    // Currently 16MHz and 8MHz chips are supported.
            clkout_en: false,
        }
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
        ufmt::uwriteln!(&mut serial, "Sent message!").unwrap();

        // Read the message back (we are in loopback mode)
        match can.read_message() {
            Ok(frame) => {
                ufmt::uwriteln!(&mut serial, "Received frame {:?}", frame).unwrap();
            }
            Err(Error::NoMessage) => ufmt::uwriteln!(&mut serial, "No message to read!").unwrap(),
            Err(_) => panic!("Oh no!"),
        }

        arduino_hal::delay_ms(1000);
    }
}
