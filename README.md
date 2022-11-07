# MCP2515

`#![no_std]` library for interacting with MCP2515 CAN controller chips.
Platform-agnostic, tested with `arduino-hal` on ATmega2560.

Inspired by the following C++ libraries:

- [`MCP_CAN_lib`](https://github.com/coryjfowler/MCP_CAN_lib)
- [`arduino-MCP2515`](https://github.com/autowp/arduino-mcp2515)

## Cargo Features

All features are disabled by default.

- `defmt` - Implements `defmt::Format` for most public types so they can be
  printed using `defmt::info!()` and relatives
- `ufmt` - Implements `ufmt::uDebug` for most public types so they can be
  printed using `ufmt::uwriteln!()` and relatives

## Examples

Examples for some common microcontrollers are available in the `examples/`
folder.

- [Arduino Uno](./examples/arduino/)
  - Easily adapted to other AVR-based boards
- [Raspberry Pi Pico](./examples/rp-pico/)
  - Easily adapted to other RP2040-based boards

## Usage

Import the relevant HAL crate for your platform. For this example I'm using
`arduino-hal` on an ATmega2560.

```rs
#![no_std]
#![no_main]

use arduino_hal::{spi::Settings, Delay, Spi};
use core::fmt::Write;
use embedded_hal::can::{ExtendedId, Frame, Id};
use mcp2515::{error::Error, frame::CanFrame, regs::OpMode, CanSpeed, McpSpeed, MCP2515};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let mut delay = Delay::new();

    let mut serial = arduino_hal::default_serial!(dp, pins, 115200);
    let (spi, cs) = Spi::new(
        dp.SPI,
        pins.d52.into_output(),
        pins.d51.into_output(),
        pins.d50.into_pull_up_input(),
        pins.d53.into_output(),
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
        writeln!(&mut serial, "Sent message!").unwrap();

        // Read the message back (we are in loopback mode)
        match can.read_message() {
            Ok(frame) => {
                writeln!(&mut serial, "Received frame {:?}", frame).unwrap();
            }
            Err(Error::NoMessage) => writeln!(&mut serial, "No message to read!").unwrap(),
            Err(_) => panic!("Oh no!"),
        }

        arduino_hal::delay_ms(1000);
    }
}
```

Output over serial:

```text
Sent message!
No message to read!
Sent message!
Received frame CanFrame { id: Extended(ExtendedId(536870911)), rtr: false, dlc: 8, data: [1, 2, 3, 4, 5, 6, 7, 8] }
Sent message!
Received frame CanFrame { id: Extended(ExtendedId(536870911)), rtr: false, dlc: 8, data: [1, 2, 3, 4, 5, 6, 7, 8] }
Sent message!
Received frame CanFrame { id: Extended(ExtendedId(536870911)), rtr: false, dlc: 8, data: [1, 2, 3, 4, 5, 6, 7, 8] }
```

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE] or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT] or <http://opensource.org/licenses/MIT>)

at your option.
