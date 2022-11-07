# mcp2515-arduino

Example utilising an MCP2515 with an Arduino Uno. Based off [avr-hal-template].

## Usage

1. Install `ravedude` -- `cargo install ravedude`
2. Run `cargo run` to build and flash the firmware to a connected Arduino and
   open a serial monitor
   1. If no matching serial port is found, you will need to tell ravedude the
      path to your Arduino serial port via `RAVEDUDE_PORT="/dev/tty.<port>" cargo run`

[avr-hal-template]: https://github.com/Rahix/avr-hal-template
