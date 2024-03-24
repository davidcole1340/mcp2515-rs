#![no_std]
#![no_main]

// Import necessary crates and modules
use embedded_hal::can::{ExtendedId, Frame, Id};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    spi::{master::Spi, SpiMode},
    Delay, IO,
}; 
use esp_println::println;
use mcp2515::{error::Error, frame::CanFrame, regs::OpMode, CanSpeed, McpSpeed, MCP2515};


// Entry point for the program
#[entry]
fn main() -> ! {
    // Take the peripherals from the ESP32
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let mut clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Initialize the IO pins
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Define SPI pins
    let sclk = io.pins.gpio18; // Clock pin
    let miso = io.pins.gpio19; // Master In Slave Out pin
    let mosi = io.pins.gpio23; // Master Out Slave In pin
    let cs_pin = io.pins.gpio5.into_push_pull_output(); // Chip Select pin for MCP2515

    // Initialize the SPI interface
    let spi = Spi::new(
        peripherals.SPI2,
        100u32.kHz(),   
        SpiMode::Mode0, 
        &mut clocks,    
    )
    .with_sck(sclk) // Bind the SCLK pin to the SPI clock
    .with_mosi(mosi) // Bind the MOSI pin to the SPI master out slave in
    .with_miso(miso);

    // Initialize the MCP2515 CAN controller with the SPI interface and CS pin
    let mut can = MCP2515::new(spi, cs_pin);
    // Initialize a delay provider
    let mut delay = Delay::new(&clocks);

    match can.init(
        &mut delay,
        mcp2515::Settings {
            mode: OpMode::Loopback,      
            can_speed: CanSpeed::Kbps100, 
            mcp_speed: McpSpeed::MHz8,   
            clkout_en: false,
        },
    ) {
        Ok(_) => println!("MCP2515 initialized successfully"),
        Err(e) => println!("Failed to initialize MCP2515: {:?}", e),
    }

    // Infinite loop to keep the program running
    loop {
        // Send a message
        let frame = CanFrame::new(
            Id::Extended(ExtendedId::MAX),
            &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
        )
        .unwrap();
        can.send_message(frame).unwrap();
        println!("Sent message!");

        // Read the message back (we are in loopback mode)
        match can.read_message() {
            Ok(frame) => println!("Received frame {:?}", frame),
            Err(Error::NoMessage) => println!("Error No message to read! = SKILL ISSUE 🫵"),
            Err(_) => panic!("Oh no!"),
        }

        delay.delay_ms(1000u32); // Delay for 1 second
    }
}
