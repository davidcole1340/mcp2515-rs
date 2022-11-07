# mcp2515-rp-pico

Example utilising an MCP2515 with a Raspberry Pi Pico. Based off [rp2040-project-template].

## Usage

1. Install dependencies

```
rustup target install thumbv6m-none-eabi
cargo install flip-link
cargo install probe-run
```

2. Run `cargo run` to build and flash the firmware to the Raspberry Pi Pico

[rp2040-project-template]: https://github.com/rp-rs/rp2040-project-template
