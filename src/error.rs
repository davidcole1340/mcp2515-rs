use core::fmt::Debug;

use embedded_hal::can::Error as CanError;

use crate::{CanSpeed, McpSpeed};

pub type Result<T, SPI, HAL> = core::result::Result<T, Error<SPI, HAL>>;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
pub enum Error<SPI: Debug, HAL: Debug> {
    /// MCP2515 did not respond to mode change.
    NewModeTimeout,
    /// Tx buffers are full and therefore cannot send another message.
    TxBusy,
    /// Failed to send a message.
    TxFailed,
    /// There was no message to be received in the Rx buffers.
    NoMessage,
    /// Received an invalid frame ID.
    InvalidFrameId,
    /// Received an invalid DLC (CAN frame data length).
    InvalidDlc,
    /// Invalid configuration options.
    InvalidConfiguration(CanSpeed, McpSpeed),
    /// SPI error.
    Spi(SPI),
    /// Error from HAL crate.
    Hal(HAL),
}

impl<SPI: Debug, HAL: Debug> CanError for Error<SPI, HAL> {
    fn kind(&self) -> embedded_hal::can::ErrorKind {
        embedded_hal::can::ErrorKind::Other
    }
}
