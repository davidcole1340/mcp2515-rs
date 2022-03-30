use core::convert::Infallible;

use embedded_hal::can::Error as CanError;
use void::Void;

use crate::{CanSpeed, McpSpeed};

pub type Result<T> = core::result::Result<T, Error>;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Error {
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
}

impl From<Infallible> for Error {
    fn from(e: Infallible) -> Self {
        match e {}
    }
}

impl From<Void> for Error {
    fn from(e: Void) -> Self {
        match e {}
    }
}

impl CanError for Error {
    fn kind(&self) -> embedded_hal::can::ErrorKind {
        embedded_hal::can::ErrorKind::Other
    }
}
