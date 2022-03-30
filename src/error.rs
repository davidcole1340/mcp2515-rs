use core::convert::Infallible;

use ufmt::derive::uDebug;
use void::Void;

use crate::{CanSpeed, McpSpeed};

pub type Result<T> = core::result::Result<T, Error>;

#[derive(uDebug, Debug)]
pub enum Error {
    NewModeTimeout,
    TxBusy,
    TxFailed,
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
