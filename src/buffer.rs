use embedded_hal::can::Id;
use modular_bitfield::prelude::*;
use ufmt::derive::uDebug;

use crate::{frame::CanFrame, regs::Register};

/// Tx buffer identification register.
#[bitfield]
#[derive(uDebug, Clone, Copy, PartialEq, Eq, BitfieldSpecifier)]
pub struct TxBufIdent {
    /// Size of data packet, 0-8. Anything above 8 is ignored.
    pub dlc: B4,
    #[skip]
    __: B2,
    /// Remote transmission request.
    pub rtr: bool,
    #[skip]
    __: B1,
    /// Extended identifier.
    pub eid: B18,
    #[skip]
    __: B1,
    /// Extended identifier enable.
    pub exide: bool,
    #[skip]
    __: B1,
    /// Standard identifier.
    pub sid: B11,
}

impl TxBufIdent {
    /// Creates a Tx buffer identification register from a CAN bus ID.
    pub fn from_frame(frame: &CanFrame) -> Self {
        // In standard mode: `exide == false` and `eid` doesn't matter. Filter goes into
        // `sid`.
        // In extended mode: `exide == true` and the lower 18 bits of the filter go into
        // `eid`. The rest (upper 11 bits) go into `sid`.
        let mut reg = TxBufIdent::new().with_dlc(frame.dlc).with_rtr(frame.rtr);
        match &frame.id {
            Id::Standard(id) => {
                reg.set_exide(false);
                reg.set_sid(id.as_raw());
            }
            Id::Extended(id) => {
                reg.set_exide(true);
                reg.set_eid(id.as_raw() & 0x3FFFF); // Lower 18 bits in EID
                reg.set_sid((id.as_raw() >> 18) as u16); // Upper 11 bits in SID
            }
        };
        reg
    }
}

crate::filter_def! {
    /// Transmit buffer.
    TxBuf(5) => {
        /// Tx buffer 0.
        B0 => [Register::TXB0SIDH, Register::TXB0SIDL, Register::TXB0EID8, Register::TXB0EID0, Register::TXB0DLC],
        /// Tx buffer 1.
        B1 => [Register::TXB1SIDH, Register::TXB1SIDL, Register::TXB1EID8, Register::TXB1EID0, Register::TXB1DLC],
        /// Tx buffer 2.
        B2 => [Register::TXB2SIDH, Register::TXB2SIDL, Register::TXB2EID8, Register::TXB2EID0, Register::TXB2DLC]
    }
}

impl TxBuf {
    /// Returns the `CTRL` register for the selected Tx buffer.
    pub const fn ctrl(self) -> Register {
        match self {
            TxBuf::B0 => Register::TXB0CTRL,
            TxBuf::B1 => Register::TXB1CTRL,
            TxBuf::B2 => Register::TXB2CTRL,
        }
    }

    /// Returns the `DATA` register for the selected Tx buffer.
    pub const fn data(self) -> Register {
        match self {
            TxBuf::B0 => Register::TXB0DATA,
            TxBuf::B1 => Register::TXB1DATA,
            TxBuf::B2 => Register::TXB2DATA,
        }
    }
}
