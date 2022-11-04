use core::fmt::Debug;

use embedded_hal::can::{ExtendedId, Frame, Id, StandardId};
use modular_bitfield::prelude::*;

use crate::{
    error::{Error, Result},
    frame::CanFrame,
    regs::Register,
};

/// Tx buffer identification register.
#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
        let mut reg = TxBufIdent::new()
            .with_dlc(frame.dlc() as u8)
            .with_rtr(frame.is_remote_frame());
        match &frame.id() {
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
        B0 => [Register::TXB0DLC, Register::TXB0EID0, Register::TXB0EID8, Register::TXB0SIDL, Register::TXB0SIDH],
        /// Tx buffer 1.
        B1 => [Register::TXB1DLC, Register::TXB1EID0, Register::TXB1EID8, Register::TXB1SIDL, Register::TXB1SIDH],
        /// Tx buffer 2.
        B2 => [Register::TXB2DLC, Register::TXB2EID0, Register::TXB2EID8, Register::TXB2SIDL, Register::TXB2SIDH]
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

/// Rx buffer identification register.
#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RxBufIdent {
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
    /// Extended identifier flag.
    pub ide: bool,
    /// Standard frame remote transfer request.
    pub srr: bool,
    /// Standard identifier.
    pub sid: B11,
}

impl RxBufIdent {
    /// Attempt to convert a Rx buffer + data into a CAN frame.
    ///
    /// # Parameters
    ///
    /// * `read_data` - Function which reads from the corresponding `DATA`
    ///   register. The function should fill the mutable slice with bytes
    ///   received from the CAN bus.
    pub fn into_frame<SPIE: Debug, HALE: Debug>(
        self,
        read_data: impl FnOnce(&mut [u8]) -> Result<(), SPIE, HALE>,
    ) -> Result<CanFrame, SPIE, HALE> {
        let id = if self.ide() {
            let id = self.eid() | ((self.sid() as u32) << 18);
            Id::Extended(ExtendedId::new(id).ok_or(Error::InvalidFrameId)?)
        } else {
            Id::Standard(StandardId::new(self.sid()).ok_or(Error::InvalidFrameId)?)
        };
        let dlc = self.dlc();
        if dlc > 8 {
            return Err(Error::InvalidDlc);
        }
        let mut frame = CanFrame {
            id,
            rtr: self.srr() && !self.ide(),
            dlc,
            data: [0; 8],
        };
        read_data(&mut frame.data[..(dlc as usize)])?;
        Ok(frame)
    }
}

crate::filter_def! {
    /// Receive buffer.
    RxBuf(5) => {
        /// Rx buffer 0.
        B0 => [Register::RXB0DLC, Register::RXB0EID0, Register::RXB0EID8, Register::RXB0SIDL, Register::RXB0SIDH],
        /// Rx buffer 1.
        B1 => [Register::RXB1DLC, Register::RXB1EID0, Register::RXB1EID8, Register::RXB1SIDL, Register::RXB1SIDH]
    }
}

impl RxBuf {
    /// Returns the `CTRL` register for the selected Rx buffer.
    pub const fn ctrl(self) -> Register {
        match self {
            RxBuf::B0 => Register::RXB0CTRL,
            RxBuf::B1 => Register::RXB1CTRL,
        }
    }

    /// Returns the `DATA` register for the selected Rx buffer.
    pub const fn data(self) -> Register {
        match self {
            RxBuf::B0 => Register::RXB0DATA,
            RxBuf::B1 => Register::RXB1DATA,
        }
    }
}
