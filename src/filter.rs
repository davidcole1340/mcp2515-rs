use embedded_hal::can::Id;
use modular_bitfield::prelude::*;

use crate::regs::Register;

/// Filter register.
///
/// Occupies 4 registers (SIDH, SIDL, EID8 and EID0).
#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, BitfieldSpecifier)]
pub struct RxFilterReg {
    // Rx filter actually consists of 4 registers (RXFn{SIDH,SIDL,EID8,EID0}),
    // however we concatenate them into one register and use `Filt` to choose `n`.
    /// Extended identifier filter.
    pub eid: B18,
    #[skip]
    __: B1,
    /// Extended identifier enable.
    pub exide: bool,
    #[skip]
    __: B1,
    /// Standard identifier filter.
    pub sid: B11,
}

impl RxFilterReg {
    /// Creates an Rx filter register from a CAN bus ID.
    pub fn from_id(id: Id) -> Self {
        // In standard mode: `exide == false` and `eid` doesn't matter. Filter goes into
        // `sid`.
        // In extended mode: `exide == true` and the lower 18 bits of the filter go into
        // `eid`. The rest (upper 11 bits) go into `sid`.
        match id {
            Id::Standard(bits) => RxFilterReg::new()
                .with_exide(false)
                .with_eid(0)
                .with_sid(bits.as_raw()),
            Id::Extended(bits) => RxFilterReg::new()
                .with_exide(true)
                .with_eid(bits.as_raw() & 0x3FFFF) // Lower 18 bits go into EID
                .with_sid((bits.as_raw() >> 18) as u16), // Upper 11 bits go into SID
        }
    }
}

/// Mask register.
///
/// Occupies 4 registers (SIDH, SIDL, EID8 and EID0).
#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq, BitfieldSpecifier)]
pub struct RxMaskReg {
    /// Extended identifier mask.
    pub eid: B18,
    #[skip]
    __: B3,
    /// Standard identifier mask.
    pub sid: B11,
}

impl RxMaskReg {
    /// Creates an Rx mask register from a CAN bus ID.
    pub fn from_id(id: Id) -> Self {
        // As above comments.
        match id {
            Id::Standard(bits) => RxMaskReg::new().with_eid(0).with_sid(bits.as_raw()),
            Id::Extended(bits) => RxMaskReg::new()
                .with_eid(bits.as_raw() & 0x3FFFF)
                .with_sid((bits.as_raw() >> 18) as u16),
        }
    }
}

crate::filter_def! {
    /// Receive filters.
    RxFilter(4) => {
        /// RXF0
        F0 => [Register::RXF0EID0, Register::RXF0EID8, Register::RXF0SIDL, Register::RXF0SIDH],
        /// RXF1
        F1 => [Register::RXF1EID0, Register::RXF1EID8, Register::RXF1SIDL, Register::RXF1SIDH],
        /// RXF2
        F2 => [Register::RXF2EID0, Register::RXF2EID8, Register::RXF2SIDL, Register::RXF2SIDH],
        /// RXF3
        F3 => [Register::RXF3EID0, Register::RXF3EID8, Register::RXF3SIDL, Register::RXF3SIDH],
        /// RXF4
        F4 => [Register::RXF4EID0, Register::RXF4EID8, Register::RXF4SIDL, Register::RXF4SIDH],
        /// RXF5
        F5 => [Register::RXF5EID0, Register::RXF5EID8, Register::RXF5SIDL, Register::RXF5SIDH]
    }
}

crate::filter_def! {
    /// Receive masks.
    RxMask(4) => {
        /// Mask 0
        Mask0 => [Register::RXM0EID0, Register::RXM0EID8, Register::RXM0SIDL, Register::RXM0SIDH],
        /// Mask 1
        Mask1 => [Register::RXM1EID0, Register::RXM1EID8, Register::RXM1SIDL, Register::RXM1SIDH]
    }
}
