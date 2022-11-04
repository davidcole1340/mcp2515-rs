//! MCP2515 registers.

use core::{fmt::Debug, ops::BitOr};

use modular_bitfield::prelude::*;

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Register {
    RXF0SIDH = 0x00,
    RXF0SIDL = 0x01,
    RXF0EID8 = 0x02,
    RXF0EID0 = 0x03,
    RXF1SIDH = 0x04,
    RXF1SIDL = 0x05,
    RXF1EID8 = 0x06,
    RXF1EID0 = 0x07,
    RXF2SIDH = 0x08,
    RXF2SIDL = 0x09,
    RXF2EID8 = 0x0A,
    RXF2EID0 = 0x0B,
    CANSTAT = 0x0E,
    CANCTRL = 0x0F,
    RXF3SIDH = 0x10,
    RXF3SIDL = 0x11,
    RXF3EID8 = 0x12,
    RXF3EID0 = 0x13,
    RXF4SIDH = 0x14,
    RXF4SIDL = 0x15,
    RXF4EID8 = 0x16,
    RXF4EID0 = 0x17,
    RXF5SIDH = 0x18,
    RXF5SIDL = 0x19,
    RXF5EID8 = 0x1A,
    RXF5EID0 = 0x1B,
    TEC = 0x1C,
    REC = 0x1D,
    RXM0SIDH = 0x20,
    RXM0SIDL = 0x21,
    RXM0EID8 = 0x22,
    RXM0EID0 = 0x23,
    RXM1SIDH = 0x24,
    RXM1SIDL = 0x25,
    RXM1EID8 = 0x26,
    RXM1EID0 = 0x27,
    CNF3 = 0x28,
    CNF2 = 0x29,
    CNF1 = 0x2A,
    CANINTE = 0x2B,
    CANINTF = 0x2C,
    EFLG = 0x2D,
    TXB0CTRL = 0x30,
    TXB0SIDH = 0x31,
    TXB0SIDL = 0x32,
    TXB0EID8 = 0x33,
    TXB0EID0 = 0x34,
    TXB0DLC = 0x35,
    TXB0DATA = 0x36,
    TXB1CTRL = 0x40,
    TXB1SIDH = 0x41,
    TXB1SIDL = 0x42,
    TXB1EID8 = 0x43,
    TXB1EID0 = 0x44,
    TXB1DLC = 0x45,
    TXB1DATA = 0x46,
    TXB2CTRL = 0x50,
    TXB2SIDH = 0x51,
    TXB2SIDL = 0x52,
    TXB2EID8 = 0x53,
    TXB2EID0 = 0x54,
    TXB2DLC = 0x55,
    TXB2DATA = 0x56,
    RXB0CTRL = 0x60,
    RXB0SIDH = 0x61,
    RXB0SIDL = 0x62,
    RXB0EID8 = 0x63,
    RXB0EID0 = 0x64,
    RXB0DLC = 0x65,
    RXB0DATA = 0x66,
    RXB1CTRL = 0x70,
    RXB1SIDH = 0x71,
    RXB1SIDL = 0x72,
    RXB1EID8 = 0x73,
    RXB1EID0 = 0x74,
    RXB1DLC = 0x75,
    RXB1DATA = 0x76,
}

pub trait Reg<const BYTES: usize>: Copy {
    /// List of addresses related to this register (or register set). LSB to
    /// MSB.
    const ADDRESSES: [Register; BYTES];

    /// Read the register into itself from a list of bytes.
    fn read(content: [u8; BYTES]) -> Self;

    /// Write the register to a list of bytes.
    fn write(self) -> [u8; BYTES];
}

/// Marker trait implemented on registers which are modifiable.
pub trait BitModifiable<const BYTES: usize>: Reg<BYTES> {}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CanCtrl {
    /// CLKOUT Presacalar
    pub clkpre: ClkPre,
    /// CLKOUT Enable
    pub clken: bool,
    /// One-shot Mode
    pub osm: bool,
    /// Abort All Pending Transmisison
    pub abat: bool,
    /// Request Operation Mode
    pub reqop: OpMode,
}

impl CanCtrl {
    /// Mask to modify the `reqop` bits.
    pub const MASK_REQOP: Self = Self::from_bytes([0b1110_0000]);
    /// Mask to modify the `clken` bit.
    pub const MASK_CLKEN: Self = Self::from_bytes([0b0000_0100]);
}

impl BitModifiable<1> for CanCtrl {}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CanStat {
    #[skip]
    __: B1,
    #[skip(setters)]
    pub icod: IntFlagCode,
    #[skip]
    __: B1,
    #[skip(setters)]
    pub opmod: OpMode,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CanIntf {
    pub rx0if: bool,
    pub rx1if: bool,
    pub tx0if: bool,
    pub tx1if: bool,
    pub tx2if: bool,
    pub errif: bool,
    pub wakif: bool,
    pub merrf: bool,
}

impl CanIntf {
    pub const MASK_RX0IF: Self = Self::from_bytes([0b0000_0001]);
    pub const MASK_RX1IF: Self = Self::from_bytes([0b0000_0010]);
    pub const MASK_TX0IF: Self = Self::from_bytes([0b0000_0100]);
    pub const MASK_TX1IF: Self = Self::from_bytes([0b0000_1000]);
    pub const MASK_TX2IF: Self = Self::from_bytes([0b0001_0000]);
    pub const MASK_ERRIF: Self = Self::from_bytes([0b0010_0000]);
    pub const MASK_WAKIF: Self = Self::from_bytes([0b0100_0000]);
    pub const MASK_MERRF: Self = Self::from_bytes([0b1000_0000]);
}

impl BitModifiable<1> for CanIntf {}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CanInte {
    pub rx0ie: bool,
    pub rx1ie: bool,
    pub tx0ie: bool,
    pub tx1ie: bool,
    pub tx2ie: bool,
    pub errie: bool,
    pub wakie: bool,
    pub merre: bool,
}

impl CanInte {
    pub const MASK_WAKIE: Self = Self::from_bytes([0b0100_0000]);
}

impl BitModifiable<1> for CanInte {}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Cnf1 {
    pub brp: B6,
    pub sjw: SyncJumpWidth,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Cnf2 {
    pub prseg: B3,
    pub phseg1: B3,
    pub sam: bool,
    pub btlmode: bool,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Cnf3 {
    pub phseg2: B3,
    #[skip]
    __: B3,
    pub wakfil: bool,
    pub sof: bool,
}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Rxb0Ctrl {
    /// Filter hit.
    // TODO(david): Data sheet says this is read-only however other libraries write to it.
    pub filhit0: bool,
    /// Read-Only copy of BUKT bit (used internally by MCP2515).
    #[skip(setters)]
    pub bukt1: bool,
    /// Rollover enable.
    pub bukt: bool,
    /// Received remote transfer request.
    #[skip(setters)]
    pub rxrtr: bool,
    #[skip]
    __: B1,
    /// Receive buffer operating mode.
    pub rxm: RecvBufOpMode,
    #[skip]
    __: B1,
}

impl Rxb0Ctrl {
    pub const MASK_RXM: Self = Self::from_bytes([0b0110_0000]);
    pub const MASK_BUKT: Self = Self::from_bytes([0b0000_0100]);
    pub const MASK_FILTHIT0: Self = Self::from_bytes([0b0000_0001]);
}

impl BitModifiable<1> for Rxb0Ctrl {}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Rxb1Ctrl {
    /// Filter hit.
    // TODO(david): Data sheet says this is read-only however other libraries write to it.
    pub filthit: FilterHit,
    /// Received remote transfer request.
    #[skip(setters)]
    pub rxrtr: bool,
    #[skip]
    __: B1,
    /// Received buffer operating mode.
    pub rxm: RecvBufOpMode,
    #[skip]
    __: B1,
}

impl Rxb1Ctrl {
    pub const MASK_RXM: Self = Self::from_bytes([0b0110_000]);
    pub const MASK_FILTHIT: Self = Self::from_bytes([0b0000_0111]);
}

impl BitModifiable<1> for Rxb1Ctrl {}

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TxbCtrl {
    pub txp: TxBufPriority,
    #[skip]
    __: B1,
    pub txreq: bool,
    #[skip(setters)]
    pub txerr: bool,
    #[skip(setters)]
    pub mloa: bool,
    #[skip(setters)]
    pub abtf: bool,
    #[skip]
    __: B1,
}

impl TxbCtrl {
    pub const MASK_TXB: Self = Self::from_bytes([0b0000_0011]);
    pub const MASK_TXREQ: Self = Self::from_bytes([0b0000_1000]);
}

///////////////////
/// Enums
///////////////////

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum TxBufPriority {
    Low,
    LowIntermediate,
    HighIntermediate,
    High,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 3]
pub enum FilterHit {
    /// Acceptance filter 0 (only if the BUKT bit is set in RXB0CTRL).
    Filter0,
    /// Acceptance filter 1 (only if the BUKT bit is set in RXB0CTRL).
    Filter1,
    /// Acceptance filter 2.
    Filter2,
    /// Acceptance filter 3.
    Fitler3,
    /// Acceptance filter 4.
    Filter4,
    /// Acceptance filter 5.
    Filter5,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum RecvBufOpMode {
    /// Receives all valid messages using either Standard or Extended
    /// Identifiers that meet filter criteria; Extended ID Filter registers,
    /// RXFnEID8:RXFnEID0, are applied to the first two bytes of data in the
    /// messages with standard IDs.
    FilterOn = 0x0,
    /// Turns masks/filters off; receives any message.
    FilterOff = 0x3,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum SyncJumpWidth {
    Tq1,
    Tq2,
    Tq3,
    Tq4,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 3]
pub enum OpMode {
    Normal,
    Sleep,
    Loopback,
    ListenOnly,
    Configuration,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum ClkPre {
    Div1,
    Div2,
    Div4,
    Div8,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 3]
pub enum IntFlagCode {
    None,
    Error,
    WakeUp,
    TXB0,
    TXB1,
    TXB2,
    RXB0,
    RXB1,
}

macro_rules! reg {
    ($($s:ty => $reg:expr),*) => {
        $(
            impl Reg<1> for $s {
                const ADDRESSES: [Register; 1] = [$reg];

                #[inline]
                fn read(content: [u8; 1]) -> Self {
                    Self::from_bytes(content)
                }

                #[inline]
                fn write(self) -> [u8; 1] {
                    self.into_bytes()
                }
            }

            impl BitOr for $s {
                type Output = Self;

                fn bitor(self, rhs: Self) -> Self::Output {
                    Self::from_bytes([
                        self.into_bytes()[0] | rhs.into_bytes()[0]
                    ])
                }
            }
        )*
    };
}

reg! {
    CanCtrl => Register::CANCTRL,
    CanStat => Register::CANSTAT,
    CanIntf => Register::CANINTF,
    CanInte => Register::CANINTE,
    Cnf1 => Register::CNF1,
    Cnf2 => Register::CNF2,
    Cnf3 => Register::CNF3,
    Rxb0Ctrl => Register::RXB0CTRL,
    Rxb1Ctrl => Register::RXB1CTRL
}
