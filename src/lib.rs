#![no_std]

pub mod buffer;
pub mod error;
pub mod filter;
pub mod frame;
pub(crate) mod macros;
pub mod regs;
pub mod stat;

use core::fmt::Debug;

use buffer::{RxBuf, RxBufIdent, TxBuf};
use embedded_hal::{
    blocking::{can::Can, delay::DelayMs, spi::Transfer},
    can::{ExtendedId, Frame, Id, StandardId},
    digital::v2::OutputPin,
};
use filter::{RxFilter, RxMask};
use frame::CanFrame;
use regs::{OpMode, Register};
use stat::Status;

use crate::{
    buffer::TxBufIdent,
    error::{Error, Result},
    filter::{RxFilterReg, RxMaskReg},
    regs::{
        CanCtrl, CanInte, CanIntf, CanStat, Cnf1, Cnf2, Cnf3, FilterHit, RecvBufOpMode, Rxb0Ctrl,
        Rxb1Ctrl, TxbCtrl,
    },
};

#[repr(u8)]
enum Instruction {
    Write = 0x2,
    Read = 0x3,
    Bitmod = 0x5,
    // LoadTX0 = 0x40,
    // LoadTX1 = 0x42,
    // LoadTX2 = 0x44,
    // RTSTX0 = 0x81,
    // RTSTX1 = 0x82,
    // RTSTX2 = 0x84,
    // RTSAll = 0x87,
    // ReadRX0 = 0x90,
    // ReadRX1 = 0x94,
    ReadStatus = 0xA0,
    // RxStatus = 0xB0,
    Reset = 0xC0,
}

/// Speed the CAN bus is operating at.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
pub enum CanSpeed {
    Kbps5,
    Kbps10,
    Kbps20,
    Kbps31_25,
    Kbps33_3,
    Kbps40,
    Kbps50,
    Kbps80,
    Kbps100,
    Kbps125,
    Kbps200,
    Kbps250,
    Kbps500,
    Kbps1000,
}

/// Speed the MCP2515 is operating at. Should match the crystal frequency
/// onboard.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
pub enum McpSpeed {
    MHz8,
    MHz16,
}

/// Settings used to initialize the MCP2515.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
pub struct Settings {
    /// Device operation mode.
    pub mode: OpMode,
    /// Device CAN speed.
    pub can_speed: CanSpeed,
    /// Device oscillator speed. Should match the clock speed of the oscillator
    /// attached to the MCP2515.
    pub mcp_speed: McpSpeed,
    /// Whether to enable the CLKOUT pin.
    pub clkout_en: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            mode: OpMode::Normal,
            can_speed: CanSpeed::Kbps100,
            mcp_speed: McpSpeed::MHz16,
            clkout_en: false,
        }
    }
}

/// MCP2515 driver.
pub struct MCP2515<SPI, CS> {
    /// SPI interface to interact with the MCP2515.
    spi: SPI,
    /// Chip select pin to select the MCP2515.
    cs: CS,
}

impl<SPI, CS, SPIE, CSE> MCP2515<SPI, CS>
where
    SPI: Transfer<u8, Error = SPIE>,
    CS: OutputPin<Error = CSE>,
    SPIE: Debug,
    CSE: Debug,
{
    /// Creates a new MCP2515 driver, initialising the chip in the process.
    ///
    /// # Configuration
    ///
    /// As this driver only takes ownership of the SPI interface, it is up to
    /// the user to create and configure the SPI interface. Namely, the MCP2515
    /// requires the following options:
    ///
    /// * **Data Order**: MSB first.
    /// * **Clock**: Check with your MCP2515 clock. Most breakout boards have an
    ///   8 MHz or 16 MHz oscillator on board. Half of the system clock rate is
    ///   good.
    /// * **Mode**: Mode 0.
    ///
    /// # Parameters
    ///
    /// * `spi` - SPI interface.
    /// * `cs` - Chip-select pin for the MCP2515.
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self { spi, cs }
    }

    /// Initializes the MCP2515 driver. This should be called once at the start
    /// of the program.
    ///
    /// # Parameters
    ///
    /// * `delay` - Delay interface from downstream HAL.
    /// * `settings` - Settings for MCP2515. See [`Settings`].
    pub fn init(
        &mut self,
        delay: &mut impl DelayMs<u8>,
        settings: Settings,
    ) -> Result<(), SPIE, CSE> {
        self.cs.set_high().map_err(Error::Hal)?;
        self.reset(delay)?;

        // Set bitrate, enable clken if required, and change into configuration mode.
        self.set_mode(OpMode::Configuration)?;
        self.set_bitrate(settings.can_speed, settings.mcp_speed, settings.clkout_en)?;
        self.set_clken(settings.clkout_en)?;

        // Clear Tx registers (TXB{O,1,2}CTRL += 14)
        let zeros = [0u8; 14];
        self.write_registers(Register::TXB0CTRL, &zeros)?;
        self.write_registers(Register::TXB1CTRL, &zeros)?;
        self.write_registers(Register::TXB2CTRL, &zeros)?;

        // Clear Rx registers
        self.write_register_addr(&[Register::RXB0CTRL], &[0])?;
        self.write_register_addr(&[Register::RXB1CTRL], &[0])?;

        // Enable interrupts for Rx buffer full, error and message errors.
        self.write_register(
            CanInte::new()
                .with_rx0ie(true)
                .with_rx1ie(true)
                .with_errie(true)
                .with_merre(true),
        )?;

        // Receive all messages that have a standard or extended identifier. Set RXF0 up
        // for RXB0 and RXF1 up for RXB1.
        self.modify_register(
            Rxb0Ctrl::new()
                .with_rxm(RecvBufOpMode::FilterOn)
                .with_bukt(true)
                .with_filhit0(false),
            Rxb0Ctrl::MASK_RXM | Rxb0Ctrl::MASK_BUKT | Rxb0Ctrl::MASK_FILTHIT0,
        )?;
        self.modify_register(
            Rxb1Ctrl::new()
                .with_rxm(RecvBufOpMode::FilterOn)
                .with_filthit(FilterHit::Filter1),
            Rxb1Ctrl::MASK_RXM | Rxb1Ctrl::MASK_FILTHIT,
        )?;

        // Clear all Rx filters and set all to standard EXCEPT for F1 which will be
        // extended filter.
        for filt in RxFilter::ALL {
            let id = if filt == RxFilter::F1 {
                Id::Extended(ExtendedId::ZERO)
            } else {
                Id::Standard(StandardId::ZERO)
            };
            self.set_filter(filt, id)?;
        }

        // Clear all Rx masks and allow extended IDs.
        for mask in RxMask::ALL {
            self.set_mask(mask, Id::Extended(ExtendedId::ZERO))?;
        }

        // Finally switch to requested mode.
        self.set_mode(settings.mode)?;

        Ok(())
    }

    /// Sets a receive filter.
    ///
    /// # Parameters
    ///
    /// * `filter` - The filter to action on.
    /// * `id` - The actual ID filter to apply to `filter`.
    pub fn set_filter(&mut self, filter: RxFilter, id: Id) -> Result<(), SPIE, CSE> {
        let regs = filter.registers();
        let data = RxFilterReg::from_id(id).into_bytes();
        debug_assert!(
            regs.len() == data.len(),
            "More registers than data retrieved from filter"
        );
        self.write_register_addr(&regs, &data)?;
        Ok(())
    }

    /// Sets a receive mask.
    ///
    /// # Parameters
    ///
    /// * `mask` - The mask to action on.
    /// * `id` - The actual ID mask to apply to `mask`.
    pub fn set_mask(&mut self, mask: RxMask, id: Id) -> Result<(), SPIE, CSE> {
        let regs = mask.registers();
        let data = RxMaskReg::from_id(id).into_bytes();
        debug_assert!(
            regs.len() == data.len(),
            "More registers than data retrieved from mask"
        );
        self.write_register_addr(&regs, &data)?;
        Ok(())
    }

    /// Configures the MCP2515 to operate at a certain CAN bitrate.
    ///
    /// # Parameters
    ///
    /// * `can_speed` - CAN speed to operate at.
    /// * `mcp_speed` - Clock speed of the MCP2515.
    /// * `clkout_en` - Whether to enable the `CLKOUT` pin.
    pub fn set_bitrate(
        &mut self,
        can_speed: CanSpeed,
        mcp_speed: McpSpeed,
        clkout_en: bool,
    ) -> Result<(), SPIE, CSE> {
        // Sourced from https://github.com/coryjfowler/MCP_CAN_lib/blob/master/mcp_can_dfs.h#L251-L363
        let (cfg1, cfg2, cfg3): (u8, u8, u8) = match (mcp_speed, can_speed) {
            (McpSpeed::MHz8, CanSpeed::Kbps5) => (0xA7, 0xF6, 0x84),
            (McpSpeed::MHz8, CanSpeed::Kbps10) => (0x93, 0xF6, 0x84),
            (McpSpeed::MHz8, CanSpeed::Kbps20) => (0x89, 0xF6, 0x84),
            (McpSpeed::MHz8, CanSpeed::Kbps31_25) => (0x87, 0xE5, 0x83),
            (McpSpeed::MHz8, CanSpeed::Kbps33_3) => (0x85, 0xF6, 0x84),
            (McpSpeed::MHz8, CanSpeed::Kbps40) => (0x84, 0xF6, 0x84),
            (McpSpeed::MHz8, CanSpeed::Kbps50) => (0x84, 0xE5, 0x83),
            (McpSpeed::MHz8, CanSpeed::Kbps80) => (0x84, 0xD3, 0x81),
            (McpSpeed::MHz8, CanSpeed::Kbps100) => (0x81, 0xF6, 0x84),
            (McpSpeed::MHz8, CanSpeed::Kbps125) => (0x81, 0xE5, 0x83),
            (McpSpeed::MHz8, CanSpeed::Kbps200) => (0x80, 0xF6, 0x84),
            (McpSpeed::MHz8, CanSpeed::Kbps250) => (0x80, 0xE5, 0x83),
            (McpSpeed::MHz8, CanSpeed::Kbps500) => (0x00, 0xD1, 0x81),
            (McpSpeed::MHz8, CanSpeed::Kbps1000) => (0x00, 0xC0, 0x80),
            (McpSpeed::MHz16, CanSpeed::Kbps5) => (0x3F, 0xFF, 0x87),
            (McpSpeed::MHz16, CanSpeed::Kbps10) => (0x67, 0xF6, 0x84),
            (McpSpeed::MHz16, CanSpeed::Kbps20) => (0x53, 0xF6, 0x74),
            (McpSpeed::MHz16, CanSpeed::Kbps33_3) => (0x4E, 0xE5, 0x83),
            (McpSpeed::MHz16, CanSpeed::Kbps40) => (0x49, 0xF6, 0x84),
            (McpSpeed::MHz16, CanSpeed::Kbps50) => (0x47, 0xF6, 0x84),
            (McpSpeed::MHz16, CanSpeed::Kbps80) => (0x44, 0xF6, 0x84),
            (McpSpeed::MHz16, CanSpeed::Kbps100) => (0x44, 0xE5, 0x83),
            (McpSpeed::MHz16, CanSpeed::Kbps125) => (0x43, 0xE5, 0x83),
            (McpSpeed::MHz16, CanSpeed::Kbps200) => (0x41, 0xF6, 0x84),
            (McpSpeed::MHz16, CanSpeed::Kbps250) => (0x41, 0xE5, 0x83),
            (McpSpeed::MHz16, CanSpeed::Kbps500) => (0x40, 0xE5, 0x83),
            (McpSpeed::MHz16, CanSpeed::Kbps1000) => (0x00, 0xCA, 0x81),
            _ => return Err(Error::InvalidConfiguration(can_speed, mcp_speed)),
        };
        let mut cfg3 = Cnf3::from_bytes([cfg3]);
        if clkout_en {
            cfg3 = cfg3.with_sof(false);
        }
        self.write_register(Cnf1::from_bytes([cfg1]))?;
        self.write_register(Cnf2::from_bytes([cfg2]))?;
        self.write_register(cfg3)?;
        Ok(())
    }

    /// Set the operation mode of the device.
    ///
    /// This will wake the device if nessacary.
    ///
    /// # Parameters
    ///
    /// * `mode` - New device mode.
    ///
    /// # Returns
    ///
    /// Nothing on success, error if waking the device or setting the new mode
    /// fails.
    pub fn set_mode(&mut self, mode: OpMode) -> Result<(), SPIE, CSE> {
        let status: CanStat = self.read_register()?;

        // If the device is currently in sleep mode, we need to wake it
        if status.opmod() == OpMode::Sleep && mode != OpMode::Sleep {
            // Ensure wake interrupt is enabled
            let caninte: CanInte = self.read_register()?;
            let int_enabled = caninte.wakie();
            if !int_enabled {
                let data = CanInte::new().with_wakie(true);
                self.modify_register(data, data)?;
            }

            // Actually wake the device
            let data = CanIntf::new().with_wakif(true);
            self.modify_register(data, CanIntf::MASK_WAKIF)?;

            // Change the device into listen only mode.
            self.set_mode_no_wake(OpMode::ListenOnly)?;

            // Disable the interrupt if it was originally disabled
            if !int_enabled {
                self.modify_register(CanInte::new().with_wakie(false), CanInte::MASK_WAKIE)?;
            }
        }

        // Clear wake flag and actually set the new mode
        self.modify_register(CanIntf::new().with_wakif(false), CanIntf::MASK_WAKIF)?;
        self.set_mode_no_wake(mode)
    }

    /// Attempts to set the operation mode without waking the device.
    ///
    /// You should only use this function when you know the device is not in
    /// sleep mode. Otherwise, use [`set_mode`].
    ///
    /// # Parameters
    ///
    /// * `mode` - New operation mode.
    ///
    /// # Returns
    ///
    /// Nothing on success, an error if the device did not respond to changing
    /// mode.
    fn set_mode_no_wake(&mut self, mode: OpMode) -> Result<(), SPIE, CSE> {
        self.modify_register(CanCtrl::new().with_reqop(mode), CanCtrl::MASK_REQOP)?;

        // Wait until status register updates with new mode. We retry 20 times, if it
        // hasn't updated by then fail.
        for _ in 0..20 {
            let canstat: CanStat = self.read_register()?;
            if canstat.opmod_or_err() == Ok(mode) {
                return Ok(());
            }
        }

        Err(Error::NewModeTimeout)
    }

    /// Enables/disables the `CLKOUT` pin on the MCP2515.
    ///
    /// # Parameters
    ///
    /// * `clken` - Whether the `CLKOUT` pin should be enabled or disabled.
    fn set_clken(&mut self, clken: bool) -> Result<(), SPIE, CSE> {
        self.modify_register(CanCtrl::new().with_clken(clken), CanCtrl::MASK_CLKEN)
    }

    /// Sends a CAN frame over the CAN bus via any available Tx buffer.
    ///
    /// # Parameters
    ///
    /// * `frame` - Frame to send.
    pub fn send_message(&mut self, frame: CanFrame) -> Result<(), SPIE, CSE> {
        let buf = self.find_free_tx_buf()?;
        self.send_message_via_buffer(buf, frame)
    }

    /// Sends a CAN frame over the CAN bus via a specific Tx buffer.
    ///
    /// # Parameters
    ///
    /// * `buf` - Tx buffer to use for transmission.
    /// * `frame` - Frame to send.
    pub fn send_message_via_buffer(
        &mut self,
        buf: TxBuf,
        frame: CanFrame,
    ) -> Result<(), SPIE, CSE> {
        // Write control registers.
        let txbuf = TxBufIdent::from_frame(&frame);
        self.write_register_addr(&buf.registers(), &txbuf.into_bytes())?;

        // Write data registers.
        self.write_registers(buf.data(), frame.data())?;

        // Set `txreq` bit in ctrl register.
        self.modify_register_addr(
            &[buf.ctrl()],
            &TxbCtrl::MASK_TXREQ.into_bytes(),
            &TxbCtrl::new().with_txreq(true).into_bytes(),
        )?;

        // Check for any errors.
        let ctrl = self.read_txb_ctrl(&buf)?;
        if ctrl.abtf() || ctrl.mloa() || ctrl.txerr() {
            Err(Error::TxFailed)
        } else {
            Ok(())
        }
    }

    /// Reads a message from the MCP2515 Rx buffers.
    pub fn read_message(&mut self) -> Result<CanFrame, SPIE, CSE> {
        let status = self.read_status()?;
        if status.rx0if() {
            self.read_message_from_buf(RxBuf::B0)
        } else if status.rx1if() {
            self.read_message_from_buf(RxBuf::B1)
        } else {
            Err(Error::NoMessage)
        }
    }

    /// Reads a message from a specific Rx buffer.
    ///
    /// # Parameters
    ///
    /// * `buf` - Rx buffer to read from.
    pub fn read_message_from_buf(&mut self, buf: RxBuf) -> Result<CanFrame, SPIE, CSE> {
        let regs = buf.registers();
        let mut ret = [0u8; 5];
        debug_assert!(regs.len() == ret.len());

        self.read_register_addr(&regs, &mut ret)?;
        let rxbuf = RxBufIdent::from_bytes(ret);

        // Read data and claer Rx interrupt flag
        let frame = rxbuf.into_frame(|ret| self.read_register_seq(buf.data(), ret))?;
        self.modify_register(
            CanIntf::new(),
            match buf {
                RxBuf::B0 => CanIntf::MASK_RX0IF,
                RxBuf::B1 => CanIntf::MASK_RX1IF,
            },
        )?;
        Ok(frame)
    }

    /// Attempts to find a free Tx buffer.
    ///
    /// # Returns
    ///
    /// An available Tx buffer on success, error if all Tx buffers were busy.
    pub fn find_free_tx_buf(&mut self) -> Result<TxBuf, SPIE, CSE> {
        for buffer in TxBuf::ALL {
            let ctrl = self.read_txb_ctrl(&buffer)?;
            if !ctrl.txreq() {
                return Ok(buffer);
            }
        }
        Err(Error::TxBusy)
    }

    /// Read the `CTRL` register of a Tx buffer.
    fn read_txb_ctrl(&mut self, buffer: &TxBuf) -> Result<TxbCtrl, SPIE, CSE> {
        let mut buf = [0u8; 1];
        self.read_register_addr(&[buffer.ctrl()], &mut buf)?;
        Ok(TxbCtrl::from_bytes(buf))
    }

    /// Resets the MCP2515.
    pub fn reset(&mut self, delay: &mut impl DelayMs<u8>) -> Result<(), SPIE, CSE> {
        self.transfer(&mut [Instruction::Reset as u8])?;
        // Sleep for 5ms after reset - if the device is in sleep mode it won't respond
        // immediately
        delay.delay_ms(5);

        Ok(())
    }

    /// Reads the status register.
    pub fn read_status(&mut self) -> Result<Status, SPIE, CSE> {
        let mut data = [Instruction::ReadStatus as u8, 0];
        self.transfer(&mut data)
            .map(|b| [b])
            .map(Status::from_bytes)
    }

    /// Read a register via a register object.
    #[inline]
    pub fn read_register<const N: usize, R: regs::Reg<N>>(&mut self) -> Result<R, SPIE, CSE> {
        let mut ret = [0u8; N];
        self.read_register_addr(&R::ADDRESSES, &mut ret)?;
        Ok(R::read(ret))
    }

    /// Reads a list of registers into an output buffer.
    ///
    /// This function reads `n` registers, where `n` is the minimum of the
    /// length of `regs` and the length of `ret`. The number of registers read
    /// is returned in a result.
    ///
    /// # Parameters
    ///
    /// * `regs` - Input registers to read.
    /// * `ret` - Output buffer to read the register content into.
    ///
    /// # Returns
    ///
    /// The number of registers read on success.
    fn read_register_addr(
        &mut self,
        regs: &[Register],
        ret: &mut [u8],
    ) -> Result<usize, SPIE, CSE> {
        let n = regs.len().min(ret.len());
        for i in 0..n {
            let mut data = [Instruction::Read as u8, regs[i] as u8, 0];
            ret[i] = self.transfer(&mut data)?;
        }
        Ok(n)
    }

    /// Reads registers starting from `reg` sequentially, moving on to the next
    /// register until `ret` is full.
    ///
    /// # Parameters
    ///
    /// * `reg` - Register to start reading from.
    /// * `ret` - Return slice to write into.
    fn read_register_seq(&mut self, reg: Register, ret: &mut [u8]) -> Result<(), SPIE, CSE> {
        let mut hdr = [Instruction::Read as u8, reg as u8];
        self.with_cs(|spi| -> Result<_, _, _> {
            spi.transfer(&mut hdr).map_err(Error::Spi)?;
            // As the MCP2515 doesn't care what we send it while reading, we can just
            // transfer `ret` as it is. The values will be overriden with received data as
            // we transfer the bytes.
            spi.transfer(ret).map_err(Error::Spi)?;
            Ok(())
        })?
    }

    /// Write to a register using a register object.
    #[inline]
    pub fn write_register<const N: usize, R: regs::Reg<N>>(
        &mut self,
        reg: R,
    ) -> Result<(), SPIE, CSE> {
        self.write_register_addr(&R::ADDRESSES, &reg.write())?;
        Ok(())
    }

    /// Write to a list of registers.
    ///
    /// This function writes to `n` registers, where `n` is the minimum of the
    /// length of `regs` and the length of `data`. The number of registers
    /// written is returned in a result.
    pub fn write_register_addr(
        &mut self,
        regs: &[Register],
        data: &[u8],
    ) -> Result<usize, SPIE, CSE> {
        let n = regs.len().min(data.len());
        for i in 0..n {
            let mut data = [Instruction::Write as u8, regs[i] as u8, data[i]];
            self.transfer(&mut data)?;
        }
        Ok(n)
    }

    /// Writes to sequential registers. Writing will start at `reg` and continue
    /// sequentially until `data` is empty.
    fn write_registers(&mut self, reg: Register, data: &[u8]) -> Result<(), SPIE, CSE> {
        let mut hdr = [Instruction::Write as u8, reg as u8];
        self.with_cs(|spi| -> Result<_, _, _> {
            spi.transfer(&mut hdr).map_err(Error::Spi)?;
            for d in data {
                let mut data = [*d];
                spi.transfer(&mut data).map_err(Error::Spi)?;
            }
            Ok(())
        })?
    }

    /// Modifies a register.
    ///
    /// # Parameters
    ///
    /// * `reg` - New register content.
    /// * `mask` - Mask register. The bits must be 1 in the positions you want
    ///   to modify.
    #[inline]
    pub fn modify_register<const N: usize, R: regs::BitModifiable<N>>(
        &mut self,
        reg: R,
        mask: R,
    ) -> Result<(), SPIE, CSE> {
        let mask = mask.write();
        let reg = reg.write();
        self.modify_register_addr(&R::ADDRESSES, &reg, &mask)?;
        Ok(())
    }

    /// Modifies n registers, where n is the minimum length of `regs`, `data`
    /// and `masks`.
    ///
    /// # Parameters
    ///
    /// * `regs` - Registers to modify.
    /// * `data` - Data to modify registers with.
    /// * `masks` - Masks to use when modifying registers.
    ///
    /// # Returns
    ///
    /// Returns the number of registers modified (n), or error on failure.
    pub fn modify_register_addr(
        &mut self,
        regs: &[Register],
        data: &[u8],
        masks: &[u8],
    ) -> Result<usize, SPIE, CSE> {
        let n = regs.len().min(data.len()).min(masks.len());
        for i in 0..n {
            let mut data = [
                Instruction::Bitmod as u8, // BIT MODIFY
                regs[i] as u8,             // Register address
                masks[i],                  // Modify mask byte
                data[i],                   // Data byte
                0,                         // Recv
            ];
            self.transfer(&mut data)?;
        }
        Ok(n)
    }

    /// Transfers an array of bytes via SPI, returning the slave response inside
    /// the given mutable bytes array.
    ///
    /// # Parameters
    ///
    /// * `bytes` - Bytes to transfer. Also used to return the data the slave
    ///   has sent.
    ///
    /// # Returns
    ///
    /// Returns the last element received from the slave. If no bytes were sent,
    /// 0 is returned.
    fn transfer(&mut self, bytes: &mut [u8]) -> Result<u8, SPIE, CSE> {
        self.with_cs(|spi| spi.transfer(bytes))?
            .map_err(Error::Spi)?;
        if let [.., data] = bytes {
            Ok(*data)
        } else {
            Ok(0)
        }
    }

    /// Calls a function `f` after bringing the chip select pin low, restoring
    /// it to high after the function has finished.
    fn with_cs<T>(&mut self, f: impl FnOnce(&mut SPI) -> T) -> Result<T, SPIE, CSE> {
        self.cs.set_low().map_err(Error::Hal)?;
        let result = f(&mut self.spi);
        self.cs.set_high().map_err(Error::Hal)?;
        Ok(result)
    }
}

impl<SPI, CS, SPIE, CSE> Can for MCP2515<SPI, CS>
where
    SPI: Transfer<u8, Error = SPIE>,
    CS: OutputPin<Error = CSE>,
    SPIE: Debug,
    CSE: Debug,
{
    type Frame = CanFrame;
    type Error = Error<SPIE, CSE>;

    #[inline]
    fn transmit(&mut self, frame: &Self::Frame) -> Result<(), SPIE, CSE> {
        self.send_message(*frame)
    }

    #[inline]
    fn receive(&mut self) -> Result<Self::Frame, SPIE, CSE> {
        self.read_message()
    }
}
