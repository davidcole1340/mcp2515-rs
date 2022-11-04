use embedded_hal::can::{Frame, Id};

/// CAN frame.
#[derive(Debug, Clone, Copy)]
pub struct CanFrame {
    /// ID of CAN frame.
    pub(crate) id: Id,
    /// Whether the frame is an RTR frame.
    pub(crate) rtr: bool,
    /// Length of data in CAN frame.
    pub(crate) dlc: u8,
    /// Data, maximum 8 bytes.
    pub(crate) data: [u8; 8],
}

#[cfg(feature = "defmt")]
impl defmt::Format for CanFrame {
    fn format(&self, fmt: defmt::Formatter) {
        // [`Id`] does not implement `defmt::Format`
        #[derive(defmt::Format)]
        enum InnerId {
            Standard(u16),
            Extended(u32),
        }

        defmt::write!(
            fmt,
            "CanFrame {{ id: {:#X}, rtr: {}, dlc: {:#X}, data: {:#X} }}",
            match self.id {
                Id::Standard(id) => InnerId::Standard(id.as_raw()),
                Id::Extended(id) => InnerId::Extended(id.as_raw()),
            },
            self.rtr,
            self.dlc,
            self.data
        );
    }
}

impl Frame for CanFrame {
    fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        if data.len() > 8 {
            return None;
        }
        let mut frame = CanFrame {
            id: id.into(),
            rtr: false,
            dlc: data.len() as u8, // Already asserted data.len() <= 8
            data: [0; 8],
        };
        frame.data[..data.len()].copy_from_slice(data);
        Some(frame)
    }

    fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        if dlc > 8 {
            return None;
        }
        Some(CanFrame {
            id: id.into(),
            rtr: true,
            dlc: dlc as u8, // Already asserted dlc <= 8
            data: [0; 8],
        })
    }

    #[inline]
    fn is_extended(&self) -> bool {
        matches!(self.id, Id::Extended(_))
    }

    #[inline]
    fn is_remote_frame(&self) -> bool {
        self.rtr
    }

    #[inline]
    fn id(&self) -> Id {
        self.id
    }

    #[inline]
    fn dlc(&self) -> usize {
        self.dlc as usize
    }

    #[inline]
    fn data(&self) -> &[u8] {
        &self.data[..self.dlc()]
    }
}
