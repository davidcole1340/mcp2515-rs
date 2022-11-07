use modular_bitfield::prelude::*;

#[bitfield]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
pub struct Status {
    #[skip(setters)]
    pub rx0if: bool,
    #[skip(setters)]
    pub rx1if: bool,
    #[skip(setters)]
    pub tx0req: bool,
    #[skip(setters)]
    pub tx0if: bool,
    #[skip(setters)]
    pub tx1req: bool,
    #[skip(setters)]
    pub tx1if: bool,
    #[skip(setters)]
    pub tx2req: bool,
    #[skip(setters)]
    pub tx2if: bool,
}
