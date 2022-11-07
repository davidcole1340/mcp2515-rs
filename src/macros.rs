#[macro_export]
macro_rules! dummy {
    ($t:expr) => {
        ()
    };
}

#[macro_export]
macro_rules! filter_def {
    (
        $(#[doc = $doc:expr])*
        $name:ident($n:expr) => {
            $(
                $(#[doc = $filt_doc:expr])*
                $filt:ident => $regs:expr
            ),*
        }
    ) => {
        $(#[doc = $doc])*
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
        pub enum $name {
            $(
                $(#[doc = $filt_doc])*
                $filt,
            )*
        }

        impl $name {
            #[doc = concat!("All valid options for [`", stringify!($name), "`].")]
            pub const ALL: [Self; <[_]>::len(&[$($crate::dummy!($filt)),*])] = [$(Self::$filt),*];

            #[doc = concat!("Returns the `SIDH`, `SIDL`, `EID8`, `EID0` registers (in that order) based on the variant of [`", stringify!($name), "`].")]
            pub const fn registers(self) -> [$crate::regs::Register; $n] {
                match self {
                    $(Self::$filt => $regs,)*
                }
            }
        }
    };
}
