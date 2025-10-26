// SPDX-License-Identifier: GPL-2.0-only

/* SPDX-License-Identifier: GPL-2.0-only */
/* macros.rs: RealTek 8169/8168/8101 ethernet driver.
 *
 * Copyright (c) 2025 Guilherme Lima <mail.guilhermenl@gmail.com>
 * Copyright (c) a lot of people too. Please respect their work.
 *
 * See MAINTAINERS file for support contact information.
 */

macro_rules! declare_mac_version {
    ($name:ident : $($variant:ident),* ;last: $last:ident ;none: $none:ident) => {
        #[derive(Debug, PartialEq, PartialOrd, Eq, Ord)]
        pub(crate) enum $name {
            $($variant,)*
            $last,
            $none,
        }

        pub(crate) const MAC_VERSION_LAST: $name = $name::$last;
    };
}
pub(crate) use declare_mac_version;

macro_rules! define_rtl_fw_op_code {
    ($enum_name:ident; $($name:ident = $val:expr),*) => {
        #[derive(Clone)]
        #[repr(u32)]
        enum $enum_name {
            $(
                $name = $val,
            )*
        }

        impl $enum_name {
            fn from_u32(n: u32) -> Result<Self, ()> {
                match n {
                    $(
                        $val => Ok(Self::$name),
                    )*
                    _ => Err(()),
                }
            }
        }
    }
}
pub(crate) use define_rtl_fw_op_code;
