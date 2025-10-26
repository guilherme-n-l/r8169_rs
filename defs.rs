// SPDX-License-Identifier: GPL-2.0-only

/* SPDX-License-Identifier: GPL-2.0-only */
/* defs.rs: RealTek 8169/8168/8101 ethernet driver.
 *
 * Copyright (c) 2025 Guilherme Lima <mail.guilhermenl@gmail.com>
 * Copyright (c) a lot of people too. Please respect their work.
 *
 * See MAINTAINERS file for support contact information.
 */

use crate::macros::*;
use kernel::{bindings, device, firmware};

declare_mac_version! {MacVesion:
    /* support for ancient RtlGigaMacVer01 has been removed */
    RtlGigaMacVer02,
    RtlGigaMacVer03,
    RtlGigaMacVer04,
    RtlGigaMacVer05,
    RtlGigaMacVer06,
    RtlGigaMacVer07,
    RtlGigaMacVer08,
    RtlGigaMacVer09,
    RtlGigaMacVer10,
    /* support for RtlGigaMacVer11 has been removed */
    /* RtlGigaMacVer12 was handled the same as VER_17 */
    /* RtlGigaMacVer13 was merged with VER_10 */
    RtlGigaMacVer14,
    /* RtlGigaMacVer16 was merged with VER_10 */
    RtlGigaMacVer17,
    RtlGigaMacVer18,
    RtlGigaMacVer19,
    RtlGigaMacVer20,
    RtlGigaMacVer21,
    RtlGigaMacVer22,
    RtlGigaMacVer23,
    RtlGigaMacVer24,
    RtlGigaMacVer25,
    RtlGigaMacVer26,
    /* support for RtlGigaMacVer27 has been removed */
    RtlGigaMacVer28,
    RtlGigaMacVer29,
    RtlGigaMacVer30,
    RtlGigaMacVer31,
    RtlGigaMacVer32,
    RtlGigaMacVer33,
    RtlGigaMacVer34,
    RtlGigaMacVer35,
    RtlGigaMacVer36,
    RtlGigaMacVer37,
    RtlGigaMacVer38,
    RtlGigaMacVer39,
    RtlGigaMacVer40,
    /* support for RtlGigaMacVer41 has been removed */
    RtlGigaMacVer42,
    RtlGigaMacVer43,
    RtlGigaMacVer44,
    /* support for RtlGigaMacVer45 has been removed */
    RtlGigaMacVer46,
    /* support for RtlGigaMacVer47 has been removed */
    RtlGigaMacVer48,
    /* support for RtlGigaMacVer49 has been removed */
    /* support for RtlGigaMacVer50 has been removed */
    RtlGigaMacVer51,
    RtlGigaMacVer52,
    /* support for RtlGigaMacVer60 has been removed */
    RtlGigaMacVer61,
    RtlGigaMacVer63,
    RtlGigaMacVer64,
    RtlGigaMacVer66,
    RtlGigaMacVer70;
    last: RtlGigaMacVer80;
    none: RtlGigaMacVerNone
}

pub(crate) struct Rtl8169Private {}
struct R8169LedClassdev {}

// pub(crate) fn r8169_apply_firmware(&self) {}
// pub(crate) fn rtl8168h_2_get_adc_bias_ioffset(&self) -> u16 {}
// pub(crate) fn rtl8168d_efuse_read(&self, reg_addr: i32) -> u8 {}
// pub(crate) fn r8169_hw_phy_config(&self, phydev: &phy_device, ver: MacVersion) {}
// pub(crate) fn r8169_get_led_name(&self, idx: i32, buf: &char, buf_len: i32) {}
// pub(crate) fn rtl8168_get_led_mode(&self) -> i32 {}
// pub(crate) fn rtl8168_led_mod_ctrl(&self, mask: u16, val: u16) -> i32 {}
// pub(crate) fn rtl8168_init_leds(ndev: &net_device) -> &R8169LedClassdev {}
// pub(crate) fn rtl8125_get_led_mode(&self, index: i32) -> i32 {}
// pub(crate) fn rtl8125_set_led_mode(&self, index: i32, mode: u16) -> i32 {}
// pub(crate) fn rtl8125_init_leds(ndev: &net_device) -> &R8169LedClassdev {}
// pub(crate) fn r8169_remove_leds(leds: &R8169LedClassdev) {}
// pub(crate) fn rtl_fw_write_firmware(&self, rtl_fw: &RtlFw) {}

pub(crate) const RTL_VER_SIZE: usize = 32;
