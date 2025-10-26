// SPDX-License-Identifier: GPL-2.0-only

/* SPDX-License-Identifier: GPL-2.0-only */
/* firmware.rs: RealTek 8169/8168/8101 ethernet driver.
 *
 * Copyright (c) 2025 Guilherme Lima <mail.guilhermenl@gmail.com>
 * Copyright (c) a lot of people too. Please respect their work.
 *
 * See MAINTAINERS file for support contact information.
 */

use crate::defs::*;
use crate::helper;
use crate::macros::*;
use kernel::str::CString;
use kernel::{bindings, device, firmware, prelude::*, types::ARef};

const FW_OPCODE_SIZE: usize =
    core::mem::size_of::<bindings::__le32 /* typeof(RtlFwPhyAction::code[0]) */>();

type RtlFwWrite = Fn(&Rtl8169Private, usize, i32);
type RtlFwRead = Fn(&Rtl8169Private, usize) -> i32;

pub(crate) struct RtlFw {
    phy_write: RtlFwWrite,
    phy_read: RtlFwRead,
    mac_mcu_write: &'static RtlFwWrite,
    mac_mcu_read: &'static RtlFwRead,
    fw: firmware::Firmware,
    fw_name: &'static CStr,
    dev: ARef<device::Device>,
    version: [u8; RTL_VER_SIZE],
    phy_action: RtlFwPhyAction,
}

define_rtl_fw_op_code! {RtlFwOpCode;
    PhyRead = 0x0,
    PhyDataOr = 0x1,
    PhyDataAnd = 0x2,
    PhyBjmpn = 0x3,
    PhyMdioChg = 0x4,
    PhyClearReadCount = 0x7,
    PhyWrite = 0x8,
    PhyReadcountEqSkip = 0x9,
    PhyCompEqSkipn = 0xa,
    PhyCompNeqSkipn = 0xb,
    PhyWritePrevious = 0xc,
    PhySkipn = 0xd,
    PhyDelayMs = 0xe
}

#[repr(C, packed)]
struct FwInfo {
    magic: u32,
    version: [u8; RTL_VER_SIZE],
    fw_start: u32, // __le32
    fw_len: u32,   // __le32
    chksum: u8,
}

impl FwInfo {
    fn from_bytes(bytes: &[u8], size: usize) -> Result<&Self, ()> {
        if size < core::mem::size_of::<FwInfo>() || !(bytes.as_ptr() as *const FwInfo).is_aligned()
        {
            return Err(());
        }
        Ok(unsafe { &*(bytes.as_ptr() as *const Self) })
    }
}

#[repr(C)]
struct RtlFwPhyAction {
    code: *const u32,
    size: usize,
}

impl RtlFw {
    fn rtl_fw_format_ok(&mut self) -> bool {
        let fw_info_data = self.fw.data();
        let fw_info_size = self.fw.size();
        let pa = &mut self.phy_action;

        if fw_info_size < FW_OPCODE_SIZE {
            return false;
        }

        let magic = u32::from_le_bytes(fw_info_data[0..4].try_into().unwrap());
        if magic == 0 {
            let checksum: u8 = fw_info_data.iter().sum();
            if checksum != 0 {
                return false;
            }
            let fw_info = match FwInfo::from_bytes(fw_info_data, fw_info_size) {
                Ok(info) => info,
                Err(_) => return false,
            };

            let start = u32::from_le(fw_info.fw_start);
            if start > fw_info_size {
                return false;
            }

            let size = u32::from_le(fw_info.fw_len);
            if size > (fw_info_size - start) / FW_OPCODE_SIZE {
                return false;
            }

            self.version.copy_from_slice(fw_info.version.as_ref());
            pa.code = unsafe { (fw_info_data.as_ptr() as *const u32).add(start) };
            pa.size = size;
        } else {
            if fw_info_size % FW_OPCODE_SIZE != 0 {
                return false;
            }
            self.version
                .copy_from_slice(self.fw_name.as_bytes_with_nul());
            pa.code = fw_info_data.as_ptr() as *const u32;
            pa.size = fw_info_size / FW_OPCODE_SIZE;
        }
        true
    }

    fn rtl_fw_data_ok(&self) -> bool {
        let pa = &self.phy_action;

        for index in 0..pa.size {
            let action = unsafe { u32::from_le(*pa.code.add(index)) };
            let val = (action & 0x0000ffff) as u16;
            let regno = ((action & 0x0fff0000) >> 16) as usize;
            let opcode = match RtlFwOpCode::from_u32(action >> 28) {
                Ok(op) => op,
                Err(_) => {
                    pr_err!("Invalid action 0x{:08x}", action);
                    return false;
                }
            };

            #[inline]
            let out_of_range_when = |cond| {
                if cond {
                    pr_err!("Out of range of firmware");
                    return false;
                }
                true
            };

            match opcode {
                RtlFwOpCode::PhyRead
                | RtlFwOpCode::PhyDataOr
                | RtlFwOpCode::PhyDataAnd
                | RtlFwOpCode::PhyClearReadCount
                | RtlFwOpCode::PhyWrite
                | RtlFwOpCode::PhyWritePrevious
                | RtlFwOpCode::PhyDelayMs => true,
                RtlFwOpCode::PhyMdioChg => out_of_range_when(val > 1),
                RtlFwOpCode::PhyBjmpn => out_of_range_when(regno > index),
                RtlFwOpCode::PhyReadcountEqSkip => out_of_range_when(index + 2 >= size),
                RtlFwOpCode::PhyCompEqSkipn
                | RtlFwOpCode::PhyCompNeqSkipn
                | RtlFwOpCode::PhySkipn => out_of_range_when(index + 1 + regno >= size),
            }
        }
    }

    fn rtl_fw_write_firmware(&self, tp: &Rtl8169Private) {
        let pa = &self.phy_action;
        let mut fw_write = &self.phy_write;
        let mut fw_read = &self.phy_read;
        let (mut predata, mut count): (i32, i32) = (0, 0);
        let mut index: usize = 0;
        while index < pa.size {
            let action = unsafe { u32::from_le(*pa.code.add(index)) };
            let data = action & 0x0000ffff;
            let regno = ((action & 0x0fff0000) >> 16) as usize;
            let opcode = match RtlFwOpCode::from_u32(action >> 28) {
                Ok(op) => op,
                Err(_) => {
                    pr_err!("Invalid action 0x{:08x}", action);
                    return;
                }
            };

            match opcode {
                RtlFwOpCode::PhyRead => {
                    predata = fw_read(tp, regno);
                    count += 1;
                }
                RtlFwOpCode::PhyDataOr => {
                    predata |= data;
                }
                RtlFwOpCode::PhyDataAnd => {
                    predata &= data;
                }
                RtlFwOpCode::PhyBjmpn => {
                    index -= regno + 1;
                }
                RtlFwOpCode::PhyMdioChg => {
                    if data != 0 {
                        fw_write = &self.mac_mcu_write;
                        fw_read = &self.mac_mcu_read;
                    } else {
                        fw_write = &self.phy_write;
                        fw_read = &self.phy_read;
                    }
                }
                RtlFwOpCode::PhyClearReadCount => {
                    count = 0;
                }
                RtlFwOpCode::PhyWrite => {
                    fw_write(tp, regno, data);
                }
                RtlFwOpCode::PhyReadcountEqSkip => {
                    if count == data {
                        index += 1
                    }
                }
                RtlFwOpCode::PhyCompEqSkipn => {
                    if predata == data {
                        index += regno;
                    }
                }
                RtlFwOpCode::PhyCompNeqSkipn => {
                    if predata != data {
                        index += regno;
                    }
                }
                RtlFwOpCode::PhyWritePrevious => {
                    fw_write(tp, regno, predata);
                }
                RtlFwOpCode::PhySkipn => {
                    index += regno;
                }
                RtlFwOpCode::PhyDelayMs => unsafe {
                    bindings::msleep(data);
                },
            }
        }
    }

    #[inline]
    fn rtl_fw_release_firmware(&self) {
        core::mem::drop(self.fw);
    }

    fn rtl_fw_request_firmware(&self) -> i32 {
        #[inline]
        let out = |rc: i32| {
            pr_warn!("Unable to load firmware ({})", rc);
            rc
        };
        firmware::Firmware::request_nowarn(self.fw_name, &self.dev)
            .map_err(|err| out(err.to_errno()))?;

        if !self.rtl_fw_format_ok() || !self.rtl_fw_data_ok() {
            core::mem::drop(self.fw);
            return out(-EINVAL);
        }

        0
    }
}
