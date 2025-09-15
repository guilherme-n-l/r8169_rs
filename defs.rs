// SPDX-License-Identifier: GPL-2.0-only
/*
 * r8169_rs: RealTek 8169/8168/8101 ethernet driver rust port.
 *
 * Copyright (c) 2025 Guilherme Lima <acc.guilhermenl@gmail.com>
 * Copyright (c) a lot of people too. Please respect their work.
 *
 * See MAINTAINERS file for support contact information.
 */

use core::mem::size_of;

use kernel::{
    bindings,
    bits::genmask_u32,
    pci,
    sizes::{SZ_1K, SZ_16K},
};

#[repr(C)]
pub struct TxDesc {
    opts1: u32,
    opts2: u32,
    addr: u64,
}

#[repr(C)]
pub struct RxDesc {
    opts1: u32,
    opts2: u32,
    addr: u64,
}

pub struct RtlChipInfo {
    mask: u16,
    val: u16,
    mac_version: MacVersion,
    name: &'static str,
    fw_name: &'static str,
}

pub struct Rtl8169Driver;

pub mod firmware {
    macro_rules! new_firmware {
        ($name:ident, $value:expr) => {
            pub const $name: &str = $value;
        };
    }

    new_firmware!(RTL8168D_1, "rtl_nic/rtl8168d-1.fw");
    new_firmware!(RTL8168D_2, "rtl_nic/rtl8168d-2.fw");
    new_firmware!(RTL8168E_1, "rtl_nic/rtl8168e-1.fw");
    new_firmware!(RTL8168E_2, "rtl_nic/rtl8168e-2.fw");
    new_firmware!(RTL8168E_3, "rtl_nic/rtl8168e-3.fw");
    new_firmware!(RTL8168F_1, "rtl_nic/rtl8168f-1.fw");
    new_firmware!(RTL8168F_2, "rtl_nic/rtl8168f-2.fw");
    new_firmware!(RTL8105E_1, "rtl_nic/rtl8105e-1.fw");
    new_firmware!(RTL8402_1, "rtl_nic/rtl8402-1.fw");
    new_firmware!(RTL8411_1, "rtl_nic/rtl8411-1.fw");
    new_firmware!(RTL8411_2, "rtl_nic/rtl8411-2.fw");
    new_firmware!(RTL8106E_1, "rtl_nic/rtl8106e-1.fw");
    new_firmware!(RTL8106E_2, "rtl_nic/rtl8106e-2.fw");
    new_firmware!(RTL8168G_2, "rtl_nic/rtl8168g-2.fw");
    new_firmware!(RTL8168G_3, "rtl_nic/rtl8168g-3.fw");
    new_firmware!(RTL8168H_2, "rtl_nic/rtl8168h-2.fw");
    new_firmware!(RTL8168FP_3, "rtl_nic/rtl8168fp-3.fw");
    new_firmware!(RTL8107E_2, "rtl_nic/rtl8107e-2.fw");
    new_firmware!(RTL8125A_3, "rtl_nic/rtl8125a-3.fw");
    new_firmware!(RTL8125B_2, "rtl_nic/rtl8125b-2.fw");
    new_firmware!(RTL8125D_1, "rtl_nic/rtl8125d-1.fw");
    new_firmware!(RTL8125D_2, "rtl_nic/rtl8125d-2.fw");
    new_firmware!(RTL8125BP_2, "rtl_nic/rtl8125bp-2.fw");
    new_firmware!(RTL8126A_2, "rtl_nic/rtl8126a-2.fw");
    new_firmware!(RTL8126A_3, "rtl_nic/rtl8126a-3.fw");
    new_firmware!(RTL8127A_1, "rtl_nic/rtl8127a-1.fw");
}

/// Maximum PCI burst, '7' is unlimited
const TX_DMA_BURST: u8 = 7;

/// 3 means InterFrameGap = the shortest one
const INTER_FRAME_GAP: u8 = 0x03;

const R8169_REGS_SIZE: usize = 256;
const R8169_RX_BUF_SIZE: usize = SZ_16K - 1;
/// Number of Tx descriptor registers
const NUM_TX_DESC: usize = 256;
/// Number of Rx descriptor registers
const NUM_RX_DESC: usize = 256;
const R8169_TX_RING_BYTES: usize = NUM_TX_DESC * size_of::<TxDesc>();
const R8169_RX_RING_BYTES: usize = NUM_RX_DESC * size_of::<RxDesc>();
// const R8169_TX_STOP_THRS: usize = MAX_SKB_FRAGS + 1;
// const R8169_TX_START_THRS: usize = 2 * R8169_TX_STOP_THRS;

const OCP_STD_PHY_BASE: u32 = 0xa400;

const RTL_CFG_NO_GBIT: u32 = 1;

pub const LEDSEL_MASK_8125: u32 = 0x23f;

pub const RX_VLAN_INNER_8125: u32 = 1 << 22;
pub const RX_VLAN_OUTER_8125: u32 = 1 << 23;
pub const RX_VLAN_8125: u32 = RX_VLAN_INNER_8125 | RX_VLAN_OUTER_8125;

pub const RX_FETCH_DFLT_8125: u32 = 8 << 27;

pub const RTL_GSO_MAX_SIZE_V1: u32 = 32000;
pub const RTL_GSO_MAX_SEGS_V1: u32 = 24;
pub const RTL_GSO_MAX_SIZE_V2: u32 = 64000;
pub const RTL_GSO_MAX_SEGS_V2: u32 = 64;

pub enum MacVersion {
    /*
    // SUPPORT REMOVED
    RtlGigaMacVer01,
    RtlGigaMacVer11,
    RtlGigaMacVer27,
    RtlGigaMacVer41,
    RtlGigaMacVer45,
    RtlGigaMacVer49,
    RtlGigaMacVer50,
    RtlGigaMacVer60,

    // MERGED
    RtlGigaMacVer12, // == RtlGigaMacVer17
    RtlGigaMacVer13, // == RtlGigaMacVer10
    RtlGigaMacVer16, // == RtlGigaMacVer10
    */
    RtlGigaMacVer02,
    RtlGigaMacVer03,
    RtlGigaMacVer04,
    RtlGigaMacVer05,
    RtlGigaMacVer06,
    RtlGigaMacVer07,
    RtlGigaMacVer08,
    RtlGigaMacVer09,
    RtlGigaMacVer10,
    RtlGigaMacVer14,
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
    RtlGigaMacVer42,
    RtlGigaMacVer43,
    RtlGigaMacVer44,
    RtlGigaMacVer46,
    RtlGigaMacVer48,
    RtlGigaMacVer51,
    RtlGigaMacVer52,
    RtlGigaMacVer61,
    RtlGigaMacVer63,
    RtlGigaMacVer64,
    RtlGigaMacVer66,
    RtlGigaMacVer70,
    RtlGigaMacVer80,
    RtlGigaMacNone,
}

impl MacVersion {
    pub fn last() -> Self {
        MacVersion::RtlGigaMacVer80
    }
}

pub mod size {
    macro_rules! new_size {
        ($name:ident, $value:expr) => {
            pub const $name: usize = $value;
        };
    }

    new_size!(
        JUMBO_4K,
        4 * super::SZ_1K
            - (super::bindings::VLAN_ETH_HLEN as usize)
            - (super::bindings::ETH_FCS_LEN as usize)
    );
    new_size!(
        JUMBO_6K,
        6 * super::SZ_1K
            - (super::bindings::VLAN_ETH_HLEN as usize)
            - (super::bindings::ETH_FCS_LEN as usize)
    );
    new_size!(
        JUMBO_7K,
        7 * super::SZ_1K
            - (super::bindings::VLAN_ETH_HLEN as usize)
            - (super::bindings::ETH_FCS_LEN as usize)
    );
    new_size!(
        JUMBO_9K,
        9 * super::SZ_1K
            - (super::bindings::VLAN_ETH_HLEN as usize)
            - (super::bindings::ETH_FCS_LEN as usize)
    );
    new_size!(
        JUMBO_16K,
        super::SZ_16K
            - (super::bindings::VLAN_ETH_HLEN as usize)
            - (super::bindings::ETH_FCS_LEN as usize)
    );
}

pub mod register {
    macro_rules! new_register {
        ($name:ident, $value:expr) => {
            pub const $name: u32 = $value;
        };
    }

    new_register!(MAC0, 0); /* Ethernet hardware address. */
    new_register!(MAC4, 4);
    new_register!(MAR0, 8); /* Multicast filter. */

    new_register!(COUNTER_ADDR_LOW, 0x10);
    new_register!(COUNTER_ADDR_HIGH, 0x14);

    new_register!(TX_DESC_START_ADDR_LOW, 0x20);
    new_register!(TX_DESC_START_ADDR_HIGH, 0x24);
    new_register!(TX_H_DESC_START_ADDR_LOW, 0x28);
    new_register!(TX_H_DESC_START_ADDR_HIGH, 0x2c);

    new_register!(FLASH, 0x30);
    new_register!(ERSR, 0x36);
    new_register!(CHIPCMD, 0x37);
    new_register!(TXPOLL, 0x38);
    new_register!(INTR_MASK, 0x3c);
    new_register!(INTR_STATUS, 0x3e);

    new_register!(TX_CONFIG, 0x40);
    new_register!(TX_CFG_AUTO_FIFO, 1 << 7); /* 8111e-vl */
    new_register!(TX_CFG_EMPTY, 1 << 11); /* 8111e-vl */

    new_register!(RX_CONFIG, 0x44);
    new_register!(RX128_INT_EN, 1 << 15); /* 8111c and later */
    new_register!(RX_MULTI_EN, 1 << 14); /* 8111c only */
    new_register!(RX_CFG_FIFO_SHIFT, 13);

    /* No threshold before first PCI xfer */
    new_register!(RX_FIFO_THRESH, 7 << RX_CFG_FIFO_SHIFT);
    new_register!(RX_EARLY_OFF, 1 << 11);
    new_register!(RX_PAUSE_SLOT_ON, 1 << 11); /* 8125b and later */
    new_register!(RX_CFG_DMA_SHIFT, 8);

    /* Unlimited maximum PCI burst. */
    new_register!(RX_DMA_BURST, 7 << RX_CFG_DMA_SHIFT);

    new_register!(CFG_9346, 0x50);
    new_register!(CONFIG0, 0x51);
    new_register!(CONFIG1, 0x52);
    new_register!(CONFIG2, 0x53);
    new_register!(CONFIG3, 0x54);
    new_register!(CONFIG4, 0x55);
    new_register!(CONFIG5, 0x56);

    new_register!(PME_SIGNAL, 1 << 5); /* 8168c and later */

    new_register!(PHY_AR, 0x60);
    new_register!(PHY_STATUS, 0x6c);
    new_register!(RX_MAX_SIZE, 0xda);
    new_register!(C_PLUS_CMD, 0xe0);
    new_register!(INTR_MITIGATE, 0xe2);

    new_register!(RTL_COALESCE_TX_USECS, super::genmask_u32(12..=15));
    new_register!(RTL_COALESCE_TX_FRAMES, super::genmask_u32(8..=11));
    new_register!(RTL_COALESCE_RX_USECS, super::genmask_u32(4..=7));
    new_register!(RTL_COALESCE_RX_FRAMES, super::genmask_u32(0..=3));

    new_register!(RTL_COALESCE_T_MAX, 0x0f);
    new_register!(RTL_COALESCE_FRAME_MAX, RTL_COALESCE_T_MAX * 4);

    new_register!(RX_DESC_ADDR_LOW, 0xe4);
    new_register!(RX_DESC_ADDR_HIGH, 0xe8);
    new_register!(EARLY_TX_THRES, 0xec); /* 8169. Unit of 32 bytes. */

    new_register!(NO_EARLY_TX, 0x3f); /* Max value : no early transmit. */

    new_register!(MAX_TX_PACKET_SIZE, 0xec); /* 8101/8168. Unit of 128 bytes. */

    new_register!(TX_PACKET_MAX, 8064 >> 7);
    new_register!(EARLY_SIZE, 0x27);

    new_register!(FUNC_EVENT, 0xf0);
    new_register!(FUNC_EVENT_MASK, 0xf4);
    new_register!(FUNC_PRESET_STATE, 0xf8);
    new_register!(IBCR0, 0xf8);
    new_register!(IBCR2, 0xf9);
    new_register!(IBIMR0, 0xfa);
    new_register!(IBISR0, 0xfb);
    new_register!(FUNC_FORCE_EVENT, 0xfc);

    pub mod rtl8168_8101 {
        new_register!(CSIDR, 0x64);
        new_register!(CSIAR, 0x68);
        new_register!(CSIAR_FLAG, 0x8000_0000);
        new_register!(CSIAR_WRITE_CMD, 0x8000_0000);
        new_register!(CSIAR_BYTE_ENABLE, 0x0000_f000);
        new_register!(CSIAR_ADDR_MASK, 0x0000_0fff);
        new_register!(PMCH, 0x6f);
        new_register!(D3COLD_NO_PLL_DOWN, 1 << 7);
        new_register!(D3HOT_NO_PLL_DOWN, 1 << 6);
        new_register!(D3_NO_PLL_DOWN, (1 << 7) | (1 << 6));
        new_register!(EPHYAR, 0x80);
        new_register!(EPHYAR_FLAG, 0x8000_0000);
        new_register!(EPHYAR_WRITE_CMD, 0x8000_0000);
        new_register!(EPHYAR_REG_MASK, 0x1f);
        new_register!(EPHYAR_REG_SHIFT, 16);
        new_register!(EPHYAR_DATA_MASK, 0xffff);
        new_register!(DLLPR, 0xd0);
        new_register!(PFM_EN, 1 << 6);
        new_register!(TX_10M_PS_EN, 1 << 7);
        new_register!(DBG_REG, 0xd1);
        new_register!(FIX_NAK_1, 1 << 4);
        new_register!(FIX_NAK_2, 1 << 3);
        new_register!(TWSI, 0xd2);
        new_register!(MCU, 0xd3);
        new_register!(NOW_IS_OOB, 1 << 7);
        new_register!(TX_EMPTY, 1 << 5);
        new_register!(RX_EMPTY, 1 << 4);
        new_register!(RXTX_EMPTY, TX_EMPTY | RX_EMPTY);
        new_register!(EN_NDP, 1 << 3);
        new_register!(EN_OOB_RESET, 1 << 2);
        new_register!(LINK_LIST_RDY, 1 << 1);
        new_register!(EFUSEAR, 0xdc);
        new_register!(EFUSEAR_FLAG, 0x8000_0000);
        new_register!(EFUSEAR_WRITE_CMD, 0x8000_0000);
        new_register!(EFUSEAR_READ_CMD, 0x0000_0000);
        new_register!(EFUSEAR_REG_MASK, 0x03ff);
        new_register!(EFUSEAR_REG_SHIFT, 8);
        new_register!(EFUSEAR_DATA_MASK, 0xff);
        new_register!(MISC_1, 0xf2);
        new_register!(PFM_D3COLD_EN, 1 << 6);
    }

    pub mod rtl8168 {
        new_register!(LED_CTRL, 0x18);
        new_register!(LED_FREQ, 0x1a);
        new_register!(EEE_LED, 0x1b);
        new_register!(ERIDR, 0x70);
        new_register!(ERIAR, 0x74);
        new_register!(ERIAR_FLAG, 0x8000_0000);
        new_register!(ERIAR_WRITE_CMD, 0x8000_0000);
        new_register!(ERIAR_READ_CMD, 0x0000_0000);
        new_register!(ERIAR_ADDR_BYTE_ALIGN, 4);
        new_register!(ERIAR_TYPE_SHIFT, 16);
        new_register!(ERIAR_EXGMAC, 0x00 << ERIAR_TYPE_SHIFT);
        new_register!(ERIAR_MSIX, 0x01 << ERIAR_TYPE_SHIFT);
        new_register!(ERIAR_ASF, 0x02 << ERIAR_TYPE_SHIFT);
        new_register!(ERIAR_OOB, 0x02 << ERIAR_TYPE_SHIFT);
        new_register!(ERIAR_MASK_SHIFT, 12);
        new_register!(ERIAR_MASK_0001, 0x1 << ERIAR_MASK_SHIFT);
        new_register!(ERIAR_MASK_0011, 0x3 << ERIAR_MASK_SHIFT);
        new_register!(ERIAR_MASK_0100, 0x4 << ERIAR_MASK_SHIFT);
        new_register!(ERIAR_MASK_0101, 0x5 << ERIAR_MASK_SHIFT);
        new_register!(ERIAR_MASK_1111, 0xf << ERIAR_MASK_SHIFT);
        new_register!(EPHY_RXER_NUM, 0x7c);
        new_register!(OCPDR, 0xb0); /* OCP GPHY access */
        new_register!(OCPDR_WRITE_CMD, 0x80000000);
        new_register!(OCPDR_READ_CMD, 0x00000000);
        new_register!(OCPDR_REG_MASK, 0x7f);
        new_register!(OCPDR_GPHY_REG_SHIFT, 16);
        new_register!(OCPDR_DATA_MASK, 0xffff);
        new_register!(OCPAR, 0xb4);
        new_register!(OCPAR_FLAG, 0x80000000);
        new_register!(OCPAR_GPHY_WRITE_CMD, 0x8000f060);
        new_register!(OCPAR_GPHY_READ_CMD, 0x0000f060);
        new_register!(GPHY_OCP, 0xb8);
        new_register!(RDSAR1, 0xd0); /* 8168c only. Undocumented on 8168dp */
        new_register!(MISC, 0xf0); /* 8168e only. */
        new_register!(TXPLA_RST, 1 << 29);
        new_register!(DISABLE_LAN_EN, 1 << 23); /* Enable GPIO pin */
        new_register!(PWM_EN, 1 << 22);
        new_register!(RXDV_GATED_EN, 1 << 19);
        new_register!(EARLY_TALLY_EN, 1 << 16);
    }

    pub mod rtl8125 {
        new_register!(LEDSEL0, 0x18);
        new_register!(INT_CFG0_8125, 0x34);
        new_register!(INT_CFG0_ENABLE_8125, 0x1);
        new_register!(INT_CFG0_CLKREQEN, 1 << 3);
        new_register!(INTR_MASK_8125, 0x38);
        new_register!(INTR_STATUS_8125, 0x3c);
        new_register!(INT_CFG1_8125, 0x7a);
        new_register!(LEDSEL2, 0x84);
        new_register!(LEDSEL1, 0x86);
        new_register!(TX_POLL_8125, 0x90);
        new_register!(LEDSEL3, 0x96);
        new_register!(MAC0_BKP, 0x19e0);
        new_register!(RSS_CTRL_8125, 0x4500);
        new_register!(Q_NUM_CTRL_8125, 0x4800);
        new_register!(EEE_TXIDLE_TIMER_8125, 0x6048);
    }
}

pub mod register_content {
    macro_rules! new_register_content {
        ($name:ident, $value:expr) => {
            pub const $name: u32 = $value;
        };
    }

    /* InterruptStatusBits */
    new_register_content!(SYS_ERR, 0x8000);
    new_register_content!(PCS_TIMEOUT, 0x4000);
    new_register_content!(SW_INT, 0x0100);
    new_register_content!(TX_DESC_UNAVAIL, 0x0080);
    new_register_content!(RX_FIFO_OVER, 0x0040);
    new_register_content!(LINK_CHG, 0x0020);
    new_register_content!(RX_OVERFLOW, 0x0010);
    new_register_content!(TX_ERR, 0x0008);
    new_register_content!(TX_OK, 0x0004);
    new_register_content!(RX_ERR, 0x0002);
    new_register_content!(RX_OK, 0x0001);

    /* RxStatusDesc */
    new_register_content!(RX_RWT, 1 << 22);
    new_register_content!(RX_RES, 1 << 21);
    new_register_content!(RX_RUNT, 1 << 20);
    new_register_content!(RX_CRC, 1 << 19);

    /* ChipCmdBits */
    new_register_content!(STOP_REQ, 0x80);
    new_register_content!(CMD_RESET, 0x10);
    new_register_content!(CMD_RX_ENB, 0x08);
    new_register_content!(CMD_TX_ENB, 0x04);
    new_register_content!(RX_BUF_EMPTY, 0x01);

    /* TXPoll register p.5 */
    new_register_content!(HPQ, 0x80); /* Poll cmd on the high prio queue */
    new_register_content!(NPQ, 0x40); /* Poll cmd on the low prio queue */
    new_register_content!(FSW_INT, 0x01); /* Forced software interrupt */

    /* Cfg9346Bits */
    new_register_content!(CFG9346_LOCK, 0x00);
    new_register_content!(CFG9346_UNLOCK, 0xc0);

    /* rx_mode_bits */
    new_register_content!(ACCEPT_ERR, 0x20);
    new_register_content!(ACCEPT_RUNT, 0x10);
    new_register_content!(RX_CONFIG_ACCEPT_ERR_MASK, 0x30);
    new_register_content!(ACCEPT_BROADCAST, 0x08);
    new_register_content!(ACCEPT_MULTICAST, 0x04);
    new_register_content!(ACCEPT_MY_PHYS, 0x02);
    new_register_content!(ACCEPT_ALL_PHYS, 0x01);
    new_register_content!(RX_CONFIG_ACCEPT_OK_MASK, 0x0f);
    new_register_content!(RX_CONFIG_ACCEPT_MASK, 0x3f);

    /* TxConfigBits */
    new_register_content!(TX_INTER_FRAME_GAP_SHIFT, 24);
    new_register_content!(TX_DMA_SHIFT, 8); /* DMA burst value (0-7) is shift this many bits */

    /* Config1 register p.24 */
    new_register_content!(LEDS1, 1 << 7);
    new_register_content!(LEDS0, 1 << 6);
    new_register_content!(SPEED_DOWN, 1 << 4);
    new_register_content!(MEMMAP, 1 << 3);
    new_register_content!(IOMAP, 1 << 2);
    new_register_content!(VPD, 1 << 1);
    new_register_content!(PM_ENABLE, 1 << 0); /* Power Management Enable */

    /* Config2 register p. 25 */
    new_register_content!(CLK_REQ_EN, 1 << 7); /* Clock Request Enable */
    new_register_content!(MSI_ENABLE, 1 << 5); /* 8169 only. Reserved in the 8168. */
    new_register_content!(PCI_CLOCK_66MHZ, 0x01);
    new_register_content!(PCI_CLOCK_33MHZ, 0x00);

    /* Config3 register p.25 */
    new_register_content!(MAGIC_PACKET, 1 << 5); /* Wake up when receives a Magic Packet */
    new_register_content!(LINK_UP, 1 << 4); /* Wake up when the cable connection is re-established */
    new_register_content!(JUMBO_EN0, 1 << 2); /* 8168 only. Reserved in the 8168b */
    new_register_content!(RDY_TO_L23, 1 << 1); /* L23 Enable */
    new_register_content!(BEACON_EN, 1 << 0); /* 8168 only. Reserved in the 8168b */

    /* Config4 register */
    new_register_content!(JUMBO_EN1, 1 << 1); /* 8168 only. Reserved in the 8168b */

    /* Config5 register p.27 */
    new_register_content!(BWF, 1 << 6); /* Accept Broadcast wakeup frame */
    new_register_content!(MWF, 1 << 5); /* Accept Multicast wakeup frame */
    new_register_content!(UWF, 1 << 4); /* Accept Unicast wakeup frame */
    new_register_content!(SPI_EN, 1 << 3);
    new_register_content!(LAN_WAKE, 1 << 1); /* LanWake enable/disable */
    new_register_content!(PME_STATUS, 1 << 0); /* PME status can be reset by PCI RST# */
    new_register_content!(ASPM_EN, 1 << 0); /* ASPM enable */

    /* CPlusCmd p.31 */
    new_register_content!(ENABLE_BIST, 1 << 15); // 8168 8101
    new_register_content!(MAC_DBGO_OE, 1 << 14); // 8168 8101
    new_register_content!(EN_ANA_PLL, 1 << 14); // 8169
    new_register_content!(NORMAL_MODE, 1 << 13); // unused
    new_register_content!(FORCE_HALF_DUP, 1 << 12); // 8168 8101
    new_register_content!(FORCE_RXFLOW_EN, 1 << 11); // 8168 8101
    new_register_content!(FORCE_TXFLOW_EN, 1 << 10); // 8168 8101
    new_register_content!(CXPL_DBG_SEL, 1 << 9); // 8168 8101
    new_register_content!(ASF, 1 << 8); // 8168 8101
    new_register_content!(PKT_CNTR_DISABLE, 1 << 7); // 8168 8101
    new_register_content!(MAC_DBGO_SEL, 0x001c); // 8168
    new_register_content!(RX_VLAN, 1 << 6);
    new_register_content!(RX_CHK_SUM, 1 << 5);
    new_register_content!(PCIDAC, 1 << 4);
    new_register_content!(PCI_MUL_RW, 1 << 3);
    new_register_content!(INTT_MASK, super::genmask_u32(0..=1));
    new_register_content!(CPCMD_MASK, NORMAL_MODE | RX_VLAN | RX_CHK_SUM | INTT_MASK);

    /* rtl8169_PHYstatus */
    new_register_content!(TBI_ENABLE, 0x80);
    new_register_content!(TX_FLOW_CTRL, 0x40);
    new_register_content!(RX_FLOW_CTRL, 0x20);
    new_register_content!(_1000BPS_F, 0x10);
    new_register_content!(_100BPS, 0x08);
    new_register_content!(_10BPS, 0x04);
    new_register_content!(LINK_STATUS, 0x02);
    new_register_content!(FULL_DUP, 0x01);

    /* ResetCounterCommand */
    new_register_content!(COUNTER_RESET, 0x1);

    /* DumpCounterCommand */
    new_register_content!(COUNTER_DUMP, 0x8);

    /* magic enable v2 */
    new_register_content!(MAGIC_PACKET_V2, 1 << 16); /* Wake up when receives a Magic Packet */
}

pub mod desc_bit {
    macro_rules! new_desc_bit {
        ($name:ident, $value:expr) => {
            pub const $name: u32 = $value;
        };
    }

    new_desc_bit!(DESC_OWN, 1 << 31); /* Descriptor is owned by NIC */
    new_desc_bit!(RING_END, 1 << 30); /* End of descriptor ring */
    new_desc_bit!(FIRST_FRAG, 1 << 29); /* First segment of a packet */
    new_desc_bit!(LAST_FRAG, 1 << 28); /* Final segment of a packet */

    pub mod tx {
        /* First doubleword. */
        new_desc_bit!(TD_LSO, 1 << 27); /* Large Send Offload */
        new_desc_bit!(TD_MSS_MAX, 0x07ff); /* MSS value */

        /* Second doubleword. */
        new_desc_bit!(TX_VLAN_TAG, 1 << 17); /* Add VLAN tag */
    }

    pub mod rx {
        /* Rx private */
        new_desc_bit!(PID1, 1 << 18); /* Protocol ID bit 1/2 */
        new_desc_bit!(PID0, 1 << 17); /* Protocol ID bit 0/2 */

        new_desc_bit!(RX_PROTO_UDP, PID1);
        new_desc_bit!(RX_PROTO_TCP, PID0);
        new_desc_bit!(RX_PROTO_IP, PID1 | PID0);
        new_desc_bit!(RX_PROTO_MASK, RX_PROTO_IP);

        new_desc_bit!(IP_FAIL, 1 << 16); /* IP checksum failed */
        new_desc_bit!(UDP_FAIL, 1 << 15); /* UDP/IP checksum failed */
        new_desc_bit!(TCP_FAIL, 1 << 14); /* TCP/IP checksum failed */

        new_desc_bit!(RX_CS_FAIL_MASK, IP_FAIL | UDP_FAIL | TCP_FAIL);

        new_desc_bit!(RX_VLAN_TAG, 1 << 16); /* VLAN tag available */
    }

    /* 8169, 8168b and 810x except 8102e. */
    pub mod v0 {
        pub mod tx {
            /* First doubleword. */
            new_desc_bit!(TD0_MSS_SHIFT, 16); /* MSS position (11 bits) */
            new_desc_bit!(TD0_TCP_CS, 1 << 16); /* Calculate TCP/IP checksum */
            new_desc_bit!(TD0_UDP_CS, 1 << 17); /* Calculate UDP/IP checksum */
            new_desc_bit!(TD0_IP_CS, 1 << 18); /* Calculate IP checksum */
        }
    }

    /* 8102e, 8168c and beyond. */
    pub mod v1 {
        pub mod tx {
            /* First doubleword. */
            new_desc_bit!(TD1_GTSENV4, 1 << 26); /* Giant Send for IPv4 */
            new_desc_bit!(TD1_GTSENV6, 1 << 25); /* Giant Send for IPv6 */
            new_desc_bit!(GTTCPHO_SHIFT, 18);
            new_desc_bit!(GTTCPHO_MAX, 0x7f);

            /* Second doubleword. */
            new_desc_bit!(TCPHO_SHIFT, 18);
            new_desc_bit!(TCPHO_MAX, 0x3ff);
            new_desc_bit!(TD1_MSS_SHIFT, 18); /* MSS position (11 bits) */
            new_desc_bit!(TD1_IPV6_CS, 1 << 28); /* Calculate IPv6 checksum */
            new_desc_bit!(TD1_IPV4_CS, 1 << 29); /* Calculate IPv4 checksum */
            new_desc_bit!(TD1_TCP_CS, 1 << 30); /* Calculate TCP/IP checksum */
            new_desc_bit!(TD1_UDP_CS, 1 << 31); /* Calculate UDP/IP checksum */
        }
    }
}

static RTL_CHIP_INFOS: &[RtlChipInfo] = &[
    /* 8127A family. */
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x6c9,
        mac_version: MacVersion::RtlGigaMacVer80,
        name: "RTL8127A",
        fw_name: firmware::RTL8127A_1,
    },
    /* 8126A family. */
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x64a,
        mac_version: MacVersion::RtlGigaMacVer70,
        name: "RTL8126A",
        fw_name: firmware::RTL8126A_3,
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x649,
        mac_version: MacVersion::RtlGigaMacVer70,
        name: "RTL8126A",
        fw_name: firmware::RTL8126A_2,
    },
    /* 8125BP family. */
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x681,
        mac_version: MacVersion::RtlGigaMacVer66,
        name: "RTL8125BP",
        fw_name: firmware::RTL8125BP_2,
    },
    /* 8125D family. */
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x689,
        mac_version: MacVersion::RtlGigaMacVer64,
        name: "RTL8125D",
        fw_name: firmware::RTL8125D_2,
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x688,
        mac_version: MacVersion::RtlGigaMacVer64,
        name: "RTL8125D",
        fw_name: firmware::RTL8125D_1,
    },
    /* 8125B family. */
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x641,
        mac_version: MacVersion::RtlGigaMacVer63,
        name: "RTL8125B",
        fw_name: firmware::RTL8125B_2,
    },
    /* 8125A family. */
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x609,
        mac_version: MacVersion::RtlGigaMacVer61,
        name: "RTL8125A",
        fw_name: firmware::RTL8125A_3,
    },
    /* RTL8117 */
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x54b,
        mac_version: MacVersion::RtlGigaMacVer52,
        name: "RTL8168fp/RTL8117",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x54a,
        mac_version: MacVersion::RtlGigaMacVer52,
        name: "RTL8168fp/RTL8117",
        fw_name: firmware::RTL8168FP_3,
    },
    /* 8168EP family. */
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x502,
        mac_version: MacVersion::RtlGigaMacVer51,
        name: "RTL8168ep/8111ep",
        fw_name: "",
    },
    /* 8168H family. */
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x541,
        mac_version: MacVersion::RtlGigaMacVer46,
        name: "RTL8168h/8111h",
        fw_name: firmware::RTL8168H_2,
    },
    /* Realtek calls it RTL8168M, but it's handled like RTL8168H */
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x6c0,
        mac_version: MacVersion::RtlGigaMacVer46,
        name: "RTL8168M",
        fw_name: firmware::RTL8168H_2,
    },
    /* 8168G family. */
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x5c8,
        mac_version: MacVersion::RtlGigaMacVer44,
        name: "RTL8411b",
        fw_name: firmware::RTL8411_2,
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x509,
        mac_version: MacVersion::RtlGigaMacVer42,
        name: "RTL8168gu/8111gu",
        fw_name: firmware::RTL8168G_3,
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x4c0,
        mac_version: MacVersion::RtlGigaMacVer40,
        name: "RTL8168g/8111g",
        fw_name: firmware::RTL8168G_2,
    },
    /* 8168F family. */
    RtlChipInfo {
        mask: 0x7c8,
        val: 0x488,
        mac_version: MacVersion::RtlGigaMacVer38,
        name: "RTL8411",
        fw_name: firmware::RTL8411_1,
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x481,
        mac_version: MacVersion::RtlGigaMacVer36,
        name: "RTL8168f/8111f",
        fw_name: firmware::RTL8168F_2,
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x480,
        mac_version: MacVersion::RtlGigaMacVer35,
        name: "RTL8168f/8111f",
        fw_name: firmware::RTL8168F_1,
    },
    /* 8168E family. */
    RtlChipInfo {
        mask: 0x7c8,
        val: 0x2c8,
        mac_version: MacVersion::RtlGigaMacVer34,
        name: "RTL8168evl/8111evl",
        fw_name: firmware::RTL8168E_3,
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x2c1,
        mac_version: MacVersion::RtlGigaMacVer32,
        name: "RTL8168e/8111e",
        fw_name: firmware::RTL8168E_1,
    },
    RtlChipInfo {
        mask: 0x7c8,
        val: 0x2c0,
        mac_version: MacVersion::RtlGigaMacVer33,
        name: "RTL8168e/8111e",
        fw_name: firmware::RTL8168E_2,
    },
    /* 8168D family. */
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x281,
        mac_version: MacVersion::RtlGigaMacVer25,
        name: "RTL8168d/8111d",
        fw_name: firmware::RTL8168D_1,
    },
    RtlChipInfo {
        mask: 0x7c8,
        val: 0x280,
        mac_version: MacVersion::RtlGigaMacVer26,
        name: "RTL8168d/8111d",
        fw_name: firmware::RTL8168D_2,
    },
    /* 8168DP family. */
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x28a,
        mac_version: MacVersion::RtlGigaMacVer28,
        name: "RTL8168dp/8111dp",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x28b,
        mac_version: MacVersion::RtlGigaMacVer31,
        name: "RTL8168dp/8111dp",
        fw_name: "",
    },
    /* 8168C family. */
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x3c9,
        mac_version: MacVersion::RtlGigaMacVer23,
        name: "RTL8168cp/8111cp",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x3c8,
        mac_version: MacVersion::RtlGigaMacVer18,
        name: "RTL8168cp/8111cp",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0x7c8,
        val: 0x3c8,
        mac_version: MacVersion::RtlGigaMacVer24,
        name: "RTL8168cp/8111cp",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x3c0,
        mac_version: MacVersion::RtlGigaMacVer19,
        name: "RTL8168c/8111c",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x3c2,
        mac_version: MacVersion::RtlGigaMacVer20,
        name: "RTL8168c/8111c",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x3c3,
        mac_version: MacVersion::RtlGigaMacVer21,
        name: "RTL8168c/8111c",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0x7c8,
        val: 0x3c0,
        mac_version: MacVersion::RtlGigaMacVer22,
        name: "RTL8168c/8111c",
        fw_name: "",
    },
    /* 8168B family. */
    RtlChipInfo {
        mask: 0x7c8,
        val: 0x380,
        mac_version: MacVersion::RtlGigaMacVer17,
        name: "RTL8168b/8111b",
        fw_name: "",
    },
    /* // This one is very old and rare, support has been removed.
    RtlChipInfo {
        mask: 0x7c8,
        val: 0x300,
        mac_version: MacVersion::RtlGigaMacVer11,
        name: "RTL8168b/8111b",
        fw_name: "",
    },
    */
    /* 8101 family. */
    RtlChipInfo {
        mask: 0x7c8,
        val: 0x448,
        mac_version: MacVersion::RtlGigaMacVer39,
        name: "RTL8106e",
        fw_name: firmware::RTL8106E_1,
    },
    RtlChipInfo {
        mask: 0x7c8,
        val: 0x440,
        mac_version: MacVersion::RtlGigaMacVer37,
        name: "RTL8402",
        fw_name: firmware::RTL8402_1,
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x409,
        mac_version: MacVersion::RtlGigaMacVer29,
        name: "RTL8105e",
        fw_name: firmware::RTL8105E_1,
    },
    RtlChipInfo {
        mask: 0x7c8,
        val: 0x408,
        mac_version: MacVersion::RtlGigaMacVer30,
        name: "RTL8105e",
        fw_name: firmware::RTL8105E_1,
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x349,
        mac_version: MacVersion::RtlGigaMacVer08,
        name: "RTL8102e",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x249,
        mac_version: MacVersion::RtlGigaMacVer08,
        name: "RTL8102e",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x348,
        mac_version: MacVersion::RtlGigaMacVer07,
        name: "RTL8102e",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x248,
        mac_version: MacVersion::RtlGigaMacVer07,
        name: "RTL8102e",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0x7cf,
        val: 0x240,
        mac_version: MacVersion::RtlGigaMacVer14,
        name: "RTL8401",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0x7c8,
        val: 0x348,
        mac_version: MacVersion::RtlGigaMacVer09,
        name: "RTL8102e/RTL8103e",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0x7c8,
        val: 0x248,
        mac_version: MacVersion::RtlGigaMacVer09,
        name: "RTL8102e/RTL8103e",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0x7c8,
        val: 0x340,
        mac_version: MacVersion::RtlGigaMacVer10,
        name: "RTL8101e/RTL8100e",
        fw_name: "",
    },
    /* 8110 family. */
    RtlChipInfo {
        mask: 0xfc8,
        val: 0x980,
        mac_version: MacVersion::RtlGigaMacVer06,
        name: "RTL8169sc/8110sc",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0xfc8,
        val: 0x180,
        mac_version: MacVersion::RtlGigaMacVer05,
        name: "RTL8169sc/8110sc",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0xfc8,
        val: 0x100,
        mac_version: MacVersion::RtlGigaMacVer04,
        name: "RTL8169sb/8110sb",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0xfc8,
        val: 0x040,
        mac_version: MacVersion::RtlGigaMacVer03,
        name: "RTL8110s",
        fw_name: "",
    },
    RtlChipInfo {
        mask: 0xfc8,
        val: 0x008,
        mac_version: MacVersion::RtlGigaMacVer02,
        name: "RTL8169s",
        fw_name: "",
    },
    /* Catch-all */
    RtlChipInfo {
        mask: 0x000,
        val: 0x000,
        mac_version: MacVersion::RtlGigaMacNone,
        name: "",
        fw_name: "",
    },
];

kernel::pci_device_table!(
    RTL8169_PCI_TABLE,
    MODULE_PCI_TABLE,
    <Rtl8169Driver as pci::Driver>::IdInfo,
    [
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x2502),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x2600),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8129),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8136),
            RTL_CFG_NO_GBIT
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8161),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8162),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8167),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8168),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_NCUBE, 0x8168),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8169),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_DLINK, 0x4300),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_DLINK, 0x4302),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_AT, 0xc107),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_USR, 0x0116),
            ()
        ),
        (
            pci::DeviceId(bindings::pci_device_id {
                vendor: bindings::PCI_VENDOR_ID_LINKSYS,
                device: 0x1032,
                subvendor: bindings::PCI_ANY_ID as u32,
                subdevice: 0x0024,
                // ..pci::DeviceId::from_id(0, 0).0
            }),
            ()
        ),
        (
            pci::DeviceId(bindings::pci_device_id {
                vendor: 0x0001,
                device: 0x8168,
                subvendor: bindings::PCI_ANY_ID as u32,
                subdevice: 0x2410,
                // ..pci::DeviceId::from_id(0, 0).0
            }),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8125),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8126),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8127),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x3000),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x5000),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x0e10),
            ()
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_ANY_ID as u32, bindings::PCI_ANY_ID as u32),
            ()
        )
    ]
);
