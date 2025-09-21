// SPDX-License-Identifier: GPL-2.0-only

use core::mem::size_of;

use kernel::{
    bindings,
    bits::genmask_u32,
    clk::Clk,
    dma::CoherentAllocation,
    net::phy::Device as PhyDevice,
    page::Page,
    pci::Device as PciDevice,
    sizes::{SZ_1K, SZ_16K},
    sync::lock::{mutex::Mutex, spinlock::SpinLock},
    transmute::{AsBytes, FromBytes},
};

macro_rules! define_flags {
    (@inner $field:ident, $n:expr;) => {};

    (@inner $field:ident, $n:expr;
        $const_name:ident, $setter:ident => $getter:ident;
        $($rest:tt)*
    ) => {
        const $const_name: u8 = 1 << $n;

        pub(crate) fn $getter(&self) -> bool {
            (self.$field & Self::$const_name) != 0
        }

        pub(crate) fn $setter(&mut self, val: bool) -> bool {
            self.$field = if val {
                self.$field | Self::$const_name
            } else {
                self.$field & !Self::$const_name
            };
            val
        }

        define_flags!(@inner $field, ($n + 1); $($rest)*);
    };

    (
        $field:ident;
        $($const_name:ident, $setter:ident => $getter:ident;)*
    ) => {
        define_flags!(@inner $field, 0; $($const_name, $setter => $getter;)*);
    };
}

macro_rules! impl_bytes {
    ($structure: ty) => {
        unsafe impl FromBytes for $structure {}
        unsafe impl AsBytes for $structure {}
    };
}

#[repr(C)]
pub(crate) struct TxDesc {
    opts1: u32,
    opts2: u32,
    addr: u64,
}

impl_bytes!(TxDesc);

#[repr(C)]
pub(crate) struct RxDesc {
    opts1: u32,
    opts2: u32,
    addr: u64,
}

impl_bytes!(RxDesc);

pub(crate) struct RtlChipInfo {
    mask: u16,
    val: u16,
    mac_version: MacVersion,
    name: &'static str,
    fw_name: &'static str,
}

pub(crate) struct RingInfo {
    // skb: T, // TODO
    len: u32,
}

#[repr(C)]
pub(crate) struct Counters {
    tx_packets: u64,
    rx_packets: u64,
    tx_errors: u64,
    rx_errors: u32,
    rx_missed: u16,
    align_errors: u16,
    tx_one_collision: u32,
    tx_multi_collision: u32,
    rx_unicast: u64,
    rx_broadcast: u64,
    rx_multicast: u32,
    tx_aborted: u16,
    tx_underrun: u16,
    /* new since RTL8125 */
    tx_octets: u64,
    rx_octets: u64,
    rx_multicast64: u64,
    tx_unicast64: u64,
    tx_broadcast64: u64,
    tx_multicast64: u64,
    tx_pause_on: u32,
    tx_pause_off: u32,
    tx_pause_all: u32,
    tx_deferred: u32,
    tx_late_collision: u32,
    tx_all_collision: u32,
    tx_aborted32: u32,
    align_errors32: u32,
    rx_frame_too_long: u32,
    rx_runt: u32,
    rx_pause_on: u32,
    rx_pause_off: u32,
    rx_pause_all: u32,
    rx_unknown_opcode: u32,
    rx_mac_error: u32,
    tx_underrun32: u32,
    rx_mac_missed: u32,
    rx_tcam_dropped: u32,
    tdu: u32,
    rdu: u32,
}

#[repr(C)]
pub(crate) struct TcOffset {
    inited: bool,
    tx_errors: u64,
    tx_multi_collision: u32,
    tx_aborted: u16,
    rx_missed: u16,
}

pub(crate) struct LedClassDev {
    // led: T, // TODO
    // ndev: T, // TODO
    index: i32,
}

pub(crate) struct Rtl8169Private {
    // void __iomem *mmio_addr;	/* memory map physical address */ // TODO
    pci_dev: PciDevice,
    // struct net_device *dev; // TODO
    phy_device: PhyDevice,
    // struct napi_struct napi; // TODO
    mac_version: MacVersion,
    rtl_dash_type: DashType,
    cur_rx: u32, /* Index into the Rx descriptor buffer of next Rx pkt. */
    cur_tx: u32, /* Index into the Tx descriptor buffer of next Rx pkt. */
    dirty_tx: u32,
    tx_desc_array: CoherentAllocation<TxDesc>, /* 256-aligned Tx descriptor ring */
    rx_desc_array: CoherentAllocation<RxDesc>, /* 256-aligned Rx descriptor ring */
    tx_phy_addr: u64,
    rx_phy_addr: u64,
    rx_databuff: [Page; NUM_RX_DESC], /* Rx data buffers */
    tx_skb: [RingInfo; NUM_TX_DESC],  /* Tx data buffers */
    tx_lpi_timer: u16,
    irq_mask: u32,
    irq: i32,
    clk: Clk,
    //
    // struct {
    // 	DECLARE_BITMAP(flags, RTL_FLAG_MAX);
    // 	struct work_struct work;
    // } wk; // TODO
    //
    mac_ocp_lock: SpinLock<()>,
    led_lock: Mutex<()>, /* serialize LED ctrl RMW access */
    flags: u8,
    // dma_addr_t counters_phys_addr; // TODO
    rtl8169_counters: Counters,
    tc_offset: TcOffset,
    saved_wolopts: u32,
    firmware: Firmware,
    // struct rtl_fw *rtl_fw; // TODO
    leds: LedClassDev,
    ocp_base: u32,
}

impl Rtl8169Private {
    define_flags! {flags;
        SUPPORTS_GMII, set_supports_gmii => supports_gmii;
        ASPM_MANAGEABLE, set_aspm_manageable => aspm_manageable;
        DASH_ENABLED, set_dash_enabled => dash_enabled;
    }
}

pub(crate) struct Rtl8169Driver;
/// Maximum PCI burst, '7' is unlimited
pub(crate) const TX_DMA_BURST: u8 = 7;

/// 3 means InterFrameGap = the shortest one
pub(crate) const INTER_FRAME_GAP: u8 = 0x03;

pub(crate) const R8169_REGS_SIZE: usize = 256;
pub(crate) const R8169_RX_BUF_SIZE: usize = SZ_16K - 1;
/// Number of Tx descriptor registers
pub(crate) const NUM_TX_DESC: usize = 256;
/// Number of Rx descriptor registers
pub(crate) const NUM_RX_DESC: usize = 256;
pub(crate) const R8169_TX_RING_BYTES: usize = NUM_TX_DESC * size_of::<TxDesc>();
pub(crate) const R8169_RX_RING_BYTES: usize = NUM_RX_DESC * size_of::<RxDesc>();
pub(crate) const R8169_TX_STOP_THRS: u32 = bindings::MAX_SKB_FRAGS as u32 + 1;
pub(crate) const R8169_TX_START_THRS: u32 = 2 * R8169_TX_STOP_THRS;

pub(crate) const OCP_STD_PHY_BASE: u32 = 0xa400;

pub(crate) const LEDSEL_MASK_8125: u32 = 0x23f;

pub(crate) const RX_VLAN_INNER_8125: u32 = 1 << 22;
pub(crate) const RX_VLAN_OUTER_8125: u32 = 1 << 23;
pub(crate) const RX_VLAN_8125: u32 = RX_VLAN_INNER_8125 | RX_VLAN_OUTER_8125;

pub(crate) const RX_FETCH_DFLT_8125: u32 = 8 << 27;

pub(crate) const RTL_GSO_MAX_SIZE_V1: u32 = 32000;
pub(crate) const RTL_GSO_MAX_SEGS_V1: u32 = 24;
pub(crate) const RTL_GSO_MAX_SIZE_V2: u32 = 64000;
pub(crate) const RTL_GSO_MAX_SEGS_V2: u32 = 64;

pub(crate) enum MacVersion {
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
    pub(crate) fn last() -> Self {
        MacVersion::RtlGigaMacVer80
    }
}

pub(crate) enum Flag {
    RtlFlagTaskResetPending,
    RtlFlagTaskTxTimeout,
    RtlFlagMax,
}

pub(crate) enum DashType {
    RtlDashNone,
    RtlDashDP,
    RtlDashEP,
    RtlDash25BP,
}

pub(crate) enum Rtl8169IdInfo {
    None,
    NoGbit,
}

pub(crate) type Firmware = &'static str;

pub(crate) mod firmware {
    macro_rules! new_firmware {
        ($name:ident, $value:expr) => {
            pub(crate) const $name: super::Firmware = $value;
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

pub(crate) mod size {
    macro_rules! new_size {
        ($name:ident, $value:expr) => {
            pub(crate) const $name: usize = $value;
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

pub(crate) mod register {
    macro_rules! new_register {
        ($name:ident, $value:expr) => {
            pub(crate) const $name: u32 = $value;
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

    pub(crate) mod rtl8168_8101 {
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

    pub(crate) mod rtl8168 {
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

    pub(crate) mod rtl8125 {
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

pub(crate) mod register_content {
    macro_rules! new_register_content {
        ($name:ident, $value:expr) => {
            pub(crate) const $name: u32 = $value;
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

pub(crate) mod desc_bit {
    macro_rules! new_desc_bit {
        ($name:ident, $value:expr) => {
            pub(crate) const $name: u32 = $value;
        };
    }

    new_desc_bit!(DESC_OWN, 1 << 31); /* Descriptor is owned by NIC */
    new_desc_bit!(RING_END, 1 << 30); /* End of descriptor ring */
    new_desc_bit!(FIRST_FRAG, 1 << 29); /* First segment of a packet */
    new_desc_bit!(LAST_FRAG, 1 << 28); /* Final segment of a packet */

    pub(crate) mod tx {
        /* First doubleword. */
        new_desc_bit!(TD_LSO, 1 << 27); /* Large Send Offload */
        new_desc_bit!(TD_MSS_MAX, 0x07ff); /* MSS value */

        /* Second doubleword. */
        new_desc_bit!(TX_VLAN_TAG, 1 << 17); /* Add VLAN tag */
    }

    pub(crate) mod rx {
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
    pub(crate) mod v0 {
        pub(crate) mod tx {
            /* First doubleword. */
            new_desc_bit!(TD0_MSS_SHIFT, 16); /* MSS position (11 bits) */
            new_desc_bit!(TD0_TCP_CS, 1 << 16); /* Calculate TCP/IP checksum */
            new_desc_bit!(TD0_UDP_CS, 1 << 17); /* Calculate UDP/IP checksum */
            new_desc_bit!(TD0_IP_CS, 1 << 18); /* Calculate IP checksum */
        }
    }

    /* 8102e, 8168c and beyond. */
    pub(crate) mod v1 {
        pub(crate) mod tx {
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
