// SPDX-License-Identifier: GPL-2.0-only

use crate::helper::*;
use core::{mem::size_of, ptr::NonNull, sync::atomic::AtomicU8};
use kernel::{
    bindings,
    bits::genmask_u32,
    c_str,
    clk::Clk,
    device::{Core, Device},
    devres::Devres,
    dma::CoherentAllocation,
    error::to_result,
    firmware::Firmware,
    impl_has_work,
    net::phy,
    page::Page,
    pci::{self, Bar},
    prelude::*,
    sizes::{SZ_1K, SZ_16K},
    str::CString,
    sync::{
        Arc,
        lock::{mutex::Mutex, spinlock::SpinLock},
    },
    time::delay::fsleep,
    transmute::{AsBytes, FromBytes},
    types::ARef,
    workqueue::{Work, WorkItem},
};
use pin_init::{PinnedDrop, pin_data};

macro_rules! __check_flag {
    ($flags:expr, $index:expr) => {
        (($flags >> $index) & 1) != 0
    };
}

macro_rules! define_flags {
    (@inner $arg:tt $field:ident, $n:expr;) => {};

    (@inner __getter $getter:ident, $field:expr, $n:expr) => {
        fn $getter(&self) -> bool {
            __check_flag!($field, $n)
        }
    };

    (@inner __atomic $field:ident, $n:expr;
        $setter:ident => $getter:ident;
    ) => {
        define_flags!(@inner __getter $getter, self.$field.load(core::sync::atomic::Ordering::SeqCst), $n);

        fn $setter(&self, val: bool) -> bool {
            __check_flag!((if val {
                    self.$field
                        .fetch_or(1 << $n, core::sync::atomic::Ordering::SeqCst)
            } else {
                    self.$field
                        .fetch_and(!(1 << $n), core::sync::atomic::Ordering::SeqCst)
            }), $n)
        }
    };

    (@inner __non_atomic $field:ident, $n:expr;
        $setter:ident => $getter:ident;
    ) => {
        define_flags!(@inner __getter $getter, self.$field, $n);

        fn $setter(&mut self, val: bool) -> bool {
            let prev = __check_flag!(self.$field, $n);
            if val {
                self.$field |= 1 << $n;
            } else {
                self.$field &= !(1 << $n);
            }
            prev
        }
    };

    (@inner $arg:tt $field:ident, $n:expr;
        $setter:ident => $getter:ident;
        $($rest:tt)*
    ) => {
        define_flags!(@inner $arg $field, $n; $setter => $getter;);
        define_flags!(@inner $arg $field, ($n + 1); $($rest)*);
    };

    (
        @inner
        $arg:tt
        $field:ident;
        $($setter:ident => $getter:ident;)*
    ) => {
        define_flags!(@inner $arg $field, 0; $($setter => $getter;)*);
    };

    (
        $field:ident;
        $($setter:ident => $getter:ident;)*
    ) => {
        define_flags!(@inner __non_atomic $field; $($setter => $getter;)*);
    };
}

macro_rules! define_atomic_flags {
    (
        $field:ident;
        $($setter:ident => $getter:ident;)*
    ) => {
        define_flags!(@inner __atomic $field; $($setter => $getter;)*);
    };
}

macro_rules! impl_bytes {
    ($structure: ty) => {
        unsafe impl FromBytes for $structure {}
        unsafe impl AsBytes for $structure {}
    };
}

macro_rules! set_netdev_dev {
    ($net: ident, $pdev: ident) => {
        unsafe {
            $net.as_ref().dev.parent = $pdev;
        }
    };
}
pub(crate) use set_netdev_dev;

macro_rules! __define_rtl_rw {
    (@inner $(#[$attr:meta])* $name:ident, ($($args:tt)*)) => {
        $(#[$attr])*
        #[inline]
        pub(crate) fn $name($($args)*)
    };

    (@inner $rw_fn:ident, $($call_args:tt)*) => {
        self.mmio_addr.try_access().ok_or(ENXIO)?.$rw_fn($($call_args)*)
    };

    ($(#[$attr:meta])* $name:ident, $rw_fn:ident, $ret:ty, ($($args:tt)*), ($($call_args:tt)*)) => {

        __define_rtl_rw!(@inner $(#[$attr])* $name, ($($args)*)) -> $ret {
            __define_rtl_rw!(@inner $rw_fn, $($call_args)*)
        }
    };

    ($(#[$attr:meta])* $name:ident, $rw_fn:ident, ($($args:tt)*), ($($call_args:tt)*)) => {
        __define_rtl_rw!(@inner $(#[$attr])* $name, ($($args)*)) {
            __define_rtl_rw!(@inner $rw_fn, $($call_args)*)
        }
    };
}

macro_rules! define_rtl_write {
    ($(#[$attr:meta])* $name:ident, $write_fn:ident, $t:ty) => {
        $(#[$attr])*
        __define_rtl_rw!(
            /// Write IO data from a given offset known at compile time.
            ///
            /// Bound checks are performed on compile time, hence if the offset is not known at compile
            /// time, the build will fail.
            $name,
            $write_fn,
            (&self, reg: usize, val: $t),
            (val, reg)
        )
    };
}

macro_rules! define_rtl_read {
    ($(#[$attr:meta])* $name:ident, $write_fn:ident, $t:ty) => {
        $(#[$attr])*
        __define_rtl_rw!(
            /// Read IO data from a given offset.
            ///
            /// Bound checks are performed on runtime, it fails if the offset (plus the type size) is
            /// out of bounds.
            $name,
            $write_fn,
            $t,
            (&self, reg: usize),
            (reg)
        )
    };
}

macro_rules! warn_once {
    ($cond: expr, $($arg:tt)*) => {
        static WARNED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
        if $cond && !WARNED.swap(true, core::sync::atomic::Ordering::Relaxed) {
            kernel::pr_warn!($($arg)*);
        }
        $cond
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
    pub(crate) mask: u16,
    pub(crate) val: u16,
    pub(crate) mac_version: MacVersion,
    pub(crate) name: &'static str,
    pub(crate) fw_name: &'static str,
}

macro_rules! new_rtl_chip_info {
    ($mask: expr, $val: expr, $mac_version: path, $name: expr, $fw_name: expr) => {
        RtlChipInfo {
            mask: $mask,
            val: $val,
            mac_version: $mac_version,
            name: $name,
            fw_name: $fw_name,
        }
    };
    ($mask: expr, $val: expr, $mac_version: path, $name: expr) => {
        new_rtl_chip_info!($mask, $val, $mac_version, $name, "")
    };
    ($mask: expr, $val: expr, $mac_version: path) => {
        new_rtl_chip_info!($mask, $val, $mac_version, "", "")
    };
    ($mac_version: path, $name: expr, $fw_name: expr) => {
        new_rtl_chip_info!(0, 0, $mac_version, $name, $fw_name)
    };
}

static RTL_CHIP_INFOS: &[RtlChipInfo] = &[
    /* 8127A family. */
    new_rtl_chip_info!(
        0x7cf,
        0x6c9,
        MacVersion::RtlGigaMacVer80,
        "RTL8127A",
        rtl_firmware_name::RTL8127A_1
    ),
    /* 8126A family. */
    new_rtl_chip_info!(
        0x7cf,
        0x64a,
        MacVersion::RtlGigaMacVer70,
        "RTL8126A",
        rtl_firmware_name::RTL8126A_3
    ),
    new_rtl_chip_info!(
        0x7cf,
        0x649,
        MacVersion::RtlGigaMacVer70,
        "RTL8126A",
        rtl_firmware_name::RTL8126A_2
    ),
    /* 8125BP family. */
    new_rtl_chip_info!(
        0x7cf,
        0x681,
        MacVersion::RtlGigaMacVer66,
        "RTL8125BP",
        rtl_firmware_name::RTL8125BP_2
    ),
    /* 8125D family. */
    new_rtl_chip_info!(
        0x7cf,
        0x689,
        MacVersion::RtlGigaMacVer64,
        "RTL8125D",
        rtl_firmware_name::RTL8125D_2
    ),
    new_rtl_chip_info!(
        0x7cf,
        0x688,
        MacVersion::RtlGigaMacVer64,
        "RTL8125D",
        rtl_firmware_name::RTL8125D_1
    ),
    /* 8125B family. */
    new_rtl_chip_info!(
        0x7cf,
        0x641,
        MacVersion::RtlGigaMacVer63,
        "RTL8125B",
        rtl_firmware_name::RTL8125B_2
    ),
    /* 8125A family. */
    new_rtl_chip_info!(
        0x7cf,
        0x609,
        MacVersion::RtlGigaMacVer61,
        "RTL8125A",
        rtl_firmware_name::RTL8125A_3
    ),
    /* RTL8117 */
    new_rtl_chip_info!(
        0x7cf,
        0x54b,
        MacVersion::RtlGigaMacVer52,
        "RTL8168fp/RTL8117"
    ),
    new_rtl_chip_info!(
        0x7cf,
        0x54a,
        MacVersion::RtlGigaMacVer52,
        "RTL8168fp/RTL8117",
        rtl_firmware_name::RTL8168FP_3
    ),
    /* 8168EP family. */
    new_rtl_chip_info!(
        0x7cf,
        0x502,
        MacVersion::RtlGigaMacVer51,
        "RTL8168ep/8111ep"
    ),
    /* 8168H family. */
    new_rtl_chip_info!(
        0x7cf,
        0x541,
        MacVersion::RtlGigaMacVer46,
        "RTL8168h/8111h",
        rtl_firmware_name::RTL8168H_2
    ),
    /* Realtek calls it RTL8168M, but it's handled like RTL8168H */
    new_rtl_chip_info!(
        0x7cf,
        0x6c0,
        MacVersion::RtlGigaMacVer46,
        "RTL8168M",
        rtl_firmware_name::RTL8168H_2
    ),
    /* 8168G family. */
    new_rtl_chip_info!(
        0x7cf,
        0x5c8,
        MacVersion::RtlGigaMacVer44,
        "RTL8411b",
        rtl_firmware_name::RTL8411_2
    ),
    new_rtl_chip_info!(
        0x7cf,
        0x509,
        MacVersion::RtlGigaMacVer42,
        "RTL8168gu/8111gu",
        rtl_firmware_name::RTL8168G_3
    ),
    new_rtl_chip_info!(
        0x7cf,
        0x4c0,
        MacVersion::RtlGigaMacVer40,
        "RTL8168g/8111g",
        rtl_firmware_name::RTL8168G_2
    ),
    /* 8168F family. */
    new_rtl_chip_info!(
        0x7c8,
        0x488,
        MacVersion::RtlGigaMacVer38,
        "RTL8411",
        rtl_firmware_name::RTL8411_1
    ),
    new_rtl_chip_info!(
        0x7cf,
        0x481,
        MacVersion::RtlGigaMacVer36,
        "RTL8168f/8111f",
        rtl_firmware_name::RTL8168F_2
    ),
    new_rtl_chip_info!(
        0x7cf,
        0x480,
        MacVersion::RtlGigaMacVer35,
        "RTL8168f/8111f",
        rtl_firmware_name::RTL8168F_1
    ),
    /* 8168E family. */
    new_rtl_chip_info!(
        0x7c8,
        0x2c8,
        MacVersion::RtlGigaMacVer34,
        "RTL8168evl/8111evl",
        rtl_firmware_name::RTL8168E_3
    ),
    new_rtl_chip_info!(
        0x7cf,
        0x2c1,
        MacVersion::RtlGigaMacVer32,
        "RTL8168e/8111e",
        rtl_firmware_name::RTL8168E_1
    ),
    new_rtl_chip_info!(
        0x7c8,
        0x2c0,
        MacVersion::RtlGigaMacVer33,
        "RTL8168e/8111e",
        rtl_firmware_name::RTL8168E_2
    ),
    /* 8168D family. */
    new_rtl_chip_info!(
        0x7cf,
        0x281,
        MacVersion::RtlGigaMacVer25,
        "RTL8168d/8111d",
        rtl_firmware_name::RTL8168D_1
    ),
    new_rtl_chip_info!(
        0x7c8,
        0x280,
        MacVersion::RtlGigaMacVer26,
        "RTL8168d/8111d",
        rtl_firmware_name::RTL8168D_2
    ),
    /* 8168DP family. */
    new_rtl_chip_info!(
        0x7cf,
        0x28a,
        MacVersion::RtlGigaMacVer28,
        "RTL8168dp/8111dp"
    ),
    new_rtl_chip_info!(
        0x7cf,
        0x28b,
        MacVersion::RtlGigaMacVer31,
        "RTL8168dp/8111dp"
    ),
    /* 8168C family. */
    new_rtl_chip_info!(
        0x7cf,
        0x3c9,
        MacVersion::RtlGigaMacVer23,
        "RTL8168cp/8111cp"
    ),
    new_rtl_chip_info!(
        0x7cf,
        0x3c8,
        MacVersion::RtlGigaMacVer18,
        "RTL8168cp/8111cp"
    ),
    new_rtl_chip_info!(
        0x7c8,
        0x3c8,
        MacVersion::RtlGigaMacVer24,
        "RTL8168cp/8111cp"
    ),
    new_rtl_chip_info!(0x7cf, 0x3c0, MacVersion::RtlGigaMacVer19, "RTL8168c/8111c"),
    new_rtl_chip_info!(0x7cf, 0x3c2, MacVersion::RtlGigaMacVer20, "RTL8168c/8111c"),
    new_rtl_chip_info!(0x7cf, 0x3c3, MacVersion::RtlGigaMacVer21, "RTL8168c/8111c"),
    new_rtl_chip_info!(0x7c8, 0x3c0, MacVersion::RtlGigaMacVer22, "RTL8168c/8111c"),
    /* 8168B family. */
    new_rtl_chip_info!(0x7c8, 0x380, MacVersion::RtlGigaMacVer17, "RTL8168b/8111b"),
    /* This one is very old and rare, support has been removed.
     * new_rtl_chip_info!(0x7c8, 0x300, MacVersion::RtlGigaMacVer11, "RTL8168b/8111b"),
     */
    /* 8101 family. */
    new_rtl_chip_info!(
        0x7c8,
        0x448,
        MacVersion::RtlGigaMacVer39,
        "RTL8106e",
        rtl_firmware_name::RTL8106E_1
    ),
    new_rtl_chip_info!(
        0x7c8,
        0x440,
        MacVersion::RtlGigaMacVer37,
        "RTL8402",
        rtl_firmware_name::RTL8402_1
    ),
    new_rtl_chip_info!(
        0x7cf,
        0x409,
        MacVersion::RtlGigaMacVer29,
        "RTL8105e",
        rtl_firmware_name::RTL8105E_1
    ),
    new_rtl_chip_info!(
        0x7c8,
        0x408,
        MacVersion::RtlGigaMacVer30,
        "RTL8105e",
        rtl_firmware_name::RTL8105E_1
    ),
    new_rtl_chip_info!(0x7cf, 0x349, MacVersion::RtlGigaMacVer08, "RTL8102e"),
    new_rtl_chip_info!(0x7cf, 0x249, MacVersion::RtlGigaMacVer08, "RTL8102e"),
    new_rtl_chip_info!(0x7cf, 0x348, MacVersion::RtlGigaMacVer07, "RTL8102e"),
    new_rtl_chip_info!(0x7cf, 0x248, MacVersion::RtlGigaMacVer07, "RTL8102e"),
    new_rtl_chip_info!(0x7cf, 0x240, MacVersion::RtlGigaMacVer14, "RTL8401"),
    new_rtl_chip_info!(
        0x7c8,
        0x348,
        MacVersion::RtlGigaMacVer09,
        "RTL8102e/RTL8103e"
    ),
    new_rtl_chip_info!(
        0x7c8,
        0x248,
        MacVersion::RtlGigaMacVer09,
        "RTL8102e/RTL8103e"
    ),
    new_rtl_chip_info!(
        0x7c8,
        0x340,
        MacVersion::RtlGigaMacVer10,
        "RTL8101e/RTL8100e"
    ),
    /* 8110 family. */
    new_rtl_chip_info!(
        0xfc8,
        0x980,
        MacVersion::RtlGigaMacVer06,
        "RTL8169sc/8110sc"
    ),
    new_rtl_chip_info!(
        0xfc8,
        0x180,
        MacVersion::RtlGigaMacVer05,
        "RTL8169sc/8110sc"
    ),
    new_rtl_chip_info!(
        0xfc8,
        0x100,
        MacVersion::RtlGigaMacVer04,
        "RTL8169sb/8110sb"
    ),
    new_rtl_chip_info!(0xfc8, 0x040, MacVersion::RtlGigaMacVer03, "RTL8110s"),
    new_rtl_chip_info!(0xfc8, 0x008, MacVersion::RtlGigaMacVer02, "RTL8169s"),
    /* Catch-all */
    new_rtl_chip_info!(0x000, 0x000, RTL_GIGA_MAC_NONE),
];

/* Chips combining a 1Gbps MAC with a 100Mbps PHY */
const RTL8106EUS_INFO: RtlChipInfo = new_rtl_chip_info!(
    MacVersion::RtlGigaMacVer43,
    "RTL8106eus",
    rtl_firmware_name::RTL8106E_2
);
const RTL8107E_INFO: RtlChipInfo = new_rtl_chip_info!(
    MacVersion::RtlGigaMacVer48,
    "RTL8107e",
    rtl_firmware_name::RTL8107E_2
);

impl RtlChipInfo {
    pub(crate) fn get_chip_version(xid: u16, gmii: bool) -> &Self {
        let p = RTL_CHIP_INFOS
            .iter()
            .find(|x| xid & x.mask == x.val)
            .unwrap_or(RTL_CHIP_INFOS.last());

        if p.mac_version == MacVersion::RtlGigaMacVer42 && !gmii {
            &RTL8106EUS_INFO
        } else if p.mac_version == MacVersion::RtlGigaMacVer46 && !gmii {
            &RTL8107E_INFO
        } else {
            p
        }
    }
}

pub(crate) struct RingInfo {
    skb: bindings::sk_buff,
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
    led: bindings::led_classdev,
    ndev: bindings::net_device,
    index: i32,
}

struct RtlCond {
    check: &Fn(&Rtl8169Private) -> bool,
    msg: &'static str,
}

mod rtl_cond {
    macro_rules! declare_rtl_cond {
        ($name: ident, $private: ident, $check: expr) => {
            const $name: super::RtlCond = super::RtlCond {
                check: |$private: &super::Rtl8169Private| $check,
                msg: stringify!($name),
            };
        };
    }

    declare_rtl_cond!(RTL_OCP_GPHY, tp, {});
    declare_rtl_cond!(RTL_PHYAR, tp, {});
    declare_rtl_cond!(RTL_OCPAR, tp, {});
    declare_rtl_cond!(RTL_EPHYAR, tp, {});
    declare_rtl_cond!(RTL_DP_OCP_READ, tp, {});
    declare_rtl_cond!(RTL_EP_OCP_READ, tp, {});
    declare_rtl_cond!(RTL_OCP_TX, tp, {});
    declare_rtl_cond!(RTL_EFUSEAR, tp, {});
    declare_rtl_cond!(RTL_COUNTERS, tp, {});
    declare_rtl_cond!(RTL_CHIPCMD, tp, {});
    declare_rtl_cond!(RTL_NPQ, tp, {});
    declare_rtl_cond!(RTL_TXCFG_EMPTY, tp, {});
    declare_rtl_cond!(RTL_RXTX_EMPTY, tp, {});
    declare_rtl_cond!(RTL_RXTX_EMPTY_2, tp, {});
    declare_rtl_cond!(RTL_CSIAR, tp, {});
    declare_rtl_cond!(RTL_MAC_OCP_E00E, tp, {});
    declare_rtl_cond!(RTL_LINK_LIST_READY, tp, {});
    declare_rtl_cond!(RTL_ERIAR, tp, {});
}

#[pin_data(PinnedDrop)]
pub(crate) struct Rtl8169Private {
    #[pin]
    pub(crate) mmio_addr: Devres<Bar>, /* memory map physical address */
    pub(crate) pci_dev: ARef<pci::Device>,
    pub(crate) dev: NonNull<bindings::net_device>,
    pub(crate) phy_device: ARef<phy::Device>,
    pub(crate) napi: bindings::napi_struct,
    pub(crate) mac_version: MacVersion,
    pub(crate) dash_type: DashType,
    pub(crate) cur_rx: u32, /* Index into the Rx descriptor buffer of next Rx pkt. */
    pub(crate) cur_tx: u32, /* Index into the Tx descriptor buffer of next Rx pkt. */
    pub(crate) dirty_tx: u32,
    pub(crate) tx_desc_array: CoherentAllocation<TxDesc>, /* 256-aligned Tx descriptor ring */
    pub(crate) rx_desc_array: CoherentAllocation<RxDesc>, /* 256-aligned Rx descriptor ring */
    pub(crate) tx_phy_addr: u64,
    pub(crate) rx_phy_addr: u64,
    pub(crate) rx_databuff: ARef<[Page; NUM_RX_DESC]>, /* Rx data buffers */
    pub(crate) tx_skb: ARef<[RingInfo; NUM_TX_DESC]>,  /* Tx data buffers */
    pub(crate) cp_cmd: u16,
    pub(crate) tx_lpi_timer: u16,
    pub(crate) irq_mask: u32,
    pub(crate) irq: i32,
    pub(crate) clk: ARef<Option<Clk>>,
    #[pin]
    pub(crate) work: work,
    pub(crate) work_flags: AtomicU8,
    #[pin]
    pub(crate) mac_ocp_lock: SpinLock<()>,
    #[pin]
    pub(crate) led_lock: Mutex<()>, /* serialize LED ctrl RMW access */
    pub(crate) flags: u8,
    pub(crate) counters_phys_addr: bindings::dma_addr_t,
    pub(crate) rtl8169_counters: ARef<Counters>,
    pub(crate) tc_offset: TcOffset,
    pub(crate) saved_wolopts: u32,
    pub(crate) fw_name: RtlFirmwareName,
    pub(crate) rtl_fw: ARef<RtlFirmware>,
    pub(crate) leds: LedClassDev,
    pub(crate) ocp_base: u32,
}

impl Rtl8169Private {
    define_flags!(flags;
        set_supports_gmii => supports_gmii;
        set_aspm_manageable => aspm_manageable;
        set_dash_enabled => dash_enabled;
    );

    define_atomic_flags! (work_flags;
        set_task_reset_pending => task_reset_pending;
        set_task_tx_timeout => task_tx_timeout;
        set_max => max;
    );

    /* read MMIO register */
    define_rtl_read!(r8, read8, u8);
    define_rtl_read!(r16, read16, u16);
    define_rtl_read!(r32, read32, u32);

    /* write MMIO register */
    define_rtl_write!(w8, write8, u8);
    define_rtl_write!(w16, write16, u16);
    define_rtl_write!(w32, write32, u32);

    #[inline]
    fn rtl_ocp_reg_failure(reg: u32) -> bool {
        warn_once!(reg & 0xffff0001 != 0, "Invalid ocp reg {:x}!\n", reg)
    }

    fn r8168_mac_ocp_read(&self, reg: u32) -> u16 {
        use register::rtl8168::*;
        if Self::rtl_ocp_reg_failure(reg) {
            return 0;
        }

        self.w32(OCPDR, reg << 15).ok();

        self.r32(OCPDR).unwrap().into()
    }

    fn r8168_mac_ocp_write(&self, reg: u32, data: u32) {
        use rtl_register::rtl8168::*;
        if Self::rtl_ocp_reg_failure(reg) {
            return;
        }

        self.w32(OCPAR, OCPAR_FLAG | (reg << 15) | data);
    }

    #[inline]
    pub(crate) fn aspm_is_safe(&self) -> bool {
        self.mac_version >= MacVersion::RtlGigaMacVer46
            && (self.r8168_mac_ocp_read(0xc0b2) & 0xf != 0)
    }

    fn rtl_loop_wait(&self, c: &RtlCond, usecs: u64, n: i32, high: bool) -> bool {
        for i in 0..=n {
            if c.check(self) == high {
                return true;
            }
            fsleep(usecs);
        }
        .as_str();
        unsafe {
            if bindings::net_ratelimit() != 0 {
                let err_msg = CString::try_from_fmt(fmt!(
                    "{} == {} (loop: {}, delay: {}).\n",
                    c.msg,
                    if high { 0 } else { 1 },
                    n,
                    usecs
                ))
                .unwrap();
                bindings::netdev_err(self.dev.as_ptr(), err_msg.as_char_ptr());
            }
        }
        false
    }

    #[inline]
    fn rtl_loop_wait_high(&self, c: &RtlCond, d: u64, n: i32) -> bool {
        self.rtl_loop_wait(c, d, n, true)
    }

    #[inline]
    fn rtl_loop_wait_low(&self, c: &RtlCond, d: u64, n: i32) -> bool {
        self.rtl_loop_wait(c, d, n, false)
    }

    #[inline]
    fn r8168fp_adjust_ocp_cmd(&self, cmd: &mut u32, typ: u32) {
        if (typ == rtl_register::rtl8168::ERIAR_OOB
            && self.mac_version == MacVersion::RtlGigaMacVer52)
        {
            *cmd |= 0xf70 << 18;
        }
    }

    pub(crate) fn rtl_get_dash_type(&self) -> DashType {
        match self.mac_version {
            MacVersion::RtlGigaMacVer28 | MacVersion::RtlGigaMacVer31 => DashType::RtlDashDP,
            MacVersion::RtlGigaMacVer51..=MacVersion::RtlGigaMacVer52 => DashType::RtlDashEP,
            MacVersion::RtlGigaMacVer66 => DashType::RtlDash25BP,
            _ => DashType::RtlDashNone,
        }
    }

    fn _rtl_eri_read(&self, addr: i32, typ: i32) -> u32 {
        use rtl_register::rtl8168::*;
        let mut cmd: u32 = ERIAR_READ_CMD | (typ as u32) | ERIAR_MASK_1111 | (addr as u32);
        self.r8168fp_adjust_ocp_cmd(&mut cmd, typ);
        self.w32(ERIAR, cmd).ok();
        if self.rtl_loop_wait_high(&rtl_cond::RTL_ERIAR, 100, 200) {
            self.r32(ERIDR)
        } else {
            !0
        }
    }

    #[inline]
    fn rtl8168_get_ocp_reg(&self) -> u16 {
        if self.mac_version == MacVersion::RtlGigaMacVer31 {
            0xb8u16
        } else {
            0x10u16
        }
    }

    fn r8168dp_ocp_read(&self, reg: u16) -> u32 {
        use rtl_register::rtl8168::*;
        self.w32(OCPAR, 0x0f << 12 | (reg & 0x0fffu16) as u32);
        if self.rtl_loop_wait_high(&rtl_cond::RTL_OCPAR, 100, 20) {
            self.r32(OCPAR).unwrap()
        } else {
            !0
        }
    }

    #[inline]
    fn r8168ep_ocp_read(&self, reg: u16) -> u32 {
        self._rtl_eri_read(reg, rtl_register::rtl8168::ERIAR_OOB)
    }

    #[inline]
    fn r8168dp_check_dash(&self) -> bool {
        self.r8168dp_ocp_read(self.rtl8168_get_ocp_reg()) & bit!(15) != 0
    }

    #[inline]
    fn r8168ep_check_dash(&self) -> bool {
        self.r8168ep_ocp_read(0x128) & bit!(0)
    }

    pub(crate) fn rtl_dash_is_enabled(&self) -> bool {
        match tp.dash_type {
            DashType::RtlDashDp => self.r8168dp_check_dash(),
            DashType::RtlDashEp | DashType::RtlDash25BP => self.r8168ep_check_dash(),
            _ => false,
        }
    }

    pub(crate) fn rtl_init_rxcfg(&self) {
        use MacVersion::*;
        use rtl_register::*;
        self.w32(
            RX_CONFIG,
            match self.mac_version {
                RtlGigaMacVer02..=RtlGigaMacVer06 | RtlGigaMacVer10..=RtlGigaMacVer17 => {
                    RX_FIFO_THRESH | RX_DMA_BURST
                }
                RtlGigaMacVer18..=RtlGigaMacVer24
                | RtlGigaMacVer34..=RtlGigaMacVer36
                | RtlGigaMacVer38 => RX128_INT_EN | RX_MULTI_EN | RX_DMA_BURST,
                RtlGigaMacVer40..=RtlGigaMacVer52 => {
                    RX128_INT_EN | RX_MULTI_EN | RX_DMA_BURST | RX_EARLY_OFF
                }
                RtlGigaMacVer61 => RX_FETCH_DFLT_8125 | RX_DMA_BURST,
                RtlGigaMacVer63..=MAC_VERSION_LAST => {
                    RX_FETCH_DFLT_8125 | RX_DMA_BURST | RX_PAUSE_SLOT_ON
                }
                _ => RX128_INT_EN | RX_DMA_BURST,
            },
        )
        .ok();
    }

    #[inline]
    fn rtl_is_8125(&self) -> bool {
        self.mac_version >= MacVersion::RtlGigaMacVer61
    }

    fn rtl_irq_disable(&self) {
        if self.rtl_is_8125() {
            self.w32(INTR_MASK_8125, 0).ok();
        } else {
            self.w16(INTR_MASK, 0).ok();
        }
    }

    fn rtl_ack_events(&self, bits: u32) {
        if self.rtl_is_8125() {
            self.w32(INTR_STATUS_8125, bits).ok();
        } else {
            self.w16(INTR_STATUS, bits).ok();
        }
    }

    #[inline]
    fn rtl_pci_commit(&self) {
        /* Read an arbitrary register to commit a preceding PCI write */
        self.r8(rtl_register::CHIP_CMD);
    }

    pub(crate) fn rtl8169_irq_mask_and_ack(&self) {
        self.rtl_irq_disable();
        self.rtl_ack_events(0xffffffff);
        self.rtl_pci_commit();
    }

    fn rtl8168ep_stop_cmac(&self) {
        use rtl_register::*;
        self.w8(IBCR2, self.r8(IBCR2) & !0x01);
        self.rtl_loop_wait_high(&rtl_cond::RTL_OCP_TX, 50000, 2000);
        self.w8(IBISR0, self.r8(IBISR0) | 0x20);
        self.w8(IBCR0, self.r8(IBCR0) & !0x01);
    }

    fn r8168_mac_ocp_modify(&self, reg: u32, mask: u16, set: u16) {
        self.r8168_mac_ocp_write(reg, (self.r8168_mac_ocp_read(reg) & !mask) | set);
    }

    #[inline]
    fn r8168g_wait_ll_share_fifo_ready(&self) {
        self.rtl_loop_wait_high(rtl_cond::RTL_LINK_LIST_READY, 100, 42);
    }

    fn rtl_wait_txrx_fifo_empty(&self) {
        use rtl_register::*;
        use rtl_register_content::*;

        match self.mac_version {
            MacVersion::RTL_GIGA_MAC_VER_40..=MacVersion::RTL_GIGA_MAC_VER_52 => {
                self.rtl_loop_wait_high(&rtl_cond::RTL_TXCFG_EMPTY, 100, 42);
                self.rtl_loop_wait_high(&rtl_cond::RTL_RXTX_EMPTY, 100, 42);
            }
            MacVersion::RTL_GIGA_MAC_VER_61..=MacVersion::RTL_GIGA_MAC_VER_61 => {
                self.rtl_loop_wait_high(&rtl_cond::RTL_RXTX_EMPTY, 100, 42);
            }
            MacVersion::RTL_GIGA_MAC_VER_63..=MacVersion::RTL_GIGA_MAC_VER_LAST => {
                self.w8(CHIP_CMD, self.r8(CHIP_CMD) | STOP_REQ);
                self.rtl_loop_wait_high(&rtl_cond::RTL_RXTX_EMPTY, 100, 42);
                self.rtl_loop_wait_high(&rtl_cond::RTL_RXTX_EMPTY_2, 100, 42);
            }
            _ => {}
        }
    }

    fn rtl_enable_rxdvgate(&self) {
        use rtl_register::rtl8168::*;

        self.w32(MISC, self.r32(MISC) | RXDV_GATED_EN);
        fsleep(2000);
        self.rtl_wait_txrx_fifo_empty();
    }

    fn rtl_hw_init_8168g(&self) {
        use rtl_register::rtl8168_8101::*;
        use rtl_register::*;
        use rtl_register_content::*;
        self.rtl_enable_rxdvgate();
        self.w8(CHIP_CMD, self.r8(CHIP_CMD) & !(CMD_TX_ENB | CMD_RX_ENB));
        unsafe {
            bindings::msleep(1);
        }
        self.w8(MCU, self.r8(MCU) & (!NOW_IS_OOB as u8));

        self.r8168_mac_ocp_modify(0xe8de, bit!(14), 0);
        self.r8168g_wait_ll_share_fifo_ready();

        self.r8168_mac_ocp_modify(0xe8de, 0, bit!(15));
        self.r8168g_wait_ll_share_fifo_ready();
    }

    fn rtl_hw_init_8125(&self) {
        use rtl_register::*;

        self.rtl_enable_rxdvgate();

        self.w8(
            CHIP_CMD,
            self.r8(CHIP_CMD) & (!(CMD_TX_ENB | CMD_RX_ENB) as u8),
        );
        unsafe {
            bindings::msleep(1);
        }
        self.w8(MCU, self.r8(MCU) & !NOW_IS_OOB);

        self.r8168_mac_ocp_modify(0xe8de, bit!(14), 0);
        self.r8168g_wait_ll_share_fifo_ready();

        self.r8168_mac_ocp_write(0xc0aa, 0x07d0);
        self.r8168_mac_ocp_write(0xc0a6, 0x0150);
        self.r8168_mac_ocp_write(0xc01e, 0x5555);
        self.r8168g_wait_ll_share_fifo_ready();
    }

    pub(crate) fn rtl_hw_initialize(&self) {
        match self.mac_version {
            MacVersion::RTL_GIGA_MAC_VER_51..=MacVersion::RTL_GIGA_MAC_VER_52 => {
                self.rtl8168ep_stop_cmac();
                self.rtl_hw_init_8168g();
            }
            MacVersion::RTL_GIGA_MAC_VER_40..=MacVersion::RTL_GIGA_MAC_VER_48 => {
                self.rtl_hw_init_8168g();
            }
            MacVersion::RTL_GIGA_MAC_VER_61..=MacVersion::RTL_GIGA_MAC_VER_LAST => {
                self.rtl_hw_init_8125();
            }
            _ => {}
        }
    }

    pub(crate) fn rtl_hw_reset(&self) {
        use rtl_register::*;
        self.w8(CHIP_CMD, CMD_RESET);
        self.rtl_loop_wait_low(&rtl_cond::RTL_CHIPCMD, 100, 100);
    }

    #[inline]
    fn rtl_unlock_config_regs(&self) {
        self.w8(rtl_register::CFG9346, rtl_register_content::CFG9346_UNLOCK);
    }

    #[inline]
    fn rtl_unlock_config_regs(&self) {
        self.w8(rtl_register::CFG9346, rtl_register_content::CFG9346_LOCK);
    }

    pub(crate) fn rtl_alloc_irq(&self) -> i32 {
        use rtl_register::*;
        use rtl_register_content::*;

        let mut flags: u32;
        match self.mac_version {
            MacVersion::RTL_GIGA_MAC_VER_02..=MacVersion::RTL_GIGA_MAC_VER_06 => {
                self.rtl_unlock_config_regs();
                self.w8(CONFIG2, self.r8(CONFIG2) & !MSI_ENABLE);
                self.rtl_lock_config_regs();
                flags = bindings::PCI_IRQ_INTX;
            }
            MacVersion::RTL_GIGA_MAC_VER_07..=MacVersion::RTL_GIGA_MAC_VER_17 => {
                flags = bindings::PCI_IRQ_INTX;
            }
            _ => {
                flags = bindings::PCI_IRQ_ALL_TYPES;
            }
        }

        unsafe { bindings::pci_alloc_irq_vectors(self.pci_dev.as_raw(), 1, 1, flags) }
    }

    fn rtl8169_mark_to_asic(desc: &RxDesc) {}

    fn rtl_reset_work(&self) {
        unsafe { netif_stop_queue(self.dev.as_ref()) };
        self.rtl8169_cleanup();

        for i in 0..NUM_RX_DESC {
            rtl8169_mark_to_asic(unsafe {
                self.rx_desc_array.start_ptr().add(i).as_ref().unwrap()
            });
        }

        unsafe { bindings::napi_enable(&mut self.napi as *mut _) }
    }

    fn rtl_task(tp: &Rtl8169Private) {
        let reset = || {
            tp.rtl_reset_work();
            unsafe { netif_wake_queue(tp.dev.as_ptr()) };
        };

        if tp.set_task_tx_timeout(false) {
            /* if chip isn't accessible, reset bus to revive it */
            if (tp.r32(rtl_register::TX_CONFIG) == !0) {
                if Err(to_result(unsafe {
                    bindings::pci_reset_bus(tp.pci_dev.as_raw())
                })) {
                    pr_err!("Can't reset secondary PCI bus, detach NIC");
                    unsafe { bindings::netif_device_detach(tp.dev.as_ptr()) }
                    return;
                }
            }

            /* ASPM compatibility issues are a typical reason for tx timeouts */
            if Ok(to_result(unsafe {
                bindings::pci_disable_link_state(
                    tp.pci_dev.as_raw(),
                    PCIE_LINK_STATE_L1 | PCIE_LINK_STATE_L0S,
                )
            })) {
                pr_err!("ASPM disabled on Tx timeout");
                reset();
                return;
            }
        }

        if tp.set_task_reset_pending(false) {
            reset();
        }
    }
}

impl_has_work! {
    impl HasWork<Self> for Rtl8169Private {self.work}
}

impl WorkItem for Rtl8169Private {
    type Pointer = Arc<Self>;
    fn run(this: Arc<Self>) {
        Self::rtl_task(&this);
    }
}

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
pub(crate) const R8169_TX_STOP_THRS: u32 = bindings::MAX_SKB_FRAGS + 1;
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

pub(crate) const NETIF_F_HIGHDMA: bindings::netdev_features_t =
    (1 as bindings::netdev_features_t) << bindings::NETIF_F_HIGHDMA_BIT;

macro_rules! declare_mac_versions {
    ($($variant:ident),* ; $last:ident) => {
        #[derive(Debug, PartialEq, PartialOrd, Eq, Ord)]
        pub(crate) enum MacVersion {
            $(
                $variant,
            )*
            $last,
            RtlGigaMacNone,
        }

        pub(crate) const MAC_VERSION_LAST: MacVersion = MacVersion::$last;
    };
}

declare_mac_versions! {
    /*
    // SUPPORT REMOVED
    RtlGigaMacVer01, RtlGigaMacVer11, RtlGigaMacVer27, RtlGigaMacVer41,
    RtlGigaMacVer45, RtlGigaMacVer49, RtlGigaMacVer50, RtlGigaMacVer60,

    // MERGED
    RtlGigaMacVer12, // == RtlGigaMacVer17
    RtlGigaMacVer13, // == RtlGigaMacVer10
    RtlGigaMacVer16, // == RtlGigaMacVer10
    */
    RtlGigaMacVer02, RtlGigaMacVer03, RtlGigaMacVer04, RtlGigaMacVer05,
    RtlGigaMacVer06, RtlGigaMacVer07, RtlGigaMacVer08, RtlGigaMacVer09,
    RtlGigaMacVer10, RtlGigaMacVer14, RtlGigaMacVer17, RtlGigaMacVer18,
    RtlGigaMacVer19, RtlGigaMacVer20, RtlGigaMacVer21, RtlGigaMacVer22,
    RtlGigaMacVer23, RtlGigaMacVer24, RtlGigaMacVer25, RtlGigaMacVer26,
    RtlGigaMacVer28, RtlGigaMacVer29, RtlGigaMacVer30, RtlGigaMacVer31,
    RtlGigaMacVer32, RtlGigaMacVer33, RtlGigaMacVer34, RtlGigaMacVer35,
    RtlGigaMacVer36, RtlGigaMacVer37, RtlGigaMacVer38, RtlGigaMacVer39,
    RtlGigaMacVer40, RtlGigaMacVer42, RtlGigaMacVer43, RtlGigaMacVer44,
    RtlGigaMacVer46, RtlGigaMacVer48, RtlGigaMacVer51, RtlGigaMacVer52,
    RtlGigaMacVer61, RtlGigaMacVer63, RtlGigaMacVer64, RtlGigaMacVer66,
    RtlGigaMacVer70; RtlGigaMacVer80
}

pub(crate) enum DashType {
    RtlDashNone,
    RtlDashDP,
    RtlDashEP,
    RtlDash25BP,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct Rtl8169IdInfo(u8);

impl Rtl8169IdInfo {
    pub(crate) const NONE: Self = Self(0);
    pub(crate) const NO_GBIT: Self = Self(1);
}

pub(crate) type RtlFirmwareName = &'static str;

pub(crate) mod rtl_firmware_name {
    macro_rules! declare_firmwares {
        ($($name:ident: $value:expr),*) => {
            $(
                pub(crate) const $name: super::RtlFirmwareName = $value;
            )*
        };
    }

    declare_firmwares! {
        RTL8168D_1: "rtl_nic/rtl8168d-1.fw",
        RTL8168D_2: "rtl_nic/rtl8168d-2.fw",
        RTL8168E_1: "rtl_nic/rtl8168e-1.fw",
        RTL8168E_2: "rtl_nic/rtl8168e-2.fw",
        RTL8168E_3: "rtl_nic/rtl8168e-3.fw",
        RTL8168F_1: "rtl_nic/rtl8168f-1.fw",
        RTL8168F_2: "rtl_nic/rtl8168f-2.fw",
        RTL8105E_1: "rtl_nic/rtl8105e-1.fw",
        RTL8402_1: "rtl_nic/rtl8402-1.fw",
        RTL8411_1: "rtl_nic/rtl8411-1.fw",
        RTL8411_2: "rtl_nic/rtl8411-2.fw",
        RTL8106E_1: "rtl_nic/rtl8106e-1.fw",
        RTL8106E_2: "rtl_nic/rtl8106e-2.fw",
        RTL8168G_2: "rtl_nic/rtl8168g-2.fw",
        RTL8168G_3: "rtl_nic/rtl8168g-3.fw",
        RTL8168H_2: "rtl_nic/rtl8168h-2.fw",
        RTL8168FP_3: "rtl_nic/rtl8168fp-3.fw",
        RTL8107E_2: "rtl_nic/rtl8107e-2.fw",
        RTL8125A_3: "rtl_nic/rtl8125a-3.fw",
        RTL8125B_2: "rtl_nic/rtl8125b-2.fw",
        RTL8125D_1: "rtl_nic/rtl8125d-1.fw",
        RTL8125D_2: "rtl_nic/rtl8125d-2.fw",
        RTL8125BP_2: "rtl_nic/rtl8125bp-2.fw",
        RTL8126A_2: "rtl_nic/rtl8126a-2.fw",
        RTL8126A_3: "rtl_nic/rtl8126a-3.fw",
        RTL8127A_1: "rtl_nic/rtl8127a-1.fw"
    }
}

#[repr(C)]
pub(crate) struct RtlPhyAction {
    code: ARef<u32>,
    size: usize,
}

pub(crate) struct RtlFirmware {
    fw: ARef<Firmware>,
    fw_name: RtlFirmwareName,
    dev: Device<Core>,
    version: &'static str,
    phy_action: RtlPhyAction,
}

pub(crate) mod size {
    macro_rules! declare_sizes {
        ($($name:ident: $value:expr),*) => {
            $(
                pub(crate) const $name: usize = $value;
            )*
        };
    }

    declare_sizes! {
        JUMBO_4K: 4 * super::SZ_1K - (super::bindings::VLAN_ETH_HLEN as usize) - (super::bindings::ETH_FCS_LEN as usize),
        JUMBO_6K: 6 * super::SZ_1K - (super::bindings::VLAN_ETH_HLEN as usize) - (super::bindings::ETH_FCS_LEN as usize),
        JUMBO_7K: 7 * super::SZ_1K - (super::bindings::VLAN_ETH_HLEN as usize) - (super::bindings::ETH_FCS_LEN as usize),
        JUMBO_9K: 9 * super::SZ_1K - (super::bindings::VLAN_ETH_HLEN as usize) - (super::bindings::ETH_FCS_LEN as usize),
        JUMBO_16K: super::SZ_16K - (super::bindings::VLAN_ETH_HLEN as usize) - (super::bindings::ETH_FCS_LEN as usize)
    }
}

pub(crate) mod rtl_register {
    macro_rules! declare_rtl_registers {
        ($($name:ident: $value:expr),*) => {
            $(
                pub(crate) const $name: u32 = $value;
            )*
        };
    }

    declare_rtl_registers! {
        MAC0: 0,  /* Ethernet hardware address. */
        MAC4: 4,
        MAR0: 8,  /* Multicast filter. */

        COUNTER_ADDR_LOW: 0x10,
        COUNTER_ADDR_HIGH: 0x14,

        TX_DESC_START_ADDR_LOW: 0x20,
        TX_DESC_START_ADDR_HIGH: 0x24,
        TX_H_DESC_START_ADDR_LOW: 0x28,
        TX_H_DESC_START_ADDR_HIGH: 0x2c,

        FLASH: 0x30,
        ERSR: 0x36,
        CHIP_CMD: 0x37,
        TXPOLL: 0x38,
        INTR_MASK: 0x3c,
        INTR_STATUS: 0x3e,

        TX_CONFIG: 0x40,
        TX_CFG_AUTO_FIFO: 1 << 7,   /* 8111e-vl */
        TX_CFG_EMPTY: 1 << 11,      /* 8111e-vl */

        RX_CONFIG: 0x44,
        RX128_INT_EN: 1 << 15,  /* 8111c and later */
        RX_MULTI_EN: 1 << 14,   /* 8111c only */
        RX_CFG_FIFO_SHIFT: 13,

        /* No threshold before first PCI xfer */
        RX_FIFO_THRESH: 7 << RX_CFG_FIFO_SHIFT,
        RX_EARLY_OFF: 1 << 11,
        RX_PAUSE_SLOT_ON: 1 << 11, /* 8125b and later */
        RX_CFG_DMA_SHIFT: 8,

        /* Unlimited maximum PCI burst. */
        RX_DMA_BURST: 7 << RX_CFG_DMA_SHIFT,

        CFG9346: 0x50,
        CONFIG0: 0x51,
        CONFIG1: 0x52,
        CONFIG2: 0x53,
        CONFIG3: 0x54,
        CONFIG4: 0x55,
        CONFIG5: 0x56,

        PME_SIGNAL: 1 << 5, /* 8168c and later */

        PHY_AR: 0x60,
        PHY_STATUS: 0x6c,
        RX_MAX_SIZE: 0xda,
        C_PLUS_CMD: 0xe0,
        INTR_MITIGATE: 0xe2,

        RTL_COALESCE_TX_USECS: super::genmask_u32(12..=15),
        RTL_COALESCE_TX_FRAMES: super::genmask_u32(8..=11),
        RTL_COALESCE_RX_USECS: super::genmask_u32(4..=7),
        RTL_COALESCE_RX_FRAMES: super::genmask_u32(0..=3),

        RTL_COALESCE_T_MAX: 0x0f,
        RTL_COALESCE_FRAME_MAX: RTL_COALESCE_T_MAX * 4,

        RX_DESC_ADDR_LOW: 0xe4,
        RX_DESC_ADDR_HIGH: 0xe8,
        EARLY_TX_THRES: 0xec,   /* 8169. Unit of 32 bytes. */

        NO_EARLY_TX: 0x3f,  /* Max value : no early transmit. */

        MAX_TX_PACKET_SIZE: 0xec,   /* 8101/8168. Unit of 128 bytes. */

        TX_PACKET_MAX: 8064 >> 7,
        EARLY_SIZE: 0x27,

        FUNC_EVENT: 0xf0,
        FUNC_EVENT_MASK: 0xf4,
        FUNC_PRESET_STATE: 0xf8,
        IBCR0: 0xf8,
        IBCR2: 0xf9,
        IBIMR0: 0xfa,
        IBISR0: 0xfb,
        FUNC_FORCE_EVENT: 0xfc
    }

    pub(crate) mod rtl8168_8101 {
        declare_rtl_registers! {
            CSIDR: 0x64,
            CSIAR: 0x68,
            CSIAR_FLAG: 0x8000_0000,
            CSIAR_WRITE_CMD: 0x8000_0000,
            CSIAR_BYTE_ENABLE: 0x0000_f000,
            CSIAR_ADDR_MASK: 0x0000_0fff,
            PMCH: 0x6f,
            D3COLD_NO_PLL_DOWN: bit!(7),
            D3HOT_NO_PLL_DOWN: bit!(6),
            D3_NO_PLL_DOWN: (bit!(7)) | (bit!(6)),
            EPHYAR: 0x80,
            EPHYAR_FLAG: 0x8000_0000,
            EPHYAR_WRITE_CMD: 0x8000_0000,
            EPHYAR_REG_MASK: 0x1f,
            EPHYAR_REG_SHIFT: 16,
            EPHYAR_DATA_MASK: 0xffff,
            DLLPR: 0xd0,
            PFM_EN: 1 << 6,
            TX_10M_PS_EN: 1 << 7,
            DBG_REG: 0xd1,
            FIX_NAK_1: 1 << 4,
            FIX_NAK_2: 1 << 3,
            TWSI: 0xd2,
            MCU: 0xd3,
            NOW_IS_OOB: 1 << 7,
            TX_EMPTY: 1 << 5,
            RX_EMPTY: 1 << 4,
            RXTX_EMPTY: TX_EMPTY | RX_EMPTY,
            EN_NDP: 1 << 3,
            EN_OOB_RESET: 1 << 2,
            LINK_LIST_RDY: 1 << 1,
            EFUSEAR: 0xdc,
            EFUSEAR_FLAG: 0x8000_0000,
            EFUSEAR_WRITE_CMD: 0x8000_0000,
            EFUSEAR_READ_CMD: 0x0000_0000,
            EFUSEAR_REG_MASK: 0x03ff,
            EFUSEAR_REG_SHIFT: 8,
            EFUSEAR_DATA_MASK: 0xff,
            MISC_1: 0xf2,
            PFM_D3COLD_EN: 1 << 6
        }
    }

    pub(crate) mod rtl8168 {
        declare_rtl_registers! {
            LED_CTRL: 0x18,
            LED_FREQ: 0x1a,
            EEE_LED: 0x1b,
            ERIDR: 0x70,
            ERIAR: 0x74,
            ERIAR_FLAG: 0x8000_0000,
            ERIAR_WRITE_CMD: 0x8000_0000,
            ERIAR_READ_CMD: 0x0000_0000,
            ERIAR_ADDR_BYTE_ALIGN: 4,
            ERIAR_TYPE_SHIFT: 16,
            ERIAR_EXGMAC: 0x00 << ERIAR_TYPE_SHIFT,
            ERIAR_MSIX: 0x01 << ERIAR_TYPE_SHIFT,
            ERIAR_ASF: 0x02 << ERIAR_TYPE_SHIFT,
            ERIAR_OOB: 0x02 << ERIAR_TYPE_SHIFT,
            ERIAR_MASK_SHIFT: 12,
            ERIAR_MASK_0001: 0x1 << ERIAR_MASK_SHIFT,
            ERIAR_MASK_0011: 0x3 << ERIAR_MASK_SHIFT,
            ERIAR_MASK_0100: 0x4 << ERIAR_MASK_SHIFT,
            ERIAR_MASK_0101: 0x5 << ERIAR_MASK_SHIFT,
            ERIAR_MASK_1111: 0xf << ERIAR_MASK_SHIFT,
            EPHY_RXER_NUM: 0x7c,
            OCPDR: 0xb0,    /* OCP GPHY access */
            OCPDR_WRITE_CMD: 0x80000000,
            OCPDR_READ_CMD: 0x00000000,
            OCPDR_REG_MASK: 0x7f,
            OCPDR_GPHY_REG_SHIFT: 16,
            OCPDR_DATA_MASK: 0xffff,
            OCPAR: 0xb4,
            OCPAR_FLAG: 0x80000000,
            OCPAR_GPHY_WRITE_CMD: 0x8000f060,
            OCPAR_GPHY_READ_CMD: 0x0000f060,
            GPHY_OCP: 0xb8,
            RDSAR1: 0xd0,   /* 8168c only. Undocumented on 8168dp */
            MISC: 0xf0,     /* 8168e only. */
            TXPLA_RST: 1 << 29,
            DISABLE_LAN_EN: 1 << 23,    /* Enable GPIO pin */
            PWM_EN: 1 << 22,
            RXDV_GATED_EN: 1 << 19,
            EARLY_TALLY_EN: 1 << 16
        }
    }

    pub(crate) mod rtl8125 {
        declare_rtl_registers! {
            LEDSEL0: 0x18,
            INT_CFG0_8125: 0x34,
            INT_CFG0_ENABLE_8125: 0x1,
            INT_CFG0_CLKREQE: bit!(3),
            INTR_MASK_8125: 0x38,
            INTR_STATUS_8125: 0x3c,
            INT_CFG1_8125: 0x7a,
            LEDSEL2: 0x84,
            LEDSEL1: 0x86,
            TX_POLL_8125: 0x90,
            LEDSEL3: 0x96,
            MAC0_BKP: 0x19e0,
            RSS_CTRL_8125: 0x4500,
            Q_NUM_CTRL_8125: 0x4800,
            EEE_TXIDLE_TIMER_8125: 0x6048
        }
    }
}

pub(crate) mod rtl_register_content {
    macro_rules! declare_rtl_register_contents {
        ($($name:ident: $value:expr),*) => {
            $(
                pub(crate) const $name: u32 = $value;
            )*
        };
    }

    declare_rtl_register_contents! {
        /* InterruptStatusBits */
        SYS_ERR: 0x8000,
        PCS_TIMEOUT: 0x4000,
        SW_INT: 0x0100,
        TX_DESC_UNAVAIL: 0x0080,
        RX_FIFO_OVER: 0x0040,
        LINK_CHG: 0x0020,
        RX_OVERFLOW: 0x0010,
        TX_ERR: 0x0008,
        TX_OK: 0x0004,
        RX_ERR: 0x0002,
        RX_OK: 0x0001,

        /* RxStatusDesc */
        RX_RWT: 1 << 22,
        RX_RES: 1 << 21,
        RX_RUNT: 1 << 20,
        RX_CRC: 1 << 19,

        /* ChipCmdBits */
        STOP_REQ: 0x80,
        CMD_RESET: 0x10,
        CMD_RX_ENB: 0x08,
        CMD_TX_ENB: 0x04,
        RX_BUF_EMPTY: 0x01,

        /* TXPoll register p.5 */
        HPQ: 0x80,      /* Poll cmd on the high prio queue */
        NPQ: 0x40,      /* Poll cmd on the low prio queue */
        FSW_INT: 0x01,  /* Forced software interrupt */

        /* Cfg9346Bits */
        CFG9346_LOCK: 0x00,
        CFG9346_UNLOCK: 0xc0,

        /* rx_mode_bits */
        ACCEPT_ERR: 0x20,
        ACCEPT_RUNT: 0x10,
        RX_CONFIG_ACCEPT_ERR_MASK: 0x30,
        ACCEPT_BROADCAST: 0x08,
        ACCEPT_MULTICAST: 0x04,
        ACCEPT_MY_PHYS: 0x02,
        ACCEPT_ALL_PHYS: 0x01,
        RX_CONFIG_ACCEPT_OK_MASK: 0x0f,
        RX_CONFIG_ACCEPT_MASK: 0x3f,

        /* TxConfigBits */
        TX_INTER_FRAME_GAP_SHIFT: 24,
        TX_DMA_SHIFT: 8,    /* DMA burst value (0-7) is shift this many bits */

        /* Config1 register p.24 */
        LEDS1: 1 << 7,
        LEDS0: 1 << 6,
        SPEED_DOWN: 1 << 4,
        MEMMAP: 1 << 3,
        IOMAP: 1 << 2,
        VPD: 1 << 1,
        PM_ENABLE: 1 << 0,  /* Power Management Enable */

        /* Config2 register p. 25 */
        CLK_REQ_EN: 1 << 7, /* Clock Request Enable */
        MSI_ENABLE: 1 << 5, /* 8169 only. Reserved in the 8168. */
        PCI_CLOCK_66MHZ: 0x01,
        PCI_CLOCK_33MHZ: 0x00,

        /* Config3 register p.25 */
        MAGIC_PACKET: 1 << 5,   /* Wake up when receives a Magic Packet */
        LINK_UP: 1 << 4,    /* Wake up when the cable connection is re-established */
        JUMBO_EN0: 1 << 2,  /* 8168 only. Reserved in the 8168b */
        RDY_TO_L23: 1 << 1, /* L23 Enable */
        BEACON_EN: 1 << 0,  /* 8168 only. Reserved in the 8168b */

        /* Config4 register */
        JUMBO_EN1: 1 << 1,  /* 8168 only. Reserved in the 8168b */

        /* Config5 register p.27 */
        BWF: 1 << 6,        /* Accept Broadcast wakeup frame */
        MWF: 1 << 5,        /* Accept Multicast wakeup frame */
        UWF: 1 << 4,        /* Accept Unicast wakeup frame */
        SPI_EN: 1 << 3,
        LAN_WAKE: 1 << 1,   /* LanWake enable/disable */
        PME_STATUS: 1 << 0, /* PME status can be reset by PCI RST# */
        ASPM_EN: 1 << 0,    /* ASPM enable */

        /* CPlusCmd p.31 */
        ENABLE_BIST: 1 << 15,       // 8168 8101
        MAC_DBGO_OE: 1 << 14,       // 8168 8101
        EN_ANA_PLL: 1 << 14,        // 8169
        NORMAL_MODE: 1 << 13,       // unused
        FORCE_HALF_DUP: 1 << 12,    // 8168 8101
        FORCE_RXFLOW_EN: 1 << 11,   // 8168 8101
        FORCE_TXFLOW_EN: 1 << 10,   // 8168 8101
        CXPL_DBG_SEL: 1 << 9,       // 8168 8101
        ASF: 1 << 8,                // 8168 8101
        PKT_CNTR_DISABLE: 1 << 7,   // 8168 8101
        MAC_DBGO_SEL: 0x001c,       // 8168
        RX_VLAN: 1 << 6,
        RX_CHK_SUM: 1 << 5,
        PCIDAC: 1 << 4,
        PCI_MUL_RW: 1 << 3,
        INTT_MASK: super::genmask_u32(0..=1),
        CPCMD_MASK: NORMAL_MODE | RX_VLAN | RX_CHK_SUM | INTT_MASK,

        /* rtl8169_PHYstatus */
        TBI_ENABLE: 0x80,
        TX_FLOW_CTRL: 0x40,
        RX_FLOW_CTRL: 0x20,
        _1000BPS_F: 0x10,
        _100BPS: 0x08,
        _10BPS: 0x04,
        LINK_STATUS: 0x02,
        FULL_DUP: 0x01,

        /* ResetCounterCommand */
        COUNTER_RESET: 0x1,

        /* DumpCounterCommand */
        COUNTER_DUMP: 0x8,

        /* magic enable v2 */
        MAGIC_PACKET_V2: 1 << 16    /* Wake up when receives a Magic Packet */
    }
}

pub(crate) mod desc_bit {
    macro_rules! declare_desc_bits {
        ($($name:ident: $value:expr),*) => {
            $(
                pub(crate) const $name: u32 = $value;
            )*
        };
    }

    declare_desc_bits! {
        DESC_OWN: 1 << 31, /* Descriptor is owned by NIC */
        RING_END: 1 << 30, /* End of descriptor ring */
        FIRST_FRAG: 1 << 29, /* First segment of a packet */
        LAST_FRAG: 1 << 28 /* Final segment of a packet */
    }

    pub(crate) mod tx {
        declare_desc_bits! {
            /* First doubleword. */
            TD_LSO: 1 << 27, /* Large Send Offload */
            TD_MSS_MAX: 0x07ff, /* MSS value */

            /* Second doubleword. */
            TX_VLAN_TAG: 1 << 17 /* Add VLAN tag */
        }
    }

    pub(crate) mod rx {
        declare_desc_bits! {
            /* Rx private */
            PID1: 1 << 18, /* Protocol ID bit 1/2 */
            PID0: 1 << 17, /* Protocol ID bit 0/2 */

            RX_PROTO_UDP: PID1,
            RX_PROTO_TCP: PID0,
            RX_PROTO_IP: PID1 | PID0,
            RX_PROTO_MASK: RX_PROTO_IP,

            IP_FAIL: 1 << 16, /* IP checksum failed */
            UDP_FAIL: 1 << 15, /* UDP/IP checksum failed */
            TCP_FAIL: 1 << 14, /* TCP/IP checksum failed */

            RX_CS_FAIL_MASK: IP_FAIL | UDP_FAIL | TCP_FAIL,

            RX_VLAN_TAG: 1 << 16 /* VLAN tag available */
        }
    }

    /* 8169, 8168b and 810x except 8102e. */
    pub(crate) mod v0 {
        pub(crate) mod tx {
            declare_desc_bits! {
                /* First doubleword. */
                TD0_MSS_SHIFT: 16, /* MSS position (11 bits) */
                TD0_TCP_CS: 1 << 16, /* Calculate TCP/IP checksum */
                TD0_UDP_CS: 1 << 17, /* Calculate UDP/IP checksum */
                TD0_IP_CS: 1 << 18 /* Calculate IP checksum */
            }
        }
    }

    /* 8102e, 8168c and beyond. */
    pub(crate) mod v1 {
        pub(crate) mod tx {
            declare_desc_bits! {
                /* First doubleword. */
                TD1_GTSENV4: 1 << 26, /* Giant Send for IPv4 */
                TD1_GTSENV6: 1 << 25, /* Giant Send for IPv6 */
                GTTCPHO_SHIFT: 18,
                GTTCPHO_MAX: 0x7f,

                /* Second doubleword. */
                TCPHO_SHIFT: 18,
                TCPHO_MAX: 0x3ff,
                TD1_MSS_SHIFT: 18, /* MSS position (11 bits) */
                TD1_IPV6_CS: 1 << 28, /* Calculate IPv6 checksum */
                TD1_IPV4_CS: 1 << 29, /* Calculate IPv4 checksum */
                TD1_TCP_CS: 1 << 30, /* Calculate TCP/IP checksum */
                TD1_UDP_CS: 1 << 31 /* Calculate UDP/IP checksum */
            }
        }
    }
}
