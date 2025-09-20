// SPDX-License-Identifier: GPL-2.0

//! Realtek 8169/8168/8101 Ethernet driver - Rust port
//!
//! Copyright (c) 2025 Guilherme Lima <acc.guilhermenl@gmail.com>
//! Copyright (c) a lot of people too. Please respect their work.
//!
//! This crate provides a PCI driver implementation for Realtek Ethernet controllers.

mod defs;
mod helper;

use crate::defs::{register::TX_CONFIG, *};
use core::{mem::size_of, ptr::NonNull, sync::atomic::AtomicU8};
use kernel::{
    bindings, c_str,
    clk::OptionalClk,
    device::Core,
    devres::Devres,
    dma::{Device, DmaMask},
    error::to_result,
    new_mutex, new_spinlock, new_work,
    pci::{self, Bar},
    prelude::*,
    sync::Mutex,
    types::ARef,
    workqueue::Work,
};
use pin_init::PinnedDrop;

kernel::pci_device_table! {
    RTL8169_PCI_TABLE,
    MODULE_PCI_TABLE,
    <Rtl8169Private as pci::Driver>::IdInfo,
    [
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x2502),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x2600),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8129),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8136),
            Rtl8169IdInfo::NO_GBIT
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8161),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8162),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8167),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8168),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_NCUBE, 0x8168),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8169),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_DLINK, 0x4300),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_DLINK, 0x4302),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_AT, 0xc107),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_USR, 0x0116),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_LINKSYS, 0x1032),
            Rtl8169IdInfo::NONE
        ),
        (pci::DeviceId::from_id(0x0001, 0x8168), Rtl8169IdInfo::NONE),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8125),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8126),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8127),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x3000),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x5000),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x0e10),
            Rtl8169IdInfo::NONE
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_ANY_ID as u32, bindings::PCI_ANY_ID as u32),
            Rtl8169IdInfo::NONE
        )
    ]
}

impl pci::Driver for Rtl8169Private {
    type IdInfo = Rtl8169IdInfo;
    const ID_TABLE: pci::IdTable<Self::IdInfo> = &RTL8169_PCI_TABLE;
    fn probe(pdev: &pci::Device<Core>, info: &Self::IdInfo) -> Result<Pin<KBox<Self>>> {
        /* enable device (incl. PCI PM wakeup and hotplug setup) */
        let pdev_raw = pdev.as_raw();
        let dev_raw = pdev.as_ref().as_raw();

        to_result(unsafe { bindings::pcim_enable_device(pdev) }).map_err(|rc| {
            pr_err!("enable failure");
            rc
        })?;

        let dev = NonNull::new(unsafe {
            bindings::devm_alloc_etherdev_mqs(dev_raw, size_of::<Self>(), 1, 1)
        })
        .ok_or(Error::from_errno(ENOMEM))?;

        set_netdev_dev!(dev, dev_raw);

        /* Get the *optional* external "ether_clk" used on some boards */
        let clk = OptionalClk::get(pdev, Some(c_str!("ether_clk")))
            .map(|clk| *clk)
            .map_err(|_| {
                pr_err!("failed to get ether_clk");
            })
            .ok();

        to_result(unsafe { bindings::pci_set_mwi(pdev_raw) }).map_err(|rc| {
            pr_err!("Mem-Wr-Inval unavailable");
            rc
        })?;

        let tp = KBox::pin_init(
            try_pin_init!(Self {
                /* use first MMIO region */
                mmio_addr <- pdev.iomap_region(0, c_str!("r8169_rs")),
                pci_dev: pdev.into(),
                dev,
                // phy_device: ARef<phy::Device>,
                // napi: bindings::napi_struct,
                mac_version: MacVersion::RtlGigaMacNone,
                rtl_dash_type: DashType::RtlDashNone,
                cur_rx: 0, /* Index into the Rx descriptor buffer of next Rx pkt. */
                cur_tx: 0, /* Index into the Tx descriptor buffer of next Rx pkt. */
                dirty_tx: 0,
                // #[pin]
                // tx_desc_array: ARef<TxDesc>, /* 256-aligned Tx descriptor ring */
                // #[pin]
                // rx_desc_array: ARef<RxDesc>, /* 256-aligned Rx descriptor ring */
                tx_phy_addr: 0,
                rx_phy_addr: 0,
                // #[pin]
                // rx_databuff: ARef<[Page; NUM_RX_DESC]>, /* Rx data buffers */
                // #[pin]
                // tx_skb: ARef<[RingInfo; NUM_TX_DESC]>, /* Tx data buffers */
                tx_lpi_timer: 0,
                irq_mask: 0,
                irq: unsafe { bindings::pci_irq_vector(pdev.as_raw(), 0) as i32 },
                clk: ARef::from(&clk),
                work <- new_work!("r8169_rs::work"),
                work_flags: AtomicU8::new(0),
                mac_ocp_lock <- new_spinlock!(()),
                led_lock <- new_mutex!(()),
                flags: 0,
                // counters_phys_addr: bindings::dma_addr_t,
                // #[pin]
                // rtl8169_counters: ARef<Counters>,
                // tc_offset: TcOffset,
                saved_wolopts: 0,
                // fw_name: RtlFirmwareName,
                // rtl_fw: ARef<RtlFirmware>,
                // leds: LedClassDev,
                ocp_base: OCP_STD_PHY_BASE,
            }),
            GFP_KERNEL,
        )?;

        let txconfig = tp.as_ref().r32(TX_CONFIG).unwrap_or(!0u32);
        if txconfig == !0u32 {
            pr_err!("PCI read failed");
            return Err(EIO);
        }

        if *info == Rtl8169IdInfo::NO_GBIT {
            tp.as_mut().set_supports_gmii(true);
        }

        let xid: u16 = (txconfig >> 20) & 0xfcf;

        let chip = RtlChipInfo::get_chip_version(xid, tp.as_ref().supports_gmii());
        if chip.mac_version == MacVersion::RtlGigaMacNone {
            pr_err!("unknown chip XID %03x, contact r8169 maintainers (see MAINTAINERS file)");
            return Err(ENODEV);
        }

        tp.as_mut().mac_version = chip.mac_version;
        tp.as_mut().fw_name = chip.fw_name;

        /* Disable ASPM L1 as that cause random device stop working
         * problems as well as full system hangs for some PCIe devices users.
         */
        tp.as_mut()
            .set_aspm_manageable(if tp.as_ref().aspm_is_safe() {
                true
            } else {
                (unsafe { bindings::pci_disable_link_state(pdev_raw, PCIE_LINK_STATE_L1 as c_int) }
                    == 0)
            });

        tp.as_mut().dash_type = tp.as_ref().rtl_get_dash_type();
        tp.as_mut()
            .set_dash_enabled(tp.as_ref().rtl_dash_is_enabled());

        tp.as_mut().cp_cmd = ((tp.as_ref().r16(rtl_register::C_PLUS_CMD).unwrap() as u32)
            & rtl_register_content::CPCMD_MASK)
            .into();

        if size_of::<bindings::dma_addr_t>() > 4
            && tp.as_ref().mac_version >= MacVersion::RtlGigaMacVer18
            && unsafe {
                pdev.dma_set_mask_and_coherent(DmaMask::new::<64>())
                    .is_err()
            }
        {
            tp.as_mut().dev.as_mut().features |= NETIF_F_HIGHDMA;
        }

        tp.as_ref().rtl_init_rxcfg();
        tp.as_ref().rtl8169_irq_mask_and_ack();

        tp.as_ref().rtl_hw_initialize();

        tp.as_ref().rtl_hw_reset();

        to_result(tp.as_ref().rtl_alloc_irq()).map_err(|rc| {
            pr_err!("Can't allocate interrupt");
            rc
        })?;

        Ok(tp)
    }
}

kernel::module_pci_driver! {
    type: Rtl8169Private,
    name: "r8169_rs",
    authors: ["Guilherme Lima"],
    description: "RealTek 8169/8168/8101 ethernet driver rust port.",
    license: "GPL v2",
}
