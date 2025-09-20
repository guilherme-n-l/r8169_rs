// SPDX-License-Identifier: GPL-2.0

//! Realtek 8169/8168/8101 Ethernet driver - Rust port
//!
//! Copyright (c) 2025 Guilherme Lima <acc.guilhermenl@gmail.com>
//! Copyright (c) a lot of people too. Please respect their work.
//!
//! This crate provides a PCI driver implementation for Realtek Ethernet controllers.

mod defs;

use crate::defs::*;
use core::pin::Pin;
use kernel::{alloc::allocator::Kmalloc, bindings, device::Core, error::Error, pci, prelude::Box};

kernel::pci_device_table!(
    RTL8169_PCI_TABLE,
    MODULE_PCI_TABLE,
    <Rtl8169Driver as pci::Driver>::IdInfo,
    [
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x2502),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x2600),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8129),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8136),
            Rtl8169IdInfo::NoGbit
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8161),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8162),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8167),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8168),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_NCUBE, 0x8168),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8169),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_DLINK, 0x4300),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_DLINK, 0x4302),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_AT, 0xc107),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_USR, 0x0116),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_LINKSYS, 0x1032),
            Rtl8169IdInfo::None
        ),
        (pci::DeviceId::from_id(0x0001, 0x8168), Rtl8169IdInfo::None),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8125),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8126),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x8127),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x3000),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x5000),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REALTEK, 0x0e10),
            Rtl8169IdInfo::None
        ),
        (
            pci::DeviceId::from_id(bindings::PCI_ANY_ID as u32, bindings::PCI_ANY_ID as u32),
            Rtl8169IdInfo::None
        )
    ]
);

impl pci::Driver for Rtl8169Driver {
    type IdInfo = Rtl8169IdInfo;
    const ID_TABLE: pci::IdTable<Self::IdInfo> = &RTL8169_PCI_TABLE;
    fn probe(
        _: &pci::Device<Core>,
        _: &<Self as pci::Driver>::IdInfo,
    ) -> Result<Pin<Box<Self, Kmalloc>>, Error> {
        todo!()
    }
}

kernel::module_pci_driver! {
    type: Rtl8169Driver,
    name: "r8169_rs",
    authors: ["Guilherme Lima"],
    description: "RealTek 8169/8168/8101 ethernet driver rust port.",
    license: "GPL v2",
}
