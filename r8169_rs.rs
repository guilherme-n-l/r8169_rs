mod defs;

use crate::defs::*;
use kernel::pci;

kernel::module_pci_driver! {
    type: Rtl8169Driver,
    name: "r8169_rs",
    authors: ["Guilherme Lima"],
    description: "RealTek 8169/8168/8101 ethernet driver rust port.",
    license: "GPL v2",
}
