use kernel::bindings::net_device;

macro_rules! bit {
    ($n: expr) => {
        (1u32 << $n)
    };
}

use bit;

/* L1 state */
pub(crate) const PCIE_LINK_STATE_L1: u32 = bit!(2);
pub(crate) const PCIE_LINK_STATE_L0S: u32 = bit!(0) | bit!(1);

unsafe extern "C" {
    pub fn rust_helper_netif_stop_queue(dev: *mut net_device);
    pub fn rust_helper_netif_wake_queue(dev: *mut net_device);
}

pub(crate) unsafe fn netif_stop_queue(dev: *mut net_device) {
    unsafe { rust_helper_netif_stop_queue(dev) };
}

pub(crate) unsafe fn netif_wake_queue(dev: *mut net_device) {
    unsafe {rust_helper_netif_wake_queue(dev) };
}
