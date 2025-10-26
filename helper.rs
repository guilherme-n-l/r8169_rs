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
    unsafe { rust_helper_netif_wake_queue(dev) };
}

#[derive(Debug)]
enum HelperCastError {
    SizeDiffer,
    Misalignment,
}

pub(crate) fn safe_cast<T>(buffer: &[u8]) -> Result<T, HelperCastError> {
    if buffer.len() != core::mem::size_of::<T>() {
        return Err(HelperCastError::SizeDiffer);
    }
    unsafe {
        let ptr = buffer.as_ptr() as *const T;
        if (ptr as usize) % core::mem::align_of::<T>() != 0 {
            return Err(HelperCastError::Misalignment);
        }

        Ok(ptr.read())
    }
}
