#ifndef R8169_RS_HELPER
#define R8169_RS_HELPER
#include <linux/netdevice.h>


/**
 *	rust helper for netif_stop_queue - stop transmitted packets
 *	@dev: network device
 *
 *	Stop upper layers calling the device hard_start_xmit routine.
 *	Used for flow control when transmit resources are unavailable.
 */
void rust_helper_netif_stop_queue(struct net_device *dev)

/**
 *	rust helper for netif_wake_queue - restart transmit
 *	@dev: network device
 *
 *	Allow upper layers to call the device hard_start_xmit routine.
 *	Used for flow control when transmit resources are available.
 */
void rust_helper_netif_wake_queue(struct net_device *dev)

#endif // R8169_RS_HELPER
