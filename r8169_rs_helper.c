#include <linux/netdevice.h>
#include "r8169_rs_helper.h"

void rust_helper_netif_stop_queue(struct net_device *dev)
{
	netif_stop_queue(dev);
}

void rust_helper_netif_wake_queue(struct net_device *dev)
{
	netif_wake_queue(dev);
}
