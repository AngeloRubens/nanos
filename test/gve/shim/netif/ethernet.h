/* Host shim for <netif/ethernet.h>. */
#ifndef GVE_HARNESS_SHIM_ETHERNET_H
#define GVE_HARNESS_SHIM_ETHERNET_H

#include <lwip.h>

err_t ethernet_input(struct pbuf *p, struct netif *netif);

#endif
