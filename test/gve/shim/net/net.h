/* Host shim for <net/net.h>: the netif_dev wrapper + the napi-id helper. */
#ifndef GVE_HARNESS_SHIM_NET_H
#define GVE_HARNESS_SHIM_NET_H

#include <harness_runtime.h>
#include <lwip.h>

closure_type(netif_dev_setup, boolean, tuple config);

typedef struct netif_dev {
    struct netif n;
    closure_struct(netif_dev_setup, setup);
} *netif_dev;

static inline void netif_dev_init(netif_dev dev) { }

u16 net_get_napi_id(u8 netif_num, u16 queue_idx);

typedef err_t (*netif_init_fn)(struct netif *netif);
typedef err_t (*netif_input_fn)(struct pbuf *p, struct netif *inp);
struct netif *netif_add(struct netif *netif, const void *ipaddr,
                        const void *netmask, const void *gw, void *state,
                        netif_init_fn init, netif_input_fn input);

#endif
