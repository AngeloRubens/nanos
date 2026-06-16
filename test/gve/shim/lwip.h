/* Host shim for <lwip.h>: minimal pbuf + netif model.  pbuf is malloc-backed;
 * pbuf_ref/pbuf_free maintain a real refcount so the harness can assert the
 * driver's ownership transfers.  See harness.c for the implementations. */
#ifndef GVE_HARNESS_SHIM_LWIP_H
#define GVE_HARNESS_SHIM_LWIP_H

#include <harness_runtime.h>

typedef s8 err_t;
#define ERR_OK   0
#define ERR_MEM  (-1)
#define ERR_IF   (-2)

typedef u16 u16_t;
typedef u8  u8_t;

#define PBUF_RAW  0
#define PBUF_RAM  0
#define PBUF_REF  1

#define NETIF_NO_INDEX 0xff

struct pbuf {
    struct pbuf *next;
    void   *payload;
    u16     tot_len;
    u16     len;
    u8      type_internal;
    u8      flags;
    u16     ref;
    u8      if_idx;
    u16     napi_id;
};

struct pbuf *pbuf_alloc(int layer, u16 length, int type);
u8   pbuf_free(struct pbuf *p);
void pbuf_ref(struct pbuf *p);
void pbuf_cat(struct pbuf *head, struct pbuf *tail);
err_t pbuf_take(struct pbuf *buf, const void *dataptr, u16 len);

#define NETIF_FLAG_UP        0x01
#define NETIF_FLAG_BROADCAST 0x02
#define NETIF_FLAG_ETHARP    0x04
#define ETH_HWADDR_LEN 6

struct netif {
    u8   name[2];
    u8   num;
    u16  mtu;
    u32  flags;
    u8   hwaddr_len;
    u8   hwaddr[ETH_HWADDR_LEN];
    void *state;
    err_t (*input)(struct pbuf *p, struct netif *inp);
    err_t (*linkoutput)(struct netif *netif, struct pbuf *p);
};

void netif_set_link_up(struct netif *netif);
void netif_set_link_down(struct netif *netif);

#endif
