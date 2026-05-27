/* gve_datapath.c — gVNIC GQI TX and RX hot path
 *
 * TX: QPL mode copies packet payload into registered bounce pages;
 *     RDA mode passes physical_from_virtual(pbuf->payload) directly to
 *     the device — no memcpy per packet.
 *     Both modes call gve_tx_fill_csum() to offload TCP/UDP checksum
 *     computation to the NIC (GVE_TXF_L4CSUM + pseudo-header seed).
 * RX: QPL mode uses a pre-registered page pool; RDA mode uses a
 *     directly allocated contiguous buffer with physical addresses.
 *     Both modes are unified via rda_base_phys (0 for QPL).
 * DQO: handled separately in gve_dqo.c.
 */

#include "gve_priv.h"

/* ------------------------------------------------------------------ */
/* TX checksum offload (GQI QPL and RDA)                                */
/* ------------------------------------------------------------------ */

/* gve_tx_fill_csum — GQI wrapper around gve_pseudo_csum (gve_priv.h).
 * l4_hdr_offset and l4_csum_offset are in 2-byte units per GQI spec:
 *   TCP chksum at byte 16 of TCP header → offset 8.
 *   UDP chksum at byte  6 of UDP header → offset 3. */
static void gve_tx_fill_csum(struct pbuf *p, struct gve_tx_pkt_desc *pkt)
{
    u8    proto;
    u16_t l4_hdr_off;
    if (!gve_pseudo_csum(p, &proto, &l4_hdr_off))
        return;
    pkt->type_flags    |= GVE_TXF_L4CSUM;
    pkt->l4_hdr_offset  = l4_hdr_off / 2;
    pkt->l4_csum_offset = (proto == GVE_IP_PROTO_TCP) ? 8 : 3;
}

/* ------------------------------------------------------------------ */
/* TX path                                                              */
/* ------------------------------------------------------------------ */

/* gve_tx_cleanup_rda / _qpl: retire completed TX descriptors.
 * Split by format so there is no raw_addressing branch in the hot path.
 * In RDA mode every retired slot has a non-NULL pending pbuf (set
 * unconditionally in gve_linkoutput_rda before tx->head++), so the
 * if (pending) guard is safe to remove. */
static void gve_tx_cleanup_rda(gve_tx_queue tx)
{
    u32 tail = be32toh(
        tx->adapter->event_counters[be32toh(tx->q_res->counter_index)]);
    for (; tx->tail != tail; tx->tail++) {
        u32 slot = tx->tail & tx->mask;
        pbuf_free(tx->pending[slot]);
        tx->pending[slot] = NULL;
    }
}

static void gve_tx_cleanup_qpl(gve_tx_queue tx)
{
    u32 tail = be32toh(
        tx->adapter->event_counters[be32toh(tx->q_res->counter_index)]);
    gve_debug("TX tail %d -> %d, QPL used %d", tx->tail, tail,
              tx->qpl_used);
    for (; tx->tail != tail; tx->tail++)
        tx->qpl_used -= tx->qpl_allocated[tx->tail & tx->mask];
}

/*
 * gve_tx_qpl_cpy: copy one pbuf segment into the TX QPL ring.
 *
 * *allocated receives the number of QPL bytes consumed (including
 * cacheline-alignment padding).
 */
static void gve_tx_qpl_cpy(gve_tx_queue tx, struct pbuf *p,
                            u32 *offset, u32 *allocated)
{
    u32 padded_len = pad(p->len, GVE_TX_PAD);
    *allocated = padded_len;

    /* Wrap-aware: check whether we need to split across the ring end. */
    u32 remain = tx->qpl_size - tx->qpl_head;
    if (padded_len > remain) {
        /* Not enough contiguous space at tail; waste the tail fragment
         * and restart from offset 0. */
        *allocated += remain;
        tx->qpl_head = 0;
    }
    *offset = tx->qpl_head;
    runtime_memcpy(tx->qpl_base + *offset, p->payload, p->len);
    tx->qpl_head += padded_len;
    if (tx->qpl_head == tx->qpl_size)
        tx->qpl_head = 0;
}

static err_t gve_linkoutput_qpl(struct netif *netif, struct pbuf *p)
{
    gve adapter = netif->state;
    u32 qidx = current_cpu()->id % adapter->num_queues;
    gve_tx_queue tx = &adapter->tx[qidx];

    if (tx->stuck)
        return ERR_IF;

    gve_tx_cleanup_qpl(tx);

    /* Pre-flight: count segments and required QPL bytes.
     * Mirrors gve_tx_qpl_cpy wrap logic: when a segment does not fit in
     * the contiguous tail of the ring, the tail fragment is wasted and
     * the copy restarts from offset 0.  Account for both the wasted tail
     * (remain) and the segment itself (padded). */
    int  seg_count  = 0;
    u32  qpl_needed = 0;
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        u32 padded   = pad(q->len, GVE_TX_PAD);
        u32 sim_head = (tx->qpl_head + qpl_needed) % tx->qpl_size;
        u32 remain   = tx->qpl_size - sim_head;
        qpl_needed  += (padded > remain) ? padded + remain : padded;
        seg_count++;
    }

    if ((tx->head - tx->tail + seg_count > adapter->tx_desc_cnt) ||
            (tx->qpl_used + qpl_needed > tx->qpl_size))
        return ERR_MEM;

    /* Seed pseudo-header checksum in the pbuf BEFORE the QPL copy so that
     * the updated checksum field lands in the QPL bounce buffer the device
     * will DMA-read. gve_tx_fill_csum also records the offload fields in
     * the descriptor. */
    u32 slot = tx->head & tx->mask;
    struct gve_tx_pkt_desc *pkt = &tx->desc[slot].pkt;
    pkt->type_flags     = GVE_TXD_STD;
    pkt->l4_csum_offset = 0;
    pkt->l4_hdr_offset  = 0;
    gve_tx_fill_csum(p, pkt);  /* may update p->payload checksum field */

    u32 offset;
    gve_tx_qpl_cpy(tx, p, &offset, &tx->qpl_allocated[slot]);
    tx->qpl_used += tx->qpl_allocated[slot];
    tx->head++;

    pkt->desc_cnt = seg_count;
    pkt->len      = htobe16(p->tot_len);
    pkt->seg_len  = htobe16(p->len);
    pkt->seg_addr = htobe64(offset);

    for (struct pbuf *q = p->next; q != NULL; q = q->next) {
        u32 seg_off;
        gve_tx_qpl_cpy(tx, q, &seg_off,
                       &tx->qpl_allocated[tx->head & tx->mask]);
        tx->qpl_used += tx->qpl_allocated[tx->head & tx->mask];

        struct gve_tx_seg_desc *seg = &tx->desc[tx->head++ & tx->mask].seg;
        seg->type_flags = GVE_TXD_SEG;
        seg->seg_len    = htobe16(q->len);
        seg->seg_addr   = htobe64(seg_off);
    }

    tx->tx_stats.cnt++;
    tx->tx_stats.bytes += p->tot_len;
    adapter->hw_stats.tx_packets++;
    adapter->hw_stats.tx_bytes += p->tot_len;
    write_barrier();
    pci_bar_write_4(&adapter->db_bar,
                    be32toh(tx->q_res->db_index) * sizeof(u32),
                    htobe32(tx->head));
    return ERR_OK;
}

/*
 * GQI-RDA TX path: seg_addr is the physical address of the pbuf payload.
 * No QPL copy — the device DMA-reads directly from the pbuf memory.
 *
 * Each segment pbuf (including chain elements q = p->next) gets its own
 * pbuf_ref(), not just the chain head.  Cleanup retires slots individually
 * via pbuf_free(pending[slot]), so each slot must hold an independent
 * reference.  If only the head were ref'd, cleanup's pbuf_free(q) would
 * bring q->ref to 0 and free q while p->next still points to it, causing
 * a use-after-free when lwIP later frees the chain.  lwIP's pbuf_free()
 * stops cascade propagation as soon as ref > 0, so the double-decrement
 * (cleanup then lwIP) is safe.
 */
static err_t gve_linkoutput_rda(struct netif *netif, struct pbuf *p)
{
    gve adapter = netif->state;
    u32 qidx = current_cpu()->id % adapter->num_queues;
    gve_tx_queue tx = &adapter->tx[qidx];

    if (tx->stuck)
        return ERR_IF;

    gve_tx_cleanup_rda(tx);

    int seg_count = 0;
    for (struct pbuf *q = p; q != NULL; q = q->next)
        seg_count++;

    if (tx->head - tx->tail + seg_count > adapter->tx_desc_cnt)
        return ERR_MEM;

    struct gve_tx_pkt_desc *pkt = &tx->desc[tx->head & tx->mask].pkt;
    pbuf_ref(p);
    tx->pending[tx->head & tx->mask] = p;
    tx->head++;

    pkt->type_flags     = GVE_TXD_STD;
    pkt->l4_csum_offset = 0;
    pkt->l4_hdr_offset  = 0;
    pkt->desc_cnt       = seg_count;
    pkt->len            = htobe16(p->tot_len);
    pkt->seg_len        = htobe16(p->len);
    pkt->seg_addr       = htobe64(physical_from_virtual(p->payload));
    gve_tx_fill_csum(p, pkt);

    for (struct pbuf *q = p->next; q != NULL; q = q->next) {
        struct gve_tx_seg_desc *seg = &tx->desc[tx->head & tx->mask].seg;
        pbuf_ref(q);
        tx->pending[tx->head & tx->mask] = q;
        tx->head++;

        seg->type_flags = GVE_TXD_SEG;
        seg->seg_len    = htobe16(q->len);
        seg->seg_addr   = htobe64(physical_from_virtual(q->payload));
    }

    tx->tx_stats.cnt++;
    tx->tx_stats.bytes += p->tot_len;
    adapter->hw_stats.tx_packets++;
    adapter->hw_stats.tx_bytes += p->tot_len;
    write_barrier();
    pci_bar_write_4(&adapter->db_bar,
                    be32toh(tx->q_res->db_index) * sizeof(u32),
                    htobe32(tx->head));
    return ERR_OK;
}

/* gve_setup_linkoutput — called once at netif init to wire the correct
 * TX function directly into netif->linkoutput, eliminating the per-packet
 * format dispatch that existed in the old gve_linkoutput() wrapper. */
void gve_setup_linkoutput(gve adapter, struct netif *netif)
{
    if (adapter->dqo)
        netif->linkoutput = gve_linkoutput_dqo;
    else if (adapter->raw_addressing)
        netif->linkoutput = gve_linkoutput_rda;
    else
        netif->linkoutput = gve_linkoutput_qpl;
}

/* ------------------------------------------------------------------ */
/* RX path                                                              */
/* ------------------------------------------------------------------ */

void gve_rx_fill(gve_rx_queue rx)
{
    gve adapter = rx->adapter;
    int slot_count;
    gve_debug("RX fill: head %d, tail %d, avail %d",
              rx->head, rx->tail, rx->qpl_available);
    for (slot_count = 0;
         (rx->head < rx->tail + adapter->rx_desc_cnt) && rx->qpl_available;
         rx->head++, slot_count++, rx->qpl_available--) {
        u64 offset = rx->qpl_head * PAGESIZE;
        if (rx->pbufs[rx->qpl_head].ref > 1) {
            /* Buffer still held by lwIP: we cannot hand the full page
             * back to the device for a new packet.  If the ring is
             * getting thin (< 1/4 of descriptors free) stop filling
             * rather than risk stalling with no available slots.
             * Otherwise post the second half of the page as a fallback
             * buffer; gve_rx_service will detect the offset mismatch
             * (qpl_offset != qpl_index * PAGESIZE) and copy the packet
             * instead of zero-copying the pbuf. */
            if (rx->tail + adapter->rx_desc_cnt - rx->head <
                    (rx->mask + 1) / 4)
                break;
            offset += PAGESIZE / 2;
        }
        rx->data[rx->head & rx->mask] = htobe64(rx->rda_base_phys + offset);
        if (++rx->qpl_head == rx->qpl_count)
            rx->qpl_head = 0;
    }
    gve_debug("filled %d slots", slot_count);
    if (slot_count) {
        write_barrier();
        pci_bar_write_4(&adapter->db_bar,
                        be32toh(rx->q_res->db_index) * sizeof(u32),
                        htobe32(rx->head));
    }
}

closure_func_basic(thunk, void, gve_rx_service)
{
    gve_rx_queue rx = struct_from_field(closure_self(),
                                        gve_rx_queue, service);
    gve adapter = rx->adapter;
    struct netif *net_if = &adapter->ndev.n;
    boolean irq_acked = false;
    spin_lock(&rx->lock);
  begin:
    gve_debug("GQI RX service tail %d", rx->tail);

    for (int iter = 0; iter < GVE_CLEAN_BUDGET; iter++) {
        int budget = GVE_RX_BUDGET;
        u32 tail = be32toh(
            adapter->event_counters[be32toh(rx->q_res->counter_index)]);
        for (; rx->tail != tail && budget > 0;
             rx->qpl_available++, rx->tail++, budget--) {
            struct gve_rx_desc *desc = &rx->desc[rx->tail & rx->mask];
            u16 length = be16toh(desc->len);
            if (length <= GVE_RX_PADDING) {
                rx->rx_stats.rx_dropped++;
                continue;
            }
            if (desc->flags_seq & (GVE_RXF_ERR | GVE_RXF_PKT_CONT)) {
                msg_err("%s: unexpected flags 0x%x", func_ss,
                        desc->flags_seq);
                rx->rx_stats.rx_dropped++;
                continue;
            }
            /* qpl_head is stable during this loop (fill runs after).
             * qpl_available counts buffers returned so far this service
             * call, so qpl_head + qpl_available is the QPL page index
             * for the current completion. */
            u32 qpl_index = rx->qpl_head + rx->qpl_available;
            if (qpl_index >= rx->qpl_count)
                qpl_index -= rx->qpl_count;
            u64 qpl_offset = be64toh(rx->data[rx->tail & rx->mask]) -
                             rx->rda_base_phys;
            void *payload = rx->qpl_base + qpl_offset + GVE_RX_PADDING;
            length -= GVE_RX_PADDING;
            struct pbuf *p;
            if (qpl_offset == (u64)qpl_index * PAGESIZE) {
                p = &rx->pbufs[qpl_index];
                p->payload = payload;
                p->len = p->tot_len = length;
                pbuf_ref(p);
            } else {
                gve_debug("RX packet copy");
                p = pbuf_alloc(PBUF_RAW, length, PBUF_RAM);
                if (p) {
                    pbuf_take(p, payload, length);
                    rx->rx_stats.rx_copy++;
                } else {
                    msg_err("%s: failed to allocate pbuf", func_ss);
                    rx->rx_stats.rx_dropped++;
                    continue;
                }
            }
            err_t err = net_if->input(p, net_if);
            if (err != ERR_OK)
                pbuf_free(p);
            rx->rx_stats.cnt++;
            rx->rx_stats.bytes += length;
            adapter->hw_stats.rx_packets++;
            adapter->hw_stats.rx_bytes += length;
        }
        if (rx->head - rx->tail <= (rx->mask + 1) / 2)
            gve_rx_fill(rx);
        if (rx->tail == tail)
            break;  /* caught up before budget — no more work */
    }
    if (!irq_acked) {
        pci_bar_write_4(&adapter->db_bar,
                        be32toh(*rx->irq_db_index) * sizeof(u32),
                        GVE_IRQ_ACK);
        irq_acked = true;
        memory_barrier();
        goto begin;
    }
    spin_unlock(&rx->lock);
}

/* gve_rx_init: called from gve_adminq.c after the queue is allocated.
 * Initialises the service closure and the per-queue spinlock. */
void gve_rx_init(gve_rx_queue rx)
{
    init_closure_func(&rx->service, thunk, gve_rx_service);
    spin_lock_init(&rx->lock);
}
