/* gve_dqo.c — gVNIC DQO (Andromeda 2.x) TX and RX hot path
 *
 * DQO differences vs GQI:
 *   - All descriptors are little-endian (no htobe conversions).
 *     TX packet desc: 16 B; TX completion: 8 B; RX buffer: 32 B; RX compl: 32 B.
 *   - TX has a separate completion ring; driver polls generation bits
 *     rather than reading the event counter.
 *   - RX uses a buffer ring (driver posts) + completion ring (device fills).
 *   - Always raw addressing (no QPL); physical_from_virtual() for all bufs.
 *   - TX checksum offload: set checksum_offload_enable in dtype_flags and
 *     seed pseudo-header in the L4 checksum field (same CHECKSUM_PARTIAL
 *     model as GQI, but via the DQO descriptor flag instead of GVE_TXF_L4CSUM).
 *
 * Nanos-specific simplifications:
 *   - Identity-mapped address space: physical_from_virtual() is trivial.
 *   - No scatter-gather: pbuf payload is physically contiguous per segment.
 *   - Single-process unikernel: no per-CPU queue affinity complexity.
 */

#include "gve_priv.h"

/* ------------------------------------------------------------------ */
/* DQO TX path                                                          */
/* ------------------------------------------------------------------ */

/*
 * gve_tx_dqo_cleanup — retire DQO TX completions by polling the
 * generation bit in the completion ring.
 *
 * When a completion is valid (generation matches expected_gen), the
 * compl_tag identifies the descriptor slot whose pbuf can be freed.
 * After consuming all entries in one ring pass the expected_gen flips.
 */
static void gve_tx_dqo_cleanup(gve_tx_dqo_queue tx)
{
    while (1) {
        u32 slot = tx->compl_head & tx->mask;
        struct gve_tx_compl_desc_dqo *c = &tx->compl[slot];

        /* id_type_gen (bytes 0-1): id[10:0] | type[13:11] | reserved[14] | gen[15]. */
        u16_t itg = c->id_type_gen;
        u8    gen = !!(itg & GVE_DQO_COMPL_GEN_BIT);  /* bit 15 */
        if (gen != tx->expected_gen)
            break;

        u8 type = (u8)((itg >> GVE_DQO_COMPL_TYPE_SHIFT) & 0x7);  /* bits[13:11] */
        if (type == GVE_DQO_COMPL_TYPE_PKT) {
            u16_t tag = c->completion_tag & tx->mask;
            tx->desc_tail += tx->seg_counts[tag];
            if (tx->pending[tag]) {
                pbuf_free(tx->pending[tag]);
                tx->pending[tag] = NULL;
            }
            tx->last_completion = now(CLOCK_ID_MONOTONIC);
        }
        /* Other completion types (miss, re-injection): no action needed. */

        tx->compl_head++;
        /* Flip expected_gen each time compl_head wraps around the ring. */
        if (!(tx->compl_head & tx->mask))
            tx->expected_gen ^= 1;
    }
}

/* gve_tx_dqo_fill_csum — DQO wrapper around gve_pseudo_csum (gve_priv.h).
 * The pseudo-header is seeded in the pbuf by the helper; DQO only needs
 * the checksum_offload_enable bit set in the descriptor. */
static void gve_tx_dqo_fill_csum(struct pbuf *p,
                                  struct gve_tx_pkt_desc_dqo *desc)
{
    u8 proto; u16_t l4_off;
    if (!gve_pseudo_csum(p, &proto, &l4_off))
        return;
    desc->dtype_flags |= GVE_DQO_TX_CSUM_EN;
}

err_t gve_linkoutput_dqo(struct netif *netif, struct pbuf *p)
{
    gve adapter = netif->state;
    u32 qidx = current_cpu()->id % adapter->num_queues;
    gve_tx_dqo_queue tx = &adapter->tx_dqo[qidx];

    if (tx->stuck)
        return ERR_IF;

    gve_tx_dqo_cleanup(tx);

    /* Count segments and check for descriptor space.
     * Space is tracked via desc_tail (descriptor slots retired), not
     * compl_head (completions consumed), so multi-segment packets are
     * accounted correctly. */
    int seg_count = 0;
    for (struct pbuf *q = p; q != NULL; q = q->next)
        seg_count++;
    if (tx->head - tx->desc_tail + seg_count > (u32)(tx->mask + 1))
        return ERR_MEM;

    /* Reference the chain head; released in gve_tx_dqo_cleanup via
     * the EOP slot's compl_tag. */
    pbuf_ref(p);
    u32 last_slot = 0;

    for (struct pbuf *q = p; q != NULL; q = q->next) {
        u32 slot = tx->head & tx->mask;
        struct gve_tx_pkt_desc_dqo *desc = &tx->desc[slot];

        desc->buf_addr    = physical_from_virtual(q->payload);
        desc->dtype_flags = GVE_DQO_TX_DTYPE_PKT;
        desc->reserved0   = 0;
        desc->reserved1   = 0;
        desc->compl_tag   = (u16_t)slot;
        desc->buf_size    = (u16_t)(q->len);

        if (q->next == NULL) {
            desc->dtype_flags |= GVE_DQO_TX_EOP | GVE_DQO_TX_REPORT;
            gve_tx_dqo_fill_csum(p, desc);
            last_slot = slot;
        }

        tx->pending[slot]    = NULL;  /* only EOP slot gets the pbuf */
        tx->seg_counts[slot] = 0;     /* only EOP slot stores seg_count */
        tx->head++;
    }
    /* EOP slot carries the pbuf chain and the segment count for cleanup. */
    tx->pending[last_slot]    = p;
    tx->seg_counts[last_slot] = (u16_t)seg_count;

    write_barrier();
    pci_bar_write_4(&adapter->db_bar,
                    be32toh(tx->q_res->db_index) * sizeof(u32),
                    htobe32(tx->head));
    return ERR_OK;
}

/* ------------------------------------------------------------------ */
/* DQO RX path                                                          */
/* ------------------------------------------------------------------ */

/*
 * gve_rx_dqo_fill — post available buffer descriptors to the device.
 * Each entry in buf_ring[] gives the device a physical buffer address
 * identified by buf_id (= ring slot index).
 */
void gve_rx_dqo_fill(gve_rx_dqo_queue rx)
{
    gve adapter = rx->adapter;
    u32 posted = 0;

    /* Post until the buffer ring is full or we run out of free slots.
     * A slot is "free" when the pbuf ref count is back to 1
     * (lwIP has released its reference). */
    while (((rx->buf_head + 1) & rx->mask) !=
           (rx->compl_head & rx->mask)) {
        u32 buf_id = rx->buf_head & rx->mask;
        if (rx->pbufs[buf_id].ref > 1)
            break;  /* buffer still owned by lwIP */

        struct gve_rx_buf_desc_dqo *bd = &rx->buf_ring[buf_id];
        bd->buf_id          = (u16_t)buf_id;
        bd->reserved0       = 0;
        bd->reserved1       = 0;
        bd->buf_addr        = rx->rda_base_phys +
                              (u64)buf_id * GVE_DQO_BUF_SIZE;
        bd->header_buf_addr = 0;
        bd->reserved2       = 0;

        rx->buf_head++;
        posted++;
    }

    if (posted) {
        write_barrier();
        pci_bar_write_4(&adapter->db_bar,
                        be32toh(rx->q_res->db_index) * sizeof(u32),
                        htobe32(rx->buf_head));
    }
}

closure_func_basic(thunk, void, gve_rx_dqo_service)
{
    gve_rx_dqo_queue rx = struct_from_field(closure_self(),
                                            gve_rx_dqo_queue, service);
    gve adapter = rx->adapter;
    struct netif *net_if = &adapter->ndev.n;
    boolean irq_acked = false;
    spin_lock(&rx->lock);
  begin:
    gve_debug("DQO RX service compl_head %d", rx->compl_head);

    while (1) {
        u32 slot = rx->compl_head & rx->mask;
        struct gve_rx_compl_desc_dqo *c = &rx->compl_ring[slot];
        /* generation is bit 14 of pkt_len_gen (bytes 4-5). */
        u8 gen = !!(c->pkt_len_gen & GVE_DQO_RX_GEN);
        if (gen != rx->expected_gen)
            break;

        if (c->err_flags & GVE_DQO_RX_ERR) {
            gve_debug("DQO RX: rx_error 0x%x, dropping", c->err_flags);
            goto next;
        }

        u16_t pkt_len = c->pkt_len_gen & GVE_DQO_RX_PKT_LEN_MASK;
        u16_t buf_id  = c->buf_id & rx->mask;

        if (pkt_len <= GVE_RX_PADDING)
            goto next;
        u16_t data_len = pkt_len - GVE_RX_PADDING;

        void *virt_buf = (u8 *)rx->qpl_base +
                         (u64)buf_id * GVE_DQO_BUF_SIZE;
        void *payload  = (u8 *)virt_buf + GVE_RX_PADDING;

        struct pbuf *pb = &rx->pbufs[buf_id];
        struct pbuf *inp;
        if (pb->ref == 1) {
            /* Zero-copy: hand the pre-allocated PBUF_REF to lwIP. */
            pb->payload   = payload;
            pb->len       = pb->tot_len = data_len;
            pbuf_ref(pb);
            inp = pb;
        } else {
            /* Buffer still held by lwIP; fall back to copy. */
            gve_debug("DQO RX: pbuf copy (ref %d)", pb->ref);
            inp = pbuf_alloc(PBUF_RAW, data_len, PBUF_RAM);
            if (inp) {
                pbuf_take(inp, payload, data_len);
            } else {
                msg_err("%s: pbuf_alloc failed", func_ss);
                goto next;
            }
        }

        err_t err = net_if->input(inp, net_if);
        if (err != ERR_OK)
            pbuf_free(inp);

      next:
        rx->compl_head++;
        if (!(rx->compl_head & rx->mask))
            rx->expected_gen ^= 1;
    }

    gve_rx_dqo_fill(rx);

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

void gve_rx_dqo_init(gve_rx_dqo_queue rx)
{
    init_closure_func(&rx->service, thunk, gve_rx_dqo_service);
    spin_lock_init(&rx->lock);
}
