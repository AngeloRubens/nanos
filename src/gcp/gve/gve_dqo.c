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
            /* Alternate-miss encoding: type=PKT but bit 15 of completion_tag set. */
            if (c->completion_tag & GVE_DQO_ALT_MISS_COMPL_BIT) {
                u16_t tag = c->completion_tag & tx->mask;
                if (!tx->miss_times[tag]) {
                    tx->miss_times[tag] = now(CLOCK_ID_MONOTONIC);
                    tx->pending_misses++;
                }
            } else {
                u16_t tag = c->completion_tag & tx->mask;
                if (!tx->seg_counts[tag]) {
                    msg_err("GVE: DQO TX invalid completion tag %u", tag);
                    tx->tx_stats.bad_compl_tag++;
                    gve_trigger_reset(tx->adapter);
                    break;
                }
                tx->desc_tail += tx->seg_counts[tag];
                if (tx->pending[tag]) {
                    pbuf_free(tx->pending[tag]);
                    tx->pending[tag] = NULL;
                }
                tx->last_completion = now(CLOCK_ID_MONOTONIC);
            }
        } else if (type == GVE_DQO_COMPL_TYPE_MISS) {
            u16_t tag = c->completion_tag & tx->mask;
            if (!tx->miss_times[tag]) {
                tx->miss_times[tag] = now(CLOCK_ID_MONOTONIC);
                tx->pending_misses++;
            }
        } else if (type == GVE_DQO_COMPL_TYPE_REINJECT) {
            u16_t tag = c->completion_tag & tx->mask;
            tx->desc_tail += tx->seg_counts[tag];
            if (tx->pending[tag]) {
                pbuf_free(tx->pending[tag]);
                tx->pending[tag] = NULL;
            }
            if (tx->miss_times[tag]) {
                tx->miss_times[tag] = 0;
                tx->pending_misses--;
            }
            tx->last_completion = now(CLOCK_ID_MONOTONIC);
        }

        tx->compl_head++;
        /* Flip expected_gen each time compl_head wraps around the ring. */
        if (!(tx->compl_head & tx->mask))
            tx->expected_gen ^= 1;
    }

    /* Per-packet watchdog: a miss completion that never gets a matching
     * reinject within GVE_TX_WATCHDOG_MS indicates a device stall.
     * The slot's pbuf is freed here to avoid a leak on reset. */
    if (tx->pending_misses) {
        timestamp now_ts  = now(CLOCK_ID_MONOTONIC);
        timestamp deadline = milliseconds(GVE_TX_WATCHDOG_MS);
        for (u32 i = 0; i <= tx->mask; i++) {
            if (!tx->miss_times[i])
                continue;
            if (now_ts - tx->miss_times[i] <= deadline)
                continue;
            msg_err("GVE: DQO TX slot %d: miss not reinjected after %d ms, "
                    "scheduling reset", i, GVE_TX_WATCHDOG_MS);
            if (tx->pending[i]) {
                pbuf_free(tx->pending[i]);
                tx->pending[i] = NULL;
            }
            tx->miss_times[i] = 0;
            tx->pending_misses--;
            tx->stuck = true;
            gve_trigger_reset(tx->adapter);
            break;
        }
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

/*
 * gve_tx_write_dqo — post one pbuf chain as DQO TX descriptors.
 * Must be called with tx->ring_mtx held.
 * p->ref has already been bumped by the br-enqueue caller; that ref
 * transfers to pending[eop_slot].
 * Returns true on success; false if descriptor space exhausted.
 */
static boolean gve_tx_write_dqo(gve_tx_dqo_queue tx, struct pbuf *p)
{
    int seg_count = 0;
    for (struct pbuf *q = p; q != NULL; q = q->next)
        seg_count++;
    if (tx->head - tx->desc_tail + seg_count > (u32)(tx->mask + 1))
        return false;

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

        tx->pending[slot]    = NULL;
        tx->seg_counts[slot] = 0;
        tx->head++;
    }
    tx->pending[last_slot]    = p;    /* ref from br-enqueue */
    tx->seg_counts[last_slot] = (u16_t)seg_count;
    return true;
}

/*
 * gve_tx_start_xmit_dqo — drain the software TX queue into the DQO HW ring.
 * Same stop/wakeup and doorbell-batching logic as gve_tx_start_xmit_gqi.
 */
static void gve_tx_start_xmit_dqo(gve_tx_dqo_queue tx)
{
    gve adapter = tx->adapter;
    spin_lock(&tx->ring_mtx);

    if (adapter->flags & (1ULL << GVE_FLAG_ONGOING_RESET)) {
        spin_unlock(&tx->ring_mtx);
        return;
    }

    gve_tx_dqo_cleanup(tx);

    if (!tx->running) {
        u32 free = (tx->mask + 1) - (tx->head - tx->desc_tail);
        if (free < GVE_TX_RESUME_THRESH) {
            spin_unlock(&tx->ring_mtx);
            return;
        }
        tx->running = true;
        tx->tx_stats.queue_wakeup++;
        memory_barrier();
    }

    u32 pkts = 0;
    struct pbuf *p;
    while ((p = dequeue(tx->br)) != INVALID_ADDRESS) {
        u16_t tot_len = p->tot_len;
        if (!gve_tx_write_dqo(tx, p)) {
            enqueue(tx->br, p);
            tx->running = false;
            tx->tx_stats.queue_stop++;
            memory_barrier();
            break;
        }

        tx->tx_stats.cnt++;
        tx->tx_stats.bytes += tot_len;
        adapter->hw_stats.tx_packets++;
        adapter->hw_stats.tx_bytes += tot_len;

        if (++pkts >= GVE_TX_DOORBELL_BATCH) {
            write_barrier();
            pci_bar_write_4(&adapter->db_bar,
                            be32toh(tx->q_res->db_index) * sizeof(u32),
                            htobe32(tx->head));
            tx->tx_stats.doorbells++;
            pkts = 0;
        }
    }
    if (pkts > 0) {
        write_barrier();
        pci_bar_write_4(&adapter->db_bar,
                        be32toh(tx->q_res->db_index) * sizeof(u32),
                        htobe32(tx->head));
        tx->tx_stats.doorbells++;
    }
    tx->acum_pkts = 0;
    spin_unlock(&tx->ring_mtx);
}

closure_func_basic(thunk, void, gve_tx_enqueue_dqo)
{
    gve_tx_dqo_queue tx = struct_from_field(closure_self(),
                                            gve_tx_dqo_queue, enqueue_task);
    gve_tx_start_xmit_dqo(tx);
}

err_t gve_linkoutput_dqo(struct netif *netif, struct pbuf *p)
{
    gve adapter = netif->state;
    if (!(netif->flags & NETIF_FLAG_UP))
        return ERR_IF;
    u32 qidx = current_cpu()->id % adapter->num_queues;
    gve_tx_dqo_queue tx = &adapter->tx_dqo[qidx];
    if (tx->stuck)
        return ERR_IF;
    pbuf_ref(p);
    if (!enqueue(tx->br, p)) {
        pbuf_free(p);
        return ERR_MEM;
    }
    async_apply_bh((thunk)&tx->enqueue_task);
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
        rx->empty_rx_queue = 0;
    } else if (((rx->buf_head + 1) & rx->mask) !=
               (rx->compl_head & rx->mask)) {
        /* Ring not full but couldn't post: lwIP holds all buffers.
         * Watchdog uses this counter to detect the deadlock. */
        rx->empty_rx_queue++;
    }
}

closure_func_basic(thunk, void, gve_rx_dqo_service)
{
    gve_rx_dqo_queue rx = struct_from_field(closure_self(),
                                            gve_rx_dqo_queue, service);
    gve adapter = rx->adapter;
    struct netif *net_if = &adapter->ndev.n;
    if (!(net_if->flags & NETIF_FLAG_UP))
        return;
    boolean irq_acked = false;
    spin_lock(&rx->lock);
  begin:
    gve_debug("DQO RX service compl_head %d", rx->compl_head);

    for (int iter = 0; iter < GVE_CLEAN_BUDGET; iter++) {
        boolean no_more = false;
        int budget = GVE_RX_BUDGET;
        for (; budget > 0; budget--) {
            u32 slot = rx->compl_head & rx->mask;
            struct gve_rx_compl_desc_dqo *c = &rx->compl_ring[slot];
            u8 gen = !!(c->pkt_len_gen & GVE_DQO_RX_GEN);
            if (gen != rx->expected_gen) {
                no_more = true;
                break;
            }

            if (c->err_flags & GVE_DQO_RX_ERR) {
                gve_debug("DQO RX: rx_error 0x%x, dropping", c->err_flags);
                rx->rx_stats.rx_dropped++;
                goto advance;
            }

            u16_t pkt_len = c->pkt_len_gen & GVE_DQO_RX_PKT_LEN_MASK;
            u16_t buf_id  = c->buf_id & rx->mask;

            if (pkt_len <= GVE_RX_PADDING)
                goto advance;
            u16_t data_len = pkt_len - GVE_RX_PADDING;

            void *virt_buf = (u8 *)rx->qpl_base +
                             (u64)buf_id * GVE_DQO_BUF_SIZE;
            void *payload  = (u8 *)virt_buf + GVE_RX_PADDING;

            struct pbuf *pb = &rx->pbufs[buf_id];
            struct pbuf *inp;
            if (pb->ref == 1) {
                pb->payload   = payload;
                pb->len       = pb->tot_len = data_len;
                pbuf_ref(pb);
                inp = pb;
            } else {
                gve_debug("DQO RX: pbuf copy (ref %d)", pb->ref);
                inp = pbuf_alloc(PBUF_RAW, data_len, PBUF_RAM);
                if (inp) {
                    pbuf_take(inp, payload, data_len);
                    rx->rx_stats.rx_copy++;
                } else {
                    msg_err("%s: pbuf_alloc failed", func_ss);
                    rx->rx_stats.rx_dropped++;
                    goto advance;
                }
            }

            {
                err_t err = net_if->input(inp, net_if);
                if (err != ERR_OK)
                    pbuf_free(inp);
            }

            rx->rx_stats.cnt++;
            rx->rx_stats.bytes += data_len;
            adapter->hw_stats.rx_packets++;
            adapter->hw_stats.rx_bytes += data_len;

          advance:
            rx->compl_head++;
            if (!(rx->compl_head & rx->mask))
                rx->expected_gen ^= 1;
        }
        /* Post new buffer descriptors after each budget pass so the device
         * always has free buffers — avoids RX starvation under high load. */
        gve_rx_dqo_fill(rx);
        if (no_more)
            break;
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

void gve_rx_dqo_init(gve_rx_dqo_queue rx)
{
    init_closure_func(&rx->service, thunk, gve_rx_dqo_service);
    spin_lock_init(&rx->lock);
}

/* gve_tx_init_dqo: called from gve_adminq.c after a DQO TX queue is
 * allocated.  Initialises the enqueue_task closure and ring spinlock. */
void gve_tx_init_dqo(gve_tx_dqo_queue tx)
{
    init_closure_func(&tx->enqueue_task, thunk, gve_tx_enqueue_dqo);
    spin_lock_init(&tx->ring_mtx);
}
