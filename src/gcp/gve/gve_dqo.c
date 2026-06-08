/* gve_dqo.c — gVNIC DQO (Andromeda 2.x) TX and RX hot path
 *
 * DQO differences vs GQI:
 *   - All descriptors are little-endian (no htobe conversions).
 *     TX packet desc: 16 B; TX completion: 8 B; RX buffer: 32 B; RX compl: 32 B.
 *   - TX has a separate completion ring; driver polls generation bits
 *     rather than reading the event counter.
 *   - RX uses a buffer ring (driver posts) + completion ring (device fills).
 *   - DQO-RDA: buffers addressed directly by physical address.
 *     DQO-QPL: buffers come from a registered queue page list (the device
 *     requires registered pages on IOMMU-restricted VMs); the descriptors
 *     still carry physical addresses, so the payload is bounce-copied to/from
 *     the QPL.  Selected per adapter->dqo_qpl.
 *   - Multi-buffer packets are reassembled with pbuf_cat up to the
 *     end_of_packet completion (same as the GQI path).
 *   - Checksums are computed in software by lwIP (no HW offload), same
 *     model as the ENA driver.
 *
 * Nanos-specific simplifications:
 *   - Identity-mapped address space: physical_from_virtual() is trivial.
 *   - No scatter-gather within a segment: each pbuf segment is physically
 *     contiguous and maps to one descriptor.
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
    int budget = GVE_TX_CLEAN_BUDGET;
    while (budget-- > 0) {
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
                /* Clear per-slot timestamps for all descriptor slots retired
                 * by this completion (EOP slot at tag, working backwards). */
                u16_t seg_cnt = tx->seg_counts[tag];
                for (u16_t k = 0; k < seg_cnt; k++)
                    tx->tx_timestamps[(u16_t)(tag - k) & tx->mask] = 0;
                tx->desc_tail += seg_cnt;
                if (tx->adapter->dqo_qpl)
                    tx->qpl_used -= tx->qpl_allocated[tag];
                if (tx->pending[tag]) {
                    pbuf_free(tx->pending[tag]);
                    tx->pending[tag] = NULL;
                }
            }
        } else if (type == GVE_DQO_COMPL_TYPE_MISS) {
            u16_t tag = c->completion_tag & tx->mask;
            if (!tx->miss_times[tag]) {
                tx->miss_times[tag] = now(CLOCK_ID_MONOTONIC);
                tx->pending_misses++;
            }
        } else if (type == GVE_DQO_COMPL_TYPE_REINJECT) {
            u16_t tag = c->completion_tag & tx->mask;
            if (!tx->seg_counts[tag]) {
                msg_err("GVE: DQO TX invalid reinject tag %u", tag);
                tx->tx_stats.bad_compl_tag++;
                gve_trigger_reset(tx->adapter);
                break;
            }
            u16_t seg_cnt = tx->seg_counts[tag];
            for (u16_t k = 0; k < seg_cnt; k++)
                tx->tx_timestamps[(u16_t)(tag - k) & tx->mask] = 0;
            tx->desc_tail += seg_cnt;
            if (tx->adapter->dqo_qpl)
                tx->qpl_used -= tx->qpl_allocated[tag];
            if (tx->pending[tag]) {
                pbuf_free(tx->pending[tag]);
                tx->pending[tag] = NULL;
            }
            if (tx->miss_times[tag]) {
                tx->miss_times[tag] = 0;
                tx->pending_misses--;
            }
        }

        tx->compl_head++;
        /* Flip expected_gen each time compl_head wraps around the ring. */
        if (!(tx->compl_head & tx->mask))
            tx->expected_gen ^= 1;
    }
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
    boolean qpl = tx->adapter->dqo_qpl;
    int seg_count = 0;
    boolean oversize = false;
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        seg_count++;
        if (q->len > GVE_DQO_BUF_SIZE)
            oversize = true;
    }

    /* The device caps a packet at GVE_TX_MAX_DATA_DESCS data descriptors.  In
     * QPL mode each segment must also fit one GVE_DQO_BUF_SIZE bounce slot (so
     * a buffer never crosses a registered QPL page).  lwIP virtually never
     * produces such chains/segments for non-TSO traffic; drop rather than
     * wedge the queue (returning false would re-enqueue it forever). */
    if (seg_count > GVE_TX_MAX_DATA_DESCS || (qpl && oversize)) {
        pbuf_free(p);
        return true;   /* consumed (dropped) */
    }

    /* Every packet is one general context descriptor (dtype 0x4) followed by
     * seg_count packet descriptors — the context descriptor is mandatory
     * (matches the official Google driver). */
    u32 total_descs = 1 + seg_count;
    if (tx->head - tx->desc_tail + total_descs > (u32)(tx->mask + 1))
        return false;

    /* DQO-QPL: the payload is bounce-copied into the registered QPL, one
     * GVE_DQO_BUF_SIZE slot per segment.  Check there is room. */
    if (qpl && tx->qpl_used + (u32)seg_count * GVE_DQO_BUF_SIZE > tx->qpl_size)
        return false;

    /* One completion tag per packet, stamped on every packet descriptor: the
     * EOP slot is unique among outstanding packets, so we reuse it as the tag
     * (the device echoes a single tag in the packet completion). */
    u16_t eop_slot = (u16_t)((tx->head + seg_count) & tx->mask);

    /* General context descriptor: no metadata, so all flex fields stay zero. */
    {
        u32 slot = tx->head & tx->mask;
        struct gve_tx_ctx_desc_dqo *ctx =
            (struct gve_tx_ctx_desc_dqo *)&tx->desc[slot];
        zero(ctx, sizeof(*ctx));
        ctx->cmd_dtype = GVE_DQO_TX_DTYPE_CTX;
        tx->pending[slot]       = NULL;
        tx->seg_counts[slot]    = 0;
        tx->tx_timestamps[slot] = now(CLOCK_ID_MONOTONIC);
        tx->head++;
    }

    u32 qpl_total = 0;
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        u32 slot = tx->head & tx->mask;
        struct gve_tx_pkt_desc_dqo *desc = &tx->desc[slot];

        if (qpl) {
            /* Copy the segment into one GVE_DQO_BUF_SIZE-aligned slot of the
             * QPL ring; the descriptor carries the slot's physical address.
             * Slots are buffer-size aligned (2 per page) so a buffer never
             * crosses a registered QPL page — matching the Google driver. */
            u32 offset = tx->qpl_head;
            runtime_memcpy((u8 *)tx->qpl_base + offset, q->payload, q->len);
            tx->qpl_head += GVE_DQO_BUF_SIZE;
            if (tx->qpl_head >= tx->qpl_size)
                tx->qpl_head = 0;
            qpl_total += GVE_DQO_BUF_SIZE;
            desc->buf_addr = tx->qpl_base_phys + offset;
        } else {
            desc->buf_addr = physical_from_virtual(q->payload);
        }
        desc->dtype_flags = GVE_DQO_TX_DTYPE_PKT;
        desc->reserved0   = 0;
        desc->reserved1   = 0;
        desc->compl_tag   = eop_slot;
        desc->buf_size    = (u16_t)(q->len);

        if (q->next == NULL)
            desc->dtype_flags |= GVE_DQO_TX_EOP;

        tx->pending[slot]       = NULL;
        tx->seg_counts[slot]    = 0;
        tx->tx_timestamps[slot] = now(CLOCK_ID_MONOTONIC);
        tx->head++;
    }

    /* Request a DESC completion only every GVE_TX_MIN_RE_INTERVAL descriptors.
     * report_event generates a (type 0x4) descriptor completion; per-packet
     * packet completions are produced regardless, so we keep these sparse to
     * avoid flooding the completion ring (matches Google). */
    u32 last_desc_idx = (tx->head - 1) & tx->mask;
    if (((last_desc_idx - tx->last_re_idx) & tx->mask) >= GVE_TX_MIN_RE_INTERVAL) {
        ((struct gve_tx_pkt_desc_dqo *)&tx->desc[last_desc_idx])->dtype_flags
            |= GVE_DQO_TX_REPORT;
        tx->last_re_idx = last_desc_idx;
    }

    /* In QPL mode the payload has been copied, so the pbuf is freed by the
     * caller and nothing is pending; we only need to release the QPL bytes on
     * completion.  In RDA mode the descriptor references the pbuf directly, so
     * it stays pending until the completion. */
    if (qpl) {
        tx->qpl_allocated[eop_slot] = qpl_total;
        tx->qpl_used += qpl_total;
        tx->pending[eop_slot] = NULL;
        pbuf_free(p);          /* payload copied into the QPL; pbuf done */
    } else {
        tx->pending[eop_slot] = p;           /* ref from br-enqueue */
    }
    tx->seg_counts[eop_slot] = (u16_t)total_descs;
    return true;
}

/*
 * gve_tx_drain_dqo — drain the DQO software TX queue into the HW ring.
 * Must be called with tx->ring_mtx held (same ownership model as ENA).
 */
static void gve_tx_drain_dqo(gve_tx_dqo_queue tx)
{
    gve adapter = tx->adapter;

    if (adapter->flags & (1ULL << GVE_FLAG_ONGOING_RESET))
        return;

    gve_tx_dqo_cleanup(tx);

    if (!tx->running) {
        u32 free = (tx->mask + 1) - (tx->head - tx->desc_tail);
        if (free < GVE_TX_RESUME_THRESH)
            return;
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
                            tx->db_idx * sizeof(u32),
                            htobe32(tx->head));
            tx->tx_stats.doorbells++;
            pkts = 0;
        }
    }
    if (pkts > 0) {
        write_barrier();
        pci_bar_write_4(&adapter->db_bar,
                        tx->db_idx * sizeof(u32),
                        htobe32(tx->head));
        tx->tx_stats.doorbells++;
    }
}

closure_func_basic(thunk, void, gve_tx_enqueue_dqo)
{
    gve_tx_dqo_queue tx = struct_from_field(closure_self(),
                                            gve_tx_dqo_queue, enqueue_task);
    spin_lock(&tx->ring_mtx);
    gve_tx_drain_dqo(tx);
    spin_unlock(&tx->ring_mtx);
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
    boolean is_empty = queue_empty(tx->br);
    if (!enqueue(tx->br, p))
        return ERR_MEM;
    pbuf_ref(p);
    if (is_empty && spin_try(&tx->ring_mtx)) {
        gve_tx_drain_dqo(tx);
        spin_unlock(&tx->ring_mtx);
    } else {
        async_apply_bh((thunk)&tx->enqueue_task);
    }
    return ERR_OK;
}

/* ------------------------------------------------------------------ */
/* DQO RX path                                                          */
/* ------------------------------------------------------------------ */

/*
 * gve_rx_dqo_fill — allocate per-packet pbufs and post buffer descriptors.
 *
 * Each slot gets a freshly allocated PBUF_RAM buffer.  The buf_id comes
 * from the free_ids circular list (same pattern as ENA free_rx_ids[]).
 * Physical address is derived per-packet via physical_from_virtual(), which
 * is trivial on Nanos (identity-mapped address space).
 */
void gve_rx_dqo_fill(gve_rx_dqo_queue rx)
{
    gve adapter = rx->adapter;
    u32 posted = 0;

    while (((rx->buf_head + 1) & rx->mask) !=
           (rx->compl_head & rx->mask)) {
        /* No free IDs — should not happen in normal operation since IDs
         * are returned before net_if->input() in the completion path. */
        if (rx->next_to_clean == rx->next_to_use)
            break;

        u16 buf_id = rx->free_ids[rx->next_to_use & rx->mask];
        u64 buf_addr;
        if (adapter->dqo_qpl) {
            /* QPL: the buffer is a fixed slot in the registered region; the
             * descriptor carries its physical address (no per-buffer pbuf). */
            buf_addr = rx->qpl_base_phys + (u64)buf_id * GVE_DQO_BUF_SIZE;
        } else {
            struct pbuf *pb = pbuf_alloc(PBUF_RAW, GVE_DQO_BUF_SIZE, PBUF_RAM);
            if (!pb) {
                rx->rx_stats.refil_partial++;
                break;
            }
            rx->pbufs[buf_id] = pb;
            buf_addr = physical_from_virtual(pb->payload);
        }
        rx->next_to_use++;

        struct gve_rx_buf_desc_dqo *bd = &rx->buf_ring[rx->buf_head & rx->mask];
        bd->buf_id          = buf_id;
        bd->reserved0       = 0;
        bd->reserved1       = 0;
        bd->buf_addr        = buf_addr;
        bd->header_buf_addr = 0;
        bd->reserved2       = 0;

        rx->buf_head++;
        posted++;
    }

    if (posted) {
        write_barrier();
        pci_bar_write_4(&adapter->db_bar,
                        rx->db_idx * sizeof(u32),
                        htobe32(rx->buf_head));
        rx->empty_rx_queue = 0;
    } else if (((rx->buf_head + 1) & rx->mask) !=
               (rx->compl_head & rx->mask)) {
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
  begin:
    if (!(net_if->flags & NETIF_FLAG_UP))
        return;
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

            u16_t buf_id   = c->buf_id & rx->mask;
            boolean eop    = !!(c->status0 & GVE_DQO_RX_EOP);
            boolean rx_err = !!(c->err_flags & GVE_DQO_RX_ERR);
            /* DQO has no leading IP-alignment pad: packet_len is this buffer's
             * data length, data at offset 0 (verified against Google). */
            u16_t pkt_len  = c->pkt_len_gen & GVE_DQO_RX_PKT_LEN_MASK;
            struct pbuf *inp;

            if (adapter->dqo_qpl) {
                /* QPL: data is in the registered page; copy it out into a fresh
                 * pbuf and recycle the buffer id immediately. */
                rx->free_ids[rx->next_to_clean & rx->mask] = buf_id;
                rx->next_to_clean++;
                if (rx_err || pkt_len == 0) {
                    if (rx_err)
                        rx->rx_stats.rx_dropped++;
                    if (rx->ctx_head) {
                        pbuf_free(rx->ctx_head);
                        rx->ctx_head = NULL;
                    }
                    goto advance;
                }
                inp = pbuf_alloc(PBUF_RAW, pkt_len, PBUF_RAM);
                if (!inp) {
                    rx->rx_stats.rx_dropped++;
                    if (rx->ctx_head) {
                        pbuf_free(rx->ctx_head);
                        rx->ctx_head = NULL;
                    }
                    goto advance;
                }
                pbuf_take(inp,
                          (u8 *)rx->qpl_base + (u64)buf_id * GVE_DQO_BUF_SIZE,
                          pkt_len);
                rx->rx_stats.rx_copy++;
            } else {
                /* RDA: the posted pbuf is the data buffer. */
                inp = rx->pbufs[buf_id];
                /* Guard against spurious completions (no pbuf was posted). */
                if (!inp) {
                    rx->rx_stats.bad_req_id++;
                    goto advance;
                }
                /* Claim the slot and return the ID before net_if->input(),
                 * same pattern as ENA (rx_info->mbuf = NULL before input). */
                rx->pbufs[buf_id] = NULL;
                rx->free_ids[rx->next_to_clean & rx->mask] = buf_id;
                rx->next_to_clean++;
                if (rx_err) {
                    gve_debug("DQO RX: rx_error 0x%x, dropping", c->err_flags);
                    rx->rx_stats.rx_dropped++;
                    pbuf_free(inp);
                    if (rx->ctx_head) {
                        pbuf_free(rx->ctx_head);
                        rx->ctx_head = NULL;
                    }
                    goto advance;
                }
                inp->len = inp->tot_len = pkt_len;
            }

            /* Accumulate the buffers of a multi-buffer packet (same pbuf_cat
             * loop as ena_rx_mbuf); deliver the chain on end_of_packet. */
            if (rx->ctx_head == NULL)
                rx->ctx_head = inp;
            else
                pbuf_cat(rx->ctx_head, inp);

            if (eop) {
                struct pbuf *pkt = rx->ctx_head;
                rx->ctx_head = NULL;
                u16_t tot = pkt->tot_len;
                if (tot == 0) {
                    pbuf_free(pkt);
                } else {
                    pkt->napi_id = net_get_napi_id(net_if->num, rx->idx);
                    err_t err = net_if->input(pkt, net_if);
                    if (err != ERR_OK)
                        pbuf_free(pkt);
                    rx->rx_stats.cnt++;
                    rx->rx_stats.bytes += tot;
                    adapter->hw_stats.rx_packets++;
                    adapter->hw_stats.rx_bytes += tot;
                }
            }

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
        /* DQO re-arms the interrupt with ACK|EVENT (matches the official
         * Google driver's NAPI path); GQI uses ACK alone (gve_rx_service). */
        pci_bar_write_4(&adapter->db_bar,
                        be32toh(*rx->irq_db_index) * sizeof(u32),
                        GVE_IRQ_ACK | GVE_IRQ_EVENT);
        irq_acked = true;
        memory_barrier();
        goto begin;
    }
}

void gve_rx_dqo_init(gve_rx_dqo_queue rx)
{
    init_closure_func(&rx->service, thunk, gve_rx_dqo_service);
}

/* gve_tx_init_dqo: called from gve_adminq.c after a DQO TX queue is
 * allocated.  Initialises the enqueue_task closure and ring spinlock. */
void gve_tx_init_dqo(gve_tx_dqo_queue tx)
{
    init_closure_func(&tx->enqueue_task, thunk, gve_tx_enqueue_dqo);
    spin_lock_init(&tx->ring_mtx);
}
