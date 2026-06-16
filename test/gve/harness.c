/* gve DQO datapath host harness.
 *
 * Includes the real src/gcp/gve/gve_dqo.c into this translation unit (its
 * datapath functions are static), provides host implementations of the
 * device/kernel glue declared by the shims, models a DQO device, and drives
 * the driver through scripted completion sequences with invariant checks.
 *
 * Built and run via test/gve/Makefile.  Uses only nanos runtime APIs
 * (rprintf, the process heap) to avoid clashing with libc headers.
 */

#include "gve_dqo.c"

/* ------------------------------------------------------------------ */
/* Test harness state and assertions                                    */
/* ------------------------------------------------------------------ */

static heap gh;                 /* global process heap */
static int  failures;
static int  checks;

#define CHECK(cond, ...) do {                                           \
    checks++;                                                           \
    if (!(cond)) {                                                      \
        failures++;                                                     \
        rprintf("  FAIL %s:%d: ", func_ss, __LINE__);                   \
        rprintf(__VA_ARGS__);                                          \
        rprintf("\n");                                                  \
    }                                                                   \
} while (0)

/* ------------------------------------------------------------------ */
/* Kernel/heap glue                                                     */
/* ------------------------------------------------------------------ */

heap heap_locked(kernel_heaps kh)        { return gh; }
heap heap_linear_backed(kernel_heaps kh) { return gh; }

int total_processors = 1;
static struct harness_cpu the_cpu = { .id = 0 };
cpuinfo current_cpu(void) { return &the_cpu; }

/* Deferred work: run synchronously (single-threaded harness). */
void async_apply(thunk t)    { apply(t); }
void async_apply_bh(thunk t) { apply(t); }

/* ------------------------------------------------------------------ */
/* pbuf model (real refcount so ownership transfers are observable)     */
/* ------------------------------------------------------------------ */

static int pbufs_live;          /* outstanding pbufs (alloc - free-to-zero) */
static int pbuf_alloc_fail_after = -1;  /* >=0: fail once the counter hits 0 */

struct pbuf *pbuf_alloc(int layer, u16 length, int type)
{
    if (pbuf_alloc_fail_after == 0)
        return 0;
    if (pbuf_alloc_fail_after > 0)
        pbuf_alloc_fail_after--;
    struct pbuf *p = allocate(gh, sizeof(*p));
    if (p == INVALID_ADDRESS)
        return 0;
    p->payload = (length ? allocate(gh, length) : 0);
    if (length && p->payload == INVALID_ADDRESS) {
        deallocate(gh, p, sizeof(*p));
        return 0;
    }
    p->next = 0;
    p->len = p->tot_len = length;
    p->ref = 1;
    p->type_internal = type;
    p->flags = 0;
    p->if_idx = NETIF_NO_INDEX;
    p->napi_id = 0;
    pbufs_live++;
    return p;
}

void pbuf_ref(struct pbuf *p) { p->ref++; }

u8 pbuf_free(struct pbuf *p)
{
    u8 freed = 0;
    while (p) {
        struct pbuf *next = p->next;
        if (--p->ref == 0) {
            if (p->payload)
                deallocate(gh, p->payload, p->len ? p->len : 1);
            deallocate(gh, p, sizeof(*p));
            pbufs_live--;
            freed++;
            p = next;          /* chain: free the rest too */
        } else {
            break;             /* still referenced: stop */
        }
    }
    return freed;
}

void pbuf_cat(struct pbuf *head, struct pbuf *tail)
{
    struct pbuf *p = head;
    while (p->next)
        p = p->next;
    p->next = tail;
    for (p = head; p; p = p->next)
        p->tot_len = head->tot_len;   /* approx; tot_len recomputed below */
    /* recompute tot_len as sum of lens */
    u16 tot = 0;
    for (p = head; p; p = p->next)
        tot += p->len;
    for (p = head; p; p = p->next)
        tot -= 0;
    head->tot_len = tot;
}

err_t pbuf_take(struct pbuf *buf, const void *dataptr, u16 len)
{
    runtime_memcpy(buf->payload, dataptr, len);
    return ERR_OK;
}

/* ------------------------------------------------------------------ */
/* netif glue                                                           */
/* ------------------------------------------------------------------ */

void netif_set_link_up(struct netif *n)   { n->flags |= NETIF_FLAG_UP; }
void netif_set_link_down(struct netif *n) { n->flags &= ~NETIF_FLAG_UP; }
u16  net_get_napi_id(u8 netif_num, u16 queue_idx) { return queue_idx + 1; }

static int rx_delivered;        /* packets handed to net_if->input */
static err_t harness_input(struct pbuf *p, struct netif *inp)
{
    rx_delivered++;
    pbuf_free(p);               /* the stack consumes it */
    return ERR_OK;
}
err_t ethernet_input(struct pbuf *p, struct netif *netif) { return harness_input(p, netif); }

/* ------------------------------------------------------------------ */
/* DQO device model                                                     */
/* ------------------------------------------------------------------ */

typedef struct dqo_device {
    /* TX completion ring the driver polls (== tx->compl). */
    struct gve_tx_compl_desc_dqo *tx_compl;
    u32 tx_compl_mask;
    u32 tx_compl_tail;          /* device write cursor */
    u8  tx_compl_gen;           /* device generation bit (starts 1) */
    u32 last_tx_doorbell;       /* last TX head the driver rang */
} *dqo_device;

static struct dqo_device the_dev;

/* The doorbell BAR write lands here; we record the TX head. */
u32 pci_bar_read_4(struct pci_bar *b, u64 offset) { return 0; }
void pci_bar_write_4(struct pci_bar *b, u64 offset, u32 val)
{
    the_dev.last_tx_doorbell = val;
}

/* Device posts one PKT completion for `tag` into the TX completion ring. */
static void dqo_dev_complete_pkt(dqo_device dev, u16 tag, u8 type)
{
    u32 slot = dev->tx_compl_tail & dev->tx_compl_mask;
    struct gve_tx_compl_desc_dqo *c = &dev->tx_compl[slot];
    u16 itg = (u16)(((u16)type & 0x7) << GVE_DQO_COMPL_TYPE_SHIFT);
    if (dev->tx_compl_gen)
        itg |= GVE_DQO_COMPL_GEN_BIT;
    c->completion_tag = tag;
    write_barrier();
    c->id_type_gen = itg;       /* gen last: announce after fields written */
    dev->tx_compl_tail++;
    if ((dev->tx_compl_tail & dev->tx_compl_mask) == 0)
        dev->tx_compl_gen ^= 1; /* flip on wrap, like real HW */
}

/* ------------------------------------------------------------------ */
/* Build a DQO TX queue (mirrors gve_create_tx_queue_dqo init, RDA mode) */
/* ------------------------------------------------------------------ */

static void make_tx_dqo_mode(gve adapter, gve_tx_dqo_queue tx, u16 desc_cnt,
                             boolean qpl)
{
    tx->desc       = allocate_zero(gh, desc_cnt * sizeof(*tx->desc));
    tx->compl      = allocate_zero(gh, desc_cnt * sizeof(*tx->compl));
    tx->pending    = allocate_zero(gh, desc_cnt * sizeof(*tx->pending));
    tx->seg_counts = allocate_zero(gh, desc_cnt * sizeof(*tx->seg_counts));
    tx->miss_times = allocate_zero(gh, desc_cnt * sizeof(*tx->miss_times));
    tx->tx_timestamps = allocate_zero(gh, desc_cnt * sizeof(*tx->tx_timestamps));
    tx->free_tags  = allocate(gh, desc_cnt * sizeof(*tx->free_tags));
    tx->br         = allocate_queue(gh, GVE_BUF_RING_SIZE);

    tx->mask = desc_cnt - 1;
    tx->head = 0;
    tx->desc_tail = 0;
    tx->compl_head = 0;
    tx->expected_gen = 1;
    tx->adapter = adapter;
    tx->stuck = false;
    tx->running = true;
    tx->last_re_idx = 0;
    tx->db_idx = 0;
    for (u16 i = 0; i < desc_cnt; i++)
        tx->free_tags[i] = i;
    tx->tags_ntu = 0;
    tx->tags_ntc = desc_cnt;

    adapter->dqo_qpl = qpl;
    if (qpl) {
        adapter->tx_pages_per_qpl = 64;
        u32 qsize = adapter->tx_pages_per_qpl * PAGESIZE;
        tx->qpl_base = allocate(gh, qsize);
        tx->qpl_base_phys = physical_from_virtual(tx->qpl_base);
        tx->qpl_num_slots = adapter->tx_pages_per_qpl * (PAGESIZE / GVE_DQO_BUF_SIZE);
        tx->qpl_slot_list = allocate(gh, tx->qpl_num_slots * sizeof(*tx->qpl_slot_list));
        for (u32 i = 0; i < tx->qpl_num_slots; i++)
            tx->qpl_slot_list[i] = (u16)i;
        tx->qpl_slot_head = tx->qpl_slot_tail = 0;
        tx->qpl_free_slots = tx->qpl_num_slots;
        tx->tag_qpl_slots = allocate(gh, desc_cnt * sizeof(*tx->tag_qpl_slots));
        tx->tag_qpl_n = allocate_zero(gh, desc_cnt * sizeof(*tx->tag_qpl_n));
    }

    zero(&tx->tx_stats, sizeof(tx->tx_stats));
    gve_tx_init_dqo(tx);

    /* wire the device model to this queue's completion ring */
    the_dev.tx_compl = tx->compl;
    the_dev.tx_compl_mask = desc_cnt - 1;
    the_dev.tx_compl_tail = 0;
    the_dev.tx_compl_gen = 1;
}

static void make_tx_dqo(gve adapter, gve_tx_dqo_queue tx, u16 desc_cnt)
{
    make_tx_dqo_mode(adapter, tx, desc_cnt, false);
}

static gve make_adapter(void)
{
    gve adapter = allocate_zero(gh, sizeof(struct gve));
    adapter->num_queues = 1;
    adapter->dqo = true;
    adapter->dqo_qpl = false;
    adapter->flags = 0;
    adapter->ndev.n.flags = NETIF_FLAG_UP;
    adapter->ndev.n.input = harness_input;
    adapter->ndev.n.num = 0;
    adapter->ndev.n.state = adapter;   /* netif_add does this in the driver */
    return adapter;
}

/* ------------------------------------------------------------------ */
/* Scenarios                                                            */
/* ------------------------------------------------------------------ */

/* A 1-segment packet posts ctx desc + 1 pkt desc (total_descs == 2). */
static struct pbuf *make_pkt(u16 len)
{
    struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_RAM);
    return p;
}

/* Scenario 1: TX RDA, single packet, in-order PKT completion.
 * Assert: pbuf retired (freed), tag returned to pool, desc_tail advanced. */
static void scenario_tx_inorder(void)
{
    rprintf("scenario_tx_inorder\n");
    gve adapter = make_adapter();
    gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
    make_tx_dqo(adapter, tx, 256);

    int live_before = pbufs_live;
    struct pbuf *p = make_pkt(100);
    int tags_free_before = (int)(tx->tags_ntc - tx->tags_ntu);

    /* linkoutput takes its own ref then enqueues; empty ring + free lock
     * drains inline: writes ctx+pkt descs, rings the doorbell. */
    /* The test plays lwIP: it owns one ref on p (from make_pkt).  The driver
     * takes its own ref at linkoutput and releases it at TX completion. */
    err_t e = gve_linkoutput_dqo(&adapter->ndev.n, p);
    CHECK(e == ERR_OK, "linkoutput returned %d", e);
    CHECK(tx->head == 2, "head expected 2 (ctx+pkt), got %d", tx->head);
    CHECK((int)(tx->tags_ntc - tx->tags_ntu) == tags_free_before - 1,
          "one tag should be taken");
    CHECK(p->ref == 2, "lwIP ref + driver ref expected (2), got %d", p->ref);

    /* The EOP descriptor carries the completion tag; read it back. */
    u16 tag = tx->desc[(0 + 1) & tx->mask].compl_tag; /* ctx@0, pkt@1 */
    CHECK(tx->seg_counts[tag] == 2, "seg_counts[%d] expected 2, got %d",
          tag, tx->seg_counts[tag]);

    /* Device completes the packet, then a drain runs cleanup. */
    dqo_dev_complete_pkt(&the_dev, tag, GVE_DQO_COMPL_TYPE_PKT);
    spin_lock(&tx->ring_mtx);
    gve_tx_dqo_cleanup(tx);
    spin_unlock(&tx->ring_mtx);

    CHECK(tx->seg_counts[tag] == 0, "tag should be retired (seg_counts 0)");
    CHECK(tx->desc_tail == 2, "desc_tail should advance by 2, got %d", tx->desc_tail);
    CHECK((int)(tx->tags_ntc - tx->tags_ntu) == tags_free_before,
          "tag should be returned to the pool");
    CHECK(p->ref == 1, "driver should have released its ref (1 left), got %d", p->ref);
    CHECK(pbufs_live == live_before + 1, "only the lwIP-owned pbuf remains");

    pbuf_free(p);   /* lwIP releases its ref */
    CHECK(pbufs_live == live_before, "pbuf fully freed after lwIP releases");
}

/* Send one packet on tx; returns its completion tag. */
static u16 send_pkt(gve_tx_dqo_queue tx, gve adapter, struct pbuf *p)
{
    u32 head_before = tx->head;
    gve_linkoutput_dqo(&adapter->ndev.n, p);
    /* EOP desc is the last one written; its compl_tag is the pool tag. */
    return tx->desc[(tx->head - 1) & tx->mask].compl_tag;
}

static void run_cleanup(gve_tx_dqo_queue tx)
{
    spin_lock(&tx->ring_mtx);
    gve_tx_dqo_cleanup(tx);
    spin_unlock(&tx->ring_mtx);
}

/* Scenario 2: a MISSed packet keeps its tag/pbuf until REINJECT, and a
 * packet sent in between gets a DIFFERENT tag (the collision the tag pool
 * exists to prevent).  Then REINJECT retires the missed packet. */
static void scenario_tx_miss_reinject(void)
{
    rprintf("scenario_tx_miss_reinject\n");
    gve adapter = make_adapter();
    gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
    make_tx_dqo(adapter, tx, 256);
    int live_before = pbufs_live;
    int free0 = (int)(tx->tags_ntc - tx->tags_ntu);

    struct pbuf *A = make_pkt(100);
    u16 tA = send_pkt(tx, adapter, A);

    /* Device MISSes A: record only, do not retire, do not return the tag. */
    dqo_dev_complete_pkt(&the_dev, tA, GVE_DQO_COMPL_TYPE_MISS);
    run_cleanup(tx);
    CHECK(tx->seg_counts[tA] == 2, "missed A not retired (seg_counts stays)");
    CHECK(tx->miss_times[tA] != 0, "miss recorded for A");
    CHECK(tx->pending[tA] == A, "A still pending");
    CHECK(A->ref == 2, "A still held by the driver");
    CHECK(tx->desc_tail == 0, "no descriptors retired on miss");
    CHECK((int)(tx->tags_ntc - tx->tags_ntu) == free0 - 1, "A's tag still out");

    /* A packet sent while A is missed must get a different tag. */
    struct pbuf *B = make_pkt(200);
    u16 tB = send_pkt(tx, adapter, B);
    CHECK(tB != tA, "B must not reuse the missed tag tA (%d)", tA);

    dqo_dev_complete_pkt(&the_dev, tB, GVE_DQO_COMPL_TYPE_PKT);
    run_cleanup(tx);
    CHECK(tx->seg_counts[tB] == 0, "B retired");
    CHECK(tx->seg_counts[tA] == 2, "A still pending after B retires");
    CHECK(tx->miss_times[tA] != 0, "A's miss still pending");

    /* REINJECT retires A. */
    dqo_dev_complete_pkt(&the_dev, tA, GVE_DQO_COMPL_TYPE_REINJECT);
    run_cleanup(tx);
    CHECK(tx->seg_counts[tA] == 0, "A retired on reinject");
    CHECK(tx->miss_times[tA] == 0, "A's miss state cleared");
    CHECK(tx->pending[tA] == NULL, "A no longer pending");
    CHECK(A->ref == 1, "driver released A's ref");
    CHECK((int)(tx->tags_ntc - tx->tags_ntu) == free0, "both tags returned");

    pbuf_free(A);
    pbuf_free(B);
    CHECK(pbufs_live == live_before, "all pbufs freed");
}

/* ------------------------------------------------------------------ */
/* RX device model + queue setup                                        */
/* ------------------------------------------------------------------ */

static u32 rx_buf_cursor;       /* device: next buf_ring entry to consume */
static u32 rx_compl_cursor;     /* device: next compl_ring entry to write */
static u8  rx_compl_gen;        /* device generation bit */

static void make_rx_dqo(gve adapter, gve_rx_dqo_queue rx, u16 num_bufs)
{
    rx->pbufs     = allocate_zero(gh, num_bufs * sizeof(*rx->pbufs));
    rx->free_ids  = allocate(gh, num_bufs * sizeof(*rx->free_ids));
    rx->buf_ring  = allocate_zero(gh, num_bufs * sizeof(*rx->buf_ring));
    rx->compl_ring = allocate_zero(gh, num_bufs * sizeof(*rx->compl_ring));
    static u32 irq_db; irq_db = 0;
    rx->irq_db_index = &irq_db;

    rx->mask = num_bufs - 1;
    rx->num_bufs = num_bufs;
    rx->buf_head = 0;
    rx->compl_head = 0;
    rx->expected_gen = 1;
    for (u16 i = 0; i < num_bufs; i++)
        rx->free_ids[i] = i;
    rx->next_to_use = 0;
    rx->next_to_clean = num_bufs;
    rx->adapter = adapter;
    rx->db_idx = 0;
    rx->idx = 0;
    rx->ctx_head = NULL;
    rx->drop_pkt = false;
    rx->db_head = 0;
    rx->first_interrupt = false;
    rx->no_interrupt_event_cnt = 0;
    rx->empty_rx_queue = 0;
    zero(&rx->rx_stats, sizeof(rx->rx_stats));
    gve_rx_dqo_init(rx);

    rx_buf_cursor = rx_compl_cursor = 0;
    rx_compl_gen = 1;
}

/* Device consumes the next posted buffer and writes a completion for it. */
static void dqo_dev_rx_complete(gve_rx_dqo_queue rx, u16 len, boolean eop,
                                boolean err)
{
    u16 buf_id = rx->buf_ring[rx_buf_cursor & rx->mask].buf_id;
    rx_buf_cursor++;
    u32 slot = rx_compl_cursor & rx->mask;
    struct gve_rx_compl_desc_dqo *c = &rx->compl_ring[slot];
    c->buf_id   = buf_id;
    c->err_flags = err ? GVE_DQO_RX_ERR : 0;
    c->status0  = eop ? GVE_DQO_RX_EOP : 0;
    write_barrier();
    c->pkt_len_gen = (u16)(len & GVE_DQO_RX_PKT_LEN_MASK) |
                     (rx_compl_gen ? GVE_DQO_RX_GEN : 0);
    rx_compl_cursor++;
    if ((rx_compl_cursor & rx->mask) == 0)
        rx_compl_gen ^= 1;
}

/* Free every pbuf the queue still holds (posted buffers + partial chain). */
static void free_rx_dqo(gve_rx_dqo_queue rx)
{
    if (rx->ctx_head) { pbuf_free(rx->ctx_head); rx->ctx_head = NULL; }
    for (u32 i = 0; i < rx->num_bufs; i++)
        if (rx->pbufs[i]) { pbuf_free(rx->pbufs[i]); rx->pbufs[i] = NULL; }
}

static void run_rx_service(gve_rx_dqo_queue rx)
{
    apply((thunk)&rx->service);
}

/* Scenario 3: RX single-buffer packet delivered in order. */
static void scenario_rx_inorder(void)
{
    rprintf("scenario_rx_inorder\n");
    gve adapter = make_adapter();
    gve_rx_dqo_queue rx = &adapter->rx_dqo[0];
    make_rx_dqo(adapter, rx, 256);
    int delivered_before = rx_delivered;

    gve_rx_dqo_fill(rx);
    CHECK(rx->buf_head > 0, "fill posted buffers, got %d", rx->buf_head);
    u16 first_buf = rx->buf_ring[0].buf_id;
    CHECK(rx->pbufs[first_buf] != NULL, "posted buffer has a pbuf");

    dqo_dev_rx_complete(rx, 100, true, false);
    run_rx_service(rx);

    CHECK(rx_delivered == delivered_before + 1, "one packet delivered");
    CHECK(rx->compl_head == 1, "compl_head advanced past the completion");
    CHECK(rx->ctx_head == NULL, "no partial chain left");

    free_rx_dqo(rx);
}

/* Scenario 4: RX multi-buffer packet (chain) delivered once at EOP. */
static void scenario_rx_chain(void)
{
    rprintf("scenario_rx_chain\n");
    gve adapter = make_adapter();
    gve_rx_dqo_queue rx = &adapter->rx_dqo[0];
    make_rx_dqo(adapter, rx, 256);
    int delivered_before = rx_delivered;

    gve_rx_dqo_fill(rx);
    dqo_dev_rx_complete(rx, 1400, false, false);  /* fragment 1, !EOP */
    dqo_dev_rx_complete(rx, 600,  true,  false);  /* fragment 2, EOP  */
    run_rx_service(rx);

    CHECK(rx_delivered == delivered_before + 1,
          "a 2-buffer packet is delivered exactly once");
    CHECK(rx->ctx_head == NULL, "chain fully delivered, no partial left");
    free_rx_dqo(rx);
}

/* Scenario 5: an error mid-chain drops to end-of-packet, and the next
 * packet is delivered cleanly (no corrupt frame from the chain tail). */
static void scenario_rx_drop_eop(void)
{
    rprintf("scenario_rx_drop_eop\n");
    gve adapter = make_adapter();
    gve_rx_dqo_queue rx = &adapter->rx_dqo[0];
    make_rx_dqo(adapter, rx, 256);
    int delivered_before = rx_delivered;

    gve_rx_dqo_fill(rx);
    dqo_dev_rx_complete(rx, 1400, false, true);   /* frag 1: error, !EOP */
    dqo_dev_rx_complete(rx, 600,  true,  false);  /* frag 2: EOP (tail)  */
    dqo_dev_rx_complete(rx, 100,  true,  false);  /* a fresh clean packet */
    run_rx_service(rx);

    CHECK(rx_delivered == delivered_before + 1,
          "only the clean packet is delivered, the dropped chain tail is not");
    CHECK(rx->drop_pkt == false, "drop state cleared at the chain's EOP");
    CHECK(rx->rx_stats.rx_dropped >= 1, "the errored packet was counted dropped");
    free_rx_dqo(rx);
}

/* Scenario 6: tag-pool / ring-cursor wrap.  Send many packets, completing
 * each before the next, so head/compl_head/tags_ntu all wrap repeatedly;
 * assert every packet retires cleanly and no completion is ever rejected. */
static void scenario_tx_wrap(void)
{
    rprintf("scenario_tx_wrap\n");
    gve adapter = make_adapter();
    gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
    make_tx_dqo(adapter, tx, 256);
    int live_before = pbufs_live;
    int free0 = (int)(tx->tags_ntc - tx->tags_ntu);

    const int N = 4000;          /* >> ring size: forces many wraps */
    for (int i = 0; i < N; i++) {
        struct pbuf *p = make_pkt(64);
        u16 tag = send_pkt(tx, adapter, p);
        dqo_dev_complete_pkt(&the_dev, tag, GVE_DQO_COMPL_TYPE_PKT);
        run_cleanup(tx);
        pbuf_free(p);            /* lwIP releases its ref */
    }
    CHECK(tx->tx_stats.bad_compl_tag == 0, "no completion ever rejected");
    CHECK(tx->tx_stats.cnt == (u64)N, "all %d packets accounted", N);
    CHECK((int)(tx->tags_ntc - tx->tags_ntu) == free0, "tag pool fully returned");
    CHECK(tx->head == tx->desc_tail, "ring fully drained (head == desc_tail)");
    CHECK(pbufs_live == live_before, "no pbuf leak across the wrap");
}

/* Build a pbuf chain of nsegs segments. */
static struct pbuf *make_chain(int nsegs, u16 seglen)
{
    struct pbuf *head = NULL, *tail = NULL;
    for (int i = 0; i < nsegs; i++) {
        struct pbuf *p = pbuf_alloc(PBUF_RAW, seglen, PBUF_RAM);
        p->tot_len = (u16)(seglen * nsegs);
        if (!head) head = p; else tail->next = p;
        tail = p;
    }
    return head;
}

/* Scenario 7: a packet with more than GVE_TX_MAX_DATA_DESCS segments is
 * dropped (consumed) rather than wedging the queue. */
static void scenario_tx_drop_oversize(void)
{
    rprintf("scenario_tx_drop_oversize\n");
    gve adapter = make_adapter();
    gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
    make_tx_dqo(adapter, tx, 256);
    int live_before = pbufs_live;
    int free0 = (int)(tx->tags_ntc - tx->tags_ntu);

    struct pbuf *p = make_chain(GVE_TX_MAX_DATA_DESCS + 1, 64);
    err_t e = gve_linkoutput_dqo(&adapter->ndev.n, p);

    CHECK(e == ERR_OK, "linkoutput accepts then drops, returns OK");
    CHECK(tx->head == 0, "no descriptors written for the dropped packet");
    CHECK((int)(tx->tags_ntc - tx->tags_ntu) == free0, "no tag consumed");
    CHECK(tx->running == true, "queue not wedged");
    CHECK(p->ref == 1, "driver released its ref on the dropped packet");

    pbuf_free(p);
    CHECK(pbufs_live == live_before, "dropped packet fully freed");
}

/* Scenario 8: RX fill tolerates a pbuf allocation failure (partial fill,
 * no crash, the failure is counted) and recovers on the next fill. */
static void scenario_rx_alloc_fail(void)
{
    rprintf("scenario_rx_alloc_fail\n");
    gve adapter = make_adapter();
    gve_rx_dqo_queue rx = &adapter->rx_dqo[0];
    make_rx_dqo(adapter, rx, 256);

    pbuf_alloc_fail_after = 5;        /* 5 succeed, then alloc fails */
    gve_rx_dqo_fill(rx);
    pbuf_alloc_fail_after = -1;       /* recover */

    CHECK(rx->buf_head == 5, "fill stopped at the alloc failure (5), got %d",
          rx->buf_head);
    CHECK(rx->rx_stats.refil_partial >= 1, "partial refill counted");

    gve_rx_dqo_fill(rx);             /* recovers, posts more */
    CHECK(rx->buf_head > 5, "fill resumes after allocations recover");
    free_rx_dqo(rx);
}

/* Scenario 9: DQO-QPL TX bounces the payload into a registered slot, frees
 * the pbuf inline, and returns the slot on completion. */
static void scenario_tx_qpl(void)
{
    rprintf("scenario_tx_qpl\n");
    gve adapter = make_adapter();
    gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
    make_tx_dqo_mode(adapter, tx, 256, true);
    int live_before = pbufs_live;
    u32 slots_before = tx->qpl_free_slots;

    struct pbuf *p = make_pkt(128);
    /* fill the payload with a known pattern to verify the bounce copy */
    for (int i = 0; i < 128; i++) ((u8 *)p->payload)[i] = (u8)(i + 1);

    u16 tag = send_pkt(tx, adapter, p);
    CHECK(p->ref == 1, "QPL frees the driver's ref inline (1 left), got %d", p->ref);
    CHECK(tx->qpl_free_slots == slots_before - 1, "one bounce slot taken");
    CHECK(tx->tag_qpl_n[tag] == 1, "one slot recorded for the tag");
    u16 sid = tx->tag_qpl_slots[tag][0];
    u8 *bounce = (u8 *)tx->qpl_base + (u64)sid * GVE_DQO_BUF_SIZE;
    boolean copied = true;
    for (int i = 0; i < 128; i++) if (bounce[i] != (u8)(i + 1)) copied = false;
    CHECK(copied, "payload bounce-copied into the QPL slot");

    dqo_dev_complete_pkt(&the_dev, tag, GVE_DQO_COMPL_TYPE_PKT);
    run_cleanup(tx);
    CHECK(tx->qpl_free_slots == slots_before, "bounce slot returned on completion");
    CHECK(tx->tag_qpl_n[tag] == 0, "slot record cleared");

    pbuf_free(p);
    CHECK(pbufs_live == live_before, "no pbuf leak (QPL)");
}

int main(int argc, char **argv)
{
    gh = init_process_runtime();
    failures = checks = 0;

    scenario_tx_inorder();
    scenario_tx_miss_reinject();
    scenario_rx_inorder();
    scenario_rx_chain();
    scenario_rx_drop_eop();
    scenario_tx_wrap();
    scenario_tx_drop_oversize();
    scenario_rx_alloc_fail();
    scenario_tx_qpl();

    rprintf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
