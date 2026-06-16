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

/* ---- register BAR (adminq) model, for the negotiation scenarios ---- */
void kernel_delay(timestamp t) { }
static int g_msix = 64;
int pci_get_msix_count(pci_dev d) { return g_msix; }

static struct pci_bar *g_reg_bar;     /* the adapter's register BAR */
static gve g_adapter;                 /* adapter being negotiated */
static u32  g_evt_cnt;                 /* adminq event counter */
static u32  g_max_tx = 8, g_max_rx = 8;
static u16  g_desc_opts[8];           /* device options to advertise */
static int  g_desc_nopts;

static void adminq_process(struct gve_adminq_command *cmd)
{
    if (be32toh(cmd->opcode) == GVE_ADMINQ_DESCRIBE_DEVICE) {
        struct gve_device_descriptor *d =
            pointer_from_u64(be64toh(cmd->describe_device.device_descriptor_addr));
        zero(d, PAGESIZE);
        d->mtu             = htobe16(1460);
        d->counters        = htobe16(16);
        d->tx_queue_entries = htobe16(256);
        d->rx_queue_entries = htobe16(256);
        d->tx_pages_per_qpl = htobe16(64);
        d->rx_pages_per_qpl = htobe16(256);
        d->num_device_options = htobe16(g_desc_nopts);
        struct gve_device_option *opt = (struct gve_device_option *)(d + 1);
        for (int i = 0; i < g_desc_nopts; i++) {
            opt->option_id = htobe16(g_desc_opts[i]);
            opt->option_length = 0;
            opt->required_features_mask = 0;
            opt++;
        }
        d->total_length = htobe16((u16)((u8 *)opt - (u8 *)d));
    }
    cmd->status = htobe32(GVE_ADMINQ_COMMAND_PASSED);
}

u32 pci_bar_read_4(struct pci_bar *b, u64 offset)
{
    if (b == g_reg_bar) {
        switch (offset) {
        case GVE_REG_ADMINQ_EVT_CNT: return htobe32(g_evt_cnt);
        case GVE_REG_MAX_TX_QUEUES:  return htobe32(g_max_tx);
        case GVE_REG_MAX_RX_QUEUES:  return htobe32(g_max_rx);
        default:                     return 0;
        }
    }
    return 0;   /* doorbell BAR */
}

void pci_bar_write_4(struct pci_bar *b, u64 offset, u32 val)
{
    if (b == g_reg_bar) {
        if (offset == GVE_REG_ADMINQ_DOORBELL) {
            u32 d = be32toh(val);
            adminq_process(&g_adapter->adminq[(d - 1) & g_adapter->adminq_mask]);
            g_evt_cnt = d;
        }
        return;
    }
    the_dev.last_tx_doorbell = val;   /* doorbell BAR */
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

static gve make_adapter_common(void)
{
    gve adapter = allocate_zero(gh, sizeof(struct gve));
    adapter->num_queues = 1;
    adapter->flags = 0;
    adapter->ndev.n.flags = NETIF_FLAG_UP;
    adapter->ndev.n.input = harness_input;
    adapter->ndev.n.num = 0;
    adapter->ndev.n.state = adapter;   /* netif_add does this in the driver */
    adapter->num_event_counters = 16;
    adapter->event_counters = allocate_zero(gh, 16 * sizeof(u32));
    return adapter;
}

static gve make_adapter(void)                 /* DQO */
{
    gve adapter = make_adapter_common();
    adapter->dqo = true;
    adapter->dqo_qpl = false;
    return adapter;
}

static gve make_adapter_gqi(boolean raw_addressing)
{
    gve adapter = make_adapter_common();
    adapter->dqo = false;
    adapter->dqo_qpl = false;
    adapter->raw_addressing = raw_addressing;
    adapter->tx_desc_cnt = 256;
    adapter->rx_desc_cnt = 256;
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

static void make_rx_dqo_mode(gve adapter, gve_rx_dqo_queue rx, u16 num_bufs,
                             boolean qpl)
{
    adapter->dqo_qpl = qpl;
    if (qpl) {
        u32 pages = (num_bufs * GVE_DQO_BUF_SIZE + PAGESIZE - 1) >> PAGELOG;
        rx->qpl_base = allocate(gh, pages * PAGESIZE);
        rx->qpl_base_phys = physical_from_virtual(rx->qpl_base);
    }
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

static void make_rx_dqo(gve adapter, gve_rx_dqo_queue rx, u16 num_bufs)
{
    make_rx_dqo_mode(adapter, rx, num_bufs, false);
}

/* Device consumes the next posted buffer and writes a completion for it. */
static void dqo_dev_rx_complete(gve_rx_dqo_queue rx, u16 len, boolean eop,
                                boolean err)
{
    u16 buf_id = rx->buf_ring[rx_buf_cursor & rx->mask].buf_id;
    rx_buf_cursor++;
    /* DQO-QPL: the device writes the payload into the registered slot the
     * driver will copy out. */
    if (rx->adapter->dqo_qpl && !err && len) {
        u8 *dst = (u8 *)rx->qpl_base + (u64)buf_id * GVE_DQO_BUF_SIZE;
        for (u16 i = 0; i < len; i++) dst[i] = (u8)(i + 7);
    }
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

/* Scenario 10: alternate-miss encoding (a PKT-type completion whose
 * completion_tag has bit 15 set is a miss, not a retirement). */
static void scenario_tx_alt_miss(void)
{
    rprintf("scenario_tx_alt_miss\n");
    gve adapter = make_adapter();
    gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
    make_tx_dqo(adapter, tx, 256);

    struct pbuf *p = make_pkt(80);
    u16 tag = send_pkt(tx, adapter, p);

    dqo_dev_complete_pkt(&the_dev, tag | GVE_DQO_ALT_MISS_COMPL_BIT,
                         GVE_DQO_COMPL_TYPE_PKT);
    run_cleanup(tx);
    CHECK(tx->miss_times[tag] != 0, "alt-miss recorded as a miss");
    CHECK(tx->seg_counts[tag] == 2, "alt-miss did not retire the packet");
    CHECK(tx->tx_stats.bad_compl_tag == 0, "alt-miss is not a bad tag");

    dqo_dev_complete_pkt(&the_dev, tag, GVE_DQO_COMPL_TYPE_REINJECT);
    run_cleanup(tx);
    CHECK(tx->seg_counts[tag] == 0, "reinject retired the alt-missed packet");
    pbuf_free(p);
}

/* Scenario 11: a stale/duplicate completion for a tag not in flight is
 * counted and skipped, the ring cursor advances, and NO reset is scheduled
 * (the trust-line fix; Google rate-limit-logs and continues). */
static void scenario_tx_stale_tag(void)
{
    rprintf("scenario_tx_stale_tag\n");
    gve adapter = make_adapter();
    gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
    make_tx_dqo(adapter, tx, 256);

    struct pbuf *p = make_pkt(80);
    u16 tag = send_pkt(tx, adapter, p);
    dqo_dev_complete_pkt(&the_dev, tag, GVE_DQO_COMPL_TYPE_PKT);
    run_cleanup(tx);                       /* tag retired */
    pbuf_free(p);

    u32 ch_before = tx->compl_head;
    u64 reset_before = (adapter->flags >> GVE_FLAG_RESETTING) & 1;
    dqo_dev_complete_pkt(&the_dev, tag, GVE_DQO_COMPL_TYPE_PKT);  /* stale */
    run_cleanup(tx);
    CHECK(tx->tx_stats.bad_compl_tag == 1, "stale tag counted");
    CHECK(tx->compl_head == ch_before + 1, "ring cursor advanced past stale");
    CHECK(((adapter->flags >> GVE_FLAG_RESETTING) & 1) == reset_before,
          "no reset scheduled for a stale completion");
}

/* deterministic xorshift PRNG for the fuzzer */
static u64 rng_state = 0x123456789abcdef0ull;
static u64 rng(void) {
    u64 x = rng_state;
    x ^= x << 13; x ^= x >> 7; x ^= x << 17;
    return rng_state = x;
}

/* Scenario 12: fuzz out-of-order completions.  Keep a bounded set of
 * outstanding packets; randomly send new ones or complete an arbitrary
 * outstanding one (by tag, i.e. out of order) — the exact stress the tag
 * pool exists for.  Assert no crash, no rejected completion, and that at
 * quiescence the tag pool and ring are fully restored with no pbuf leak. */
static void scenario_tx_fuzz(void)
{
    rprintf("scenario_tx_fuzz\n");
    gve adapter = make_adapter();
    gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
    make_tx_dqo(adapter, tx, 256);
    int live_before = pbufs_live;
    int free0 = (int)(tx->tags_ntc - tx->tags_ntu);

    enum { CAP = 40 };
    struct pbuf *out_p[CAP];
    u16          out_tag[CAP];
    int n = 0;

    for (int step = 0; step < 20000; step++) {
        boolean do_send = (n < CAP) && (n == 0 || (rng() & 1));
        if (do_send) {
            struct pbuf *p = make_pkt(32 + (rng() & 63));
            out_tag[n] = send_pkt(tx, adapter, p);
            out_p[n] = p;
            n++;
        } else {
            int i = (int)(rng() % (u64)n);     /* complete an arbitrary one */
            dqo_dev_complete_pkt(&the_dev, out_tag[i], GVE_DQO_COMPL_TYPE_PKT);
            run_cleanup(tx);
            pbuf_free(out_p[i]);
            out_p[i] = out_p[n - 1];
            out_tag[i] = out_tag[n - 1];
            n--;
        }
    }
    /* drain the remainder */
    while (n > 0) {
        dqo_dev_complete_pkt(&the_dev, out_tag[n - 1], GVE_DQO_COMPL_TYPE_PKT);
        run_cleanup(tx);
        pbuf_free(out_p[n - 1]);
        n--;
    }

    CHECK(tx->tx_stats.bad_compl_tag == 0, "fuzz: no completion rejected");
    CHECK((int)(tx->tags_ntc - tx->tags_ntu) == free0, "fuzz: tag pool restored");
    CHECK(tx->head == tx->desc_tail, "fuzz: ring fully drained");
    CHECK(pbufs_live == live_before, "fuzz: no pbuf leak");
}

/* ------------------------------------------------------------------ */
/* GQI datapath (gve_datapath.c, separate TU): event-counter completion */
/* ------------------------------------------------------------------ */

static void make_tx_gqi(gve adapter, gve_tx_queue tx, u16 desc_cnt)
{
    boolean rda = adapter->raw_addressing;
    tx->desc = allocate_zero(gh, desc_cnt * sizeof(*tx->desc));
    tx->tx_timestamps = allocate_zero(gh, desc_cnt * sizeof(*tx->tx_timestamps));
    tx->br = allocate_queue(gh, GVE_BUF_RING_SIZE);
    if (rda) {
        tx->qpl_base = NULL;
        tx->pending = allocate_zero(gh, desc_cnt * sizeof(*tx->pending));
    } else {
        adapter->tx_pages_per_qpl = 64;
        u32 qsize = adapter->tx_pages_per_qpl * PAGESIZE;
        tx->qpl_base = allocate(gh, qsize);
        tx->qpl_allocated = allocate_zero(gh, desc_cnt * sizeof(*tx->qpl_allocated));
        tx->qpl_head = 0;
        tx->qpl_used = 0;
        tx->qpl_size = qsize;
    }
    tx->mask = desc_cnt - 1;
    tx->head = tx->tail = 0;
    tx->adapter = adapter;
    tx->stuck = false;
    tx->running = true;
    tx->event_counter_idx = 0;
    tx->db_idx = 0;
    zero(&tx->tx_stats, sizeof(tx->tx_stats));
    gve_tx_init_gqi(tx);
}

/* GQI device: advance the event counter to N descriptors completed. */
static void gqi_dev_tx_complete_all(gve adapter, gve_tx_queue tx)
{
    adapter->event_counters[tx->event_counter_idx] = htobe32(tx->head);
}

static void run_tx_gqi_cleanup(gve_tx_queue tx)
{
    apply((thunk)&tx->enqueue_task);   /* drains (br empty) -> cleanup */
}

/* Scenario 13: GQI-RDA TX, single packet, event-counter completion. */
static void scenario_gqi_tx_rda(void)
{
    rprintf("scenario_gqi_tx_rda\n");
    gve adapter = make_adapter_gqi(true);
    gve_tx_queue tx = &adapter->tx[0];
    make_tx_gqi(adapter, tx, 256);
    gve_setup_linkoutput(adapter, &adapter->ndev.n);
    int live_before = pbufs_live;

    struct pbuf *p = make_pkt(100);
    err_t e = adapter->ndev.n.linkoutput(&adapter->ndev.n, p);
    CHECK(e == ERR_OK, "gqi-rda linkoutput ok");
    CHECK(tx->head == 1, "one descriptor written, got %d", tx->head);
    CHECK(p->ref == 2, "lwIP + driver ref");
    CHECK(tx->pending[0] == p, "pbuf pending in RDA slot");

    gqi_dev_tx_complete_all(adapter, tx);
    run_tx_gqi_cleanup(tx);
    CHECK(tx->tail == tx->head, "all descriptors retired");
    CHECK(tx->pending[0] == NULL, "pending slot cleared");
    CHECK(p->ref == 1, "driver released its ref");

    pbuf_free(p);
    CHECK(pbufs_live == live_before, "no leak (gqi-rda)");
}

/* Scenario 14: GQI-QPL TX, single packet, payload copied into the QPL
 * byte FIFO, qpl_used released on completion. */
static void scenario_gqi_tx_qpl(void)
{
    rprintf("scenario_gqi_tx_qpl\n");
    gve adapter = make_adapter_gqi(false);
    gve_tx_queue tx = &adapter->tx[0];
    make_tx_gqi(adapter, tx, 256);
    gve_setup_linkoutput(adapter, &adapter->ndev.n);
    int live_before = pbufs_live;

    struct pbuf *p = make_pkt(128);
    for (int i = 0; i < 128; i++) ((u8 *)p->payload)[i] = (u8)(i + 3);
    err_t e = adapter->ndev.n.linkoutput(&adapter->ndev.n, p);
    CHECK(e == ERR_OK, "gqi-qpl linkoutput ok");
    CHECK(tx->head == 1, "one descriptor written");
    CHECK(tx->qpl_used > 0, "QPL bytes consumed");
    CHECK(p->ref == 1, "QPL frees the pbuf inline after the copy");
    boolean copied = true;
    for (int i = 0; i < 128; i++)
        if (((u8 *)tx->qpl_base)[i] != (u8)(i + 3)) copied = false;
    CHECK(copied, "payload copied into the QPL FIFO");

    gqi_dev_tx_complete_all(adapter, tx);
    run_tx_gqi_cleanup(tx);
    CHECK(tx->tail == tx->head, "all descriptors retired");
    CHECK(tx->qpl_used == 0, "QPL bytes released on completion");
    CHECK(p->ref == 1, "only the lwIP ref remains");
    pbuf_free(p);          /* lwIP releases its ref */
    CHECK(pbufs_live == live_before, "no leak (gqi-qpl)");
}

/* GQI RX setup (mirrors gve_create_rx_queue; n = rx_desc_cnt ==
 * rx_data_slot_cnt, the GQI invariant). */
static u32 gqi_rx_cursor;        /* device: next desc/data slot to fill */

static void make_rx_gqi(gve adapter, gve_rx_queue rx, u16 n)
{
    boolean rda = adapter->raw_addressing;
    adapter->rx_desc_cnt = n;
    adapter->rx_data_slot_cnt = n;
    u16 num_pages = n;

    rx->qpl_base = allocate(gh, num_pages * PAGESIZE);
    rx->rda_base_phys = rda ? physical_from_virtual(rx->qpl_base) : 0;
    rx->qpl_available = rx->qpl_count = num_pages;
    rx->pbufs = allocate(gh, rx->qpl_count * sizeof(*rx->pbufs));
    rx->desc  = allocate_zero(gh, n * sizeof(*rx->desc));
    rx->data  = allocate_zero(gh, n * sizeof(*rx->data));
    static u32 irq_db; irq_db = 0;
    rx->irq_db_index = &irq_db;

    rx->mask = n - 1;
    rx->head = rx->tail = 0;
    rx->qpl_head = 0;
    for (u32 i = 0; i < rx->qpl_count; i++) {
        struct pbuf *pb = &rx->pbufs[i];
        pb->next = NULL; pb->type_internal = PBUF_REF; pb->flags = 0;
        pb->ref = 1; pb->if_idx = NETIF_NO_INDEX;
    }
    rx->adapter = adapter;
    rx->event_counter_idx = 1;
    rx->db_idx = 0;
    rx->idx = 0;
    rx->ctx_head = NULL;
    rx->drop_pkt = false;
    rx->first_interrupt = false;
    rx->no_interrupt_event_cnt = 0;
    rx->empty_rx_queue = 0;
    zero(&rx->rx_stats, sizeof(rx->rx_stats));
    gve_rx_init(rx);
    gve_rx_fill(rx);
    gqi_rx_cursor = 0;
}

/* GQI device: write packet data into the posted QPL slot and a descriptor,
 * then advance the event counter.  flags carries ERR / PKT_CONT (be16). */
static void gqi_dev_rx_complete(gve_rx_queue rx, u16 pkt_len, u16 flags)
{
    u32 slot = gqi_rx_cursor & rx->mask;
    u64 offset = be64toh(rx->data[slot]) - rx->rda_base_phys;
    /* write the payload after the 2-byte alignment pad */
    u8 *dst = (u8 *)rx->qpl_base + offset + GVE_RX_PADDING;
    for (u16 i = 0; i < pkt_len; i++) dst[i] = (u8)(i + 1);
    rx->desc[slot].len = htobe16(pkt_len + GVE_RX_PADDING);
    rx->desc[slot].flags_seq = flags;
    gqi_rx_cursor++;
    rx->adapter->event_counters[rx->event_counter_idx] = htobe32(gqi_rx_cursor);
}

/* Scenario 15: GQI-RDA RX single-buffer packet (zero-copy fast path). */
static void scenario_gqi_rx(void)
{
    rprintf("scenario_gqi_rx\n");
    gve adapter = make_adapter_gqi(true);
    gve_rx_queue rx = &adapter->rx[0];
    make_rx_gqi(adapter, rx, 64);
    gve_setup_linkoutput(adapter, &adapter->ndev.n); /* sets net_if->num path */
    int delivered_before = rx_delivered;

    CHECK(rx->head == 64, "fill posted all buffers, got %d", rx->head);
    u32 pool0_ref = rx->pbufs[0].ref;

    gqi_dev_rx_complete(rx, 100, 0);    /* normal single-buffer packet */
    apply((thunk)&rx->service);

    CHECK(rx_delivered == delivered_before + 1, "one packet delivered");
    CHECK(rx->tail == 1, "tail advanced past the completion");
    CHECK(rx->pbufs[0].ref == pool0_ref,
          "zero-copy pool pbuf recycled (ref back to %d)", pool0_ref);
}

/* Scenario 16: GQI RX multi-buffer chain delivered once at the final
 * (non-CONT) buffer. */
static void scenario_gqi_rx_chain(void)
{
    rprintf("scenario_gqi_rx_chain\n");
    gve adapter = make_adapter_gqi(true);
    gve_rx_queue rx = &adapter->rx[0];
    make_rx_gqi(adapter, rx, 64);
    int delivered_before = rx_delivered;
    int live_before = pbufs_live;

    gqi_dev_rx_complete(rx, 1400, GVE_RXF_PKT_CONT);  /* fragment 1 */
    gqi_dev_rx_complete(rx, 600,  0);                 /* fragment 2 (final) */
    apply((thunk)&rx->service);

    CHECK(rx_delivered == delivered_before + 1, "chain delivered exactly once");
    CHECK(rx->ctx_head == NULL, "no partial chain left");
    CHECK(rx->tail == 2, "both completions consumed");
    CHECK(pbufs_live == live_before, "chain copy pbufs freed after delivery");
}

/* Scenario 17: GQI-QPL RX (same service path as RDA with base 0). */
static void scenario_gqi_rx_qpl(void)
{
    rprintf("scenario_gqi_rx_qpl\n");
    gve adapter = make_adapter_gqi(false);  /* QPL */
    gve_rx_queue rx = &adapter->rx[0];
    make_rx_gqi(adapter, rx, 64);
    int delivered_before = rx_delivered;

    gqi_dev_rx_complete(rx, 120, 0);
    apply((thunk)&rx->service);
    CHECK(rx_delivered == delivered_before + 1, "gqi-qpl rx delivered");
    CHECK(rx->tail == 1, "completion consumed");
}

/* Scenario 18: DQO-QPL RX copies the payload out of the registered slot
 * into a fresh pbuf and recycles the buffer id. */
static void scenario_dqo_rx_qpl(void)
{
    rprintf("scenario_dqo_rx_qpl\n");
    gve adapter = make_adapter();
    adapter->dqo_qpl = true;
    gve_rx_dqo_queue rx = &adapter->rx_dqo[0];
    make_rx_dqo_mode(adapter, rx, 256, true);
    int delivered_before = rx_delivered;
    int live_before = pbufs_live;

    gve_rx_dqo_fill(rx);                 /* posts QPL slots, no pbufs */
    CHECK(pbufs_live == live_before, "QPL fill allocates no pbufs");

    dqo_dev_rx_complete(rx, 128, true, false);
    run_rx_service(rx);
    CHECK(rx_delivered == delivered_before + 1, "dqo-qpl rx delivered (copied out)");
    CHECK(rx->compl_head == 1, "completion consumed");
    CHECK(pbufs_live == live_before, "copy-out pbuf freed on delivery");
}

/* ------------------------------------------------------------------ */
/* Device negotiation / fallback (gve_adminq.c, third TU)               */
/* ------------------------------------------------------------------ */

static gve negotiate(const u16 *opts, int nopts)
{
    gve adapter = make_adapter_common();
    adapter->contiguous = gh;
    adapter->adminq = allocate(gh, PAGESIZE);
    adapter->adminq_mask = PAGESIZE / sizeof(struct gve_adminq_command) - 1;
    adapter->adminq_head = 0;
    adapter->adminq_running = true;
    adapter->reg_bar.vaddr = pointer_from_u64(0x1000);
    adapter->db_bar.vaddr  = pointer_from_u64(0x2000);
    g_reg_bar = &adapter->reg_bar;
    g_adapter = adapter;
    g_evt_cnt = 0;
    g_desc_nopts = nopts;
    for (int i = 0; i < nopts; i++) g_desc_opts[i] = opts[i];

    boolean ok = gve_describe_device(adapter);
    CHECK(ok, "describe_device succeeded");
    return adapter;
}

/* Scenario 19: format negotiation and the GQI-QPL fallback. */
static void scenario_negotiate_formats(void)
{
    rprintf("scenario_negotiate_formats\n");

    /* No options at all -> GQI-QPL fallback (the common case on older
     * devices; the allocate_zero fix makes this deterministic). */
    gve a = negotiate(NULL, 0);
    CHECK(!a->dqo && !a->dqo_qpl && !a->raw_addressing,
          "no options -> GQI-QPL fallback (all format flags false)");
    CHECK(!a->rss_supported, "no RSS option -> rss_supported false");

    u16 o_dqo_rda[] = { GVE_DEV_OPT_ID_DQO_RDA };
    a = negotiate(o_dqo_rda, 1);
    CHECK(a->dqo && !a->dqo_qpl, "DQO_RDA option -> DQO-RDA");

    u16 o_dqo_qpl[] = { GVE_DEV_OPT_ID_DQO_QPL };
    a = negotiate(o_dqo_qpl, 1);
    CHECK(a->dqo && a->dqo_qpl, "DQO_QPL option -> DQO-QPL");

    u16 o_gqi_rda[] = { GVE_DEV_OPT_ID_GQI_RDA };
    a = negotiate(o_gqi_rda, 1);
    CHECK(!a->dqo && a->raw_addressing, "GQI_RDA option -> GQI-RDA");

    /* Priority: DQO-RDA beats GQI-RDA when both are offered. */
    u16 o_both[] = { GVE_DEV_OPT_ID_GQI_RDA, GVE_DEV_OPT_ID_DQO_RDA };
    a = negotiate(o_both, 2);
    CHECK(a->dqo, "DQO-RDA wins over GQI-RDA (priority)");

    u16 o_rss[] = { GVE_DEV_OPT_ID_RSS_CONFIG };
    a = negotiate(o_rss, 1);
    CHECK(a->rss_supported, "RSS_CONFIG option -> rss_supported");
    CHECK(!a->dqo && !a->raw_addressing, "RSS option alone keeps GQI-QPL");
}

/* Scenario 20: queue-count negotiation and the dev_max==0 fallback. */
static void scenario_negotiate_queue_count(void)
{
    rprintf("scenario_negotiate_queue_count\n");
    gve a = make_adapter_common();
    a->reg_bar.vaddr = pointer_from_u64(0x1000);
    g_reg_bar = &a->reg_bar;
    int saved_tp = total_processors;

    /* device reports 0 max queues -> fall back to 1 */
    g_max_tx = g_max_rx = 0; g_msix = 64; total_processors = 8;
    CHECK(gve_calc_num_queues(a, 0) == 1, "dev_max 0 -> 1 queue");

    /* min over device max / msix / cpu */
    g_max_tx = g_max_rx = 8; g_msix = 64; total_processors = 4;
    CHECK(gve_calc_num_queues(a, 0) == 4, "capped by cpu count (4)");

    g_max_tx = g_max_rx = 8; g_msix = 64; total_processors = 16;
    CHECK(gve_calc_num_queues(a, 0) == 8, "capped by device max (8)");

    g_max_tx = g_max_rx = 32; g_msix = 9; total_processors = 16;
    CHECK(gve_calc_num_queues(a, 0) == 4, "capped by MSI-X ((9-1)/2 = 4)");

    total_processors = saved_tp;
    g_max_tx = g_max_rx = 8; g_msix = 64;
}

/* Scenario 21: DQO TX multi-segment packet (chain -> ctx + N pkt descs,
 * one tag, retired as a unit). */
static void scenario_dqo_tx_multiseg(void)
{
    rprintf("scenario_dqo_tx_multiseg\n");
    gve adapter = make_adapter();
    gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
    make_tx_dqo(adapter, tx, 256);
    int live_before = pbufs_live;
    int free0 = (int)(tx->tags_ntc - tx->tags_ntu);

    struct pbuf *p = make_chain(3, 200);     /* 3 segments */
    u16 tag = send_pkt(tx, adapter, p);
    CHECK(tx->head == 4, "ctx + 3 pkt descs written, got %d", tx->head);
    CHECK(tx->seg_counts[tag] == 4, "4 descriptors recorded for the tag");
    CHECK(tx->pending[tag] == p, "chain held pending in RDA");

    dqo_dev_complete_pkt(&the_dev, tag, GVE_DQO_COMPL_TYPE_PKT);
    run_cleanup(tx);
    CHECK(tx->desc_tail == 4, "all 4 descriptors retired");
    CHECK((int)(tx->tags_ntc - tx->tags_ntu) == free0, "tag returned");

    pbuf_free(p);
    CHECK(pbufs_live == live_before, "multi-seg chain fully freed");
}

/* Scenario 22: GQI-RDA TX multi-segment packet (pkt + seg descriptors,
 * each segment slot holds its own pbuf reference). */
static void scenario_gqi_tx_multiseg(void)
{
    rprintf("scenario_gqi_tx_multiseg\n");
    gve adapter = make_adapter_gqi(true);
    gve_tx_queue tx = &adapter->tx[0];
    make_tx_gqi(adapter, tx, 256);
    gve_setup_linkoutput(adapter, &adapter->ndev.n);
    int live_before = pbufs_live;

    struct pbuf *p = make_chain(3, 200);
    err_t e = adapter->ndev.n.linkoutput(&adapter->ndev.n, p);
    CHECK(e == ERR_OK, "gqi multi-seg linkoutput ok");
    CHECK(tx->head == 3, "pkt + 2 seg descriptors written, got %d", tx->head);
    CHECK(tx->pending[0] == p && tx->pending[1] == p->next, "chain segments pending");

    gqi_dev_tx_complete_all(adapter, tx);
    run_tx_gqi_cleanup(tx);
    CHECK(tx->tail == tx->head, "all segments retired");

    pbuf_free(p);
    CHECK(pbufs_live == live_before, "no leak across segment refs");
}

/* Scenario 23: TX backpressure — fill the ring until the queue stops, then
 * a completion lets a queued packet through (queue_wakeup). */
static void scenario_dqo_tx_backpressure(void)
{
    rprintf("scenario_dqo_tx_backpressure\n");
    gve adapter = make_adapter();
    gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
    make_tx_dqo(adapter, tx, 16);            /* small ring */
    int live_before = pbufs_live;

    /* send (without completing) until the queue stops accepting */
    int sent = 0;
    struct pbuf *kept[32];
    while (tx->running && sent < 32) {
        struct pbuf *p = make_pkt(64);
        kept[sent++] = p;
        adapter->ndev.n.state = adapter;
        gve_linkoutput_dqo(&adapter->ndev.n, p);
    }
    CHECK(!tx->running, "queue stopped under backpressure");
    CHECK(tx->tx_stats.queue_stop >= 1, "queue_stop counted");
    CHECK(!queue_empty(tx->br), "a packet is held in the software queue");

    /* complete the outstanding packets, then kick the drain */
    for (u32 t = tx->tags_ntu; t != tx->tags_ntc; t++) { }   /* (no-op) */
    /* complete in submission order via the descriptor EOP tags */
    u32 done = 0;
    while (tx->desc_tail != tx->head && done < 64) {
        u16 tag = tx->desc[(tx->desc_tail + 1) & tx->mask].compl_tag;
        if (tx->seg_counts[tag]) {
            dqo_dev_complete_pkt(&the_dev, tag, GVE_DQO_COMPL_TYPE_PKT);
            run_cleanup(tx);
        } else {
            run_cleanup(tx);
        }
        done++;
    }
    apply((thunk)&tx->enqueue_task);         /* wakeup + drain the held pkt */
    CHECK(tx->running, "queue resumed after completions");
    CHECK(tx->tx_stats.queue_wakeup >= 1, "queue_wakeup counted");
    CHECK(queue_empty(tx->br), "software queue drained");

    /* drain everything to balance pbufs */
    while (tx->desc_tail != tx->head) {
        u16 tag = tx->desc[(tx->desc_tail + 1) & tx->mask].compl_tag;
        dqo_dev_complete_pkt(&the_dev, tag, GVE_DQO_COMPL_TYPE_PKT);
        run_cleanup(tx);
    }
    for (int i = 0; i < sent; i++) pbuf_free(kept[i]);
    CHECK(pbufs_live == live_before, "no leak under backpressure");
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
    scenario_tx_alt_miss();
    scenario_tx_stale_tag();
    scenario_tx_fuzz();
    scenario_gqi_tx_rda();
    scenario_gqi_tx_qpl();
    scenario_gqi_rx();
    scenario_gqi_rx_chain();
    scenario_gqi_rx_qpl();
    scenario_dqo_rx_qpl();
    scenario_negotiate_formats();
    scenario_negotiate_queue_count();
    scenario_dqo_tx_multiseg();
    scenario_gqi_tx_multiseg();
    scenario_dqo_tx_backpressure();

    rprintf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
