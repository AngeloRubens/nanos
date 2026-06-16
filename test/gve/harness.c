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

struct pbuf *pbuf_alloc(int layer, u16 length, int type)
{
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

static void make_tx_dqo(gve adapter, gve_tx_dqo_queue tx, u16 desc_cnt)
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
    zero(&tx->tx_stats, sizeof(tx->tx_stats));
    gve_tx_init_dqo(tx);

    /* wire the device model to this queue's completion ring */
    the_dev.tx_compl = tx->compl;
    the_dev.tx_compl_mask = desc_cnt - 1;
    the_dev.tx_compl_tail = 0;
    the_dev.tx_compl_gen = 1;
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

int main(int argc, char **argv)
{
    gh = init_process_runtime();
    failures = checks = 0;

    scenario_tx_inorder();
    scenario_tx_miss_reinject();

    rprintf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
