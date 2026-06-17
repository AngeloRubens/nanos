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
/* Descriptor layout cross-check against the official Google driver.    */
/*                                                                      */
/* These _Static_asserts are the one check independent of the driver    */
/* itself: the sizes and field offsets are transcribed as literals from */
/* the public Google headers (gve_desc.h, gve_desc_dqo.h, gve_adminq.h  */
/* in GoogleCloudPlatform/compute-virtual-ethernet-linux), so a         */
/* reordered field or wrong type in gve_priv.h fails the build even     */
/* though the driver and the device model would agree with each other.  */
/* Only fields the driver/device actually read or write are pinned.     */
/* ------------------------------------------------------------------ */
#define OFF(t, m) __builtin_offsetof(t, m)

/* --- GQI: gve_desc.h --- */
_Static_assert(sizeof(struct gve_tx_pkt_desc) == 16, "gve_tx_pkt_desc 16B");
_Static_assert(OFF(struct gve_tx_pkt_desc, l4_csum_offset) == 1, "");
_Static_assert(OFF(struct gve_tx_pkt_desc, l4_hdr_offset) == 2, "");
_Static_assert(OFF(struct gve_tx_pkt_desc, len) == 4, "");
_Static_assert(OFF(struct gve_tx_pkt_desc, seg_len) == 6, "");
_Static_assert(OFF(struct gve_tx_pkt_desc, seg_addr) == 8, "");
_Static_assert(sizeof(struct gve_tx_seg_desc) == 16, "gve_tx_seg_desc 16B");
_Static_assert(OFF(struct gve_tx_seg_desc, mss) == 4, "");
_Static_assert(OFF(struct gve_tx_seg_desc, seg_len) == 6, "");
_Static_assert(OFF(struct gve_tx_seg_desc, seg_addr) == 8, "");
_Static_assert(sizeof(struct gve_rx_desc) == 64, "gve_rx_desc 64B");
_Static_assert(OFF(struct gve_rx_desc, len) == 60, "");
_Static_assert(OFF(struct gve_rx_desc, flags_seq) == 62, "");

/* --- adminq: gve_adminq.h --- */
_Static_assert(sizeof(struct gve_queue_resources) == 64, "gve_queue_resources 64B");
_Static_assert(OFF(struct gve_queue_resources, db_index) == 0, "");
_Static_assert(OFF(struct gve_queue_resources, counter_index) == 4, "");

/* --- DQO: gve_desc_dqo.h --- */
_Static_assert(sizeof(struct gve_tx_pkt_desc_dqo) == 16, "gve_tx_pkt_desc_dqo 16B");
_Static_assert(OFF(struct gve_tx_pkt_desc_dqo, dtype_flags) == 8, "");
_Static_assert(OFF(struct gve_tx_pkt_desc_dqo, compl_tag) == 12, "");
_Static_assert(OFF(struct gve_tx_pkt_desc_dqo, buf_size) == 14, "");
_Static_assert(sizeof(struct gve_tx_ctx_desc_dqo) == 16, "gve_tx_ctx_desc_dqo 16B");
_Static_assert(OFF(struct gve_tx_ctx_desc_dqo, cmd_dtype) == 8, "");
_Static_assert(sizeof(struct gve_tx_compl_desc_dqo) == 8, "gve_tx_compl_desc_dqo 8B");
_Static_assert(OFF(struct gve_tx_compl_desc_dqo, completion_tag) == 2, "");
_Static_assert(sizeof(struct gve_rx_buf_desc_dqo) == 32, "gve_rx_buf_desc_dqo 32B");
_Static_assert(OFF(struct gve_rx_buf_desc_dqo, buf_addr) == 8, "");
_Static_assert(OFF(struct gve_rx_buf_desc_dqo, header_buf_addr) == 16, "");
_Static_assert(sizeof(struct gve_rx_compl_desc_dqo) == 32, "gve_rx_compl_desc_dqo 32B");
_Static_assert(OFF(struct gve_rx_compl_desc_dqo, err_flags) == 1, "");
_Static_assert(OFF(struct gve_rx_compl_desc_dqo, pkt_len_gen) == 4, "");
_Static_assert(OFF(struct gve_rx_compl_desc_dqo, status0) == 8, "");
_Static_assert(OFF(struct gve_rx_compl_desc_dqo, buf_id) == 12, "");

#undef OFF

/* ------------------------------------------------------------------ */
/* Test harness state and assertions                                    */
/* ------------------------------------------------------------------ */

static heap gh;                 /* global process heap */
static int  failures;
static int  checks;

/* Concurrency harness (GVE_HARNESS_SMP): a mutex serialises pbuf alloc/free
 * and the refcount, modelling lwIP's SYS_ARCH_PROTECT and the kernel's locked
 * general heap.  Without it, concurrent service/linkoutput threads would race
 * in the (host) allocator itself and TSan would flag the heap rather than the
 * driver logic under test.  In the normal build these are no-ops. */
#ifdef GVE_HARNESS_SMP
#include <pthread.h>
static pthread_mutex_t g_heap_mtx = PTHREAD_MUTEX_INITIALIZER;
#define HEAP_LOCK()   pthread_mutex_lock(&g_heap_mtx)
#define HEAP_UNLOCK() pthread_mutex_unlock(&g_heap_mtx)
#else
#define HEAP_LOCK()   ((void)0)
#define HEAP_UNLOCK() ((void)0)
#endif

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

/* Failing-heap wrapper: delegates to gh but returns INVALID after a set
 * number of allocations, so the driver's alloc-failure cascades and the
 * ring-size backoff can be exercised. */
static int g_alloc_fail_after = -1;       /* -1 = never fail */
static u64 failheap_alloc(struct heap *h, bytes b)
{
    if (g_alloc_fail_after == 0)
        return INVALID_PHYSICAL;
    if (g_alloc_fail_after > 0)
        g_alloc_fail_after--;
    return gh->alloc(gh, b);
}
static void failheap_dealloc(struct heap *h, u64 a, bytes b)
{
    gh->dealloc(gh, a, b);
}
static struct heap g_failheap;
static heap fail_heap(void)
{
    g_failheap.alloc = failheap_alloc;
    g_failheap.dealloc = failheap_dealloc;
    g_failheap.pagesize = gh->pagesize;
    return &g_failheap;
}

int total_processors = 1;
static struct harness_cpu the_cpu = { .id = 0 };
cpuinfo current_cpu(void) { return &the_cpu; }
static void set_cpu(u32 id) { the_cpu.id = id; }

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
    HEAP_LOCK();
    if (pbuf_alloc_fail_after == 0) {
        HEAP_UNLOCK();
        return 0;
    }
    if (pbuf_alloc_fail_after > 0)
        pbuf_alloc_fail_after--;
    struct pbuf *p = allocate(gh, sizeof(*p));
    if (p == INVALID_ADDRESS) {
        HEAP_UNLOCK();
        return 0;
    }
    p->payload = (length ? allocate(gh, length) : 0);
    if (length && p->payload == INVALID_ADDRESS) {
        deallocate(gh, p, sizeof(*p));
        HEAP_UNLOCK();
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
    HEAP_UNLOCK();
    return p;
}

void pbuf_ref(struct pbuf *p) { HEAP_LOCK(); p->ref++; HEAP_UNLOCK(); }

u8 pbuf_free(struct pbuf *p)
{
    u8 freed = 0;
    HEAP_LOCK();
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
    HEAP_UNLOCK();
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
static int g_input_hold;        /* 1 = lwIP keeps the pbuf (models a held ref) */
static int g_input_err;         /* 1 = input returns !=ERR_OK (driver frees) */
static struct pbuf *g_last_input;  /* last pbuf handed to input (for inspection) */
static u16 g_last_input_totlen;    /* its tot_len, captured before any free */
static u16 g_last_input_napi;      /* its napi_id, captured before any free */
static err_t harness_input(struct pbuf *p, struct netif *inp)
{
    rx_delivered++;
    g_last_input = p;
    g_last_input_totlen = p->tot_len;
    g_last_input_napi = p->napi_id;
    if (g_input_err)
        return ERR_MEM;         /* the stack rejects it; the driver frees p */
    if (!g_input_hold)
        pbuf_free(p);           /* the stack consumes it */
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
static struct gve_adminq_command *g_adminq;  /* the device's view of the ring */
static u32  g_adminq_mask;
static u32  g_evt_cnt;                 /* adminq event counter */
static u32  g_max_tx = 8, g_max_rx = 8;
static u16  g_desc_opts[8];           /* device options to advertise */
static int  g_desc_nopts;
static u32  g_adminq_fail_op;          /* opcode to fail (0 = none) */
static int  g_adminq_no_answer;        /* 1 = device never writes a status */
static u32  g_rss_lut[128];           /* captured from CONFIGURE_RSS */
static u16  g_rss_lut_size;
static u8   g_rss_key[40];            /* captured Toeplitz key */
static u16  g_rss_key_size;
static u16  g_rss_hash_types;
static u8   g_rss_hash_alg;
static u32  g_dev_status;             /* value returned for DEVICE_STATUS */

/* Process one admin-queue command: build any response the driver reads back,
 * then mark it passed.  Enough commands are handled to drive the full
 * lifecycle (describe / configure-resources / create-queues / rss / page
 * lists / teardown). */
static void adminq_process(struct gve_adminq_command *cmd)
{
    u32 op = be32toh(cmd->opcode);
    switch (op) {
    case GVE_ADMINQ_DESCRIBE_DEVICE: {
        struct gve_device_descriptor *d =
            pointer_from_u64(be64toh(cmd->describe_device.device_descriptor_addr));
        zero(d, PAGESIZE);
        d->mtu              = htobe16(1460);
        d->counters         = htobe16(16);
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
        break;
    }
    case GVE_ADMINQ_CREATE_TX_QUEUE: {
        struct gve_queue_resources *q =
            pointer_from_u64(be64toh(cmd->create_tx_queue.queue_resources_addr));
        u32 qid = be32toh(cmd->create_tx_queue.queue_id);
        q->db_index      = htobe32(qid);
        q->counter_index = htobe32(qid);
        break;
    }
    case GVE_ADMINQ_CREATE_RX_QUEUE: {
        struct gve_queue_resources *q =
            pointer_from_u64(be64toh(cmd->create_rx_queue.queue_resources_addr));
        u32 qid = be32toh(cmd->create_rx_queue.queue_id);
        q->db_index      = htobe32(8 + qid);
        q->counter_index = htobe32(8 + qid);
        break;
    }
    case GVE_ADMINQ_CONFIGURE_RSS: {
        u32 *lut = pointer_from_u64(be64toh(cmd->configure_rss.hash_lut_addr));
        u16 n = be16toh(cmd->configure_rss.hash_lut_size);
        g_rss_lut_size = n;
        for (u16 i = 0; i < n && i < 128; i++)
            g_rss_lut[i] = be32toh(lut[i]);
        g_rss_hash_types = be16toh(cmd->configure_rss.hash_types);
        g_rss_hash_alg   = cmd->configure_rss.hash_alg;
        g_rss_key_size   = be16toh(cmd->configure_rss.hash_key_size);
        u8 *key = pointer_from_u64(be64toh(cmd->configure_rss.hash_key_addr));
        for (u16 i = 0; i < g_rss_key_size && i < sizeof(g_rss_key); i++)
            g_rss_key[i] = key[i];
        break;
    }
    default:
        break;   /* verify / cfg-resources / ptype / page-list / destroy */
    }
    if (op == g_adminq_fail_op)
        cmd->status = htobe32(GVE_ADMINQ_COMMAND_ERROR_ABORTED);
    else
        cmd->status = htobe32(GVE_ADMINQ_COMMAND_PASSED);
}

/* ---- lifecycle link/stub glue (only the full-lifecycle scenario uses it) ---- */
timerqueue kernel_timers;
static pci_probe g_probe;            /* recorded by register_pci_driver */
static struct netif *g_life_netif;   /* captured by netif_add */

void register_pci_driver(pci_probe p, pci_remove remove) { g_probe = p; }
static int g_msix_avail = 64;             /* vectors pci_enable_msix reports */
int  pci_enable_msix(pci_dev dev) { return g_msix_avail; }
static int g_msix_setup_fail_after = -1;  /* >=0: fail that many calls in */
static int g_msix_slots[2 * GVE_MAX_IO_QUEUES + 1];  /* slots requested */
static int g_msix_nslots;
u64  pci_setup_msix_aff(pci_dev dev, int s, thunk h, sstring n, range a)
{
    if (g_msix_setup_fail_after == 0)
        return INVALID_PHYSICAL;
    if (g_msix_setup_fail_after > 0)
        g_msix_setup_fail_after--;
    if (g_msix_nslots < (int)(sizeof(g_msix_slots) / sizeof(g_msix_slots[0])))
        g_msix_slots[g_msix_nslots++] = s;
    return 0;
}
void pci_teardown_msix(pci_dev dev, int msi_slot) { }
void pci_disable_msix(pci_dev dev) { }
void pci_bar_deinit(struct pci_bar *b) { }
void pci_enable_io_and_memory(pci_dev dev) { }
u16  pci_get_vendor(pci_dev dev) { return PCI_VENDOR_ID_GOOGLE; }
u16  pci_get_device(pci_dev dev) { return PCI_DEV_ID_GVNIC; }

void pci_bar_init(pci_dev dev, struct pci_bar *b, int bar, bytes o, bytes l)
{
    b->vaddr = pointer_from_u64(0x10000 + bar);
    if (bar == GVE_REGISTER_BAR) {
        g_reg_bar = b;             /* route adminq register access here */
        /* capture the ring: the PFN write is lossy on a 64-bit host (the
         * physical address does not fit u32 >> PAGELOG), so recover the
         * adapter from its embedded reg_bar instead.  adapter->adminq is
         * already allocated by this point in gve_init. */
        gve adapter = (gve)((u8 *)b - offsetof(gve, reg_bar));
        g_adminq = adapter->adminq;
        g_adminq_mask = PAGESIZE / sizeof(struct gve_adminq_command) - 1;
    }
}

struct netif *netif_add(struct netif *netif, const void *ip, const void *nm,
                        const void *gw, void *state, netif_init_fn init,
                        netif_input_fn input)
{
    netif->state = state;
    netif->input = input;
    g_life_netif = netif;
    if (init) init(netif);
    return netif;
}

u32 pci_bar_read_4(struct pci_bar *b, u64 offset)
{
    if (b == g_reg_bar) {
        switch (offset) {
        case GVE_REG_ADMINQ_EVT_CNT: return htobe32(g_evt_cnt);
        case GVE_REG_MAX_TX_QUEUES:  return htobe32(g_max_tx);
        case GVE_REG_MAX_RX_QUEUES:  return htobe32(g_max_rx);
        case GVE_REG_DEVICE_STATUS:  return g_dev_status;
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
            /* A device that never replies neither processes the command nor
             * advances the event counter, so gve_adminq_wait polls the
             * unchanged counter to its retry limit and times out. */
            if (!g_adminq_no_answer) {
                adminq_process(&g_adminq[(d - 1) & g_adminq_mask]);
                g_evt_cnt = d;
            }
        }
        return;
    }
    /* Doorbell BAR — telemetry only.  Atomic so concurrent service/drain
     * threads in the TSan harness don't race on this harness-side field
     * (it is not driver state). */
    __atomic_store_n(&the_dev.last_tx_doorbell, val, __ATOMIC_RELAXED);
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

/* Device RX cursors, per queue (indexed by rx->idx) so multiple RX queues
 * can be driven independently. */
static u32 rx_buf_cursor[GVE_MAX_IO_QUEUES];    /* next buf_ring entry to consume */
static u32 rx_compl_cursor[GVE_MAX_IO_QUEUES];  /* next compl_ring entry to write */
static u8  rx_compl_gen[GVE_MAX_IO_QUEUES];     /* device generation bit */

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
    rx->idx = (u16)(rx - adapter->rx_dqo);   /* array position = queue index */
    rx->ctx_head = NULL;
    rx->drop_pkt = false;
    rx->db_head = 0;
    rx->first_interrupt = false;
    rx->no_interrupt_event_cnt = 0;
    rx->empty_rx_queue = 0;
    zero(&rx->rx_stats, sizeof(rx->rx_stats));
    gve_rx_dqo_init(rx);

    rx_buf_cursor[rx->idx] = rx_compl_cursor[rx->idx] = 0;
    rx_compl_gen[rx->idx] = 1;
}

static void make_rx_dqo(gve adapter, gve_rx_dqo_queue rx, u16 num_bufs)
{
    make_rx_dqo_mode(adapter, rx, num_bufs, false);
}

/* Device consumes the next posted buffer and writes a completion for it. */
static void dqo_dev_rx_complete(gve_rx_dqo_queue rx, u16 len, boolean eop,
                                boolean err)
{
    u16 q = rx->idx;
    u16 buf_id = rx->buf_ring[rx_buf_cursor[q] & rx->mask].buf_id;
    rx_buf_cursor[q]++;
    /* DQO-QPL: the device writes the payload into the registered slot the
     * driver will copy out. */
    if (rx->adapter->dqo_qpl && !err && len) {
        u8 *dst = (u8 *)rx->qpl_base + (u64)buf_id * GVE_DQO_BUF_SIZE;
        for (u16 i = 0; i < len; i++) dst[i] = (u8)(i + 7);
    }
    u32 slot = rx_compl_cursor[q] & rx->mask;
    struct gve_rx_compl_desc_dqo *c = &rx->compl_ring[slot];
    c->buf_id   = buf_id;
    c->err_flags = err ? GVE_DQO_RX_ERR : 0;
    c->status0  = eop ? GVE_DQO_RX_EOP : 0;
    write_barrier();
    c->pkt_len_gen = (u16)(len & GVE_DQO_RX_PKT_LEN_MASK) |
                     (rx_compl_gen[q] ? GVE_DQO_RX_GEN : 0);
    rx_compl_cursor[q]++;
    if ((rx_compl_cursor[q] & rx->mask) == 0)
        rx_compl_gen[q] ^= 1;
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
    adapter->tx_desc_cnt = desc_cnt;   /* the GQI space check uses this */
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
static boolean gqi_dev_in_packet;   /* device-side: currently mid-packet? */

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
    gqi_dev_in_packet = false;
}

/* GQI device: write packet data into the posted QPL slot and a descriptor,
 * then advance the event counter.  flags carries ERR / PKT_CONT (be16).
 * Real GQI prepends the 2-byte IP-alignment pad on the FIRST buffer of a
 * packet only (Google gve_rx.c), so the model pads only when starting a new
 * packet; pkt_len is the data length the device delivers in this buffer. */
static void gqi_dev_rx_complete(gve_rx_queue rx, u16 pkt_len, u16 flags)
{
    u32 slot = gqi_rx_cursor & rx->mask;
    u64 offset = be64toh(rx->data[slot]) - rx->rda_base_phys;
    u16 pad = gqi_dev_in_packet ? 0 : GVE_RX_PADDING;   /* pad first buffer only */
    u8 *dst = (u8 *)rx->qpl_base + offset + pad;
    for (u16 i = 0; i < pkt_len; i++) dst[i] = (u8)(i + 1);
    rx->desc[slot].len = htobe16(pkt_len + pad);
    rx->desc[slot].flags_seq = flags;
    /* PKT_CONT means more buffers follow -> stay in-packet; otherwise the
     * next buffer starts a fresh packet. */
    gqi_dev_in_packet = !!(flags & GVE_RXF_PKT_CONT);
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
    g_adminq = adapter->adminq;
    g_adminq_mask = adapter->adminq_mask;
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

/* ------------------------------------------------------------------ */
/* Full lifecycle: init_gve -> probe -> setup -> watchdog -> reset      */
/* ------------------------------------------------------------------ */

/* Scenario 24: drive the real bring-up end to end, then fire the watchdog
 * and let it reset.  This is the only scenario that runs gve_main.c's
 * probe/setup/watchdog/reset (their closures can only be initialised by the
 * driver's own lifecycle, not from the harness). */
static void scenario_lifecycle(void)
{
    rprintf("scenario_lifecycle\n");

    /* device presents a DQO-RDA option, 8 max queues, plenty of MSI-X */
    g_desc_nopts = 1;
    g_desc_opts[0] = GVE_DEV_OPT_ID_DQO_RDA;
    g_max_tx = g_max_rx = 8;
    g_msix = 64;
    int saved_tp = total_processors;
    total_processors = 2;
    g_adminq = NULL;

    /* init_gve registers the probe; apply it with a fake gVNIC device */
    init_gve((kernel_heaps)0);
    CHECK(g_probe != 0, "init_gve registered a probe");
    boolean ok = apply(g_probe, (pci_dev)pointer_from_u64(0x42));
    CHECK(ok, "probe matched and brought the device up to netif_add");

    gve adapter = g_life_netif->state;
    CHECK(adapter->dqo && !adapter->dqo_qpl, "negotiated DQO-RDA");

    /* net.c applies the setup closure once the config tuple is available */
    boolean up = apply((netif_dev_setup)&adapter->ndev.setup, (tuple)0);
    CHECK(up, "setup completed (queues, interrupts, RSS, watchdog)");
    CHECK((adapter->flags >> GVE_FLAG_DEVICE_RUNNING) & 1, "DEVICE_RUNNING set");
    CHECK(adapter->num_queues == 2, "queue count capped by cpus (2), got %d",
          adapter->num_queues);

    /* a healthy tick must NOT reset */
    u64 wd0 = adapter->dev_stats.wd_expired;
    apply((timer_handler)&adapter->watchdog_task, (u64)0, (u64)1);
    CHECK(adapter->dev_stats.wd_expired == wd0, "healthy watchdog tick: no reset");

    /* a MISS that never reinjects: the watchdog must reset the adapter */
    gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
    tx->miss_times[5] = now(CLOCK_ID_MONOTONIC) -
                        milliseconds(GVE_TX_WATCHDOG_MS + 1000);
    u64 wd_before = adapter->dev_stats.wd_expired;

    apply((timer_handler)&adapter->watchdog_task, (u64)0, (u64)1);

    CHECK(adapter->dev_stats.wd_expired == wd_before + 1,
          "watchdog detected the stuck queue and counted a reset");
    CHECK((adapter->flags >> GVE_FLAG_DEVICE_RUNNING) & 1,
          "adapter is RUNNING again after the reset recreated the queues");
    CHECK(!adapter->tx_dqo[0].stuck, "recreated queue is not stuck");

    total_processors = saved_tp;
}

/* Scenario 25: GQI-QPL lifecycle — the no-option fallback registers page
 * lists at create; an explicit reset unregisters and recreates them. */
static void scenario_lifecycle_gqi_qpl(void)
{
    rprintf("scenario_lifecycle_gqi_qpl\n");
    g_desc_nopts = 0;                 /* no options -> GQI-QPL fallback */
    g_max_tx = g_max_rx = 8;
    g_msix = 64;
    int saved_tp = total_processors;
    total_processors = 1;
    g_adminq = NULL; g_probe = 0; g_life_netif = 0;

    init_gve((kernel_heaps)0);
    boolean ok = apply(g_probe, (pci_dev)pointer_from_u64(0x42));
    CHECK(ok, "gqi-qpl probe up to netif_add");
    gve adapter = g_life_netif->state;
    CHECK(!adapter->dqo && !adapter->raw_addressing, "GQI-QPL fallback selected");

    boolean up = apply((netif_dev_setup)&adapter->ndev.setup, (tuple)0);
    CHECK(up, "gqi-qpl setup: page lists registered, queues created");
    CHECK((adapter->flags >> GVE_FLAG_DEVICE_RUNNING) & 1, "running");

    /* explicit reset: teardown unregisters the page lists, recreate
     * re-registers them — exercise the QPL teardown/recreate path. */
    gve_trigger_reset(adapter);
    CHECK((adapter->flags >> GVE_FLAG_DEVICE_RUNNING) & 1,
          "running again after the QPL reset");
    CHECK(!((adapter->flags >> GVE_FLAG_RESETTING) & 1), "reset flag cleared");

    total_processors = saved_tp;
}

/* complete and retire every outstanding single-segment packet on a DQO TX
 * queue (re-points the device completion ring at this queue first). */
static void drain_tx_dqo_all(gve_tx_dqo_queue tx)
{
    the_dev.tx_compl = tx->compl;
    the_dev.tx_compl_mask = tx->mask;
    the_dev.tx_compl_tail = tx->compl_head;
    the_dev.tx_compl_gen = tx->expected_gen;
    while (tx->desc_tail != tx->head) {
        u16 tag = tx->desc[(tx->desc_tail + 1) & tx->mask].compl_tag;
        dqo_dev_complete_pkt(&the_dev, tag, GVE_DQO_COMPL_TYPE_PKT);
        run_cleanup(tx);
    }
}

/* Scenario 26: per-CPU TX queue dispatch — a packet from CPU n goes to
 * queue n % num_queues, and the queues stay independent (own tags). */
static void scenario_multiqueue_dispatch(void)
{
    rprintf("scenario_multiqueue_dispatch\n");
    gve adapter = make_adapter();
    adapter->num_queues = 4;
    for (int q = 0; q < 4; q++)
        make_tx_dqo(adapter, &adapter->tx_dqo[q], 256);
    int live_before = pbufs_live;

    struct pbuf *kept[8];
    for (int cpu = 0; cpu < 8; cpu++) {
        set_cpu(cpu);
        struct pbuf *p = make_pkt(64);
        kept[cpu] = p;
        u32 q = cpu % 4;
        u32 head_before = adapter->tx_dqo[q].head;
        gve_linkoutput_dqo(&adapter->ndev.n, p);
        CHECK(adapter->tx_dqo[q].head == head_before + 2,
              "cpu %d -> queue %d (head advanced)", cpu, q);
    }
    set_cpu(0);
    for (int q = 0; q < 4; q++)
        CHECK(adapter->tx_dqo[q].head == 4,
              "queue %d received exactly 2 packets, got %d",
              q, adapter->tx_dqo[q].head);
    /* isolation: each queue used its own tag pool (tags 0 and 1) */
    for (int q = 0; q < 4; q++)
        CHECK(adapter->tx_dqo[q].tags_ntu == 2, "queue %d took 2 tags", q);

    for (int q = 0; q < 4; q++)
        drain_tx_dqo_all(&adapter->tx_dqo[q]);
    for (int cpu = 0; cpu < 8; cpu++)
        pbuf_free(kept[cpu]);
    CHECK(pbufs_live == live_before, "no leak across queues");
}

/* Scenario 27: the RSS indirection table is built round-robin over the RX
 * queues (the driver's contribution to multi-queue RX; the device's actual
 * Toeplitz steering is a hardware behaviour, measured on real GCP HW). */
static void scenario_multiqueue_rss(void)
{
    rprintf("scenario_multiqueue_rss\n");
    gve adapter = make_adapter_common();
    adapter->contiguous = gh;
    adapter->num_queues = 6;
    adapter->rss_supported = true;
    adapter->adminq = allocate(gh, PAGESIZE);
    adapter->adminq_mask = PAGESIZE / sizeof(struct gve_adminq_command) - 1;
    adapter->adminq_head = 0;
    adapter->adminq_running = true;
    adapter->reg_bar.vaddr = pointer_from_u64(0x1000);
    g_reg_bar = &adapter->reg_bar;
    g_adminq = adapter->adminq;
    g_adminq_mask = adapter->adminq_mask;
    g_evt_cnt = 0;
    g_rss_lut_size = 0;

    boolean ok = gve_configure_rss(adapter);
    CHECK(ok, "configure_rss succeeded");
    CHECK(g_rss_lut_size == GVE_RSS_INDIR_SIZE, "LUT size %d", GVE_RSS_INDIR_SIZE);
    boolean roundrobin = true;
    for (int i = 0; i < g_rss_lut_size; i++)
        if (g_rss_lut[i] != (u32)(i % 6)) roundrobin = false;
    CHECK(roundrobin, "LUT spreads round-robin over the 6 RX queues");
}

/* Scenario 28: GQI RX error and drop-to-EOP paths. */
static void scenario_gqi_rx_errors(void)
{
    rprintf("scenario_gqi_rx_errors\n");
    gve adapter = make_adapter_gqi(true);
    gve_rx_queue rx = &adapter->rx[0];
    make_rx_gqi(adapter, rx, 64);
    int delivered_before = rx_delivered;

    /* single-buffer error packet -> dropped */
    gqi_dev_rx_complete(rx, 100, GVE_RXF_ERR);
    /* errored fragment mid-chain -> drop the rest to EOP */
    gqi_dev_rx_complete(rx, 1400, GVE_RXF_ERR | GVE_RXF_PKT_CONT);
    gqi_dev_rx_complete(rx, 600, 0);                /* EOP of dropped chain */
    /* a clean packet still delivered afterwards */
    gqi_dev_rx_complete(rx, 120, 0);
    apply((thunk)&rx->service);

    CHECK(rx_delivered == delivered_before + 1, "only the clean packet delivered");
    CHECK(rx->rx_stats.rx_dropped >= 2, "errored packets counted dropped");
    CHECK(!rx->drop_pkt, "drop state cleared at EOP");
}

/* Scenario 30: GQI-QPL TX byte-FIFO wrap (qpl_head wraps around qpl_size). */
static void scenario_gqi_qpl_wrap(void)
{
    rprintf("scenario_gqi_qpl_wrap\n");
    gve adapter = make_adapter_gqi(false);     /* QPL */
    gve_tx_queue tx = &adapter->tx[0];
    make_tx_gqi(adapter, tx, 256);
    gve_setup_linkoutput(adapter, &adapter->ndev.n);
    int live_before = pbufs_live;

    /* send enough bytes (large packets) to wrap the QPL byte FIFO twice */
    u32 sent = 0;
    for (int i = 0; i < 4 * (int)(tx->qpl_size / 2048); i++) {
        struct pbuf *p = make_pkt(2000);
        adapter->ndev.n.linkoutput(&adapter->ndev.n, p);
        gqi_dev_tx_complete_all(adapter, tx);
        run_tx_gqi_cleanup(tx);
        pbuf_free(p);            /* lwIP's ref (QPL freed the driver's inline) */
        sent++;
    }
    CHECK(sent > 0 && tx->qpl_head < tx->qpl_size, "qpl_head wrapped and stayed in range");
    CHECK(tx->qpl_used == 0, "all QPL bytes released after completions");
    CHECK(pbufs_live == live_before, "no leak across the QPL wrap");
}

/* Scenario 31: GQI TX backpressure (ring fills, queue stops, then resumes). */
static void scenario_gqi_tx_backpressure(void)
{
    rprintf("scenario_gqi_tx_backpressure\n");
    gve adapter = make_adapter_gqi(true);
    gve_tx_queue tx = &adapter->tx[0];
    make_tx_gqi(adapter, tx, 16);
    gve_setup_linkoutput(adapter, &adapter->ndev.n);
    int live_before = pbufs_live;

    struct pbuf *kept[64]; int n = 0;
    while (tx->running && n < 64) {
        struct pbuf *p = make_pkt(64);
        kept[n++] = p;
        adapter->ndev.n.linkoutput(&adapter->ndev.n, p);
    }
    CHECK(!tx->running, "GQI queue stopped under backpressure");
    CHECK(tx->tx_stats.queue_stop >= 1, "queue_stop counted");

    gqi_dev_tx_complete_all(adapter, tx);       /* complete what's posted */
    apply((thunk)&tx->enqueue_task);            /* wakeup + drain held pkt */
    CHECK(tx->running, "GQI queue resumed");
    CHECK(tx->tx_stats.queue_wakeup >= 1, "queue_wakeup counted");

    /* drain fully */
    while (tx->tail != tx->head) {
        gqi_dev_tx_complete_all(adapter, tx);
        apply((thunk)&tx->enqueue_task);
    }
    for (int i = 0; i < n; i++) pbuf_free(kept[i]);
    CHECK(pbufs_live == live_before, "no leak (gqi backpressure)");
}

/* Bring an adapter up through the real lifecycle and return it. */
static gve bring_up(u16 opt, int ncpu)
{
    g_desc_nopts = opt ? 1 : 0;
    g_desc_opts[0] = opt;
    g_max_tx = g_max_rx = 8;
    g_msix = 64;
    total_processors = ncpu;
    g_adminq = NULL; g_probe = 0; g_life_netif = 0;
    init_gve((kernel_heaps)0);
    apply(g_probe, (pci_dev)pointer_from_u64(0x42));
    gve adapter = g_life_netif->state;
    apply((netif_dev_setup)&adapter->ndev.setup, (tuple)0);
    return adapter;
}

/* Scenario 32: management interrupt (link up/down, device-requested reset)
 * and the RX interrupt trampoline. */
static void scenario_main_irqs(void)
{
    rprintf("scenario_main_irqs\n");
    int saved_tp = total_processors;
    gve adapter = bring_up(GVE_DEV_OPT_ID_DQO_RDA, 2);
    struct netif *nif = &adapter->ndev.n;

    g_dev_status = GVE_DEVICE_STATUS_LINK_STATUS;       /* link up */
    apply((thunk)&adapter->mgmt_irq_handler);
    CHECK(nif->flags & NETIF_FLAG_UP, "mgmt IRQ brought the link up");

    g_dev_status = 0;                                   /* link down */
    apply((thunk)&adapter->mgmt_irq_handler);
    CHECK(!(nif->flags & NETIF_FLAG_UP), "mgmt IRQ took the link down");

    /* device-requested reset (live migration): mgmt IRQ schedules a reset */
    g_dev_status = GVE_DEVICE_STATUS_RESET;
    apply((thunk)&adapter->mgmt_irq_handler);
    g_dev_status = 0;
    CHECK((adapter->flags >> GVE_FLAG_DEVICE_RUNNING) & 1,
          "running again after a device-requested reset");

    /* RX interrupt trampoline marks first_interrupt and runs the service */
    adapter->rx_dqo[0].first_interrupt = false;
    apply((thunk)&adapter->rx_dqo[0].irq_handler);
    CHECK(adapter->rx_dqo[0].first_interrupt, "RX IRQ set first_interrupt");

    total_processors = saved_tp;
}

/* Scenario 33: the GQI watchdog branch (a healthy tick on a GQI adapter). */
static void scenario_gqi_watchdog(void)
{
    rprintf("scenario_gqi_watchdog\n");
    int saved_tp = total_processors;
    gve adapter = bring_up(0, 1);          /* GQI-QPL fallback, 1 queue */
    CHECK(!adapter->dqo, "GQI adapter for the GQI watchdog path");
    u64 wd0 = adapter->dev_stats.wd_expired;
    apply((timer_handler)&adapter->watchdog_task, (u64)0, (u64)1);
    CHECK(adapter->dev_stats.wd_expired == wd0, "healthy GQI watchdog: no reset");

    /* GQI RX interrupt trampoline: marks first_interrupt, runs the service. */
    adapter->rx[0].first_interrupt = false;
    apply((thunk)&adapter->rx[0].irq_handler);
    CHECK(adapter->rx[0].first_interrupt, "GQI RX IRQ set first_interrupt");

    total_processors = saved_tp;
}

/* Scenario 34: watchdog stuck-TX and no-RX-interrupt detection. */
static void scenario_watchdog_detect(void)
{
    rprintf("scenario_watchdog_detect\n");
    int saved_tp = total_processors;

    /* no-interrupt RX: a non-empty completion ring with first_interrupt
     * never set makes the watchdog count missed interrupts. */
    gve a = bring_up(GVE_DEV_OPT_ID_DQO_RDA, 1);
    gve_rx_dqo_queue rx = &a->rx_dqo[0];
    u32 slot = rx->compl_head & rx->mask;
    rx->compl_ring[slot].pkt_len_gen =
        rx->expected_gen ? GVE_DQO_RX_GEN : 0;     /* CQ appears non-empty */
    for (int t = 0; t < GVE_MAX_NO_INTERRUPT_ITERATIONS; t++) {
        rx->first_interrupt = false;
        a->next_monitored_tx_qid = 0;
        apply((timer_handler)&a->watchdog_task, (u64)0, (u64)1);
    }
    CHECK(rx->no_interrupt_event_cnt >= 2, "watchdog counted missed RX interrupts");

    /* stuck-TX: more than the threshold of packets timed out -> reset. */
    gve b = bring_up(GVE_DEV_OPT_ID_DQO_RDA, 1);
    gve_tx_dqo_queue tx = &b->tx_dqo[0];
    timestamp old = now(CLOCK_ID_MONOTONIC) - milliseconds(GVE_TX_WATCHDOG_MS + 1000);
    for (int i = 0; i <= GVE_TX_STUCK_THRESHOLD + 1; i++)
        tx->tx_timestamps[i] = old;
    u64 wd0 = b->dev_stats.wd_expired;
    b->next_monitored_tx_qid = 0;
    apply((timer_handler)&b->watchdog_task, (u64)0, (u64)1);
    CHECK(b->dev_stats.wd_expired == wd0 + 1, "stuck-TX threshold -> reset");

    total_processors = saved_tp;
}

/* Bring an adapter through probe (describe) + queue-count + cfg-resources,
 * stopping BEFORE setup_queues so a failing heap can be injected there. */
static gve bring_up_to_cfg(u16 opt, int ncpu)
{
    g_desc_nopts = opt ? 1 : 0;
    g_desc_opts[0] = opt;
    g_max_tx = g_max_rx = 8;
    g_msix = 64;
    total_processors = ncpu;
    g_adminq = NULL; g_probe = 0; g_life_netif = 0;
    init_gve((kernel_heaps)0);
    apply(g_probe, (pci_dev)pointer_from_u64(0x42));
    gve adapter = g_life_netif->state;
    adapter->num_queues = gve_calc_num_queues(adapter, (tuple)0);
    gve_cfg_device_resources(adapter);
    return adapter;
}

/* Scenario 35: queue setup under allocation failure — the create-queue
 * error cascades and the ring-size backoff (the driver halves and retries
 * down to GVE_MIN_RING_SIZE, then gives up). */
static void scenario_setup_alloc_fail(void)
{
    rprintf("scenario_setup_alloc_fail\n");
    int saved_tp = total_processors;

    gve a = bring_up_to_cfg(GVE_DEV_OPT_ID_DQO_RDA, 2);
    a->contiguous = fail_heap();
    a->general = fail_heap();
    g_alloc_fail_after = 6;            /* fail mid first-queue create */
    CHECK(!gve_setup_queues(a), "DQO setup fails (early alloc failure)");
    g_alloc_fail_after = 20;           /* create one queue, fail the next */
    CHECK(!gve_setup_queues(a), "DQO setup fails (inter-queue failure)");
    g_alloc_fail_after = -1;
    a->contiguous = gh; a->general = gh;

    gve b = bring_up_to_cfg(0, 1);     /* GQI-QPL fallback */
    b->contiguous = fail_heap();
    b->general = fail_heap();
    g_alloc_fail_after = 4;
    CHECK(!gve_setup_queues(b), "GQI-QPL setup fails under alloc failure");
    g_alloc_fail_after = -1;
    b->contiguous = gh; b->general = gh;

    /* Failed reset: a full adapter is reset, but the re-create allocation
     * fails (cfg succeeds, setup_queues does not), so the reset takes its
     * failure epilogue — DEVICE_RUNNING cleared, RESETTING/ONGOING_RESET
     * left set (the adapter is dead until restart). */
    gve c = bring_up(GVE_DEV_OPT_ID_DQO_RDA, 1);
    c->contiguous = fail_heap();
    c->general = fail_heap();
    g_alloc_fail_after = 8;            /* cfg+ptype pass, setup_queues fails */
    gve_trigger_reset(c);              /* async_apply_bh runs synchronously */
    g_alloc_fail_after = -1;
    c->contiguous = gh; c->general = gh;
    CHECK(!(c->flags & (1ULL << GVE_FLAG_DEVICE_RUNNING)),
          "failed reset clears DEVICE_RUNNING");
    CHECK(c->flags & (1ULL << GVE_FLAG_ONGOING_RESET),
          "failed reset leaves ONGOING_RESET set (adapter dead)");

    total_processors = saved_tp;
}

/* Bring an adapter through probe (gve_init: describe) only, so the netif
 * setup closure can be applied by hand with failures injected. */
static gve bring_up_probe_only(u16 opt, int ncpu)
{
    g_desc_nopts = opt ? 1 : 0;
    g_desc_opts[0] = opt;
    g_max_tx = g_max_rx = 8;
    g_msix = 64; g_msix_avail = 64; g_msix_setup_fail_after = -1;
    total_processors = ncpu;
    g_adminq = NULL; g_probe = 0; g_life_netif = 0;
    init_gve((kernel_heaps)0);
    apply(g_probe, (pci_dev)pointer_from_u64(0x42));
    return g_life_netif->state;
}

/* Scenario 35b: gve_setup failure paths — cfg failure, interrupt-init
 * failure (too few vectors, mgmt/RX MSI-X setup failure), queue setup
 * failure (the err_intr deinit path), plus the success tail with the link
 * up and a device-requested reset caught during bring-up. */
static void scenario_setup_failpaths(void)
{
    rprintf("scenario_setup_failpaths\n");
    int saved_tp = total_processors;
    u32 saved_status = g_dev_status;

    /* cfg fails: the device rejects CONFIGURE_DEVICE_RESOURCES (covers the
     * cfg cleanup path that deallocates both arrays). */
    gve a = bring_up_probe_only(GVE_DEV_OPT_ID_DQO_RDA, 1);
    g_adminq_fail_op = GVE_ADMINQ_CONFIGURE_DEVICE_RESOURCES;
    CHECK(!apply((netif_dev_setup)&a->ndev.setup, (tuple)0),
          "setup fails when the device rejects cfg-resources");
    g_adminq_fail_op = 0;

    /* too few MSI-X vectors. */
    gve b = bring_up_probe_only(GVE_DEV_OPT_ID_DQO_RDA, 1);
    g_msix_avail = 2;                  /* need 2*nq+1 = 3 */
    CHECK(!apply((netif_dev_setup)&b->ndev.setup, (tuple)0),
          "setup fails with too few MSI-X vectors");
    g_msix_avail = 64;

    /* mgmt MSI-X setup fails (first pci_setup_msix call). */
    gve c = bring_up_probe_only(GVE_DEV_OPT_ID_DQO_RDA, 1);
    g_msix_setup_fail_after = 0;
    CHECK(!apply((netif_dev_setup)&c->ndev.setup, (tuple)0),
          "setup fails when mgmt MSI-X setup fails");
    g_msix_setup_fail_after = -1;

    /* RX MSI-X setup fails (mgmt ok, first RX slot fails -> teardown). */
    gve d = bring_up_probe_only(GVE_DEV_OPT_ID_DQO_RDA, 1);
    g_msix_setup_fail_after = 1;
    CHECK(!apply((netif_dev_setup)&d->ndev.setup, (tuple)0),
          "setup fails when RX MSI-X setup fails");
    g_msix_setup_fail_after = -1;

    /* queue setup fails after interrupts are up -> err_intr deinit path. */
    gve e = bring_up_probe_only(GVE_DEV_OPT_ID_DQO_RDA, 1);
    e->contiguous = fail_heap(); e->general = fail_heap();
    g_alloc_fail_after = 12;           /* cfg+ptype pass, queues fail */
    CHECK(!apply((netif_dev_setup)&e->ndev.setup, (tuple)0),
          "setup fails at queue creation (deinit interrupts path)");
    g_alloc_fail_after = -1; e->contiguous = gh; e->general = gh;

    /* success tail: link up and a device-requested reset during setup. */
    gve f = bring_up_probe_only(GVE_DEV_OPT_ID_DQO_RDA, 1);
    g_dev_status = GVE_DEVICE_STATUS_LINK_STATUS | GVE_DEVICE_STATUS_RESET;
    boolean ok = apply((netif_dev_setup)&f->ndev.setup, (tuple)0);
    g_dev_status = saved_status;
    CHECK(ok, "setup succeeds with link up + device-requested reset");
    CHECK(f->flags & (1ULL << GVE_FLAG_DEVICE_RUNNING),
          "device-requested-reset-during-setup recovered");

    total_processors = saved_tp;
}

/* Scenario 36: watchdog branches not reached by the healthy/stuck DQO
 * cases — the per-tick TX wakeup and RX empty-ring kicks, the
 * first_interrupt-seen reset, the GQI stuck-TX and no-interrupt resets, and
 * the DQO miss-reinject timeout that frees the pbuf and resets. */
static void scenario_watchdog_branches(void)
{
    rprintf("scenario_watchdog_branches\n");
    int saved_tp = total_processors;
    timestamp old = now(CLOCK_ID_MONOTONIC) -
                    milliseconds(GVE_TX_WATCHDOG_MS + 1000);

    /* DQO healthy tick: TX wakeup (full ring free, not running), RX
     * empty-ring kick, and the first_interrupt-seen branch — no reset. */
    gve a = bring_up(GVE_DEV_OPT_ID_DQO_RDA, 1);
    gve_tx_dqo_queue atx = &a->tx_dqo[0];
    gve_rx_dqo_queue arx = &a->rx_dqo[0];
    atx->running = false; atx->stuck = false;
    atx->head = atx->desc_tail = 0;
    arx->first_interrupt = true;
    arx->empty_rx_queue = 3;
    u64 er0 = arx->rx_stats.empty_rx_ring, wd0 = a->dev_stats.wd_expired;
    a->next_monitored_tx_qid = 0;
    apply((timer_handler)&a->watchdog_task, (u64)0, (u64)1);
    CHECK(a->dev_stats.wd_expired == wd0, "DQO healthy watchdog: no reset");
    CHECK(arx->rx_stats.empty_rx_ring == er0 + 1, "DQO watchdog empty-ring kick");
    CHECK(arx->empty_rx_queue == 0 && !arx->first_interrupt,
          "DQO watchdog cleared empty/first-interrupt state");

    /* GQI healthy tick: same wakeup + empty-ring branches. */
    gve b = bring_up(0, 1);
    gve_tx_queue btx = &b->tx[0];
    gve_rx_queue brx = &b->rx[0];
    btx->running = false; btx->stuck = false; btx->head = 0;
    b->event_counters[btx->event_counter_idx] = htobe32(0);
    brx->first_interrupt = true;
    brx->empty_rx_queue = 3;
    u64 ber0 = brx->rx_stats.empty_rx_ring, bwd0 = b->dev_stats.wd_expired;
    b->next_monitored_tx_qid = 0;
    apply((timer_handler)&b->watchdog_task, (u64)0, (u64)1);
    CHECK(b->dev_stats.wd_expired == bwd0, "GQI healthy watchdog: no reset");
    CHECK(brx->rx_stats.empty_rx_ring == ber0 + 1, "GQI watchdog empty-ring kick");

    /* GQI stuck-TX over threshold -> reset. */
    gve c = bring_up(0, 1);
    gve_tx_queue ctx = &c->tx[0];
    ctx->tail = 0; ctx->head = GVE_TX_STUCK_THRESHOLD + 2;
    for (u32 i = 0; i < ctx->head; i++)
        ctx->tx_timestamps[i & ctx->mask] = old;
    u64 cwd0 = c->dev_stats.wd_expired;
    c->next_monitored_tx_qid = 0;
    apply((timer_handler)&c->watchdog_task, (u64)0, (u64)1);
    CHECK(c->dev_stats.wd_expired == cwd0 + 1, "GQI stuck-TX threshold -> reset");

    /* DQO miss-reinject timeout: a MISS that never got a REINJECT within
     * the deadline frees the held pbuf and resets. */
    gve d = bring_up(GVE_DEV_OPT_ID_DQO_RDA, 1);
    gve_tx_dqo_queue dtx = &d->tx_dqo[0];
    dtx->pending[3] = make_pkt(100);
    dtx->miss_times[3] = old;
    u64 dwd0 = d->dev_stats.wd_expired;
    d->next_monitored_tx_qid = 0;
    apply((timer_handler)&d->watchdog_task, (u64)0, (u64)1);
    CHECK(d->dev_stats.wd_expired == dwd0 + 1, "DQO miss timeout -> reset");
    CHECK(dtx->miss_times[3] == 0, "DQO watchdog cleared the timed-out miss");

    /* DQO no-interrupt: ticking past GVE_MAX_NO_INTERRUPT_ITERATIONS with a
     * non-empty CQ and no first_interrupt triggers a reset (and recovers). */
    gve e = bring_up(GVE_DEV_OPT_ID_DQO_RDA, 1);
    gve_rx_dqo_queue erx = &e->rx_dqo[0];
    for (int t = 0; t <= GVE_MAX_NO_INTERRUPT_ITERATIONS; t++) {
        erx->first_interrupt = false;
        u32 slot = erx->compl_head & erx->mask;
        erx->compl_ring[slot].pkt_len_gen = erx->expected_gen ? GVE_DQO_RX_GEN : 0;
        e->next_monitored_tx_qid = 0;
        apply((timer_handler)&e->watchdog_task, (u64)0, (u64)1);
    }
    CHECK(e->flags & (1ULL << GVE_FLAG_DEVICE_RUNNING),
          "DQO no-interrupt watchdog reset and recovered");

    /* GQI no-interrupt: event counter past tail, no first_interrupt. */
    gve f = bring_up(0, 1);
    gve_rx_queue frx = &f->rx[0];
    for (int t = 0; t <= GVE_MAX_NO_INTERRUPT_ITERATIONS; t++) {
        frx->first_interrupt = false;
        f->event_counters[frx->event_counter_idx] = htobe32(frx->tail + 1);
        f->next_monitored_tx_qid = 0;
        apply((timer_handler)&f->watchdog_task, (u64)0, (u64)1);
    }
    CHECK(f->flags & (1ULL << GVE_FLAG_DEVICE_RUNNING),
          "GQI no-interrupt watchdog reset and recovered");

    total_processors = saved_tp;
}

/* Scenario 37: DQO TX completions whose tag is not in flight — the
 * alternate-miss, MISS and REINJECT encodings each count a bad tag and are
 * skipped without a reset (the trust-line fix; the PKT-stale encoding is
 * covered by scenario_tx_stale_tag). */
static void scenario_tx_stale_types(void)
{
    rprintf("scenario_tx_stale_types\n");
    gve adapter = make_adapter();
    gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
    make_tx_dqo(adapter, tx, 256);
    u64 reset_before = (adapter->flags >> GVE_FLAG_RESETTING) & 1;
    u16 free_tag = 200;                /* nothing was ever sent on this tag */

    dqo_dev_complete_pkt(&the_dev, free_tag | GVE_DQO_ALT_MISS_COMPL_BIT,
                         GVE_DQO_COMPL_TYPE_PKT);     /* alt-miss, stale */
    dqo_dev_complete_pkt(&the_dev, free_tag, GVE_DQO_COMPL_TYPE_MISS);
    dqo_dev_complete_pkt(&the_dev, free_tag, GVE_DQO_COMPL_TYPE_REINJECT);
    run_cleanup(tx);

    CHECK(tx->tx_stats.bad_compl_tag == 3, "three stale-type tags counted");
    CHECK(tx->compl_head == 3, "ring cursor advanced past all three");
    CHECK(((adapter->flags >> GVE_FLAG_RESETTING) & 1) == reset_before,
          "no reset for stale alt-miss/miss/reinject completions");
}

/* Scenario 38: more than GVE_TX_DOORBELL_BATCH packets drained in one pass
 * cross the mid-batch doorbell write (then the tail doorbell), for both DQO
 * and GQI. */
static void scenario_tx_doorbell_batch(void)
{
    rprintf("scenario_tx_doorbell_batch\n");
    const int N = GVE_TX_DOORBELL_BATCH + 1;   /* 65: forces a mid-batch ring */

    /* DQO-RDA. */
    {
        gve adapter = make_adapter();
        gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
        make_tx_dqo(adapter, tx, 256);
        int live_before = pbufs_live;
        struct pbuf *pkts[N];
        for (int i = 0; i < N; i++) {
            pkts[i] = make_pkt(64);
            pbuf_ref(pkts[i]);              /* the ref linkoutput would take */
            enqueue(tx->br, pkts[i]);
        }
        apply((thunk)&tx->enqueue_task);   /* one drain of all N packets */
        CHECK(tx->tx_stats.doorbells >= 2, "DQO drain rang a mid-batch + tail doorbell");
        CHECK(tx->tx_stats.cnt == (u64)N, "DQO drained all %d packets", N);
        for (u16 t = 0; t <= tx->mask; t++)
            if (tx->seg_counts[t])
                dqo_dev_complete_pkt(&the_dev, t, GVE_DQO_COMPL_TYPE_PKT);
        run_cleanup(tx);
        for (int i = 0; i < N; i++)
            pbuf_free(pkts[i]);
        CHECK(pbufs_live == live_before, "DQO batch: no pbuf leak");
    }

    /* GQI-RDA. */
    {
        gve adapter = make_adapter_gqi(true);
        gve_tx_queue tx = &adapter->tx[0];
        make_tx_gqi(adapter, tx, 256);
        gve_setup_linkoutput(adapter, &adapter->ndev.n);
        int live_before = pbufs_live;
        struct pbuf *pkts[N];
        for (int i = 0; i < N; i++) {
            pkts[i] = make_pkt(64);
            pbuf_ref(pkts[i]);
            enqueue(tx->br, pkts[i]);
        }
        apply((thunk)&tx->enqueue_task);
        CHECK(tx->tx_stats.doorbells >= 2, "GQI drain rang a mid-batch + tail doorbell");
        CHECK(tx->tx_stats.cnt == (u64)N, "GQI drained all %d packets", N);
        gqi_dev_tx_complete_all(adapter, tx);
        run_tx_gqi_cleanup(tx);
        for (int i = 0; i < N; i++)
            pbuf_free(pkts[i]);
        CHECK(pbufs_live == live_before, "GQI batch: no pbuf leak");
    }
}

/* Scenario 39: linkoutput on a full software TX queue returns ERR_MEM and
 * drops the reference it took (lwIP keeps owning the pbuf), for both DQO
 * and GQI. */
static void scenario_tx_enqueue_full(void)
{
    rprintf("scenario_tx_enqueue_full\n");

    /* DQO. */
    {
        gve adapter = make_adapter();
        gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
        make_tx_dqo(adapter, tx, 256);
        int live_before = pbufs_live;
        while (enqueue(tx->br, (void *)tx)) ;   /* fill to capacity */
        struct pbuf *p = make_pkt(100);
        err_t r = gve_linkoutput_dqo(&adapter->ndev.n, p);
        CHECK(r == ERR_MEM, "DQO linkoutput on a full queue returns ERR_MEM");
        CHECK(p->ref == 1, "DQO linkoutput dropped its own ref on failure");
        pbuf_free(p);
        CHECK(pbufs_live == live_before, "DQO enqueue-full: no pbuf leak");
    }

    /* GQI. */
    {
        gve adapter = make_adapter_gqi(true);
        gve_tx_queue tx = &adapter->tx[0];
        make_tx_gqi(adapter, tx, 256);
        gve_setup_linkoutput(adapter, &adapter->ndev.n);
        int live_before = pbufs_live;
        while (enqueue(tx->br, (void *)tx)) ;
        struct pbuf *p = make_pkt(100);
        err_t r = adapter->ndev.n.linkoutput(&adapter->ndev.n, p);
        CHECK(r == ERR_MEM, "GQI linkoutput on a full queue returns ERR_MEM");
        CHECK(p->ref == 1, "GQI linkoutput dropped its own ref on failure");
        pbuf_free(p);
        CHECK(pbufs_live == live_before, "GQI enqueue-full: no pbuf leak");
    }
}

/* Scenario 40: DQO-RDA RX edge cases — a completion for an unposted buffer
 * (bad_req_id), an error on the second fragment of a started chain (frees
 * the partial chain), a zero-length packet (freed at EOP), and a packet the
 * stack rejects (driver frees it). */
static void scenario_rx_dqo_edge(void)
{
    rprintf("scenario_rx_dqo_edge\n");
    gve adapter = make_adapter();
    gve_rx_dqo_queue rx = &adapter->rx_dqo[0];
    make_rx_dqo(adapter, rx, 256);
    int delivered_before = rx_delivered;
    gve_rx_dqo_fill(rx);

    /* spurious completion: drop the posted pbuf so the slot looks unposted. */
    u16 bid = rx->buf_ring[rx_buf_cursor[rx->idx] & rx->mask].buf_id;
    struct pbuf *lost = rx->pbufs[bid];
    rx->pbufs[bid] = NULL;
    pbuf_free(lost);
    dqo_dev_rx_complete(rx, 100, true, false);
    run_rx_service(rx);
    CHECK(rx->rx_stats.bad_req_id == 1, "unposted buffer counted bad_req_id");

    /* good fragment then an error fragment: the partial chain is freed. */
    dqo_dev_rx_complete(rx, 1400, false, false);   /* frag 1: ctx_head set */
    dqo_dev_rx_complete(rx, 600,  true,  true);    /* frag 2: error at EOP */
    run_rx_service(rx);
    CHECK(rx->ctx_head == NULL, "errored chain tail freed the partial packet");
    CHECK(rx->rx_stats.rx_dropped >= 1, "errored chain counted dropped");

    /* zero-length packet: delivered path frees it (tot_len == 0). */
    int dropped_at = rx_delivered;
    dqo_dev_rx_complete(rx, 0, true, false);
    run_rx_service(rx);
    CHECK(rx_delivered == dropped_at, "zero-length packet not handed to input");

    /* a packet the stack rejects: the driver frees it. */
    int live_before = pbufs_live;
    g_input_err = 1;
    dqo_dev_rx_complete(rx, 120, true, false);
    run_rx_service(rx);
    g_input_err = 0;
    CHECK(rx_delivered == dropped_at + 1, "rejected packet still reached input once");
    CHECK(pbufs_live <= live_before, "driver freed the rejected packet");

    free_rx_dqo(rx);
}

/* Scenario 41: GQI RX edge cases — an error on a started chain (frees the
 * partial), a runt buffer (length <= pad) dropped, a multi-buffer packet
 * whose continuation allocation fails, and a packet the stack rejects. */
static void scenario_rx_gqi_edge(void)
{
    rprintf("scenario_rx_gqi_edge\n");
    gve adapter = make_adapter_gqi(true);
    gve_rx_queue rx = &adapter->rx[0];
    make_rx_gqi(adapter, rx, 64);

    /* good first fragment then an error continuation: partial chain freed. */
    gqi_dev_rx_complete(rx, 1400, GVE_RXF_PKT_CONT);     /* frag 1 -> ctx_head */
    gqi_dev_rx_complete(rx, 600,  GVE_RXF_ERR);          /* frag 2: error EOP */
    apply((thunk)&rx->service);
    CHECK(rx->ctx_head == NULL, "GQI errored continuation freed the partial");

    /* runt buffer: length <= pad is dropped. */
    u64 dropped0 = rx->rx_stats.rx_dropped;
    gqi_dev_rx_complete(rx, 0, 0);                       /* len = pad only */
    apply((thunk)&rx->service);
    CHECK(rx->rx_stats.rx_dropped > dropped0, "runt buffer (<= pad) dropped");

    /* multi-buffer packet whose continuation pbuf_alloc fails. */
    u64 dropped1 = rx->rx_stats.rx_dropped;
    gqi_dev_rx_complete(rx, 1400, GVE_RXF_PKT_CONT);     /* frag 1 */
    gqi_dev_rx_complete(rx, 600,  0);                    /* frag 2 (final) */
    pbuf_alloc_fail_after = 1;          /* frag 1 copy ok, frag 2 fails */
    apply((thunk)&rx->service);
    pbuf_alloc_fail_after = -1;
    CHECK(rx->rx_stats.rx_dropped > dropped1, "chain dropped on continuation alloc fail");
    CHECK(rx->ctx_head == NULL, "no partial chain left after alloc fail");

    /* a packet the stack rejects: the driver frees it. */
    int delivered_at = rx_delivered;
    g_input_err = 1;
    gqi_dev_rx_complete(rx, 120, 0);
    apply((thunk)&rx->service);
    g_input_err = 0;
    CHECK(rx_delivered == delivered_at + 1, "rejected GQI packet reached input once");
}

/* Scenario 42: DQO-QPL RX drop paths — an error fragment recycles its slot
 * and arms drop-to-EOP (whose subsequent buffers take the QPL recycle
 * branch), and a continuation whose copy-out allocation fails frees the
 * partial chain. */
static void scenario_rx_dqo_qpl_edge(void)
{
    rprintf("scenario_rx_dqo_qpl_edge\n");
    gve adapter = make_adapter();
    gve_rx_dqo_queue rx = &adapter->rx_dqo[0];
    make_rx_dqo_mode(adapter, rx, 256, true);    /* DQO-QPL */
    int delivered_before = rx_delivered;
    gve_rx_dqo_fill(rx);

    /* error fragment (!EOP) drops and arms drop_pkt; the following buffer
     * takes the QPL drop-recycle branch; EOP clears the drop. */
    dqo_dev_rx_complete(rx, 1400, false, true);    /* frag 1: error, !EOP */
    dqo_dev_rx_complete(rx, 600,  true,  false);   /* frag 2: recycled, EOP */
    run_rx_service(rx);
    CHECK(rx->drop_pkt == false, "QPL drop-to-EOP cleared at end of packet");
    CHECK(rx->rx_stats.rx_dropped >= 1, "QPL error fragment counted dropped");

    /* good fragment then a continuation whose copy-out alloc fails. */
    u64 dropped1 = rx->rx_stats.rx_dropped;
    dqo_dev_rx_complete(rx, 1400, false, false);   /* frag 1: copied, ctx_head */
    dqo_dev_rx_complete(rx, 600,  true,  false);   /* frag 2: copy will fail */
    pbuf_alloc_fail_after = 1;          /* frag 1 copy ok, frag 2 fails */
    run_rx_service(rx);
    pbuf_alloc_fail_after = -1;
    CHECK(rx->rx_stats.rx_dropped > dropped1, "QPL continuation alloc fail dropped");
    CHECK(rx->ctx_head == NULL, "QPL partial chain freed on alloc fail");

    free_rx_dqo(rx);
}

/* Scenario 43: sweep the allocation-failure point across every queue-create
 * cascade.  Each step fails one allocation deeper than the last (on a fresh
 * adapter), so the create functions' err_after_* cleanup labels are unwound
 * from progressively deeper points until the rings come up.  Covers the
 * TX/RX create-and-destroy error paths for all four queue formats, including
 * the QPL page-list and slot-list allocations. */
static void sweep_setup_fail(u16 opt)
{
    int saved_tp = total_processors;
    boolean reached_success = false;
    for (int k = 0; k < 48 && !reached_success; k++) {
        gve a = bring_up_to_cfg(opt, 1);
        a->contiguous = fail_heap();
        a->general = fail_heap();
        g_alloc_fail_after = k;
        boolean ok = gve_setup_queues(a);
        g_alloc_fail_after = -1;
        a->contiguous = gh; a->general = gh;
        if (ok)
            reached_success = true;    /* every cascade depth exercised */
        else
            CHECK(!(a->flags & (1ULL << GVE_FLAG_DEVICE_RUNNING)),
                  "setup-queues failure leaves DEVICE_RUNNING clear (k=%d)", k);
    }
    CHECK(reached_success, "setup-queues sweep reaches a working ring (opt 0x%x)", opt);
    total_processors = saved_tp;
}

static void scenario_setup_fail_sweep(void)
{
    rprintf("scenario_setup_fail_sweep\n");
    sweep_setup_fail(GVE_DEV_OPT_ID_DQO_RDA);
    sweep_setup_fail(GVE_DEV_OPT_ID_DQO_QPL);
    sweep_setup_fail(GVE_DEV_OPT_ID_GQI_RDA);
    sweep_setup_fail(0);                       /* GQI-QPL */
}

/* Full bring-up (probe + setup) advertising an explicit set of device
 * options (so RSS-capable adapters can be built). */
static gve bring_up_opts(const u16 *opts, int n, int ncpu)
{
    g_desc_nopts = n;
    for (int i = 0; i < n; i++)
        g_desc_opts[i] = opts[i];
    g_max_tx = g_max_rx = 8;
    g_msix = 64; g_msix_avail = 64; g_msix_setup_fail_after = -1;
    total_processors = ncpu;
    g_adminq = NULL; g_probe = 0; g_life_netif = 0;
    init_gve((kernel_heaps)0);
    apply(g_probe, (pci_dev)pointer_from_u64(0x42));
    gve adapter = g_life_netif->state;
    apply((netif_dev_setup)&adapter->ndev.setup, (tuple)0);
    return adapter;
}

/* Scenario 44: admin-queue command-failure paths — describe failing at
 * probe (the adapter is torn down and no netif added), the ptype-map and
 * RSS commands failing during setup, and cfg/ptype/RSS failing during a
 * reset. */
static void scenario_cmd_failures(void)
{
    rprintf("scenario_cmd_failures\n");
    int saved_tp = total_processors;
    const u16 dqo_rss[] = { GVE_DEV_OPT_ID_DQO_RDA, GVE_DEV_OPT_ID_RSS_CONFIG };

    /* describe fails -> gve_init returns false -> probe deallocates. */
    g_desc_nopts = 1; g_desc_opts[0] = GVE_DEV_OPT_ID_DQO_RDA;
    g_max_tx = g_max_rx = 8; g_msix = 64; g_msix_avail = 64;
    g_msix_setup_fail_after = -1; total_processors = 1;
    g_adminq = NULL; g_probe = 0; g_life_netif = 0;
    g_adminq_fail_op = GVE_ADMINQ_DESCRIBE_DEVICE;
    init_gve((kernel_heaps)0);
    boolean pr = apply(g_probe, (pci_dev)pointer_from_u64(0x42));
    g_adminq_fail_op = 0;
    CHECK(!pr, "probe fails when DESCRIBE_DEVICE is rejected");
    CHECK(g_life_netif == 0, "no netif added on describe failure");

    /* setup: ptype-map fails (DQO requires it before queues). */
    gve a = bring_up_probe_only(GVE_DEV_OPT_ID_DQO_RDA, 1);
    g_adminq_fail_op = GVE_ADMINQ_GET_PTYPE_MAP;
    CHECK(!apply((netif_dev_setup)&a->ndev.setup, (tuple)0),
          "setup fails when GET_PTYPE_MAP is rejected");
    g_adminq_fail_op = 0;

    /* setup: RSS fails — best-effort, setup still succeeds. */
    g_desc_nopts = 2; g_desc_opts[0] = dqo_rss[0]; g_desc_opts[1] = dqo_rss[1];
    g_max_tx = g_max_rx = 8; g_msix = 64; g_msix_avail = 64;
    g_msix_setup_fail_after = -1; total_processors = 1;
    g_adminq = NULL; g_probe = 0; g_life_netif = 0;
    init_gve((kernel_heaps)0);
    apply(g_probe, (pci_dev)pointer_from_u64(0x42));
    gve b = g_life_netif->state;
    CHECK(b->rss_supported, "RSS option advertised -> rss_supported");
    g_adminq_fail_op = GVE_ADMINQ_CONFIGURE_RSS;
    CHECK(apply((netif_dev_setup)&b->ndev.setup, (tuple)0),
          "setup still succeeds when RSS config is rejected (best-effort)");
    g_adminq_fail_op = 0;

    /* reset: cfg-resources fails. */
    gve c = bring_up(GVE_DEV_OPT_ID_DQO_RDA, 1);
    g_adminq_fail_op = GVE_ADMINQ_CONFIGURE_DEVICE_RESOURCES;
    gve_trigger_reset(c);
    g_adminq_fail_op = 0;
    CHECK(!(c->flags & (1ULL << GVE_FLAG_DEVICE_RUNNING)),
          "reset with cfg failure leaves the adapter down");

    /* reset: ptype-map fails. */
    gve d = bring_up(GVE_DEV_OPT_ID_DQO_RDA, 1);
    g_adminq_fail_op = GVE_ADMINQ_GET_PTYPE_MAP;
    gve_trigger_reset(d);
    g_adminq_fail_op = 0;
    CHECK(!(d->flags & (1ULL << GVE_FLAG_DEVICE_RUNNING)),
          "reset with ptype failure leaves the adapter down");

    /* reset: RSS fails — best-effort, reset completes. */
    gve e = bring_up_opts(dqo_rss, 2, 1);
    g_adminq_fail_op = GVE_ADMINQ_CONFIGURE_RSS;
    gve_trigger_reset(e);
    g_adminq_fail_op = 0;
    CHECK(e->flags & (1ULL << GVE_FLAG_DEVICE_RUNNING),
          "reset completes despite RSS config failure");

    total_processors = saved_tp;
}

/* Scenario 45: an admin-queue command that the device never answers times
 * out, marks the queue dead, and makes later commands fast-fail. */
static void scenario_adminq_timeout(void)
{
    rprintf("scenario_adminq_timeout\n");
    int saved_tp = total_processors;

    g_desc_nopts = 1; g_desc_opts[0] = GVE_DEV_OPT_ID_DQO_RDA;
    g_max_tx = g_max_rx = 8; g_msix = 64; g_msix_avail = 64;
    g_msix_setup_fail_after = -1; total_processors = 1;
    g_adminq = NULL; g_probe = 0; g_life_netif = 0;
    g_evt_cnt = 0;                     /* fresh device: counter starts at 0 so
                                        * the unanswered command stays behind */
    g_adminq_no_answer = 1;            /* device stops replying */
    init_gve((kernel_heaps)0);
    boolean pr = apply(g_probe, (pci_dev)pointer_from_u64(0x42));
    g_adminq_no_answer = 0;
    CHECK(!pr, "probe fails when the device never answers the admin queue");

    total_processors = saved_tp;
}

/* Scenario 45b: watchdog deadline-scan edges that are NOT a timeout — a DQO
 * miss recorded recently (still within GVE_TX_WATCHDOG_MS) is skipped, not
 * reset; and the GQI stuck scan skips zero (seg-descriptor / already-retired)
 * timestamp slots while a recent packet stamp is below the deadline. */
static void scenario_watchdog_deadline_edges(void)
{
    rprintf("scenario_watchdog_deadline_edges\n");
    int saved_tp = total_processors;
    timestamp recent = now(CLOCK_ID_MONOTONIC);

    /* DQO: a fresh miss within the deadline must not trip a reset. */
    gve a = bring_up(GVE_DEV_OPT_ID_DQO_RDA, 1);
    gve_tx_dqo_queue atx = &a->tx_dqo[0];
    atx->seg_counts[3] = 2;            /* tag 3 in flight */
    atx->miss_times[3] = recent;       /* missed just now (not expired) */
    a->rx_dqo[0].first_interrupt = true;
    a->next_monitored_tx_qid = 0;
    u64 wd0 = a->dev_stats.wd_expired;
    apply((timer_handler)&a->watchdog_task, (u64)0, (u64)1);
    CHECK(a->dev_stats.wd_expired == wd0, "DQO recent miss: no reset");
    CHECK(atx->miss_times[3] == recent, "DQO recent miss still pending");

    /* GQI: in-flight packet with a recent pkt stamp and zero seg-slot stamps;
     * the scan skips the zero slots and finds nothing stuck. */
    gve b = bring_up(0, 1);
    gve_tx_queue btx = &b->tx[0];
    btx->tail = 0; btx->head = 3;
    btx->tx_timestamps[0] = recent;    /* packet slot, recent */
    btx->tx_timestamps[1] = 0;         /* seg slot -> skipped */
    btx->tx_timestamps[2] = 0;         /* seg slot -> skipped */
    b->rx[0].first_interrupt = true;
    b->next_monitored_tx_qid = 0;
    u64 bwd0 = b->dev_stats.wd_expired;
    apply((timer_handler)&b->watchdog_task, (u64)0, (u64)1);
    CHECK(b->dev_stats.wd_expired == bwd0, "GQI recent stamp + seg slots: no reset");
    CHECK(!btx->stuck, "GQI queue not marked stuck");

    total_processors = saved_tp;
}

/* Scenario 46: GQI-QPL multi-segment TX writes the per-segment seg
 * descriptors, and a copy that crosses the QPL byte-FIFO end wraps to the
 * front. */
static void scenario_gqi_qpl_multiseg(void)
{
    rprintf("scenario_gqi_qpl_multiseg\n");
    gve adapter = make_adapter_gqi(false);     /* GQI-QPL */
    gve_tx_queue tx = &adapter->tx[0];
    make_tx_gqi(adapter, tx, 256);
    gve_setup_linkoutput(adapter, &adapter->ndev.n);
    int live_before = pbufs_live;

    /* Push the byte FIFO close to the end so the next copy wraps. */
    tx->qpl_head = tx->qpl_size - 128;
    struct pbuf *p = make_chain(3, 200);       /* 600 bytes, 3 segments */
    err_t e = adapter->ndev.n.linkoutput(&adapter->ndev.n, p);
    CHECK(e == ERR_OK, "GQI-QPL multi-seg linkoutput ok");
    CHECK(tx->head == 3, "pkt + 2 seg descriptors written, got %d", tx->head);
    CHECK(tx->qpl_head < tx->qpl_size, "QPL head wrapped and stayed in range");

    gqi_dev_tx_complete_all(adapter, tx);
    run_tx_gqi_cleanup(tx);
    CHECK(tx->tail == tx->head, "all segments retired");
    pbuf_free(p);
    CHECK(pbufs_live == live_before, "no leak across the QPL multi-seg wrap");
}

/* Scenario 47: GQI RX — a runt buffer in the middle of a started chain
 * frees the partial, and a multi-buffer packet the stack rejects is freed
 * by the driver. */
static void scenario_gqi_rx_midchain(void)
{
    rprintf("scenario_gqi_rx_midchain\n");
    gve adapter = make_adapter_gqi(true);
    gve_rx_queue rx = &adapter->rx[0];
    make_rx_gqi(adapter, rx, 64);

    /* good first fragment then a runt continuation (length <= pad). */
    gqi_dev_rx_complete(rx, 1400, GVE_RXF_PKT_CONT);   /* frag 1 -> ctx_head */
    gqi_dev_rx_complete(rx, 0,    GVE_RXF_PKT_CONT);   /* runt continuation */
    gqi_dev_rx_complete(rx, 600,  0);                  /* EOP of the chain */
    apply((thunk)&rx->service);
    CHECK(rx->ctx_head == NULL, "runt continuation freed the partial chain");

    /* a multi-buffer packet the stack rejects: the driver frees the chain. */
    int delivered_at = rx_delivered;
    g_input_err = 1;
    gqi_dev_rx_complete(rx, 1400, GVE_RXF_PKT_CONT);
    gqi_dev_rx_complete(rx, 600,  0);
    apply((thunk)&rx->service);
    g_input_err = 0;
    CHECK(rx_delivered == delivered_at + 1, "rejected GQI chain reached input once");
}

/* Scenario 48: more DQO RX branches — a QPL error on a started chain frees
 * the partial, and processing a full ring's worth of completions flips the
 * expected generation bit at the wrap. */
static void scenario_dqo_rx_more(void)
{
    rprintf("scenario_dqo_rx_more\n");

    /* QPL: good fragment then an error fragment (ctx_head was set). */
    {
        gve adapter = make_adapter();
        gve_rx_dqo_queue rx = &adapter->rx_dqo[0];
        make_rx_dqo_mode(adapter, rx, 256, true);   /* DQO-QPL */
        gve_rx_dqo_fill(rx);
        dqo_dev_rx_complete(rx, 1400, false, false);  /* frag 1: copied */
        dqo_dev_rx_complete(rx, 600,  false, true);   /* frag 2: error */
        dqo_dev_rx_complete(rx, 1,    true,  false);  /* EOP, recycled */
        run_rx_service(rx);
        CHECK(rx->ctx_head == NULL, "QPL error on a started chain freed the partial");
        free_rx_dqo(rx);
    }

    /* RDA: a small ring processed past a full wrap flips expected_gen. */
    {
        gve adapter = make_adapter();
        gve_rx_dqo_queue rx = &adapter->rx_dqo[0];
        make_rx_dqo(adapter, rx, 8);                 /* tiny ring -> quick wrap */
        u8 gen0 = rx->expected_gen;
        boolean flipped = false;
        for (int i = 0; i < 16; i++) {
            gve_rx_dqo_fill(rx);
            dqo_dev_rx_complete(rx, 100, true, false);
            run_rx_service(rx);
            if (rx->expected_gen != gen0)
                flipped = true;
        }
        CHECK(flipped, "expected_gen flipped at the RX completion-ring wrap");
        free_rx_dqo(rx);
    }
}

/* Scenario 49: queue teardown frees held resources — an undrained software
 * TX queue, in-flight pending pbufs, a partial RX chain and a still-held RX
 * pbuf — for GQI-RDA, GQI-QPL and DQO-RDA. */
static void scenario_teardown_held(void)
{
    rprintf("scenario_teardown_held\n");
    int saved_tp = total_processors;

    /* GQI-RDA. */
    {
        const u16 opts[] = { GVE_DEV_OPT_ID_GQI_RDA };
        gve adapter = bring_up_opts(opts, 1, 1);
        gve_tx_queue tx = &adapter->tx[0];
        gve_rx_queue rx = &adapter->rx[0];
        struct pbuf *q = make_pkt(64); pbuf_ref(q);
        enqueue(tx->br, q);                          /* undrained -> freed */
        struct pbuf *p = make_pkt(100);
        adapter->ndev.n.linkoutput(&adapter->ndev.n, p);  /* pending[0] = p */
        rx->ctx_head = make_pkt(200);                /* partial chain */
        rx->pbufs[0].ref++;                          /* still-held warning */
        gve_teardown_queues(adapter);
        CHECK(tx->br == NULL && tx->q_res == NULL, "GQI-RDA TX torn down");
        CHECK(rx->q_res == NULL, "GQI-RDA RX torn down");
        pbuf_free(q);
        pbuf_free(p);
    }

    /* GQI-QPL: the non-raw teardown unregisters the page lists. */
    {
        gve adapter = bring_up(0, 1);                /* GQI-QPL */
        gve_rx_queue rx = &adapter->rx[0];
        rx->ctx_head = make_pkt(200);
        gve_teardown_queues(adapter);
        CHECK(adapter->tx[0].qpl_base == NULL, "GQI-QPL TX QPL released");
        CHECK(rx->qpl_base == NULL, "GQI-QPL RX QPL released");
    }

    /* DQO-RDA: pending pbufs and a partial RX chain freed at teardown. */
    {
        gve adapter = bring_up(GVE_DEV_OPT_ID_DQO_RDA, 1);
        gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
        gve_rx_dqo_queue rx = &adapter->rx_dqo[0];
        struct pbuf *q = make_pkt(64); pbuf_ref(q);
        enqueue(tx->br, q);
        struct pbuf *p = make_pkt(100);
        send_pkt(tx, adapter, p);                    /* leaves pending[tag] */
        gve_rx_dqo_fill(rx);                         /* post RX pbufs */
        rx->ctx_head = make_pkt(200);
        gve_teardown_queues(adapter);
        CHECK(tx->br == NULL && tx->q_res == NULL, "DQO-RDA TX torn down");
        CHECK(rx->q_res == NULL && rx->pbufs == NULL, "DQO-RDA RX torn down");
        pbuf_free(q);
        pbuf_free(p);
    }

    total_processors = saved_tp;
}

/* count the buffers in a pbuf chain */
static int pbuf_chain_len(struct pbuf *p)
{
    int n = 0;
    for (; p != NULL; p = p->next) n++;
    return n;
}

/* Scenario 50: a frame the device splits across several fixed-size
 * (GVE_DQO_BUF_SIZE = 2 KB) RX buffers is reassembled into one pbuf chain of
 * the right total length and buffer count — for DQO-RDA, DQO-QPL and GQI.
 * The earlier chain scenarios used arbitrary 2-buffer splits; this one uses
 * full buffer-sized fragments and a long chain, the realistic shape for a
 * jumbo frame over 2 KB buffers. */
static void scenario_rx_fixed_buffer_split(void)
{
    rprintf("scenario_rx_fixed_buffer_split\n");
    /* four full 2 KB buffers + an 808-byte tail = 8 KB + 808 = a 8.99 KB frame */
    const u16 full = GVE_DQO_BUF_SIZE;     /* 2048 */
    const u16 tail = 808;
    const u32 expect = (u32)full * 4 + tail;

    /* DQO-RDA. */
    {
        gve adapter = make_adapter();
        gve_rx_dqo_queue rx = &adapter->rx_dqo[0];
        make_rx_dqo(adapter, rx, 256);
        gve_rx_dqo_fill(rx);
        g_input_hold = 1; g_last_input = NULL;
        dqo_dev_rx_complete(rx, full, false, false);
        dqo_dev_rx_complete(rx, full, false, false);
        dqo_dev_rx_complete(rx, full, false, false);
        dqo_dev_rx_complete(rx, full, false, false);
        dqo_dev_rx_complete(rx, tail, true,  false);
        run_rx_service(rx);
        g_input_hold = 0;
        CHECK(g_last_input_totlen == expect,
              "DQO-RDA 5-buffer split reassembled to %u, got %u",
              expect, g_last_input_totlen);
        CHECK(pbuf_chain_len(g_last_input) == 5, "DQO-RDA chain has 5 buffers");
        if (g_last_input) pbuf_free(g_last_input);
        free_rx_dqo(rx);
    }

    /* DQO-QPL (copy-out path). */
    {
        gve adapter = make_adapter();
        gve_rx_dqo_queue rx = &adapter->rx_dqo[0];
        make_rx_dqo_mode(adapter, rx, 256, true);
        gve_rx_dqo_fill(rx);
        g_input_hold = 1; g_last_input = NULL;
        dqo_dev_rx_complete(rx, full, false, false);
        dqo_dev_rx_complete(rx, full, false, false);
        dqo_dev_rx_complete(rx, full, false, false);
        dqo_dev_rx_complete(rx, full, false, false);
        dqo_dev_rx_complete(rx, tail, true,  false);
        run_rx_service(rx);
        g_input_hold = 0;
        CHECK(g_last_input_totlen == expect,
              "DQO-QPL 5-buffer split reassembled to %u, got %u",
              expect, g_last_input_totlen);
        CHECK(pbuf_chain_len(g_last_input) == 5, "DQO-QPL chain has 5 buffers");
        if (g_last_input) pbuf_free(g_last_input);
        free_rx_dqo(rx);
    }

    /* GQI: the device strips a 2-byte pad on the first buffer only, so the
     * first fragment carries (buffer - pad) data and continuations a full
     * buffer.  Feed the per-buffer data lengths the device delivers. */
    {
        gve adapter = make_adapter_gqi(true);
        gve_rx_queue rx = &adapter->rx[0];
        make_rx_gqi(adapter, rx, 64);
        u16 first = full - GVE_RX_PADDING;     /* 2046 of data on buffer 1 */
        g_input_hold = 1; g_last_input = NULL;
        gqi_dev_rx_complete(rx, first, GVE_RXF_PKT_CONT);
        gqi_dev_rx_complete(rx, full,  GVE_RXF_PKT_CONT);
        gqi_dev_rx_complete(rx, full,  GVE_RXF_PKT_CONT);
        gqi_dev_rx_complete(rx, tail,  0);
        apply((thunk)&rx->service);
        g_input_hold = 0;
        u32 gqi_expect = (u32)first + full + full + tail;
        CHECK(g_last_input_totlen == gqi_expect,
              "GQI 4-buffer split reassembled to %u, got %u",
              gqi_expect, g_last_input_totlen);
        CHECK(pbuf_chain_len(g_last_input) == 4, "GQI chain has 4 buffers");
        if (g_last_input) pbuf_free(g_last_input);
    }
}

/* Scenario 51: DQO-QPL TX rejects a single segment larger than one fixed
 * bounce slot (GVE_DQO_BUF_SIZE), since a buffer may not cross a registered
 * QPL page.  Such a segment is dropped (not re-queued forever) and the
 * queue stays usable.  (The RDA too-many-segments drop is scenario_tx_drop_
 * oversize; this is the QPL fixed-slot constraint.) */
static void scenario_tx_qpl_oversize(void)
{
    rprintf("scenario_tx_qpl_oversize\n");
    gve adapter = make_adapter();
    gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
    make_tx_dqo_mode(adapter, tx, 256, true);      /* DQO-QPL */
    int live_before = pbufs_live;
    int free0 = (int)(tx->tags_ntc - tx->tags_ntu);
    u32 slots0 = tx->qpl_free_slots;

    struct pbuf *p = make_pkt(GVE_DQO_BUF_SIZE + 1000);   /* 3 KB > 2 KB slot */
    err_t e = gve_linkoutput_dqo(&adapter->ndev.n, p);
    CHECK(e == ERR_OK, "QPL oversize linkoutput accepts then drops, returns OK");
    CHECK(tx->head == 0, "no descriptors written for the oversize segment");
    CHECK((int)(tx->tags_ntc - tx->tags_ntu) == free0, "no tag consumed");
    CHECK(tx->qpl_free_slots == slots0, "no bounce slot taken");
    CHECK(tx->running == true, "queue not wedged by the oversize drop");
    CHECK(p->ref == 1, "driver released its ref on the dropped segment");

    /* the queue still works: a normal packet goes through afterwards. */
    struct pbuf *q = make_pkt(128);
    u16 tag = send_pkt(tx, adapter, q);
    CHECK(tx->head == 2, "a normal packet sends after the oversize drop");
    dqo_dev_complete_pkt(&the_dev, tag, GVE_DQO_COMPL_TYPE_PKT);
    run_cleanup(tx);

    pbuf_free(p);
    pbuf_free(q);
    CHECK(pbufs_live == live_before, "no leak across oversize drop + normal send");
}

/* Scenario 53: ring-size backoff does not permanently shrink the rings.
 * A setup under sustained allocation failure backs off to the minimum and
 * gives up (leaving the working sizes small); the next clean setup restores
 * them to the device-reported canonical sizes (the ENA set_io_rings_size
 * alignment — earlier the working sizes were mutated in place and never
 * restored, so rings only ever shrank across resets). */
static void scenario_ring_size_restore(void)
{
    rprintf("scenario_ring_size_restore\n");
    int saved_tp = total_processors;

    gve a = bring_up_to_cfg(GVE_DEV_OPT_ID_DQO_RDA, 1);
    u32 tx_dev = a->tx_desc_cnt_dev, rx_dev = a->rx_desc_cnt_dev;

    a->contiguous = fail_heap(); a->general = fail_heap();
    g_alloc_fail_after = 2;            /* fail early -> backoff to give-up */
    CHECK(!gve_setup_queues(a), "setup gives up under sustained alloc failure");
    CHECK(a->tx_desc_cnt < tx_dev, "working TX ring shrank during backoff (%u < %u)",
          a->tx_desc_cnt, tx_dev);
    g_alloc_fail_after = -1;
    a->contiguous = gh; a->general = gh;

    CHECK(gve_setup_queues(a), "the next clean setup succeeds");
    CHECK(a->tx_desc_cnt == tx_dev, "TX ring restored to canonical %u, got %u",
          tx_dev, a->tx_desc_cnt);
    CHECK(a->rx_desc_cnt == rx_dev, "RX ring restored to canonical %u, got %u",
          rx_dev, a->rx_desc_cnt);

    total_processors = saved_tp;
}

/* Scenario 54: RSS uses a fresh random Toeplitz key on each configure (anti
 * hash-flooding) and declares the Toeplitz algorithm and the TCP/UDP v4/v6
 * hash types Google's driver uses. */
static void scenario_rss_key_fresh(void)
{
    rprintf("scenario_rss_key_fresh\n");
    const u16 opts[] = { GVE_DEV_OPT_ID_DQO_RDA, GVE_DEV_OPT_ID_RSS_CONFIG };
    gve a = bring_up_opts(opts, 2, 1);
    CHECK(a->rss_supported, "RSS supported");

    u8 key1[40];
    CHECK(gve_configure_rss(a), "first configure_rss ok");
    CHECK(g_rss_key_size == GVE_RSS_KEY_SIZE, "40-byte key");
    CHECK(g_rss_hash_alg == GVE_RSS_HASH_ALG_TOEPLITZ, "Toeplitz algorithm");
    CHECK(g_rss_hash_types == GVE_RSS_HASH_TYPES, "TCP/UDP v4+v6 hash types");
    runtime_memcpy(key1, g_rss_key, sizeof(key1));

    CHECK(gve_configure_rss(a), "second configure_rss ok");
    boolean differs = false;
    for (int i = 0; i < GVE_RSS_KEY_SIZE; i++)
        if (g_rss_key[i] != key1[i]) differs = true;
    CHECK(differs, "a fresh random key is generated on each configure");

    boolean nonzero = false;
    for (int i = 0; i < GVE_RSS_KEY_SIZE; i++)
        if (g_rss_key[i]) nonzero = true;
    CHECK(nonzero, "the key is not all-zero");
}

/* Scenario 55: no TX checksum offload — the driver leaves the descriptor
 * checksum fields zero and lets lwIP compute checksums in software (the
 * ENA-aligned model; guards against re-introducing the reverted L4 offload).
 * Verified on both GQI and DQO descriptors after a send. */
static void scenario_no_csum_offload(void)
{
    rprintf("scenario_no_csum_offload\n");

    /* GQI: pkt descriptor csum offsets stay zero. */
    {
        gve adapter = make_adapter_gqi(true);
        gve_tx_queue tx = &adapter->tx[0];
        make_tx_gqi(adapter, tx, 256);
        gve_setup_linkoutput(adapter, &adapter->ndev.n);
        struct pbuf *p = make_pkt(100);
        adapter->ndev.n.linkoutput(&adapter->ndev.n, p);
        struct gve_tx_pkt_desc *d = &tx->desc[0].pkt;
        CHECK(d->l4_csum_offset == 0 && d->l4_hdr_offset == 0,
              "GQI pkt descriptor carries no checksum offload");
        gqi_dev_tx_complete_all(adapter, tx);
        run_tx_gqi_cleanup(tx);
        pbuf_free(p);
    }

    /* DQO: the packet descriptor has no checksum-enable bits set (dtype only). */
    {
        gve adapter = make_adapter();
        gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
        make_tx_dqo(adapter, tx, 256);
        struct pbuf *p = make_pkt(100);
        u16 tag = send_pkt(tx, adapter, p);
        struct gve_tx_pkt_desc_dqo *d = &tx->desc[1];   /* pkt desc (after ctx) */
        CHECK((d->dtype_flags & 0x1f) == GVE_DQO_TX_DTYPE_PKT,
              "DQO packet descriptor dtype is PKT");
        /* the only extra bit allowed is end_of_packet; no checksum bits. */
        CHECK((d->dtype_flags & ~GVE_DQO_TX_EOP) == GVE_DQO_TX_DTYPE_PKT,
              "DQO packet descriptor sets no flags beyond dtype + EOP");
        /* the context descriptor (where Google carries csum/TSO metadata) is
         * all-zero except its dtype, and the TSO bit (cmd_dtype bit 5) clear. */
        struct gve_tx_ctx_desc_dqo *ctx = (struct gve_tx_ctx_desc_dqo *)&tx->desc[0];
        CHECK(ctx->cmd_dtype == GVE_DQO_TX_DTYPE_CTX,
              "context descriptor carries only its dtype (no TSO/csum bit)");
        boolean flex_zero = true;
        for (int i = 0; i < 8; i++) if (ctx->flex_hi[i]) flex_zero = false;
        for (int i = 0; i < 4; i++) if (ctx->flex_lo[i]) flex_zero = false;
        CHECK(flex_zero, "context descriptor metadata flex fields are zero");
        dqo_dev_complete_pkt(&the_dev, tag, GVE_DQO_COMPL_TYPE_PKT);
        run_cleanup(tx);
        pbuf_free(p);
    }
}

/* Scenario 56: DQO report_event — the driver sets the REPORT bit sparsely
 * (at least GVE_TX_MIN_RE_INTERVAL descriptors apart, per Google), not on
 * every packet, and advances last_re_idx. */
static void scenario_dqo_report_event(void)
{
    rprintf("scenario_dqo_report_event\n");
    gve adapter = make_adapter();
    gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
    make_tx_dqo(adapter, tx, 256);

    const int N = 40;                  /* 40 pkts * 2 descs = 80 descriptors */
    struct pbuf *pkts[N];
    u16 tags[N];
    for (int i = 0; i < N; i++) {
        pkts[i] = make_pkt(64);
        tags[i] = send_pkt(tx, adapter, pkts[i]);
    }

    int reports = 0, last = -1, min_gap = 1 << 30;
    for (u32 s = 0; s < tx->head; s++) {
        if (tx->desc[s].dtype_flags & GVE_DQO_TX_REPORT) {
            if (last >= 0 && (int)s - last < min_gap) min_gap = (int)s - last;
            last = (int)s;
            reports++;
        }
    }
    CHECK(reports >= 1 && reports < N,
          "REPORT bit set sparsely (%d reports over %d packets)", reports, N);
    CHECK(min_gap >= GVE_TX_MIN_RE_INTERVAL,
          "reports at least %d descriptors apart (min gap %d)",
          GVE_TX_MIN_RE_INTERVAL, min_gap == (1 << 30) ? -1 : min_gap);
    CHECK(tx->last_re_idx != 0, "last_re_idx advanced");

    for (int i = 0; i < N; i++) {
        dqo_dev_complete_pkt(&the_dev, tags[i], GVE_DQO_COMPL_TYPE_PKT);
        run_cleanup(tx);
        pbuf_free(pkts[i]);
    }
}

/* Scenario 57: DQO descriptor stream layout — every packet is one mandatory
 * general context descriptor (dtype 0x4) followed by its packet
 * descriptor(s), and the same completion tag is stamped on every packet
 * descriptor of the packet (matches the official Google driver). */
static void scenario_dqo_desc_layout(void)
{
    rprintf("scenario_dqo_desc_layout\n");
    gve adapter = make_adapter();
    gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
    make_tx_dqo(adapter, tx, 256);

    /* single-segment packet: ctx desc then one pkt desc. */
    struct pbuf *p = make_pkt(100);
    u16 tag = send_pkt(tx, adapter, p);
    struct gve_tx_ctx_desc_dqo *ctx = (struct gve_tx_ctx_desc_dqo *)&tx->desc[0];
    CHECK((ctx->cmd_dtype & 0x1f) == GVE_DQO_TX_DTYPE_CTX,
          "packet starts with a general context descriptor (dtype 0x4)");
    CHECK((tx->desc[1].dtype_flags & 0x1f) == GVE_DQO_TX_DTYPE_PKT,
          "the context descriptor is followed by a packet descriptor");
    CHECK(tx->desc[1].compl_tag == tag, "packet descriptor carries the compl tag");
    dqo_dev_complete_pkt(&the_dev, tag, GVE_DQO_COMPL_TYPE_PKT);
    run_cleanup(tx);
    pbuf_free(p);

    /* multi-segment packet: ctx desc then one pkt desc per segment, all with
     * the same tag. */
    u32 h0 = tx->head;
    struct pbuf *c = make_chain(3, 200);
    u16 tag2 = send_pkt(tx, adapter, c);
    struct gve_tx_ctx_desc_dqo *ctx2 =
        (struct gve_tx_ctx_desc_dqo *)&tx->desc[h0 & tx->mask];
    CHECK((ctx2->cmd_dtype & 0x1f) == GVE_DQO_TX_DTYPE_CTX,
          "multi-seg packet also starts with a context descriptor");
    boolean all_tag = true;
    for (u32 s = h0 + 1; s < tx->head; s++)
        if (tx->desc[s & tx->mask].compl_tag != tag2) all_tag = false;
    CHECK(all_tag, "every segment's packet descriptor carries the same tag");
    CHECK(tx->head - h0 == 4, "ctx + 3 segment descriptors written");
    dqo_dev_complete_pkt(&the_dev, tag2, GVE_DQO_COMPL_TYPE_PKT);
    run_cleanup(tx);
    pbuf_free(c);
}

/* Scenario 58: TX doorbell value format differs by protocol — GQI rings a
 * free-running big-endian producer index, DQO rings a masked little-endian
 * ring index (Google: iowrite32be for GQI, iowrite32 of the masked tail for
 * DQO). */
static void scenario_doorbell_format(void)
{
    rprintf("scenario_doorbell_format\n");

    /* GQI: big-endian free-running head. */
    {
        gve adapter = make_adapter_gqi(true);
        gve_tx_queue tx = &adapter->tx[0];
        make_tx_gqi(adapter, tx, 256);
        gve_setup_linkoutput(adapter, &adapter->ndev.n);
        the_dev.last_tx_doorbell = 0;
        struct pbuf *p = make_pkt(100);
        adapter->ndev.n.linkoutput(&adapter->ndev.n, p);
        CHECK(be32toh(the_dev.last_tx_doorbell) == tx->head,
              "GQI doorbell is the big-endian head (%u)", tx->head);
        CHECK(the_dev.last_tx_doorbell != tx->head,
              "GQI doorbell is byte-swapped (not native LE)");
        gqi_dev_tx_complete_all(adapter, tx);
        run_tx_gqi_cleanup(tx);
        pbuf_free(p);
    }

    /* DQO: little-endian masked ring index. */
    {
        gve adapter = make_adapter();
        gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
        make_tx_dqo(adapter, tx, 256);
        the_dev.last_tx_doorbell = 0xffffffff;
        struct pbuf *p = make_pkt(100);
        u16 tag = send_pkt(tx, adapter, p);
        CHECK(the_dev.last_tx_doorbell == (tx->head & tx->mask),
              "DQO doorbell is the masked LE ring index (%u)", tx->head & tx->mask);
        dqo_dev_complete_pkt(&the_dev, tag, GVE_DQO_COMPL_TYPE_PKT);
        run_cleanup(tx);
        pbuf_free(p);
    }
}

/* Scenario 59: GQI RX fill is u32-wrap-safe — with head/tail near the 32-bit
 * wrap, the `head - tail < cnt` predicate keeps posting buffers (the old
 * `head < tail + cnt` form overflowed and deadlocked RX after 2^32 posted
 * buffers per queue). */
static void scenario_rx_fill_wrap(void)
{
    rprintf("scenario_rx_fill_wrap\n");
    gve adapter = make_adapter_gqi(true);
    gve_rx_queue rx = &adapter->rx[0];
    make_rx_gqi(adapter, rx, 64);

    /* Drive head/tail right up to the u32 wrap with a fully available QPL. */
    u32 base = 0xfffffff0;
    rx->head = rx->tail = base;
    rx->qpl_head = 0;
    rx->qpl_available = rx->qpl_count;
    gve_rx_fill(rx);

    u32 posted = rx->head - base;      /* wrap-safe subtraction */
    CHECK(posted == adapter->rx_desc_cnt,
          "fill posted a full ring across the wrap (%u of %u)",
          posted, adapter->rx_desc_cnt);
    CHECK(rx->head < base, "head wrapped past 2^32 (now %u)", rx->head);
    CHECK(rx->head - rx->tail == adapter->rx_desc_cnt,
          "head-tail predicate stays correct across the wrap");
}

/* ------------------------------------------------------------------ */
/* Concurrency scenarios (GVE_HARNESS_SMP, run under ThreadSanitizer).   */
/* With the real atomic spinlocks from shim/lock.h, these drive the      */
/* driver's lock-protected state from multiple threads so TSan can       */
/* confirm ring_mtx / service_lock actually serialise it.                */
/* ------------------------------------------------------------------ */
#ifdef GVE_HARNESS_SMP
#define CONC_NPROD    4
#define CONC_PERPROD  300

static gve          g_conc_adapter;
static struct pbuf *g_conc_tx_pkts[CONC_NPROD][CONC_PERPROD];

static void *conc_tx_producer(void *arg)
{
    long id = (long)arg;
    for (int i = 0; i < CONC_PERPROD; i++)
        gve_linkoutput_dqo(&g_conc_adapter->ndev.n, g_conc_tx_pkts[id][i]);
    return NULL;
}

/* Scenario C1: many producer threads transmit on one DQO-RDA queue at
 * once.  Each linkoutput enqueues on the lock-free br, then drains under
 * ring_mtx; the drain's accesses to head / desc_tail / the tag pool / the
 * descriptor ring must be serialised by ring_mtx.  TSan reports a race if
 * any of that escapes the lock. */
static void scenario_conc_tx(void)
{
    rprintf("scenario_conc_tx\n");
    gve adapter = make_adapter();              /* DQO-RDA */
    gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
    make_tx_dqo(adapter, tx, 4096);            /* holds all descs, no backpressure */
    adapter->num_queues = 1;
    g_conc_adapter = adapter;

    int total = CONC_NPROD * CONC_PERPROD;
    int live_before = pbufs_live;
    for (int t = 0; t < CONC_NPROD; t++)
        for (int i = 0; i < CONC_PERPROD; i++)
            g_conc_tx_pkts[t][i] = make_pkt(64);

    pthread_t th[CONC_NPROD];
    for (long t = 0; t < CONC_NPROD; t++)
        pthread_create(&th[t], NULL, conc_tx_producer, (void *)t);
    for (int t = 0; t < CONC_NPROD; t++)
        pthread_join(th[t], NULL);

    /* All packets were accepted and turned into ctx+pkt descriptor pairs;
     * the tag pool advanced by exactly one tag per packet (no double-take,
     * no lost slot — the invariant ring_mtx protects). */
    CHECK(tx->head == (u32)(total * 2), "all %d packets written (head=%u)",
          total, tx->head);
    CHECK((u32)(tx->tags_ntu - 0) == (u32)total, "one tag taken per packet (%u)",
          tx->tags_ntu);

    /* Drain the completions and free everything.  Cleanup is budgeted
     * (GVE_TX_CLEAN_BUDGET per call), so retire in a loop until the ring
     * is empty. */
    for (u16 t = 0; t <= tx->mask; t++)
        if (tx->seg_counts[t])
            dqo_dev_complete_pkt(&the_dev, t, GVE_DQO_COMPL_TYPE_PKT);
    for (int guard = 0; tx->compl_head < (u32)total && guard < total + 16; guard++)
        run_cleanup(tx);
    for (int t = 0; t < CONC_NPROD; t++)
        for (int i = 0; i < CONC_PERPROD; i++)
            pbuf_free(g_conc_tx_pkts[t][i]);
    CHECK(pbufs_live == live_before, "no pbuf leak across concurrent TX");
}

static gve_rx_dqo_queue g_conc_rx;
static void *conc_rx_service(void *arg)
{
    (void)arg;
    for (int i = 0; i < 50; i++)
        apply((thunk)&g_conc_rx->service);
    return NULL;
}

/* Scenario C2: two threads run the RX service concurrently while a batch of
 * completions is pending.  service_lock must serialise the body so the two
 * runs do not both consume the same completion or corrupt compl_head /
 * ctx_head / the free-id pool — the dual-source double-run the design
 * guards against.  Each packet must be delivered exactly once. */
static void scenario_conc_rx(void)
{
    rprintf("scenario_conc_rx\n");
    gve adapter = make_adapter();
    gve_rx_dqo_queue rx = &adapter->rx_dqo[0];
    make_rx_dqo(adapter, rx, 256);
    adapter->num_queues = 1;
    g_conc_rx = rx;

    int delivered_before = rx_delivered;
    const int N = 200;
    gve_rx_dqo_fill(rx);
    for (int i = 0; i < N; i++)
        dqo_dev_rx_complete(rx, 100, true, false);

    pthread_t a, b;
    pthread_create(&a, NULL, conc_rx_service, NULL);
    pthread_create(&b, NULL, conc_rx_service, NULL);
    pthread_join(a, NULL);
    pthread_join(b, NULL);

    CHECK(rx_delivered == delivered_before + N,
          "every completion delivered exactly once (%d of %d)",
          rx_delivered - delivered_before, N);
    CHECK(rx->compl_head == (u32)N, "compl_head consumed all completions (%u)",
          rx->compl_head);
    CHECK(rx->ctx_head == NULL, "no partial chain left after concurrent service");
    free_rx_dqo(rx);
}
#endif /* GVE_HARNESS_SMP */

/* Scenario 60: RX multiqueue — four RX queues each consume their own
 * completions independently (own compl_head / buffer pool) and deliver
 * packets tagged with their own per-queue napi_id. */
static void scenario_rx_multiqueue(void)
{
    rprintf("scenario_rx_multiqueue\n");
    gve adapter = make_adapter();
    adapter->num_queues = 4;
    for (int q = 0; q < 4; q++)
        make_rx_dqo(adapter, &adapter->rx_dqo[q], 256);
    int delivered_before = rx_delivered;
    int live_before = pbufs_live;
    int counts[4] = { 5, 12, 3, 20 };

    for (int q = 0; q < 4; q++) {
        gve_rx_dqo_queue rx = &adapter->rx_dqo[q];
        gve_rx_dqo_fill(rx);
        for (int i = 0; i < counts[q]; i++)
            dqo_dev_rx_complete(rx, 100, true, false);
    }
    int total = 0;
    for (int q = 0; q < 4; q++) {
        g_last_input_napi = 0xffff;
        run_rx_service(&adapter->rx_dqo[q]);
        CHECK(adapter->rx_dqo[q].compl_head == (u32)counts[q],
              "queue %d consumed exactly its %d completions (%u)",
              q, counts[q], adapter->rx_dqo[q].compl_head);
        CHECK(g_last_input_napi == (u16)(q + 1),
              "queue %d packets tagged napi_id %d (got %d)",
              q, q + 1, g_last_input_napi);
        total += counts[q];
    }
    CHECK(rx_delivered == delivered_before + total,
          "every queue delivered its packets (%d total)", total);
    for (int q = 0; q < 4; q++)
        free_rx_dqo(&adapter->rx_dqo[q]);
    CHECK(pbufs_live == live_before, "no leak across the RX queues");
}

/* Scenario 61: RX cleaning budget — one service call retires a bounded
 * number of completions (two sweeps of GVE_CLEAN_BUDGET x GVE_RX_BUDGET
 * around the IRQ re-arm), and a second call finishes the rest, so a single
 * run cannot monopolise the runloop CPU. */
static void scenario_rx_budget(void)
{
    rprintf("scenario_rx_budget\n");
    gve adapter = make_adapter();
    gve_rx_dqo_queue rx = &adapter->rx_dqo[0];
    make_rx_dqo(adapter, rx, 2048);
    gve_rx_dqo_fill(rx);
    const int per_call = 2 * GVE_CLEAN_BUDGET * GVE_RX_BUDGET;   /* 1024 */
    const int N = per_call + 100;
    for (int i = 0; i < N; i++)
        dqo_dev_rx_complete(rx, 100, true, false);

    run_rx_service(rx);
    CHECK(rx->compl_head == (u32)per_call,
          "one service call bounded to %d completions (got %u)",
          per_call, rx->compl_head);
    run_rx_service(rx);
    CHECK(rx->compl_head == (u32)N, "a second call drains the rest (%u)",
          rx->compl_head);
    free_rx_dqo(rx);
}

/* Scenario 62: TX cleaning budget — one cleanup retires at most
 * GVE_TX_CLEAN_BUDGET completions; the rest wait for the next call. */
static void scenario_tx_cleanup_budget(void)
{
    rprintf("scenario_tx_cleanup_budget\n");
    gve adapter = make_adapter();
    gve_tx_dqo_queue tx = &adapter->tx_dqo[0];
    make_tx_dqo(adapter, tx, 1024);
    int live_before = pbufs_live;
    const int N = 200;
    struct pbuf *pkts[200];
    u16 tags[200];
    for (int i = 0; i < N; i++) {
        pkts[i] = make_pkt(64);
        tags[i] = send_pkt(tx, adapter, pkts[i]);
    }
    for (int i = 0; i < N; i++)
        dqo_dev_complete_pkt(&the_dev, tags[i], GVE_DQO_COMPL_TYPE_PKT);

    run_cleanup(tx);
    CHECK(tx->compl_head == (u32)GVE_TX_CLEAN_BUDGET,
          "one cleanup bounded to %d completions (got %u)",
          GVE_TX_CLEAN_BUDGET, tx->compl_head);
    while (tx->compl_head < (u32)N)
        run_cleanup(tx);
    CHECK(tx->compl_head == (u32)N, "subsequent cleanups retire the rest");
    for (int i = 0; i < N; i++)
        pbuf_free(pkts[i]);
    CHECK(pbufs_live == live_before, "no leak across the budgeted cleanup");
}

/* Scenario 63: watchdog queue rotation — with 8 queues and
 * GVE_TX_MONITORED_QUEUES=4 checked per tick, the cursor advances by 4 each
 * tick and wraps after two ticks (a full sweep), so all queues are monitored
 * over ceil(N/4) ticks without checking them all every tick. */
static void scenario_watchdog_rotation(void)
{
    rprintf("scenario_watchdog_rotation\n");
    int saved_tp = total_processors;
    gve a = bring_up(GVE_DEV_OPT_ID_DQO_RDA, 8);
    CHECK(a->num_queues == 8, "8 queues for the rotation test (got %d)",
          a->num_queues);
    for (u32 q = 0; q < a->num_queues; q++)
        a->rx_dqo[q].first_interrupt = true;     /* avoid the no-interrupt reset */
    a->next_monitored_tx_qid = 0;
    u64 wd0 = a->dev_stats.wd_expired;

    apply((timer_handler)&a->watchdog_task, (u64)0, (u64)1);
    CHECK(a->next_monitored_tx_qid == GVE_TX_MONITORED_QUEUES,
          "tick 1 advanced the cursor to %d (got %d)",
          GVE_TX_MONITORED_QUEUES, a->next_monitored_tx_qid);
    apply((timer_handler)&a->watchdog_task, (u64)0, (u64)1);
    CHECK(a->next_monitored_tx_qid == 0,
          "tick 2 wrapped the cursor back to 0 (got %d)",
          a->next_monitored_tx_qid);
    CHECK(a->dev_stats.wd_expired == wd0, "healthy queues: no reset during rotation");
    total_processors = saved_tp;
}

/* Scenario 64: per-queue MSI-X layout — bring-up installs one mgmt IRQ at
 * slot 2N and one RX IRQ per queue at slots N..2N-1 (the TX notify slots
 * 0..N-1 are allocated but have no handler — TX completion is polled). */
static void scenario_msix_layout(void)
{
    rprintf("scenario_msix_layout\n");
    int saved_tp = total_processors;
    g_msix_nslots = 0;
    gve a = bring_up(GVE_DEV_OPT_ID_DQO_RDA, 4);
    u32 nq = a->num_queues;
    CHECK(nq == 4, "4 queues (got %d)", nq);
    CHECK(g_msix_nslots == (int)(nq + 1),
          "nq+1 MSI-X handlers installed (mgmt + one per RX queue), got %d",
          g_msix_nslots);
    CHECK(g_msix_slots[0] == (int)(2 * nq),
          "management IRQ at slot 2N=%d (got %d)", 2 * nq, g_msix_slots[0]);
    boolean rx_ok = true;
    for (u32 i = 0; i < nq; i++) {
        boolean found = false;
        for (int j = 1; j < g_msix_nslots; j++)
            if (g_msix_slots[j] == (int)(nq + i))
                found = true;
        if (!found)
            rx_ok = false;
    }
    CHECK(rx_ok, "each RX queue i has an MSI-X handler at slot N+i");
    total_processors = saved_tp;
}

int main(int argc, char **argv)
{
    gh = init_process_runtime();
    kernel_timers = allocate_timerqueue(gh, 0, ss("harness"));
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
    scenario_lifecycle();
    scenario_lifecycle_gqi_qpl();
    scenario_multiqueue_dispatch();
    scenario_multiqueue_rss();
    scenario_gqi_rx_errors();
    scenario_gqi_qpl_wrap();
    scenario_gqi_tx_backpressure();
    scenario_main_irqs();
    scenario_gqi_watchdog();
    scenario_watchdog_detect();
    scenario_setup_alloc_fail();
    scenario_setup_failpaths();
    scenario_watchdog_branches();
    scenario_tx_stale_types();
    scenario_tx_doorbell_batch();
    scenario_tx_enqueue_full();
    scenario_rx_dqo_edge();
    scenario_rx_gqi_edge();
    scenario_rx_dqo_qpl_edge();
    scenario_setup_fail_sweep();
    scenario_cmd_failures();
    scenario_adminq_timeout();
    scenario_watchdog_deadline_edges();
    scenario_gqi_qpl_multiseg();
    scenario_gqi_rx_midchain();
    scenario_dqo_rx_more();
    scenario_teardown_held();
    scenario_rx_fixed_buffer_split();
    scenario_tx_qpl_oversize();
    scenario_ring_size_restore();
    scenario_rss_key_fresh();
    scenario_no_csum_offload();
    scenario_dqo_report_event();
    scenario_dqo_desc_layout();
    scenario_doorbell_format();
    scenario_rx_fill_wrap();
    scenario_rx_multiqueue();
    scenario_rx_budget();
    scenario_tx_cleanup_budget();
    scenario_watchdog_rotation();
    scenario_msix_layout();
#ifdef GVE_HARNESS_SMP
    scenario_conc_tx();
    scenario_conc_rx();
#endif

    rprintf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
