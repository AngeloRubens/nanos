/* gve_priv.h — private types and declarations shared across gve_*.c
 *
 * Included by gve_main.c, gve_adminq.c, gve_datapath.c, and gve_dqo.c.
 * Not part of the public driver API (use gve.h for that).
 *
 * Queue formats supported:
 *   GQI-QPL   — TX/RX via registered bounce pages (QPL)
 *   GQI-RDA   — TX/RX via direct physical addresses (no QPL copy)
 *   DQO-RDA   — Andromeda 2.x: completely different descriptor set,
 *               separate TX-completion and RX-buffer/completion rings,
 *               little-endian, generation-bit polling.
 *
 * TSO is not implementable: lwIP always segments TCP at MSS before
 * calling linkoutput.  TX checksum offload is available for GQI
 * (GVE_TXF_L4CSUM) and DQO (checksum_offload_enable bit).
 */

#include <kernel.h>
#include <lwip.h>
#include <lwip/prot/ip4.h>
#include <lwip/prot/ip6.h>
#include <lwip/prot/tcp.h>
#include <lwip/prot/udp.h>
#include <netif/ethernet.h>
#include <pci.h>

#include "gve.h"

/* ------------------------------------------------------------------ */
/* Debug                                                                */
/* ------------------------------------------------------------------ */

//#define GVE_DEBUG
#ifdef GVE_DEBUG
#define gve_debug(x, ...) do { rprintf("GVE: " x "\n", ##__VA_ARGS__); } while (0)
#else
#define gve_debug(x, ...)
#endif

/* ------------------------------------------------------------------ */
/* PCI layout                                                           */
/* ------------------------------------------------------------------ */

#define PCI_VENDOR_ID_GOOGLE    0x1ae0
#define PCI_DEV_ID_GVNIC        0x0042

#define GVE_REGISTER_BAR    0
#define GVE_DOORBELL_BAR    2

/* Register BAR offsets */
#define GVE_REG_DEVICE_STATUS   0x00
#define GVE_REG_DRIVER_STATUS   0x04
#define GVE_REG_MAX_TX_QUEUES   0x08
#define GVE_REG_MAX_RX_QUEUES   0x0C
#define GVE_REG_ADMINQ_PFN      0x10
#define GVE_REG_ADMINQ_DOORBELL 0x14
#define GVE_REG_ADMINQ_EVT_CNT  0x18
#define GVE_REG_DRIVER_VERSION  0x1F

#define GVE_DEVICE_STATUS_RESET         htobe32(U32_FROM_BIT(1))
#define GVE_DEVICE_STATUS_LINK_STATUS   htobe32(U32_FROM_BIT(2))
#define GVE_DEVICE_STATUS_REPORT_STATS  htobe32(U32_FROM_BIT(3))

/* ------------------------------------------------------------------ */
/* Queue limits and driver constants                                    */
/* ------------------------------------------------------------------ */

/*
 * Maximum number of IO queue pairs the driver will allocate.
 * Each pair requires 2 MSI-X vectors (TX + RX); the management
 * vector is additional, so total MSI-X needed = 2*N + 1.
 */
#define GVE_MAX_IO_QUEUES   16

/* Watchdog: if TX event counter does not advance for this many ms,
 * declare the queue stuck and stop transmitting on it. */
#define GVE_TX_WATCHDOG_MS  5000

/* Cacheline boundary used for QPL copy padding.
 * DEFAULT_CACHELINE_SIZE is 64 on both x86_64 and aarch64 in nanos.
 * On GCP Axion (128-byte cacheline) bump this define if needed. */
#define GVE_TX_PAD  DEFAULT_CACHELINE_SIZE

/* Per-invocation RX budget: completions processed per inner loop pass.
 * CLEAN_BUDGET bounds the outer retry count when the ring saturates budget. */
#define GVE_RX_BUDGET    64
#define GVE_CLEAN_BUDGET  8

/* Sentinel QPL id: tells the device to use direct physical addresses
 * (GQI-RDA mode) rather than a registered page list. */
#define GVE_RAW_ADDRESSING_QPL_ID   0xFFFFFFFFu

/* Device option IDs advertised by the device in gve_device_descriptor. */
#define GVE_DEV_OPT_ID_GQI_RAW_ADDRESSING  0x1  /* legacy RDA option */
#define GVE_DEV_OPT_ID_GQI_RDA             0x2
#define GVE_DEV_OPT_ID_GQI_QPL             0x3
#define GVE_DEV_OPT_ID_DQO_RDA             0x4

/* ------------------------------------------------------------------ */
/* Admin queue opcodes and structs                                      */
/* ------------------------------------------------------------------ */

enum gve_adminq_opcode {
    GVE_ADMINQ_DESCRIBE_DEVICE = 1,
    GVE_ADMINQ_CONFIGURE_DEVICE_RESOURCES,
    GVE_ADMINQ_REGISTER_PAGE_LIST,
    GVE_ADMINQ_UNREGISTER_PAGE_LIST,
    GVE_ADMINQ_CREATE_TX_QUEUE,
    GVE_ADMINQ_CREATE_RX_QUEUE,
    GVE_ADMINQ_DESTROY_TX_QUEUE,
    GVE_ADMINQ_DESTROY_RX_QUEUE,
    GVE_ADMINQ_DECONFIGURE_DEVICE_RESOURCES,
    GVE_ADMINQ_SET_DRIVER_PARAMETER = 11,
    GVE_ADMINQ_REPORT_STATS,
    GVE_ADMINQ_REPORT_LINK_SPEED,
    GVE_ADMINQ_GET_PTYPE_MAP,
};

enum gve_adminq_status {
    GVE_ADMINQ_COMMAND_UNSET = 0,
    GVE_ADMINQ_COMMAND_PASSED,
    GVE_ADMINQ_COMMAND_ERROR_ABORTED = -16,
    GVE_ADMINQ_COMMAND_ERROR_ALREADY_EXISTS,
    GVE_ADMINQ_COMMAND_ERROR_CANCELLED,
    GVE_ADMINQ_COMMAND_ERROR_DATALOSS,
    GVE_ADMINQ_COMMAND_ERROR_DEADLINE_EXCEEDED,
    GVE_ADMINQ_COMMAND_ERROR_FAILED_PRECONDITION,
    GVE_ADMINQ_COMMAND_ERROR_INTERNAL_ERROR,
    GVE_ADMINQ_COMMAND_ERROR_INVALID_ARGUMENT,
    GVE_ADMINQ_COMMAND_ERROR_NOT_FOUND,
    GVE_ADMINQ_COMMAND_ERROR_OUT_OF_RANGE,
    GVE_ADMINQ_COMMAND_ERROR_PERMISSION_DENIED,
    GVE_ADMINQ_COMMAND_ERROR_UNAUTHENTICATED,
    GVE_ADMINQ_COMMAND_ERROR_RESOURCE_EXHAUSTED,
    GVE_ADMINQ_COMMAND_ERROR_UNAVAILABLE,
    GVE_ADMINQ_COMMAND_ERROR_UNIMPLEMENTED,
    GVE_ADMINQ_COMMAND_ERROR_UNKNOWN_ERROR,
};

struct gve_device_descriptor {
    u64 max_registered_pages;
    u16 reserved1;
    u16 tx_queue_entries;
    u16 rx_queue_entries;
    u16 default_num_queues;
    u16 mtu;
    u16 counters;
    u16 tx_pages_per_qpl;
    u16 rx_pages_per_qpl;
    u8  mac[ETH_HWADDR_LEN];
    u16 num_device_options;
    u16 total_length;
    u8  reserved2[6];
} __attribute__((packed));

/* Variable-length option records that follow gve_device_descriptor.
 * option_length bytes of option-specific data follow each header. */
struct gve_device_option {
    u16 option_id;
    u16 option_length;
    u32 required_features_mask;
} __attribute__((packed));

struct gve_adminq_describe_device {
    u64 device_descriptor_addr;
    u32 device_descriptor_version;
    u32 available_length;
} __attribute__((packed));

enum gve_queue_format {
    GVE_QUEUE_FORMAT_UNSPECIFIED = 0,
    GVE_GQI_RDA_FORMAT,
    GVE_GQI_QPL_FORMAT,
    GVE_DQO_RDA_FORMAT,
};

struct gve_adminq_configure_device_resources {
    u64 counter_array;
    u64 irq_db_addr;
    u32 num_counters;
    u32 num_irq_dbs;
    u32 irq_db_stride;
    u32 ntfy_blk_msix_base_idx;
    u8  queue_format;   /* enum gve_queue_format */
} __attribute__((packed));

struct gve_adminq_register_page_list {
    u32 page_list_id;
    u32 num_pages;
    u64 page_address_list_addr;
    u64 page_size;
} __attribute__((packed));

struct gve_adminq_unregister_page_list {
    u32 page_list_id;
} __attribute__((packed));

struct gve_adminq_create_tx_queue {
    u32 queue_id;
    u32 reserved;
    u64 queue_resources_addr;
    u64 tx_ring_addr;
    u32 queue_page_list_id;
    u32 ntfy_id;
    u64 tx_comp_ring_addr;
    u16 tx_ring_size;
    u16 tx_comp_ring_size;
    u8  padding[4];
} __attribute__((packed));

struct gve_adminq_create_rx_queue {
    u32 queue_id;
    u32 index;
    u32 reserved;
    u32 ntfy_id;
    u64 queue_resources_addr;
    u64 rx_desc_ring_addr;
    u64 rx_data_ring_addr;
    u32 queue_page_list_id;
    u16 rx_ring_size;
    u16 packet_buffer_size;
    u16 rx_buff_ring_size;
    u8  enable_rsc;
    u8  padding[5];
} __attribute__((packed));

struct gve_adminq_destroy_tx_queue {
    u32 queue_id;
} __attribute__((packed));

struct gve_adminq_destroy_rx_queue {
    u32 queue_id;
} __attribute__((packed));

struct gve_adminq_command {
    u32 opcode;
    u32 status;
    union {
        struct gve_adminq_describe_device           describe_device;
        struct gve_adminq_configure_device_resources cfg_dev_resources;
        struct gve_adminq_register_page_list        register_page_list;
        struct gve_adminq_unregister_page_list      unregister_page_list;
        struct gve_adminq_create_tx_queue           create_tx_queue;
        struct gve_adminq_create_rx_queue           create_rx_queue;
        struct gve_adminq_destroy_tx_queue          destroy_tx_queue;
        struct gve_adminq_destroy_rx_queue          destroy_rx_queue;
        u8 padding[56];     /* struct size = 64 bytes */
    };
} __attribute__((packed));

/* ------------------------------------------------------------------ */
/* IRQ / event counter structs                                          */
/* ------------------------------------------------------------------ */

#define GVE_IRQ_EVENT   htobe32(U32_FROM_BIT(29))
#define GVE_IRQ_MASK    htobe32(U32_FROM_BIT(30))
#define GVE_IRQ_ACK     htobe32(U32_FROM_BIT(31))

/*
 * IRQ DB slots: one TX + one RX per queue pair, plus one management.
 * Layout: [0 .. num_queues-1] = TX, [num_queues .. 2*num_queues-1] = RX,
 *         [2*num_queues] = management.
 */
#define GVE_IRQ_DB_TX(n, q)     ((q))
#define GVE_IRQ_DB_RX(n, q)     ((n) + (q))
#define GVE_IRQ_DB_MGMT(n)      (2 * (n))
#define GVE_IRQ_DB_COUNT(n)     (2 * (n) + 1)

struct gve_irq_db {
    u32 index;
} __attribute__((aligned(64)));   /* device expects 64-byte alignment */

struct gve_queue_resources {
    u32 db_index;
    u32 counter_index;
    u8  reserved[56];
} __attribute__((packed));

/* ------------------------------------------------------------------ */
/* GQI TX descriptor formats                                            */
/* ------------------------------------------------------------------ */

#define GVE_TXD_STD 0x00
#define GVE_TXD_TSO 0x10   /* TSO: not used (lwIP always segments at MSS) */
#define GVE_TXD_SEG 0x20
#define GVE_TXD_MTD 0x30

/* GQI TX checksum offload: set in type_flags to enable CHECKSUM_PARTIAL.
 * Driver seeds the L4 checksum field with the pseudo-header sum;
 * device adds the payload checksum and completes the field. */
#define GVE_TXF_L4CSUM  0x01

/* ------------------------------------------------------------------ */
/* DQO descriptor formats (Andromeda 2.x, little-endian)               */
/* ------------------------------------------------------------------ */

/*
 * DQO TX packet descriptor — 16 bytes, little-endian.
 * dtype_flags byte: bits[4:0]=dtype, bit[5]=end_of_packet,
 *                   bit[6]=checksum_offload_enable, bit[7]=report_event.
 */
struct gve_tx_pkt_desc_dqo {
    u64 buf_addr;
    u8  dtype_flags;
    u8  reserved0;
    u16 reserved1;
    u16 compl_tag;  /* echoed back in TX completion */
    u16 buf_size;   /* bits[13:0]=bytes */
} __attribute__((packed));

/* dtype must be GVE_DQO_TX_DTYPE_PKT (0xc) for packet descriptors. */
#define GVE_DQO_TX_DTYPE_PKT    0x0cu
#define GVE_DQO_TX_EOP          0x20u   /* end_of_packet */
#define GVE_DQO_TX_CSUM_EN      0x40u   /* checksum_offload_enable */
#define GVE_DQO_TX_REPORT       0x80u   /* report_event: generate TX completion */

/*
 * DQO TX completion descriptor — 8 bytes, little-endian.
 *
 * Layout matches Linux gve_tx_compl_desc (gve_desc_dqo.h):
 *   bytes 0-1: id[10:0] | type[13:11] | reserved[14] | generation[15]
 *   bytes 2-3: completion_tag echoed from TX packet descriptor
 *   bytes 4-7: reserved
 *
 * generation is bit 15 of id_type_gen; type is bits[13:11].
 */
struct gve_tx_compl_desc_dqo {
    u16 id_type_gen;      /* bytes 0-1: id | type | reserved | generation */
    u16 completion_tag;   /* bytes 2-3: compl_tag echoed from TX descriptor */
    u32 reserved;
} __attribute__((packed));

/* Alternate miss completion encoding: hardware may set bit 15 of
 * completion_tag instead of using type=GVE_DQO_COMPL_TYPE_MISS. */
#define GVE_DQO_ALT_MISS_COMPL_BIT  0x8000u  /* bit 15 of completion_tag */

#define GVE_DQO_COMPL_GEN_BIT       0x8000u  /* bit 15 of id_type_gen */
#define GVE_DQO_COMPL_TYPE_SHIFT    11       /* type field in bits[13:11] */
#define GVE_DQO_COMPL_TYPE_PKT      0x2u     /* normal packet completion */
#define GVE_DQO_COMPL_TYPE_DESC     0x4u     /* descriptor completion (context descriptors) */
#define GVE_DQO_COMPL_TYPE_MISS     0x1u     /* miss path: device buffered packet */
#define GVE_DQO_COMPL_TYPE_REINJECT 0x3u     /* reinjection: buffered packet delivered */

/*
 * DQO RX buffer descriptor — 32 bytes, little-endian.
 * Matches Linux gve_rx_desc_dqo.  Driver posts these to tell the device
 * where to write incoming packet data.  header_buf_addr is zeroed because
 * split-header mode is not used.
 */
struct gve_rx_buf_desc_dqo {
    u16 buf_id;           /* returned in RX completion to identify buffer */
    u16 reserved0;
    u32 reserved1;
    u64 buf_addr;         /* physical address of the receive buffer */
    u64 header_buf_addr;  /* 0 = no separate header buffer */
    u64 reserved2;
} __attribute__((packed));

/*
 * DQO RX completion descriptor — 32 bytes, little-endian.
 * Matches Linux gve_rx_compl_desc_dqo.
 *
 * Byte layout (driver-relevant fields only):
 *   byte  0:     rxdid[3:0] | reserved[7:4]
 *   byte  1:     loopback | ipv6_ex_add | rx_error[2] | reserved[7:3]
 *   bytes 2-3:   packet_type | errors (unused by driver)
 *   bytes 4-5:   packet_len[13:0] | generation[14] | buffer_queue_id[15]
 *   bytes 6-11:  header_len, flags (unused by driver)
 *   bytes 12-13: buf_id
 *   bytes 14-31: hash, timestamps, reserved
 */
struct gve_rx_compl_desc_dqo {
    u8   reserved0;       /* byte  0 */
    u8   err_flags;       /* byte  1: bit[2]=rx_error */
    u16  reserved1;       /* bytes 2-3 */
    u16  pkt_len_gen;     /* bytes 4-5: packet_len[13:0] | generation[14] */
    u16  reserved2;       /* bytes 6-7 */
    u8   reserved3[4];    /* bytes 8-11 */
    u16  buf_id;          /* bytes 12-13 */
    u8   reserved4[18];   /* bytes 14-31 */
} __attribute__((packed));

#define GVE_DQO_RX_GEN          0x4000u  /* generation bit in pkt_len_gen */
#define GVE_DQO_RX_PKT_LEN_MASK 0x3FFFu  /* packet_len field in pkt_len_gen */
#define GVE_DQO_RX_ERR          0x04u    /* rx_error bit in err_flags */

/* Per-buffer size used for DQO RX.  Sized for standard MTU 1500:
 * 1500 + 14 (Ethernet) + 2 (GVE_RX_PADDING) = 1516 < 2048.
 * Jumbo frames (MTU 9000) are not supported in this driver. */
#define GVE_DQO_BUF_SIZE    (PAGESIZE / 2)

struct gve_tx_pkt_desc {
    u8  type_flags;
    u8  l4_csum_offset;
    u8  l4_hdr_offset;
    u8  desc_cnt;
    u16 len;
    u16 seg_len;
    u64 seg_addr;
} __attribute__((packed));

struct gve_tx_seg_desc {
    u8  type_flags;
    u8  l3_offset;
    u16 reserved;
    u16 mss;
    u16 seg_len;
    u64 seg_addr;
} __attribute__((packed));

/* ------------------------------------------------------------------ */
/* RX descriptor format                                                 */
/* ------------------------------------------------------------------ */

#define GVE_RXF_FRAG        htobe16(1 << 6)
#define GVE_RXF_IPV4        htobe16(1 << 7)
#define GVE_RXF_IPV6        htobe16(1 << 8)
#define GVE_RXF_TCP         htobe16(1 << 9)
#define GVE_RXF_UDP         htobe16(1 << 10)
#define GVE_RXF_ERR         htobe16(1 << 11)
#define GVE_RXF_PKT_CONT    htobe16(1 << 13)

/* 2-byte pad prepended by device to align IP header */
#define GVE_RX_PADDING  2

struct gve_rx_desc {
    u8  padding[48];
    u32 rss_hash;
    u16 mss;
    u16 reserved;
    u8  hdr_len;
    u8  hdr_off;
    u16 csum;
    u16 len;
    u16 flags_seq;
} __attribute__((packed));

/* ------------------------------------------------------------------ */
/* Per-queue and adapter statistics                                     */
/* ------------------------------------------------------------------ */

struct gve_stats_tx {
    u64 cnt;              /* packets sent */
    u64 bytes;            /* bytes sent */
    u64 bad_compl_tag;    /* completion with invalid/unexpected tag */
    u64 queue_stop;       /* times TX ring stopped (HW ring full) */
    u64 queue_wakeup;     /* times TX ring woken after stop */
    u64 missing_tx_comp;  /* packets past completion deadline */
    u64 doorbells;        /* doorbell writes issued */
};

struct gve_stats_rx {
    u64 cnt;            /* packets received */
    u64 bytes;          /* bytes received */
    u64 rx_copy;        /* packets received via copy path */
    u64 rx_dropped;     /* packets dropped (error or alloc failure) */
    u64 refil_partial;  /* RX refill posted fewer buffers than requested */
    u64 empty_rx_ring;  /* watchdog-triggered refill (ring fully empty) */
    u64 bad_req_id;     /* completion with out-of-range buffer id */
};

struct gve_stats_dev {
    u64 wd_expired;     /* watchdog-triggered resets */
};

struct gve_hw_stats {
    u64 rx_packets;
    u64 tx_packets;
    u64 rx_bytes;
    u64 tx_bytes;
};

/* TX software queue constants (buf_ring pattern, same as ENA). */
#define GVE_BUF_RING_SIZE       4096   /* software TX queue depth */
#define GVE_TX_STOP_THRESH      4      /* stop when < this many HW slots free */
#define GVE_TX_RESUME_THRESH    8      /* wake when >= this many HW slots free */
#define GVE_TX_DOORBELL_BATCH   64     /* max packets per doorbell write */

/* Adapter-level atomic flags (use atomic_test_and_set_bit / atomic_clear_bit). */
#define GVE_FLAG_RESETTING      0      /* reset has been scheduled */
#define GVE_FLAG_ONGOING_RESET  1      /* reset is actively running */

/* ------------------------------------------------------------------ */
/* GQI per-queue structs                                                */
/* ------------------------------------------------------------------ */

typedef struct gve_tx_queue {
    u32 head, tail;

    /*
     * QPL (queue page list) state.
     *
     * qpl_used tracks how many QPL bytes are currently in-flight
     * (owned by the device).  This replaces the old "qpl_available"
     * / wrap-compare idiom that had the integer-overflow bug described
     * in the file header.
     *
     * qpl_head is the write cursor into the QPL ring (bytes).
     * qpl_size is the total QPL size in bytes (num_pages * PAGESIZE).
     * qpl_used <= qpl_size always.
     */
    u32 qpl_head;       /* next byte to write in the QPL ring */
    u32 qpl_used;       /* bytes currently owned by device */
    u32 qpl_size;       /* total QPL ring size in bytes */

    struct gve *adapter;
    u16  mask;
    void *qpl_base;

    union {
        struct gve_tx_pkt_desc pkt;
        struct gve_tx_seg_desc seg;
    } *desc;

    u32 *qpl_allocated;     /* QPL mode: bytes allocated per descriptor slot */
    struct pbuf **pending;  /* RDA mode: in-flight pbuf per descriptor slot */
    struct gve_queue_resources *q_res;

    struct gve_stats_tx tx_stats;

    /* Watchdog: timestamp of last successful TX event-counter advance */
    timestamp last_completion;
    boolean   stuck;

    /* Software TX queue (buf_ring pattern, same as ENA). */
    queue            br;          /* software queue absorbs burst when HW ring full */
    boolean          running;     /* false = HW ring full, drain paused */
    u32              acum_pkts;   /* packets batched since last doorbell */
    struct spinlock  ring_mtx;
    closure_struct(thunk, enqueue_task);
} *gve_tx_queue;

typedef struct gve_rx_queue {
    struct spinlock lock;
    u32 head, tail;
    u32 qpl_head, qpl_available;
    u64 rda_base_phys;  /* RDA: physical_from_virtual(qpl_base); 0 in QPL mode */
    struct gve *adapter;
    u16  mask;
    void *qpl_base;
    u32  qpl_count;
    struct gve_rx_desc       *desc;
    u64                      *data;
    struct pbuf              *pbufs;
    u32                      *irq_db_index;
    closure_struct(thunk, irq_handler);
    closure_struct(thunk, service);
    struct gve_queue_resources *q_res;

    struct gve_stats_rx rx_stats;

    /* Watchdog: MSIX miss and empty-ring detection (same as ENA). */
    boolean first_interrupt;      /* set to true on first IRQ arrival */
    u16   no_interrupt_event_cnt; /* increments if completions arrive but no IRQ */
    int   empty_rx_queue;         /* consecutive watchdog ticks with ring empty */
} *gve_rx_queue;

/* ------------------------------------------------------------------ */
/* DQO per-queue structs                                                */
/* ------------------------------------------------------------------ */

/*
 * DQO TX queue.
 *
 * TX descriptors are submitted to desc[] and advanced via head.
 * The device writes completions to compl[], which the driver reads
 * via compl_head.  expected_gen tracks the generation bit convention:
 * 0 → 1 → 0 → ... with each full pass through the ring.
 *
 * desc_tail tracks how many descriptor ring slots have been freed.
 * A completion for a packet of seg_count descriptors frees seg_count
 * slots.  We store seg_count in seg_counts[] indexed by compl_tag so
 * that gve_tx_dqo_cleanup can advance desc_tail by the right amount.
 * Space check: head - desc_tail + seg_count <= desc_cnt.
 */
typedef struct gve_tx_dqo_queue {
    u32  head;               /* next descriptor slot to write */
    u32  desc_tail;          /* descriptor slots retired (freed) */
    u32  compl_head;         /* next completion ring entry to read */
    u8   expected_gen;       /* expected generation bit (0 or 1) */

    struct gve *adapter;
    u16  mask;               /* desc_cnt - 1 */

    struct gve_tx_pkt_desc_dqo  *desc;
    struct gve_tx_compl_desc_dqo *compl;
    struct pbuf                 **pending;    /* pbuf per compl_tag slot */
    u16                         *seg_counts;  /* seg_count per compl_tag slot */
    timestamp                   *miss_times;  /* non-zero after miss, cleared on reinject */
    u16                          pending_misses;
    struct gve_queue_resources  *q_res;

    struct gve_stats_tx tx_stats;

    timestamp last_completion;
    boolean   stuck;

    queue            br;
    boolean          running;
    u32              acum_pkts;
    struct spinlock  ring_mtx;
    closure_struct(thunk, enqueue_task);
} *gve_tx_dqo_queue;

/*
 * DQO RX queue.
 *
 * buf_ring[] is posted to the device (buffer descriptors).
 * compl_ring[] is written by the device (completion descriptors).
 * buf_head tracks the next slot in buf_ring to fill.
 * compl_head tracks the next completion to consume.
 * expected_gen tracks generation bit convention.
 */
typedef struct gve_rx_dqo_queue {
    struct spinlock lock;
    u32  buf_head;           /* next buf_ring slot to post */
    u32  compl_head;         /* next compl_ring entry to consume */
    u8   expected_gen;       /* expected generation bit */

    struct gve *adapter;
    u16  mask;               /* num_bufs - 1 */
    u32  num_bufs;

    void *qpl_base;          /* virtual base of all RX buffers */
    u64  rda_base_phys;      /* physical_from_virtual(qpl_base) */

    struct gve_rx_buf_desc_dqo   *buf_ring;
    struct gve_rx_compl_desc_dqo *compl_ring;
    struct pbuf                  *pbufs;     /* one PBUF_REF per buf slot */

    u32 *irq_db_index;
    closure_struct(thunk, irq_handler);
    closure_struct(thunk, service);
    struct gve_queue_resources *q_res;

    struct gve_stats_rx rx_stats;

    boolean first_interrupt;
    u16   no_interrupt_event_cnt;
    int   empty_rx_queue;
} *gve_rx_dqo_queue;

/* ------------------------------------------------------------------ */
/* Adapter struct                                                       */
/* ------------------------------------------------------------------ */

typedef struct gve {
    struct netif_dev ndev;
    heap general, contiguous;
    pci_dev pdev;
    struct pci_bar reg_bar;
    struct pci_bar db_bar;
    struct gve_adminq_command *adminq;
    u32    adminq_head;
    u32    adminq_mask;
    u16    tx_desc_cnt, rx_desc_cnt;
    u16    tx_pages_per_qpl, rx_data_slot_cnt;
    u16    num_event_counters;
    u32   *event_counters;
    struct gve_irq_db *irq_db_indices;
    u32    num_queues;
    boolean raw_addressing;  /* GQI-RDA when true; GQI-QPL when false */
    boolean dqo;             /* DQO format (Andromeda 2.x) overrides GQI */

    /* GQI queues (used when !dqo) */
    struct gve_tx_queue tx[GVE_MAX_IO_QUEUES];
    struct gve_rx_queue rx[GVE_MAX_IO_QUEUES];

    /* DQO queues (used when dqo) */
    struct gve_tx_dqo_queue tx_dqo[GVE_MAX_IO_QUEUES];
    struct gve_rx_dqo_queue rx_dqo[GVE_MAX_IO_QUEUES];

    closure_struct(thunk, mgmt_irq_handler);
    closure_struct(thunk, link_up_task);
    closure_struct(thunk, link_down_task);
    closure_struct(thunk, reset_handler);
    u64 flags;                    /* GVE_FLAG_* bits, accessed atomically */
    struct spinlock global_lock;  /* held during full reset sequence */
    struct timer watchdog_timer;
    closure_struct(timer_handler, watchdog_task);
    struct gve_hw_stats hw_stats;
    struct gve_stats_dev dev_stats;
    u16 mtu;
} *gve;

/* Schedule a reset exactly once; subsequent callers are no-ops. */
static inline void gve_trigger_reset(gve adapter)
{
    if (!atomic_test_and_set_bit(&adapter->flags, GVE_FLAG_RESETTING))
        async_apply_bh((thunk)&adapter->reset_handler);
}

/* ------------------------------------------------------------------ */
/* Checksum offload shared helper                                       */
/* ------------------------------------------------------------------ */

#define GVE_ETHERTYPE_IP4   0x0800u
#define GVE_ETHERTYPE_IP6   0x86DDu
#define GVE_IP_PROTO_TCP    6u
#define GVE_IP_PROTO_UDP    17u

/* gve_pseudo_csum — parse pbuf for TCP/UDP, compute pseudo-header
 * checksum and seed the L4 checksum field (CHECKSUM_PARTIAL model).
 * Returns false for non-TCP/UDP or too-short packets.
 * On true: L4 checksum field already seeded; *proto and *l4_off filled
 * for callers that need them to program TX descriptor offload fields. */
static inline boolean gve_pseudo_csum(struct pbuf *p,
                                       u8 *proto_out, u16_t *l4_off_out)
{
    if (p->len < 14 + 20)
        return false;

    u8 *frame = (u8 *)p->payload;
    u16_t ethertype = ((u16_t)frame[12] << 8) | frame[13];
    u8    proto;
    u16_t l4_hdr_off;
    u32_t sum = 0;

    if (ethertype == GVE_ETHERTYPE_IP4) {
        struct ip_hdr *iph = (struct ip_hdr *)(frame + 14);
        proto = IPH_PROTO(iph);
        if (proto != GVE_IP_PROTO_TCP && proto != GVE_IP_PROTO_UDP)
            return false;
        u8_t ihl = IPH_HL_BYTES(iph);
        l4_hdr_off = 14 + ihl;
        u16_t l4_len = be16toh(IPH_LEN(iph)) - ihl;
        const u8_t *sb = (const u8_t *)&iph->src;
        const u8_t *db = (const u8_t *)&iph->dest;
        sum = (((u32_t)sb[0] << 8) | sb[1]) + (((u32_t)sb[2] << 8) | sb[3]);
        sum += (((u32_t)db[0] << 8) | db[1]) + (((u32_t)db[2] << 8) | db[3]);
        sum += (u32_t)proto + (u32_t)l4_len;
    } else if (ethertype == GVE_ETHERTYPE_IP6) {
        if (p->len < 14 + 40)
            return false;
        struct ip6_hdr *ip6h = (struct ip6_hdr *)(frame + 14);
        proto = IP6H_NEXTH(ip6h);
        if (proto != GVE_IP_PROTO_TCP && proto != GVE_IP_PROTO_UDP)
            return false;
        l4_hdr_off = 14 + 40;
        u16_t l4_len = IP6H_PLEN(ip6h);
        const u8_t *sb = (const u8_t *)ip6h->src.addr;
        const u8_t *db = (const u8_t *)ip6h->dest.addr;
        for (int i = 0; i < 8; i++) {
            sum += (((u32_t)sb[2*i] << 8) | sb[2*i+1]);
            sum += (((u32_t)db[2*i] << 8) | db[2*i+1]);
        }
        sum += (u32_t)l4_len + (u32_t)proto;
    } else {
        return false;
    }

    while (sum >> 16) sum = (sum & 0xffff) + (sum >> 16);
    u16_t pseudo_csum = htobe16(~(u16_t)sum);

    if (proto == GVE_IP_PROTO_TCP)
        ((struct tcp_hdr *)(frame + l4_hdr_off))->chksum = pseudo_csum;
    else
        ((struct udp_hdr *)(frame + l4_hdr_off))->chksum = pseudo_csum;

    *proto_out  = proto;
    *l4_off_out = l4_hdr_off;
    return true;
}

/* ------------------------------------------------------------------ */
/* Inter-file function declarations                                     */
/* ------------------------------------------------------------------ */

/* gve_adminq.c */
boolean gve_describe_device(gve adapter);
boolean gve_cfg_device_resources(gve adapter);
void    gve_free_device_resources(gve adapter);
u32     gve_calc_num_queues(gve adapter, tuple config);
boolean gve_setup_queues(gve adapter);
void    gve_teardown_queues(gve adapter);

/* gve_datapath.c */
void  gve_setup_linkoutput(gve adapter, struct netif *netif);
void  gve_rx_fill(gve_rx_queue rx);
void  gve_rx_init(gve_rx_queue rx);
void  gve_tx_init_gqi(gve_tx_queue tx);

/* gve_dqo.c */
err_t gve_linkoutput_dqo(struct netif *netif, struct pbuf *p);
void  gve_rx_dqo_fill(gve_rx_dqo_queue rx);
void  gve_rx_dqo_init(gve_rx_dqo_queue rx);
void  gve_tx_init_dqo(gve_tx_dqo_queue tx);
