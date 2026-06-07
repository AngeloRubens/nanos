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
 * calling linkoutput.  TX/RX checksums are computed in software by lwIP
 * (same model as the ENA driver): no hardware checksum offload.
 */

#include <kernel.h>
#include <lwip.h>
#include <net/net.h>
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

/* Watchdog: if a TX descriptor slot has been outstanding for this many ms
 * with no completion, declare the queue stuck and schedule a reset. */
#define GVE_TX_WATCHDOG_MS          5000

/* Watchdog timer fires at this interval (matches ENA 1-second cadence).
 * Decoupled from GVE_TX_WATCHDOG_MS so the deadline and polling frequency
 * can be tuned independently. */
#define GVE_WATCHDOG_INTERVAL_MS    1000

/* If this many consecutive watchdog ticks pass without an RX interrupt,
 * the MSI-X vector may have been lost — schedule a reset. */
#define GVE_MAX_NO_INTERRUPT_ITERATIONS  3

/* Number of per-slot TX timeouts required to trigger a reset.
 * Matches ENA DEFAULT_TX_CMP_THRESHOLD: avoids spurious resets from
 * transient single-slot stalls; requires sustained queue exhaustion. */
#define GVE_TX_STUCK_THRESHOLD          128

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

/* Admin queue polling: exponential backoff from GVE_ADMINQ_MIN_POLL_US to
 * GVE_ADMINQ_MAX_POLL_US (mirrors ENA ena_delay_exponential_backoff_us).
 * On timeout, gve_adminq_wait sets adminq_running = false. */
#define GVE_ADMINQ_MIN_POLL_US  100
#define GVE_ADMINQ_MAX_POLL_US  5000

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
    GVE_ADMINQ_VERIFY_DRIVER_COMPATIBILITY = 15,
};

/* Driver capability flags advertised in VERIFY_DRIVER_COMPATIBILITY.  We
 * declare only the queue formats we actually implement, so the device never
 * offers one we cannot handle (e.g. DQO-QPL, flexible buffer sizes). */
#define GVE_CAP1_GQI_QPL    (1ull << 0)
#define GVE_CAP1_GQI_RDA    (1ull << 1)
#define GVE_CAP1_DQO_RDA    (1ull << 3)
#define GVE_DRIVER_CAPABILITY_FLAGS1 \
    (GVE_CAP1_GQI_QPL | GVE_CAP1_GQI_RDA | GVE_CAP1_DQO_RDA)

#define GVE_VERSION_STR_LEN 128

struct gve_driver_info {
    u8  os_type;        /* 1 = Linux-compatible protocol */
    u8  driver_major;
    u8  driver_minor;
    u8  driver_sub;
    u32 os_version_major;
    u32 os_version_minor;
    u32 os_version_sub;
    u64 driver_capability_flags[4];
    u8  os_version_str1[GVE_VERSION_STR_LEN];
    u8  os_version_str2[GVE_VERSION_STR_LEN];
} __attribute__((packed));

struct gve_adminq_verify_driver_compatibility {
    u64 driver_info_len;
    u64 driver_info_addr;
} __attribute__((packed));

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

struct gve_adminq_get_ptype_map {
    u64 ptype_map_len;
    u64 ptype_map_addr;
} __attribute__((packed));

/* Packet-type map: GVE_NUM_PTYPES (10-bit space) entries of 2 bytes each. */
#define GVE_NUM_PTYPES      1024
#define GVE_PTYPE_MAP_SIZE  (GVE_NUM_PTYPES * 2)

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
        struct gve_adminq_get_ptype_map             get_ptype_map;
        struct gve_adminq_verify_driver_compatibility verify_driver_compat;
        u8 padding[56];     /* struct size = 64 bytes */
    };
} __attribute__((packed));

/* ------------------------------------------------------------------ */
/* IRQ / event counter structs                                          */
/* ------------------------------------------------------------------ */

#define GVE_IRQ_ACK     htobe32(U32_FROM_BIT(31))
#define GVE_IRQ_MASK    htobe32(U32_FROM_BIT(30))
#define GVE_IRQ_EVENT   htobe32(U32_FROM_BIT(29))

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
#define GVE_DQO_TX_REPORT       0x80u   /* report_event: request a DESC completion */

/*
 * DQO TX general context descriptor — 16 bytes, little-endian.
 * The device requires a context descriptor (dtype 0x4) ahead of the packet
 * descriptors of every TX packet (matches the official Google driver, which
 * always emits gve_tx_general_context_desc_dqo).  We carry no metadata, so
 * the flex fields are left zero; only cmd_dtype must be set.
 */
struct gve_tx_ctx_desc_dqo {
    u8  flex_hi[8];     /* bytes 0-7: metadata flex (zero) */
    u8  cmd_dtype;      /* byte 8: dtype[4:0] | tso[5] */
    u8  reserved0;      /* byte 9 */
    u16 reserved1;      /* bytes 10-11 */
    u8  flex_lo[4];     /* bytes 12-15: metadata flex (zero) */
} __attribute__((packed));

#define GVE_DQO_TX_DTYPE_CTX    0x04u   /* general context descriptor dtype */

/* report_event spacing: the device requires DESC completions (requested via
 * report_event) to be at least this many descriptors apart (Google
 * GVE_TX_MIN_RE_INTERVAL). */
#define GVE_TX_MIN_RE_INTERVAL  32

/* Max data descriptors per TX packet (Google GVE_TX_MAX_DATA_DESCS). */
#define GVE_TX_MAX_DATA_DESCS   10

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
#define GVE_TX_RESUME_THRESH    8      /* wake when >= this many HW slots free */
#define GVE_TX_DOORBELL_BATCH   64     /* max packets per doorbell write */

/* TX completion budget: maximum completions retired per gve_tx_cleanup call
 * (matches ENA TX_BUDGET=128).  Bounds the time spent in a single cleanup
 * invocation so that long completion bursts do not starve other BH work. */
#define GVE_TX_CLEAN_BUDGET     128

/* Watchdog queue rotation: number of TX/RX queue pairs checked per watchdog
 * tick (matches ENA DEFAULT_TX_MONITORED_QUEUES=4).  A full pass over all
 * queues completes in ceil(num_queues / GVE_TX_MONITORED_QUEUES) ticks. */
#define GVE_TX_MONITORED_QUEUES 4

/* Ring size backoff: minimum ring size when allocation fails under memory
 * pressure.  Must be a power-of-two; device-reported sizes are always
 * powers-of-two so halving keeps the invariant. */
#define GVE_MIN_RING_SIZE       64

/* Adapter-level atomic flags (use atomic_test_and_set_bit / atomic_clear_bit). */
#define GVE_FLAG_RESETTING      0      /* reset has been scheduled */
#define GVE_FLAG_ONGOING_RESET  1      /* reset is actively running */
#define GVE_FLAG_DEVICE_RUNNING 2      /* queues allocated and device operational */

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

    /* Watchdog: per-slot submission timestamp (same model as ENA tx_buf->timestamp).
     * Set when a descriptor slot is posted; cleared on completion.
     * Watchdog iterates outstanding slots [tail, head) and flags any slot
     * whose age exceeds GVE_TX_WATCHDOG_MS. */
    timestamp *tx_timestamps;
    boolean    stuck;
    u32        event_counter_idx;  /* cached from q_res->counter_index at create */
    u32        db_idx;             /* cached from q_res->db_index at create */

    /* Software TX queue (buf_ring pattern, same as ENA). */
    queue            br;          /* software queue absorbs burst when HW ring full */
    boolean          running;     /* false = HW ring full, drain paused */
    struct spinlock  ring_mtx;
    closure_struct(thunk, enqueue_task);
} *gve_tx_queue;

typedef struct gve_rx_queue {
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
    u32   event_counter_idx;      /* cached from q_res->counter_index at create */
    u32   db_idx;                 /* cached from q_res->db_index at create */
    u16   idx;                    /* RX queue index, for SO_INCOMING_NAPI_ID */
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

    /* Watchdog: per-slot submission timestamp (same model as GQI above). */
    timestamp *tx_timestamps;
    boolean    stuck;

    u32              last_re_idx;  /* desc idx of last report_event (DESC compl spacing) */

    queue            br;
    boolean          running;
    u32              db_idx;   /* cached from q_res->db_index at create */
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
    u32  buf_head;           /* next buf_ring slot to post */
    u32  compl_head;         /* next compl_ring entry to consume */
    u8   expected_gen;       /* expected generation bit */

    struct gve *adapter;
    u16  mask;               /* num_bufs - 1 */
    u32  num_bufs;

    struct gve_rx_buf_desc_dqo   *buf_ring;
    struct gve_rx_compl_desc_dqo *compl_ring;
    struct pbuf                 **pbufs;     /* per-slot pbuf pointer (NULL when free) */
    u16                          *free_ids;  /* circular free-buffer-id list */
    u32                           next_to_use;   /* free_ids take cursor (post path) */
    u32                           next_to_clean; /* free_ids put cursor (completion path) */

    u32 *irq_db_index;
    closure_struct(thunk, irq_handler);
    closure_struct(thunk, service);
    struct gve_queue_resources *q_res;
    u32                          db_idx;  /* cached from q_res->db_index at create */

    struct gve_stats_rx rx_stats;

    boolean first_interrupt;
    u16   no_interrupt_event_cnt;
    int   empty_rx_queue;
    u16   idx;                    /* RX queue index, for SO_INCOMING_NAPI_ID */
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
    boolean adminq_running;   /* cleared on timeout; reset before any admin cmds */
    u16    tx_desc_cnt, rx_desc_cnt;
    u16    tx_pages_per_qpl, rx_data_slot_cnt;

    /* Device-reported (canonical) ring sizes. The working fields above may
     * be halved by gve_setup_queues backoff under memory pressure;
     * gve_setup_queues resets the working fields to these before each
     * attempt, so a transient shortage does not shrink the rings forever.
     * Mirrors ENA requested_*_ring_size + set_io_rings_size (ena.c:1042). */
    u16    tx_desc_cnt_dev, rx_desc_cnt_dev;
    u16    tx_pages_per_qpl_dev, rx_data_slot_cnt_dev;
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
    u32 next_monitored_tx_qid;   /* rotation cursor for watchdog TX/RX checks */
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
/* Inter-file function declarations                                     */
/* ------------------------------------------------------------------ */

/* gve_adminq.c */
boolean gve_verify_driver_compatibility(gve adapter);
boolean gve_describe_device(gve adapter);
boolean gve_cfg_device_resources(gve adapter);
void    gve_free_device_resources(gve adapter);
boolean gve_get_ptype_map_dqo(gve adapter);
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
