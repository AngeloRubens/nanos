# gve DQO datapath host harness

A host-side test harness that compiles the **real** driver datapath
(`src/gcp/gve/gve_dqo.c`) against a software model of a DQO gVNIC device,
and drives it through scripted and fuzzed completion sequences.

This exercises the layer that QEMU/CI boot smokes cannot reach: QEMU has no
gVNIC, so `gve_probe` no-matches and the datapath never runs.  It is also the
only pre-hardware tool that exercises the lifecycle/concurrency logic
(tag-pool reuse across ring wrap, MISS/REINJECT ordering, allocation
failures, counter wraps) — the class where code review proved fallible.

## How it works

The runtime primitives (heap, queue, closures, symbols, `now`, `zero`) come
from the nanos runtime + the `unix_process` host simulator, exactly as
`test/unit` does.  The arch primitives (barriers, spinlocks, byteswap) come
from `src/x86_64` because the host is x86_64.  Only the device/glue layer is
shimmed (`shim/`): pbuf, netif, pci (the PCI BAR is backed by the device
model), timers, and the async work queues.

`gve_dqo.c` is `#include`d directly, so the code under test is the shipping
driver, not a copy.

## Coverage

59 scenarios (381 checks) exercise every queue format in both directions
(GQI-QPL/RDA, DQO-RDA/QPL TX and RX), multi-segment packets, TX
backpressure, the MISS/REINJECT tag pool and its edge encodings, RX chaining
and drop-to-EOP, an out-of-order completion fuzzer, the device-option
negotiation / format fallback (via an admin-queue model), the full
lifecycle — init_gve → probe → describe → setup → watchdog → reset — for
both a DQO-RDA device and the GQI-QPL fallback, the multi-queue driver logic
(per-CPU TX queue dispatch with cross-queue isolation, and the round-robin
RSS indirection table), and — via a failing-heap wrapper that returns
INVALID after a set number of allocations — the queue-create error cascades,
the ring-size backoff (halve and retry down to GVE_MIN_RING_SIZE), and the
failed-reset epilogue (DEVICE_RUNNING cleared, RESETTING/ONGOING_RESET left
set), the gve_setup failure paths (cfg-resources rejected, too few MSI-X
vectors, mgmt/RX MSI-X setup failure with interrupt teardown, queue-create
failure through the deinit-interrupts path, and a device-requested reset
caught during bring-up), and every watchdog branch (per-tick TX wakeup and
RX empty-ring kicks, the GQI stuck-TX and no-interrupt resets, the DQO
miss-reinject timeout and no-interrupt resets, all with recovery), the TX
hot-path edges (stale alt-miss/miss/reinject completion tags, the mid-batch
doorbell write when more than GVE_TX_DOORBELL_BATCH packets drain in one
pass, and linkoutput returning ERR_MEM on a full software queue), and the RX
error/drop branches (DQO-RDA bad_req_id, errored continuation freeing a
partial chain, zero-length and stack-rejected packets; GQI runt buffers and
continuation-alloc failures; DQO-QPL drop-to-EOP recycle and copy-out alloc
failure).  Finally a sweep walks the allocation-failure point across every
queue-create cascade for all four formats, unwinding the create functions'
err_after_* cleanup labels from progressively deeper points until the rings
come up (the QPL page-list and slot-list allocations included).  Finally the
admin-queue command-failure paths (describe failing at probe, ptype/RSS/cfg
failing during setup and reset, and a command the device never answers
running the exponential-backoff poll to its retry limit, timing out and
marking the queue dead — plus the watchdog deadline-scan edges that are not a
timeout: a DQO miss still within GVE_TX_WATCHDOG_MS and zero seg-descriptor
timestamp slots, both skipped without a reset), the GQI-QPL multi-segment TX seg
descriptors with a byte-FIFO wrap, more GQI/DQO RX chain-error branches, and
queue teardown freeing held resources (an undrained software queue, in-flight
pending pbufs, a partial RX chain and a still-held RX pbuf) for GQI-RDA,
GQI-QPL and DQO-RDA.  Finally, fixed-buffer packet splitting: a frame the
device splits across several full 2 KB (GVE_DQO_BUF_SIZE) RX buffers is
reassembled into one chain of the right total length and buffer count (a long
chain at the real buffer dimension, not the earlier arbitrary 2-buffer
splits) for DQO-RDA, DQO-QPL and GQI (which pads only the first buffer), plus
the DQO-QPL TX rejection of a single segment larger than one fixed bounce
slot.

A final set asserts protocol/behaviour the line coverage cannot — what the
driver actually emits and the invariants it must hold, checked against the
ENA patterns and the official Google driver: ring-size backoff restores the
canonical sizes after transient pressure (no permanent shrink); RSS uses a
fresh random Toeplitz key per configure with the Toeplitz algorithm and the
TCP/UDP v4+v6 hash types; no TX checksum offload (descriptor csum fields and
the DQO context-descriptor flex fields stay zero — guarding the reverted L4
offload); DQO sets the report_event bit sparsely (>= 32 descriptors apart),
not per packet; the DQO stream is a mandatory context descriptor (dtype 0x4)
then packet descriptors all carrying the one completion tag; the TX doorbell
is big-endian free-running for GQI and a masked little-endian index for DQO;
and the GQI RX fill predicate stays correct across the u32 wrap.

Measured line coverage (gcov): gve_dqo.c 98%, gve_datapath.c 94%,
gve_adminq.c 96%, gve_main.c 99%.  To measure: compile the four driver
files (gve_dqo.c is carried by harness.c via #include) and harness.c with
`--coverage`, link, run, then `gcov -n` the objects.

Ring geometry and per-packet cost: a sweep builds TX and RX queues at every
supported descriptor-ring size (64 = GVE_MIN_RING_SIZE, 256, 1024, 4096) in
all four formats and round-trips a packet through each with no leak; and an
allocation-accounting scenario asserts the TX-RDA hot path makes zero heap
allocations per packet (it maps the pbuf's physical address — a regression
that added a per-packet alloc would be caught) via a counting heap wrapper.

The multi-queue and scheduling machinery is exercised directly: per-CPU TX
dispatch (CPU n -> queue n % nq) with cross-queue tag isolation; RX
multi-queue, where four RX queues consume their own completions independently
and deliver packets tagged with their own per-queue napi_id; the RX cleaning
budget, where one service call retires a bounded number of completions (two
sweeps of GVE_CLEAN_BUDGET x GVE_RX_BUDGET around the IRQ re-arm) and a second
finishes the rest; the TX cleaning budget (<= GVE_TX_CLEAN_BUDGET per call);
the watchdog queue rotation (8 queues, GVE_TX_MONITORED_QUEUES=4 per tick, the
cursor advancing 0->4->0 over a full sweep); the buf_ring backpressure cycle
(ring full -> queue_stop -> completion -> queue_wakeup); and the per-queue
MSI-X layout (mgmt IRQ at slot 2N, one RX IRQ per queue at N..2N-1, the polled
TX notify slots 0..N-1 left without a handler).

A block of `_Static_assert`s at the top of `harness.c` pins the descriptor
struct sizes and the offsets of every field the driver/device read or write
to literals transcribed from the official Google headers (gve_desc.h,
gve_desc_dqo.h, gve_adminq.h).  This is the one check independent of both the
driver and the device model: a reordered field or wrong type in gve_priv.h
fails the build even though the driver and the model would still agree with
each other.

`make -C test/gve sanitize` builds and runs the suite under
AddressSanitizer + UBSan — an independent check of the memory and
undefined-behaviour properties the functional assertions do not cover
(use-after-free, buffer overflow, bad frees) across the ref-ownership,
wrap-stress and out-of-order fuzzer paths.  It runs clean.  The build passes
`-fno-sanitize=alignment`: the only over-aligned type is struct gve_irq_db
(aligned(64) — "device expects 64-byte alignment"), which on hardware comes
from the page-backed contiguous heap; the host harness backs every heap with
the general mcache (not 64-byte aligned), so that one UBSan check is a harness
artifact, not a driver bug.

`make -C test/gve tsan` builds with real atomic spinlocks (shim/lock.h under
GVE_HARNESS_SMP — the normal build compiles the runtime's locks to no-ops)
and runs two extra concurrency scenarios under ThreadSanitizer: four producer
threads transmitting on one DQO-RDA queue (ring_mtx must serialise the
head/desc_tail/tag-pool/descriptor-ring writes), and two threads running the
RX service at once (service_lock must serialise it so each completion is
delivered exactly once — the dual-source double-run the design guards
against), four threads each transmitting on their own CPU and own TX queue
(per-queue ring_mtx independence, no false sharing), and many threads
requesting a reset at once (the RESETTING flag admits exactly one — reset
single-flight).  TSan reports no data races; the functional invariants hold.
(The datapath-vs-teardown window during a reset is a documented accepted risk
with ENA precedent, and the harness runs deferred work synchronously, so it is
not contended here — that is closed only by the GCP hardware test.)  The
lock-free runtime queue's internals are suppressed (tsan.supp) since its
inline-asm CAS is opaque to TSan and it is a vetted primitive, not driver
code.  TSan needs ASLR off, so the target wraps the run in `setarch -R`.

`make -C test/gve litmus` runs the memory-ordering litmus matrix
(litmus.c, a standalone program with no runtime deps that mirrors the
driver's barrier macros).  Two patterns on pinned cores stand in for the
driver's ordering contracts: MP (message passing) — a producer writes the
body, write_barrier, sets a flag; a consumer waits the flag, read_barrier,
reads the body — the completion gen-bit / RX event-counter / doorbell shape
(the sibling of the audited arm64 fix bc3855ae); and SB (store buffering) —
each thread stores then loads the other's store with a full barrier between —
the TX stop/wake recheck.  litmus-check.sh builds three variants and asserts
the matrix per arch: the full build never violates; stripping the SB full
barrier violates on both arches (StoreLoad reorders everywhere); stripping the
MP read barrier violates ONLY on arm64, with x86 staying clean as the TSO
control.  This turns "the barriers are present" into "the barriers are present
AND necessary", and is wired into CI as `litmus-x86` and `litmus-arm64` jobs
(the arm64 job on a native runner).  It validates the ordering contract and
the architecture, not the driver's barrier placement (that is the static
presence check and review); CPU-CPU ordering stands in for device DMA, which
together with the backend remains a GCP-hardware question.

What the harness cannot test (hardware behaviour): whether the device
actually steers RX flows across queues per the RSS table — that is Toeplitz
hashing in the NIC, and is the open question (#2165) answered only by
per-queue RX counters on real GCP hardware.

The remaining uncovered lines are the GQI RX non-zero-copy fallback (the
half-page buffer lands at a ring slot the in-order device model never
reaches), the slow-path async_apply enqueue (a no-op to reach with the
single-threaded UP spinlock model), and a few QPL slot-wrap and msg_err
diagnostic lines.

## Build & run

    make -C test/gve run        # functional suite (463 checks)
    make -C test/gve sanitize   # same scenarios under ASan + UBSan
    make -C test/gve tsan       # concurrency scenarios under ThreadSanitizer
    make -C test/gve litmus     # memory-ordering litmus matrix

All four run in CI (`.github/workflows/gve-ci.yml`): `dqo-harness` (run),
`harness-sanitize`, `harness-tsan` and `litmus-x86` on x86 runners, plus
`litmus-arm64` on a native arm64 runner.
