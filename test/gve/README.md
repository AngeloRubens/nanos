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

52 scenarios (348 checks) exercise every queue format in both directions
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

Measured line coverage (gcov): gve_dqo.c 98%, gve_datapath.c 94%,
gve_adminq.c 96%, gve_main.c 99%.  To measure: compile the four driver
files (gve_dqo.c is carried by harness.c via #include) and harness.c with
`--coverage`, link, run, then `gcov -n` the objects.

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

    make -C test/gve run
