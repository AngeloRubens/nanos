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

25 scenarios exercise every queue format in both directions (GQI-QPL/RDA,
DQO-RDA/QPL TX and RX), multi-segment packets, TX backpressure, the
MISS/REINJECT tag pool and its edge encodings, RX chaining and drop-to-EOP,
allocation-failure tolerance, an out-of-order completion fuzzer, the
device-option negotiation / format fallback (via an admin-queue model), and
the full lifecycle — init_gve → probe → describe → setup → watchdog →
reset — for both a DQO-RDA device and the GQI-QPL fallback.  The lifecycle
links `gve_main.c` and drives the driver's own bring-up so its watchdog and
reset closures get initialised (closure `_fill_*` helpers are static per
translation unit, so only the driver can initialise them).

Minor remaining gaps (datapath only): the GQI RX held-pbuf copy fallback,
the GQI QPL byte-FIFO wrap, and the io-queues manifest cap.

## Build & run

    make -C test/gve run
