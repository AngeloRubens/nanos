/* Host shim for <kernel.h>: pulls the real nanos runtime (types, barriers,
 * byteswap, spinlocks, closures, heap, queue, symbol, timer, now/zero) and
 * declares the handful of kernel-glue symbols the gve datapath uses.  The
 * implementations live in harness.c. */
#ifndef GVE_HARNESS_SHIM_KERNEL_H
#define GVE_HARNESS_SHIM_KERNEL_H

#include <harness_runtime.h>

/* heaps */
typedef struct kernel_heaps *kernel_heaps;
heap heap_locked(kernel_heaps kh);
heap heap_linear_backed(kernel_heaps kh);

/* DMA: identity-mapped in the harness */
static inline u64 physical_from_virtual(void *v) { return u64_from_pointer(v); }

/* deferred work: the harness runs these synchronously or records them */
void async_apply(thunk t);
void async_apply_bh(thunk t);

/* cpu */
typedef struct harness_cpu { u32 id; } *cpuinfo;
cpuinfo current_cpu(void);
extern int total_processors;

/* busy-wait: a no-op in the harness (the device model responds at once) */
void kernel_delay(timestamp t);

#endif
