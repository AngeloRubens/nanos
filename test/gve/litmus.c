/* Memory-ordering litmus tests for the gve barrier discipline.
 *
 * Standalone (no runtime dependencies): it mirrors the driver's barrier
 * macros and the two ordering patterns the driver relies on, then runs them
 * concurrently on pinned cores so that each barrier's *absence* is detectable
 * on the architecture that needs it.  This turns "the barriers are present"
 * (a static check) into "the barriers are present AND necessary".
 *
 * Barrier macros mirror src/x86_64/machine.h and src/aarch64/machine.h:
 *   read_barrier  : lfence (x86)      / dmb ld (arm64)
 *   write_barrier : sfence (x86)      / dmb st (arm64)
 *   memory_barrier: mfence (x86)      / dmb sy (arm64)
 *
 * Patterns:
 *  MP (message passing) — the completion gen-bit / RX event-counter / doorbell
 *     shape: producer writes the body, write_barrier, sets the flag; consumer
 *     waits the flag, read_barrier, reads the body.  A missing read_barrier
 *     lets a weakly-ordered CPU (arm64) observe the flag set but the body
 *     stale.  x86 (TSO) never reorders load-load, so it is the CONTROL:
 *     stripping the read barrier must NOT produce a violation on x86.
 *     This is the sibling of the audited arm64 fix (gve_datapath.c GQI RX
 *     read_barrier after the event counter, commit bc3855ae).
 *  SB (store buffering) — the TX stop/wake recheck: each thread stores then
 *     loads the other's store with a full barrier between.  StoreLoad
 *     reorders on BOTH x86 and arm64, so stripping the full barrier must
 *     produce a violation on both (a lost wakeup / permanent queue stall).
 *
 * Build variants (Makefile): the full build keeps every barrier (must pass
 * everywhere); -DLITMUS_NO_RMB strips the MP read barrier (must fail only on
 * arm64); -DLITMUS_NO_MB strips the SB full barrier (must fail on both).
 *
 * Exit: 0 = no violations; 1 = violations seen; 77 = skipped (need >= 2 cores).
 */
#define _GNU_SOURCE
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

typedef uint32_t u32;

#if defined(__x86_64__)
# define LIT_RMB() __asm__ __volatile__("lfence" ::: "memory")
# define LIT_WMB() __asm__ __volatile__("sfence" ::: "memory")
# define LIT_MB()  __asm__ __volatile__("mfence" ::: "memory")
# define LIT_ARCH  "x86_64"
#elif defined(__aarch64__)
# define LIT_RMB() __asm__ __volatile__("dmb ld" ::: "memory")
# define LIT_WMB() __asm__ __volatile__("dmb st" ::: "memory")
# define LIT_MB()  __asm__ __volatile__("dmb sy" ::: "memory")
# define LIT_ARCH  "aarch64"
#else
# error "litmus: unsupported architecture"
#endif
/* Compiler barrier: kept even when the hardware fence is stripped, so the
 * test isolates HARDWARE reordering (not compiler reordering). */
#define LIT_CC() __asm__ __volatile__("" ::: "memory")

#ifdef LITMUS_NO_RMB
# define MP_RMB() LIT_CC()
#else
# define MP_RMB() LIT_RMB()
#endif
#ifdef LITMUS_NO_WMB
# define MP_WMB() LIT_CC()
#else
# define MP_WMB() LIT_WMB()
#endif
#ifdef LITMUS_NO_MB
# define SB_MB()  LIT_CC()
#else
# define SB_MB()  LIT_MB()
#endif

static long iters = 2000000;

/* Cache-line separated so the reorder windows are real (no false sharing
 * forcing coherence traffic that would hide the effect). */
static volatile u32 mp_x __attribute__((aligned(64)));
static volatile u32 mp_y __attribute__((aligned(64)));
static volatile u32 sb_x __attribute__((aligned(64)));
static volatile u32 sb_y __attribute__((aligned(64)));
static volatile u32 sb_r0 __attribute__((aligned(64)));
static volatile u32 sb_r1 __attribute__((aligned(64)));
static long mp_violations, sb_violations;

/* Tight two-thread sense-reversing barrier: releases both threads within a
 * few cycles of each other (unlike pthread_barrier_wait, whose wake latency
 * decorrelates the threads and hides the store-buffering window).  Uses
 * acquire/release atomics so the count reset is visible before the release is
 * observed — otherwise on a weak arch the released thread could see the new
 * sense but a stale count and corrupt the next round (the barrier must itself
 * be correct so it never injects a false violation). */
static int bar_count, bar_sense;
static void rendezvous(int *sense)
{
    *sense = !*sense;
    if (__atomic_add_fetch(&bar_count, 1, __ATOMIC_ACQ_REL) == 2) {
        __atomic_store_n(&bar_count, 0, __ATOMIC_RELAXED);
        __atomic_store_n(&bar_sense, *sense, __ATOMIC_RELEASE);  /* release peer */
    } else {
        while (__atomic_load_n(&bar_sense, __ATOMIC_ACQUIRE) != *sense)
            LIT_CC();
    }
}

static void pin(int cpu)
{
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
}

/* Thread 0: MP writer, SB store-0. */
static void *t0(void *arg)
{
    (void)arg;
    int sense = 0;
    pin(0);
    for (long i = 0; i < iters; i++) {
        rendezvous(&sense);            /* trial start (cells reset by t1) */
        mp_x = 1;
        MP_WMB();
        mp_y = 1;
        rendezvous(&sense);            /* trial end */

        rendezvous(&sense);            /* SB trial start */
        sb_x = 1;
        SB_MB();
        sb_r0 = sb_y;
        rendezvous(&sense);
    }
    return NULL;
}

/* Thread 1: MP reader, SB store-1, and the violation checks. */
static void *t1(void *arg)
{
    (void)arg;
    int sense = 0;
    pin(1);
    for (long i = 0; i < iters; i++) {
        mp_x = 0;
        mp_y = 0;
        rendezvous(&sense);            /* reset visible to t0 across the barrier */
        while (mp_y == 0)
            LIT_CC();
        MP_RMB();
        if (mp_x == 0)                 /* saw the flag but the stale body */
            mp_violations++;
        rendezvous(&sense);

        sb_x = 0;
        sb_y = 0;
        rendezvous(&sense);
        sb_y = 1;
        SB_MB();
        sb_r1 = sb_x;
        rendezvous(&sense);
        if (sb_r0 == 0 && sb_r1 == 0)  /* both saw the other's stale store */
            sb_violations++;
    }
    return NULL;
}

int main(int argc, char **argv)
{
    if (argc > 1)
        iters = atol(argv[1]);

    long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
    if (ncpu < 2) {
        printf("litmus [%s]: SKIP (need >= 2 online CPUs, have %ld)\n",
               LIT_ARCH, ncpu);
        return 77;
    }

    pthread_t a, b;
    pthread_create(&a, NULL, t0, NULL);
    pthread_create(&b, NULL, t1, NULL);
    pthread_join(a, NULL);
    pthread_join(b, NULL);

    printf("litmus [%s] iters=%ld  MP(read-order) violations=%ld  "
           "SB(store-load) violations=%ld\n",
           LIT_ARCH, iters, mp_violations, sb_violations);
    return (mp_violations || sb_violations) ? 1 : 0;
}
