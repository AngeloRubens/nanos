/* Giving anonymous memory back the way a JVM heap does, from more than one
   thread, while others fault it in again.

   This is the collector that most people run. G1, Parallel, Serial and
   Shenandoah keep the heap in anonymous memory: reserved MAP_PRIVATE, committed
   by mapping read-write over the range with MAP_FIXED, and given back by mapping
   PROT_NONE over it the same way -- os::pd_uncommit_memory in HotSpot, which is
   an mmap and not an madvise. There is no file behind any of it, so none of the
   page cache is involved: what is left is the process's own vmap lock and the
   page tables, and both are taken with interrupts disabled while an unmap opens
   a TLB shootdown that every processor has to join.

   A thread waiting for either of those locks with interrupts disabled never
   services the shootdown, so it never joins, so whoever opened it never
   finishes. Two processors are enough, which is what the CI gives.

   The companion test vmap_flush_race does this against a file mapping; this one
   removes the file, so that what it finds cannot be blamed on the page cache.

   A kernel that closes the cycle stops the machine, so the round is printed
   before the work: a log that ends says which round it ended in. */

#define _GNU_SOURCE
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/sysinfo.h>
#include <unistd.h>

#include "../test_utils.h"

#define MAX_CPUS    16
#define PAGESIZE    4096
#define HEAP_PAGES  2048
#define HEAP_BYTES  (HEAP_PAGES * PAGESIZE)
#define ROUNDS      64

static pthread_t threads[MAX_CPUS];
static volatile uint8_t *heap;
static volatile int done;
static int np;

static int kidcnt;
static pthread_cond_t kid_cv;
static pthread_cond_t sync_cv;
static pthread_mutex_t sync_mut;

static void wait_for_sync(void)
{
    pthread_mutex_lock(&sync_mut);
    kidcnt++;
    pthread_cond_signal(&kid_cv);
    pthread_cond_wait(&sync_cv, &sync_mut);
    pthread_mutex_unlock(&sync_mut);
}

static void wait_for_children(void)
{
    pthread_mutex_lock(&sync_mut);
    while (kidcnt < np)
        pthread_cond_wait(&kid_cv, &sync_mut);
    kidcnt = 0;
    pthread_mutex_unlock(&sync_mut);
}

static void wake_children(void)
{
    pthread_mutex_lock(&sync_mut);
    pthread_cond_broadcast(&sync_cv);
    pthread_mutex_unlock(&sync_mut);
}

/* Uncommit and commit, exactly as HotSpot does them: PROT_NONE over the range to
   give it back, read-write over it to take it again. Nothing else. */
static void uncommit(void *addr, size_t len)
{
    test_assert(mmap(addr, len, PROT_NONE,
                     MAP_FIXED | MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE,
                     -1, 0) != MAP_FAILED);
}

static void commit(void *addr, size_t len)
{
    test_assert(mmap(addr, len, PROT_READ | PROT_WRITE,
                     MAP_FIXED | MAP_PRIVATE | MAP_ANONYMOUS,
                     -1, 0) != MAP_FAILED);
}

static void *cpu_thread(void *v)
{
    int id = (int)((uintptr_t)v);
    size_t share = HEAP_BYTES / np;
    size_t base = share * id;

    while (!done) {
        wait_for_sync();
        if (done)
            break;
        if (id & 1) {
            for (size_t off = base; off < base + share; off += PAGESIZE)
                heap[off] = (uint8_t)(id + 1);
        } else {
            uncommit((void *)(heap + base), share);
            commit((void *)(heap + base), share);
        }
    }
    return NULL;
}

int main(int argc, char **argv)
{
    np = get_nprocs();
    if (np > MAX_CPUS)
        np = MAX_CPUS;
    printf("There are %d processors available\n", np);
    if (np < 2) {
        printf("anon uncommit race needs more than one processor -- skipped\n");
        return EXIT_SUCCESS;
    }

    pthread_cond_init(&kid_cv, NULL);
    pthread_cond_init(&sync_cv, NULL);
    pthread_mutex_init(&sync_mut, NULL);

    /* Reserved the way a heap is reserved, then committed. */
    heap = mmap(NULL, HEAP_BYTES, PROT_NONE,
                MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    test_assert(heap != MAP_FAILED);
    commit((void *)heap, HEAP_BYTES);
    memset((void *)heap, 0x11, HEAP_BYTES);

    for (long i = 0; i < np; i++)
        test_assert(pthread_create(&threads[i], NULL, cpu_thread, (void *)i) == 0);

    for (int round = 0; round < ROUNDS; round++) {
        printf("round %d\n", round);
        fflush(stdout);
        wait_for_children();
        wake_children();
        /* This thread drives the rounds and touches nothing. An earlier version
           wrote the whole heap here, which is a range another thread may be
           holding at PROT_NONE at that moment: the write is then a segfault the
           test earned rather than a defect it found, and it showed up as a user
           fault with error code 7 -- present, write, user mode. The contention
           that matters is already between the children: the odd ones fault the
           pages in while the even ones are giving them back. */
    }

    wait_for_children();
    done = 1;
    wake_children();
    for (int i = 0; i < np; i++)
        test_assert(pthread_join(threads[i], NULL) == 0);

    test_assert(munmap((void *)heap, HEAP_BYTES) == 0);
    printf("anon uncommit race test passed\n");
    return EXIT_SUCCESS;
}
