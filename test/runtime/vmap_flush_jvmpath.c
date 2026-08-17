/* Giving pages back the way HotSpot does -- PROT_NONE mapped over the range with
   MAP_FIXED, then the file mapped back over it -- from more than one thread,
   while the file behind them is synced.

   The companion test vmap_flush_race does the same with madvise(MADV_DONTNEED).
   This one takes the route the JVM actually takes, os::pd_uncommit_memory
   followed by a fresh mmap of the backing file, which on nanos goes through
   process_remove_range_locked() rather than straight into vmap_unmap_page_range()
   -- the same vmap lock and the same shootdown, reached by the other door.

   Every thread owns a share of the mapping and touches nothing but its own: a
   range that one thread has mapped PROT_NONE is not readable by another, here or
   on Linux, and a test that reads it is testing nothing. What overlaps is the
   unmapping and the faulting and the sync, which is what closes the cycle.

   A kernel that closes the cycle stops the machine, so the round is printed
   before the work: a log that ends says which round it ended in. */

#define _GNU_SOURCE
#include <pthread.h>
#include <sched.h>
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
#define MAP_PAGES   2048
#define MAP_BYTES   (MAP_PAGES * PAGESIZE)
#define ROUNDS      64

static pthread_t threads[MAX_CPUS];
static volatile uint8_t *m;
static volatile int done;
static int np;
static int fd;
static size_t share;

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

/* The way a JVM gives a range back and takes it again: PROT_NONE over it with
   MAP_FIXED, then the backing file mapped over it at the same offset. */
static void work(int id)
{
    size_t base = share * id;

    test_assert(mmap((void *)(m + base), share, PROT_NONE,
                     MAP_FIXED | MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE,
                     -1, 0) != MAP_FAILED);
    test_assert(mmap((void *)(m + base), share, PROT_READ | PROT_WRITE,
                     MAP_FIXED | MAP_SHARED, fd, base) != MAP_FAILED);
    for (size_t off = base; off < base + share; off += PAGESIZE)
        m[off] = (uint8_t)(id + 1);
}

static void *cpu_thread(void *v)
{
    int id = (int)((uintptr_t)v);

    while (!done) {
        wait_for_sync();
        if (done)
            break;
        work(id);
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
        printf("vmap flush jvmpath needs more than one processor -- skipped\n");
        return EXIT_SUCCESS;
    }

    share = (MAP_BYTES / (np + 1)) & ~(size_t)(PAGESIZE - 1);

    pthread_cond_init(&kid_cv, NULL);
    pthread_cond_init(&sync_cv, NULL);
    pthread_mutex_init(&sync_mut, NULL);

    fd = memfd_create("race", 0);
    test_assert(fd >= 0);
    test_assert(ftruncate(fd, MAP_BYTES) == 0);
    m = mmap(NULL, MAP_BYTES, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    test_assert(m != MAP_FAILED);
    memset((void *)m, 0x11, MAP_BYTES);

    for (long i = 0; i < np; i++)
        test_assert(pthread_create(&threads[i], NULL, cpu_thread, (void *)i) == 0);

    for (int round = 0; round < ROUNDS; round++) {
        printf("round %d\n", round);
        fflush(stdout);
        wait_for_children();
        wake_children();
        /* The other way into the shootdown, from this thread, while they are in
           the middle of theirs. The sync walks the whole file; a share that is
           anonymous at that moment is skipped rather than read. */
        test_assert(msync((void *)m, MAP_BYTES, MS_SYNC) == 0);
        test_assert(fsync(fd) == 0);
        work(np);
    }

    wait_for_children();
    done = 1;
    wake_children();
    for (int i = 0; i < np; i++)
        test_assert(pthread_join(threads[i], NULL) == 0);

    test_assert(munmap((void *)m, MAP_BYTES) == 0);
    test_assert(close(fd) == 0);
    printf("vmap flush jvmpath test passed\n");
    return EXIT_SUCCESS;
}
