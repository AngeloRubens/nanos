/* What a garbage collector that shrinks its heap actually does to a memory
   filesystem, in the smallest form that still does it.

   ZGC keeps its heap in a memfd, maps it, and has a thread that punches holes in
   it to give memory back while the collector and the application go on faulting
   on the same file and writing to it. Every piece of that is ordinary; it is
   having them at once, on one node, that finds things -- the page cache calls
   into the filesystem to write a page back while holding the node's own lock,
   the filesystem takes a lock that can sleep, a punch takes pages away from
   under a walk that is counting them, and a sync completes on whichever context
   got there first.

   None of it needs an unusual system call. The threads here do what a collector
   does, only closer together: two punching, one writing and syncing, three
   touching pages back in.

   Failures come in three shapes, so the test says where it is as it goes: an
   assertion inside the kernel, a fault, or a machine that stops. The round is
   printed before the work, and the counters after it, so a log that ends says
   both which round it ended in and whether the threads were still moving.

   Two processors are enough, which is what the CI gives. */

#define _GNU_SOURCE
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/sysinfo.h>
#include <unistd.h>

#include "../test_utils.h"

#define PAGESIZE        4096
#define FILE_PAGES      2048
#define FILE_BYTES      (FILE_PAGES * PAGESIZE)
#define ROUNDS          200
#define PUNCHERS        2
#define TOUCHERS        3
#define CHUNK           (32 * 1024)

static int fd;
static volatile uint8_t *map;
static volatile int stop;
static volatile unsigned long punches, touches, writes;

static unsigned long rnd(unsigned long *seed)
{
    *seed = *seed * 6364136223846793005ull + 1442695040888963407ull;
    return *seed >> 33;
}

/* Gives ranges of the file back, which is the uncommitter. */
static void *puncher(void *arg)
{
    unsigned long seed = (unsigned long)(uintptr_t)arg + 1;

    while (!stop) {
        unsigned long lo = rnd(&seed) % FILE_PAGES;
        unsigned long len = 1 + rnd(&seed) % (FILE_PAGES - lo);
        test_assert(fallocate(fd, FALLOC_FL_PUNCH_HOLE | FALLOC_FL_KEEP_SIZE,
                              lo * PAGESIZE, len * PAGESIZE) == 0);
        __sync_fetch_and_add(&punches, 1);
    }
    return NULL;
}

/* Writes and syncs: this is what makes the page cache commit, which is what
   calls into the filesystem with the node held. */
static void *writer(void *arg)
{
    uint8_t buf[CHUNK];
    unsigned long seed = 0x9e3779b9;

    memset(buf, 0xa5, sizeof(buf));
    while (!stop) {
        off_t off = (rnd(&seed) % (FILE_BYTES / sizeof(buf))) * sizeof(buf);
        test_assert(pwrite(fd, buf, sizeof(buf), off) == sizeof(buf));
        test_assert(fsync(fd) == 0);
        __sync_fetch_and_add(&writes, 1);
    }
    return NULL;
}

/* Faults the pages back in through the mapping. A page a punch took away comes
   back as zero, which is allowed; a page that comes back as anything else is
   not, and that is what is checked. */
static void *toucher(void *arg)
{
    unsigned long seed = (unsigned long)(uintptr_t)arg + 0x1234;

    while (!stop) {
        unsigned long p = rnd(&seed) % FILE_PAGES;
        uint8_t v = map[p * PAGESIZE];
        test_assert((v == 0) || (v == 0xa5) || (v == 0x5c));
        map[p * PAGESIZE] = 0x5c;
        __sync_fetch_and_add(&touches, 1);
    }
    return NULL;
}

int main(int argc, char **argv)
{
    pthread_t pt[PUNCHERS], tt[TOUCHERS], wt;

    printf("There are %d processors available\n", get_nprocs());

    fd = memfd_create("punchrace", 0);
    test_assert(fd >= 0);
    test_assert(ftruncate(fd, FILE_BYTES) == 0);
    map = mmap(NULL, FILE_BYTES, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    test_assert(map != MAP_FAILED);
    memset((void *)map, 0xa5, FILE_BYTES);
    test_assert(msync((void *)map, FILE_BYTES, MS_SYNC) == 0);

    printf("threads starting\n");
    fflush(stdout);
    for (long i = 0; i < PUNCHERS; i++)
        test_assert(pthread_create(&pt[i], NULL, puncher, (void *)i) == 0);
    for (long i = 0; i < TOUCHERS; i++)
        test_assert(pthread_create(&tt[i], NULL, toucher, (void *)i) == 0);
    test_assert(pthread_create(&wt, NULL, writer, NULL) == 0);

    for (int round = 0; round < ROUNDS; round++) {
        printf("round %d\n", round);
        fflush(stdout);
        /* The collector's own sync, crossing everything above. */
        test_assert(fsync(fd) == 0);
        struct stat st;
        test_assert(fstat(fd, &st) == 0);
        printf("  punches %lu touches %lu writes %lu blocks %ld\n",
               punches, touches, writes, (long)st.st_blocks);
        fflush(stdout);
        usleep(100000);
    }

    stop = 1;
    for (int i = 0; i < PUNCHERS; i++)
        test_assert(pthread_join(pt[i], NULL) == 0);
    for (int i = 0; i < TOUCHERS; i++)
        test_assert(pthread_join(tt[i], NULL) == 0);
    test_assert(pthread_join(wt, NULL) == 0);

    test_assert(munmap((void *)map, FILE_BYTES) == 0);
    test_assert(close(fd) == 0);
    printf("tmpfs punch race test passed\n");
    return EXIT_SUCCESS;
}
