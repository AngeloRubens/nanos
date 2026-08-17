/* Several threads syncing the same file at once.

   A sync whose node is already busy does not do the work itself: the caller's
   completion is queued behind whoever got there first, and applied when that
   one finishes. The two things that have to be true for that to happen at all
   are that the node is busy and that the caller asked for a completion, which
   is fsync and fdatasync and not msync or the write-back timer.

   That window is why a file in memory finds nothing: the commit is a memcpy and
   the node is busy for no time worth the name. This test uses a file on the root
   filesystem, where committing is real I/O and the node stays busy long enough
   for the next sync to arrive and be queued -- and it uses one syncing thread
   per processor, because a completion cannot queue behind another one if there
   is only ever one.

   The failure this looks for is a completion given back on whichever context
   finished the write rather than on the one that asked for it. In nanos that is
   an assertion, and assertions are not compiled out (runtime.h:69-82), so it is
   a halt of the guest rather than a wrong answer. */

#define _GNU_SOURCE
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/sysinfo.h>
#include <unistd.h>

#include "../test_utils.h"

#define MAX_CPUS    16
#define FILE_BYTES  (4 * 1024 * 1024)  /* needs EXTRA_MKFS_OPTS="-s 64m" */
#define CHUNK       (64 * 1024)
#define ROUNDS      48

static pthread_t threads[MAX_CPUS];
static volatile int done;
static volatile unsigned long syncs;
static int np;
static int fd;

static void *syncer(void *arg)
{
    uint8_t buf[CHUNK];
    unsigned long seed = (unsigned long)(uintptr_t)arg * 2654435761u + 1;
    uint8_t mark = (uint8_t)((uintptr_t)arg + 1);

    memset(buf, mark, sizeof(buf));
    while (!done) {
        /* Dirty something, then ask for it to be committed. With every thread
           doing this on one file, the second and later callers find the node
           busy and have their completion queued. */
        seed = seed * 6364136223846793005ull + 1442695040888963407ull;
        off_t off = (off_t)((seed >> 33) % (FILE_BYTES / sizeof(buf))) * sizeof(buf);
        test_assert(pwrite(fd, buf, sizeof(buf), off) == sizeof(buf));
        if ((uintptr_t)arg & 1)
            test_assert(fdatasync(fd) == 0);
        else
            test_assert(fsync(fd) == 0);
        __sync_fetch_and_add(&syncs, 1);
    }
    return NULL;
}

int main(int argc, char **argv)
{
    np = get_nprocs();
    if (np > MAX_CPUS)
        np = MAX_CPUS;
    printf("There are %d processors available\n", np);

    /* On the root filesystem, not in memory: committing has to take long enough
       for the node to still be busy when the next caller arrives. */
    fd = open("/fsync_race.dat", O_RDWR | O_CREAT | O_TRUNC, 0644);
    test_assert(fd >= 0);
    uint8_t buf[CHUNK];
    memset(buf, 0x11, sizeof(buf));
    for (off_t off = 0; off < FILE_BYTES; off += sizeof(buf))
        test_assert(pwrite(fd, buf, sizeof(buf), off) == sizeof(buf));
    test_assert(fsync(fd) == 0);

    printf("threads starting\n");
    fflush(stdout);
    for (long i = 0; i < (np > 2 ? np : 2); i++)
        test_assert(pthread_create(&threads[i], NULL, syncer, (void *)i) == 0);

    for (int round = 0; round < ROUNDS; round++) {
        printf("round %d: syncs %lu\n", round, syncs);
        fflush(stdout);
        test_assert(fsync(fd) == 0);
        usleep(50000);
    }

    done = 1;
    for (int i = 0; i < (np > 2 ? np : 2); i++)
        test_assert(pthread_join(threads[i], NULL) == 0);

    test_assert(close(fd) == 0);
    test_assert(unlink("/fsync_race.dat") == 0);
    printf("fsync completion race test passed\n");
    return EXIT_SUCCESS;
}
