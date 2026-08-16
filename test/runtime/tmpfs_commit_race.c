/* Several threads writing and syncing the same file on a memory filesystem,
   some of them through a mapping.

   Writing back a dirty page on a memory filesystem calls into the filesystem
   from inside the page cache, and the filesystem takes a lock that can sleep;
   the page cache holds the node's own lock across that call, which a context
   that sleeps must not do. And the completion of a sync that was queued behind
   somebody else's write is applied wherever the write finished rather than on
   the context that asked for it.

   Neither shows up when one thread writes and syncs a file, because nothing is
   contended and nothing is queued. Both want the same thing: more than one
   writer on one file, and a sync crossing them. Two processors are enough,
   which is what the CI gives (VCPUS=2).

   A kernel that has either of these stops with an assertion rather than
   silently, so this one names itself when it fails; the round printed before
   the work is for the case where it stops without saying anything at all. */

#define _GNU_SOURCE
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "../test_utils.h"

#define FILE_BYTES  (4 * 1024 * 1024)
#define CHUNK       (32 * 1024)
#define ROUNDS      48
#define PAGE        4096

static int fd;
static uint8_t *map;
static volatile int stop;

/* Writes through the file, which dirties pages the page cache will hand to the
   filesystem when somebody syncs. */
static void *writer(void *arg)
{
    uint8_t buf[CHUNK];
    uint8_t mark = (uint8_t)(uintptr_t)arg;

    memset(buf, mark, sizeof(buf));
    while (!stop) {
        for (off_t off = 0; off < FILE_BYTES; off += sizeof(buf))
            test_assert(pwrite(fd, buf, sizeof(buf), off) == sizeof(buf));
    }
    return NULL;
}

/* Dirties the same pages from the other side, so that the write-back has more
   than one source to serialise. */
static void *mapper(void *arg)
{
    while (!stop) {
        for (size_t off = 0; off < FILE_BYTES; off += PAGE)
            map[off] = 0x77;
        test_assert(msync(map, FILE_BYTES, MS_ASYNC) == 0);
    }
    return NULL;
}

/* The sync that has to cross the writers: this is the one that goes into the
   filesystem holding the node, and the one whose completion is queued behind
   whatever write got there first. */
static void *syncer(void *arg)
{
    while (!stop) {
        test_assert(fsync(fd) == 0);
        test_assert(fdatasync(fd) == 0);
    }
    return NULL;
}

int main(int argc, char **argv)
{
    pthread_t t[4];

    fd = memfd_create("commit", 0);
    test_assert(fd >= 0);
    test_assert(ftruncate(fd, FILE_BYTES) == 0);
    map = mmap(NULL, FILE_BYTES, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    test_assert(map != MAP_FAILED);

    test_assert(pthread_create(&t[0], NULL, writer, (void *)(uintptr_t)0x11) == 0);
    test_assert(pthread_create(&t[1], NULL, writer, (void *)(uintptr_t)0x22) == 0);
    test_assert(pthread_create(&t[2], NULL, mapper, NULL) == 0);
    test_assert(pthread_create(&t[3], NULL, syncer, NULL) == 0);

    for (int round = 0; round < ROUNDS; round++) {
        printf("round %d\n", round);
        fflush(stdout);
        test_assert(fsync(fd) == 0);
        /* Truncating down and back up gives the write-back something to finish
           against a node whose pages are going away underneath it. */
        test_assert(ftruncate(fd, FILE_BYTES / 2) == 0);
        test_assert(ftruncate(fd, FILE_BYTES) == 0);
    }

    stop = 1;
    for (int i = 0; i < 4; i++)
        test_assert(pthread_join(t[i], NULL) == 0);

    test_assert(munmap(map, FILE_BYTES) == 0);
    test_assert(close(fd) == 0);
    printf("tmpfs commit race test passed\n");
    return EXIT_SUCCESS;
}
