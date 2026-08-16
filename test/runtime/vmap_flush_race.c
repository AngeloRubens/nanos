/* madvise(MADV_DONTNEED) on a shared mapping while another thread faults on it
   and a third syncs it.

   Each of the three is ordinary on its own. Together they close a cycle:
   madvise takes the process's vmap lock with interrupts disabled and then
   unmaps, which opens a TLB shootdown that every processor has to join; a
   second processor waiting for that same lock with interrupts disabled never
   services the shootdown, so it never joins, so the first never finishes and
   never releases the lock. Two processors are enough, which is what the CI
   gives (VCPUS=2).

   Nothing here is unusual for a program that returns memory to the system --
   which every garbage collector does -- and nothing here punches a hole, so
   this exercises the kernel as it is rather than any filesystem feature.

   A kernel that closes the cycle stops the machine, so the failure is a boot
   that stops printing rather than a message. The line printed before each round
   is there so that a log that ends says which round it ended in. */

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

#define MAP_BYTES   (8 * 1024 * 1024)
#define ROUNDS      64
#define PAGE        4096

static int fd;
static uint8_t *map;
static volatile int stop;

/* Gives the pages back, over and over: this is the thread that takes the vmap
   lock and opens the shootdown. */
static void *dropper(void *arg)
{
    while (!stop) {
        test_assert(madvise(map, MAP_BYTES, MADV_DONTNEED) == 0);
        for (size_t off = 0; off < MAP_BYTES; off += MAP_BYTES / 4)
            test_assert(madvise(map + off, MAP_BYTES / 4, MADV_DONTNEED) == 0);
    }
    return NULL;
}

/* Brings them back, which is what makes the next round have something to drop,
   and is the thread that waits for the same lock. */
static void *toucher(void *arg)
{
    uint8_t mark = (uint8_t)(uintptr_t)arg;

    while (!stop) {
        for (size_t off = 0; off < MAP_BYTES; off += PAGE)
            map[off] = mark;
    }
    return NULL;
}

/* Writes the dirty pages back, which is the other way into the shootdown. */
static void *syncer(void *arg)
{
    while (!stop) {
        test_assert(msync(map, MAP_BYTES, MS_SYNC) == 0);
        test_assert(fsync(fd) == 0);
    }
    return NULL;
}

int main(int argc, char **argv)
{
    pthread_t t[4];

    fd = memfd_create("race", 0);
    test_assert(fd >= 0);
    test_assert(ftruncate(fd, MAP_BYTES) == 0);
    map = mmap(NULL, MAP_BYTES, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    test_assert(map != MAP_FAILED);
    memset(map, 0x11, MAP_BYTES);

    /* Before the threads, so that a log which ends here says the machine stopped
       as soon as they started rather than that the test never got going. */
    printf("threads starting\n");
    fflush(stdout);

    test_assert(pthread_create(&t[0], NULL, dropper, NULL) == 0);
    test_assert(pthread_create(&t[1], NULL, toucher, (void *)(uintptr_t)0x22) == 0);
    test_assert(pthread_create(&t[2], NULL, toucher, (void *)(uintptr_t)0x33) == 0);
    test_assert(pthread_create(&t[3], NULL, syncer, NULL) == 0);

    for (int round = 0; round < ROUNDS; round++) {
        /* Printed before the work, so that a log which stops says where. */
        printf("round %d\n", round);
        fflush(stdout);
        for (size_t off = 0; off < MAP_BYTES; off += PAGE)
            map[off] = 0x44;
        test_assert(madvise(map, MAP_BYTES, MADV_DONTNEED) == 0);
    }

    stop = 1;
    for (int i = 0; i < 4; i++)
        test_assert(pthread_join(t[i], NULL) == 0);

    test_assert(munmap(map, MAP_BYTES) == 0);
    test_assert(close(fd) == 0);
    printf("vmap flush race test passed\n");
    return EXIT_SUCCESS;
}
