/* Two threads faulting the same page of a shared mapping while a third reads it
   through a private one.

   A private mapping of a file gets its own copy of a page, and when it is the
   only reader the page cache hands the page over rather than copying it --
   which takes the record out of the node. Meanwhile two processors faulting the
   same page of the shared mapping race to map it: one wins, and the other gives
   its reference back through pagecache_release_page. That function takes the
   node's lock, looks the page up, and when the lookup finds nothing -- which is
   what the private mapping just arranged -- returns from inside the lock.

   Nothing crashes. Every thread that faults on that node afterwards waits behind
   a lock nobody will release, while everything that does not touch the node
   carries on: the counters printed each round say which it is. */

#define _GNU_SOURCE
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/sysinfo.h>
#include <unistd.h>

#include "../test_utils.h"

#define PAGESIZE    4096
#define FILE_PAGES  512
#define FILE_BYTES  (FILE_PAGES * PAGESIZE)
#define ROUNDS      40

static int fd;
static volatile uint8_t *shared;
static volatile uint8_t *private_map;
static volatile int stop;
static volatile unsigned long shared_faults, private_faults, drops;

static void *shared_toucher(void *arg)
{
    unsigned long n = 0;
    while (!stop) {
        for (int p = 0; p < FILE_PAGES; p++)
            shared[p * PAGESIZE] = 0x33;
        n++;
        __sync_fetch_and_add(&shared_faults, FILE_PAGES);
    }
    return NULL;
}

/* The private side, which is what takes records out of the node. */
static void *private_reader(void *arg)
{
    while (!stop) {
        for (int p = 0; p < FILE_PAGES; p++)
            (void)private_map[p * PAGESIZE];
        __sync_fetch_and_add(&private_faults, FILE_PAGES);
    }
    return NULL;
}

/* Gives the pages back so that both sides have to fault them in again. */
static void *dropper(void *arg)
{
    while (!stop) {
        test_assert(madvise((void *)shared, FILE_BYTES, MADV_DONTNEED) == 0);
        __sync_fetch_and_add(&drops, 1);
    }
    return NULL;
}

int main(int argc, char **argv)
{
    pthread_t t[4];

    printf("There are %d processors available\n", get_nprocs());
    fd = memfd_create("relrace", 0);
    test_assert(fd >= 0);
    test_assert(ftruncate(fd, FILE_BYTES) == 0);
    shared = mmap(NULL, FILE_BYTES, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    test_assert(shared != MAP_FAILED);
    private_map = mmap(NULL, FILE_BYTES, PROT_READ, MAP_PRIVATE, fd, 0);
    test_assert(private_map != MAP_FAILED);
    memset((void *)shared, 0x11, FILE_BYTES);

    printf("threads starting\n");
    fflush(stdout);
    test_assert(pthread_create(&t[0], NULL, shared_toucher, NULL) == 0);
    test_assert(pthread_create(&t[1], NULL, shared_toucher, NULL) == 0);
    test_assert(pthread_create(&t[2], NULL, private_reader, NULL) == 0);
    test_assert(pthread_create(&t[3], NULL, dropper, NULL) == 0);

    for (int round = 0; round < ROUNDS; round++) {
        printf("round %d: shared %lu private %lu drops %lu\n",
               round, shared_faults, private_faults, drops);
        fflush(stdout);
        usleep(200000);
    }

    stop = 1;
    for (int i = 0; i < 4; i++)
        test_assert(pthread_join(t[i], NULL) == 0);
    test_assert(munmap((void *)shared, FILE_BYTES) == 0);
    test_assert(munmap((void *)private_map, FILE_BYTES) == 0);
    test_assert(close(fd) == 0);
    printf("release page race test passed\n");
    return EXIT_SUCCESS;
}
