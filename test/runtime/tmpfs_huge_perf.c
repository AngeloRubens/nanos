/* What the block mappings are worth, on the shape of work a collector does to its
   heap: commit a range of a memory file, fault it in, punch it back out, do it again.

   The two granularities are compared by running this against the same kernel twice,
   once with the manifest's transparent_hugepage left alone and once with it set to
   "never" -- the gate the file-backed fault path shares with anonymous memory. The
   binary does not change between the two, so the difference is the granularity and
   nothing else.

   No timing is asserted. A test that fails on a slow machine is a test that gets
   turned off; this one checks that the work was done correctly and prints the rates
   for the two runs to be read side by side. */

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "../test_utils.h"

#define HUGE_SIZE   (2 * 1024 * 1024)
#define REGION      (128 * 1024 * 1024)
#define GRANULES    (REGION / HUGE_SIZE)
#define CHURN_ROUNDS 4

#ifndef PAGESIZE
#define PAGESIZE    4096
#endif

static double now_s(void)
{
    struct timespec ts;

    test_assert(clock_gettime(CLOCK_MONOTONIC, &ts) == 0);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static double rate_mbs(size_t bytes, double secs)
{
    return secs > 0 ? (bytes / (1024.0 * 1024.0)) / secs : 0;
}

/* One byte per page, which is what makes the kernel fault each one in. */
static void touch(unsigned char *p, size_t len, unsigned char v)
{
    for (size_t off = 0; off < len; off += PAGESIZE)
        p[off] = v;
}

static void check(unsigned char *p, size_t len, unsigned char v)
{
    for (size_t off = 0; off < len; off += PAGESIZE)
        test_assert(p[off] == v);
}

static unsigned char *reserve_aligned(size_t len)
{
    unsigned char *p = mmap(NULL, len + HUGE_SIZE, PROT_NONE,
                            MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    test_assert(p != MAP_FAILED);
    return (unsigned char *)(((uintptr_t)p + HUGE_SIZE - 1) & ~((uintptr_t)HUGE_SIZE - 1));
}

int main(int argc, char **argv)
{
    int fd = memfd_create("hugeperf", 0);
    test_assert(fd >= 0);
    test_assert(ftruncate(fd, REGION) == 0);

    unsigned char *base = reserve_aligned(REGION);
    test_assert(mmap(base, REGION, PROT_READ | PROT_WRITE,
                     MAP_SHARED | MAP_FIXED, fd, 0) == base);

    /* Faulting a whole region in from nothing. */
    double t0 = now_s();
    touch(base, REGION, 0x5a);
    double fault_s = now_s() - t0;
    check(base, REGION, 0x5a);

    /* And the cycle a collector repeats: give a granule back, take it again. Each
       round punches every granule and faults it in afresh, which is the uncommit and
       recommit path rather than the first-touch one. */
    size_t churn_bytes = 0;
    double churn_s = 0;
    for (int round = 0; round < CHURN_ROUNDS; round++) {
        unsigned char v = 0x10 + round;
        double c0 = now_s();
        for (int g = 0; g < GRANULES; g++) {
            off_t off = (off_t)g * HUGE_SIZE;
            test_assert(fallocate(fd, FALLOC_FL_PUNCH_HOLE | FALLOC_FL_KEEP_SIZE,
                                  off, HUGE_SIZE) == 0);
            touch(base + off, HUGE_SIZE, v);
        }
        churn_s += now_s() - c0;
        churn_bytes += REGION;
        check(base, REGION, v);
    }

    printf("region %d MiB, %d granules of %d KiB\n",
           REGION / (1024 * 1024), GRANULES, HUGE_SIZE / 1024);
    printf("first fault:  %.3f s  %.1f MiB/s\n", fault_s, rate_mbs(REGION, fault_s));
    printf("punch+fault:  %.3f s  %.1f MiB/s  (%d rounds)\n",
           churn_s, rate_mbs(churn_bytes, churn_s), CHURN_ROUNDS);

    test_assert(munmap(base, REGION) == 0);
    test_assert(close(fd) == 0);
    printf("tmpfs_huge_perf: OK\n");
    return EXIT_SUCCESS;
}
