/* Punching a hole in a memory filesystem has to give the memory back, and has to
   leave the file usable afterwards.

   The first is what st_blocks is for: a filesystem that keeps the pages of a hole
   reports the same block count before and after, and a collector that shrinks its
   heap by punching holes in a memfd -- which is what ZGC's uncommitter does --
   frees nothing at all.

   The second is the part that does not look like it needs a test. On a memory
   filesystem the pages of a file are the file, so writing to one pins its pages
   and the pages of a punched range are given back and unpinned. If the record of
   such a page is left behind holding no memory, the next write pins it again and
   the next unpin releases it a second time, handing an address that was never
   allocated to the page heap. Hence the write-punch-write-close below: each step
   is ordinary, and it is only the order that finds it. */

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "../test_utils.h"

#define FILE_SIZE   (4 * 1024 * 1024)
#define CHUNK       (64 * 1024)
#define SECTOR_SIZE 512

static blkcnt_t blocks_of(int fd)
{
    struct stat st;
    test_assert(fstat(fd, &st) == 0);
    return st.st_blocks;
}

/* The blocks of a file on a memory filesystem are counted from the ranges its
   write handler has been given, and the page cache hands those over when it
   commits, not when write() returns -- so a file that has only been written to
   holds no blocks yet, and fsync is what makes the count mean anything. Keeping
   the count as a running total instead of walking the ranges does not change
   that: the total moves where the ranges do, which is inside the write-back. */
static void fill(int fd, size_t size)
{
    uint8_t buf[CHUNK];

    memset(buf, 0xa5, sizeof(buf));
    test_assert(lseek(fd, 0, SEEK_SET) == 0);
    for (size_t off = 0; off < size; off += sizeof(buf))
        test_assert(write(fd, buf, sizeof(buf)) == sizeof(buf));
    test_assert(fsync(fd) == 0);
}

/* A hole gives its blocks back, and reads back as zeroes. */
static void test_punch_frees_blocks(void)
{
    int fd = memfd_create("punch", 0);
    test_assert(fd >= 0);
    test_assert(ftruncate(fd, FILE_SIZE) == 0);
    fill(fd, FILE_SIZE);

    blkcnt_t full = blocks_of(fd);
    test_assert(full == FILE_SIZE / SECTOR_SIZE);

    test_assert(fallocate(fd, FALLOC_FL_PUNCH_HOLE | FALLOC_FL_KEEP_SIZE,
                          0, FILE_SIZE / 2) == 0);
    blkcnt_t holed = blocks_of(fd);
    test_assert(holed == full / 2);

    /* The size is kept, and what the hole covered is zero. */
    test_assert(lseek(fd, 0, SEEK_END) == FILE_SIZE);
    uint8_t buf[CHUNK];
    test_assert(lseek(fd, 0, SEEK_SET) == 0);
    test_assert(read(fd, buf, sizeof(buf)) == sizeof(buf));
    for (size_t i = 0; i < sizeof(buf); i++)
        test_assert(buf[i] == 0);

    /* And what it did not cover is untouched. */
    test_assert(lseek(fd, FILE_SIZE / 2, SEEK_SET) == FILE_SIZE / 2);
    test_assert(read(fd, buf, sizeof(buf)) == sizeof(buf));
    for (size_t i = 0; i < sizeof(buf); i++)
        test_assert(buf[i] == 0xa5);

    /* The whole file, and then nothing left to give back. */
    test_assert(fallocate(fd, FALLOC_FL_PUNCH_HOLE | FALLOC_FL_KEEP_SIZE,
                          0, FILE_SIZE) == 0);
    test_assert(blocks_of(fd) == 0);

    test_assert(close(fd) == 0);
}

/* Writing over a punched range, and then dropping the file. Each step is
   ordinary; the pages of the range are pinned by the first write, given back and
   unpinned by the punch, pinned again by the second write, and released when the
   file goes. A record left behind with no memory is released twice here. */
static void test_rewrite_after_punch(void)
{
    int fd = memfd_create("rewrite", 0);
    test_assert(fd >= 0);
    test_assert(ftruncate(fd, FILE_SIZE) == 0);

    for (int round = 0; round < 4; round++) {
        fill(fd, FILE_SIZE);
        test_assert(blocks_of(fd) == FILE_SIZE / SECTOR_SIZE);
        test_assert(fallocate(fd, FALLOC_FL_PUNCH_HOLE | FALLOC_FL_KEEP_SIZE,
                              0, FILE_SIZE) == 0);
        test_assert(blocks_of(fd) == 0);
        fill(fd, FILE_SIZE);
        test_assert(blocks_of(fd) == FILE_SIZE / SECTOR_SIZE);

        uint8_t buf[CHUNK];
        test_assert(lseek(fd, 0, SEEK_SET) == 0);
        test_assert(read(fd, buf, sizeof(buf)) == sizeof(buf));
        for (size_t i = 0; i < sizeof(buf); i++)
            test_assert(buf[i] == 0xa5);
    }

    test_assert(close(fd) == 0);
}

/* The same range through a mapping rather than through write(), because a hole
   punched under a live mapping has to reach the page tables as well as the file:
   what was mapped and dirty must read back as zero without the mapping being
   torn down first. */
static void test_punch_under_mapping(void)
{
    int fd = memfd_create("mapped", 0);
    test_assert(fd >= 0);
    test_assert(ftruncate(fd, FILE_SIZE) == 0);

    uint8_t *p = mmap(NULL, FILE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    test_assert(p != MAP_FAILED);
    memset(p, 0x5a, FILE_SIZE);
    test_assert(msync(p, FILE_SIZE, MS_SYNC) == 0);

    test_assert(fallocate(fd, FALLOC_FL_PUNCH_HOLE | FALLOC_FL_KEEP_SIZE,
                          0, FILE_SIZE / 2) == 0);
    for (size_t i = 0; i < FILE_SIZE / 2; i++)
        test_assert(p[i] == 0);
    for (size_t i = FILE_SIZE / 2; i < FILE_SIZE; i++)
        test_assert(p[i] == 0x5a);

    /* Written again through the mapping, so the pages come back. */
    memset(p, 0x3c, FILE_SIZE / 2);
    for (size_t i = 0; i < FILE_SIZE / 2; i++)
        test_assert(p[i] == 0x3c);

    test_assert(munmap(p, FILE_SIZE) == 0);
    test_assert(close(fd) == 0);
}

int main(int argc, char **argv)
{
    test_punch_frees_blocks();
    test_rewrite_after_punch();
    test_punch_under_mapping();
    printf("tmpfs punch test passed\n");
    return EXIT_SUCCESS;
}
