# Testi delle PR — serie uncommit/ZGC

Il **cappello va in cima a ogni PR, identico**. Poi la sezione della singola patch.

---

## CAPPELLO (identico in tutte e nove)

> **Where this comes from.** This is one of nine patches that came out of making
> `fallocate(FALLOC_FL_PUNCH_HOLE)` on a memory filesystem give the pages back,
> so that a JVM heap can shrink. ZGC keeps its heap in a memfd and its
> uncommitter punches holes in it to return memory to the system; on nanos the
> punch cleared the range and freed nothing, so the heap never shrank and the
> feature was inert.
>
> Making it free the pages put the kernel under a pattern it had not had before:
> memory being returned and faulted back in continuously, from several threads,
> for as long as the process runs. That pattern **did not create** the defects
> this series fixes -- every one of them is in code that predates this work, and
> the first two stop the binary nanos ships today with none of my changes applied
> -- but it makes them fire in minutes instead of never.
>
> They are sent separately because each is an independent defect with its own
> justification, and each stands without the others. Read in this order:
>
> 1. `kernel: do not wait for the vmap lock deaf to a pending flush`
> 2. `kernel: do not wait for the page table lock deaf to a pending flush`
> 3. `pagecache: do not walk a range through the pages it has not got`
> 4. `pagecache: do not wait for a node lock from inside the page table walk`
> 5. `pagecache: do not leave with the node's lock when the page has gone`
> 6. `pagecache: give nothing back for a page that is already free`
> 7. `pagecache: give a queued sync completion back on its own context`
> 8. `pagecache: let the node go before calling into the filesystem`
> 9. `fs: give back the pages of a punched hole on a memory filesystem`
>
> The last is the feature. The eight before it are what it takes for the feature
> to run: with the punch and without them, a JVM using ZGC stops the machine.
> **They are not all specific to ZGC**, and where they are not, the patch says so
> -- several are on paths any program walks, which is why two of them stop the
> stock kernel on a workload that never punches a hole at all.
>
> Everything below was measured, on x86-64, with `make VCPUS=2 test-noaccel`,
> which is this project's own CI configuration.

---

## L'argomento della serie — in cima a tutte

`test/runtime/vmap_flush_jvmpath` returns pages the way HotSpot does --
`PROT_NONE` mapped over the range with `MAP_FIXED`, which is
`os::pd_uncommit_memory` on every collector -- while other threads fault them
back in and the mapping is synced. With `VCPUS=2`:

| tree | result |
|---|---|
| master, unpatched | **hangs at round 0, 5 times out of 5** |
| only the lock patches, master's pagecache | **hangs at round 0, 3 of 3** |
| only the pagecache patches, master's locks | **hangs at round 0, 3 of 3** |
| all nine | **64 rounds, 13 of 14** |

Neither half is enough on its own. That is the reason they are sent together
rather than the locks first and the rest later.

# La scala misurata — nel testo delle PR 1, 2 e 3

`test/runtime/vmap_flush_race` (new, built on the barrier and shape of
`tlbshootdown.c`) returns pages from more than one thread while others fault on
them and the mapping is synced:

| tree | rounds of 64 completed |
|---|---|
| master, unpatched | **0** -- stops before the first |
| + #1 | **28** -- the same cycle, now on the page table lock |
| + #2 | **43** -- faults at 0x3b in the traverse |
| + #3 | **64, passes** |

None of the three alone makes it pass. `tlbshootdown` passes on all of them: it
unmaps from one thread while the others read, and the cycle needs two returning
pages at once.

---

## 1 — vmap lock

**Test:** `test/runtime/vmap_flush_race` (new). Red on master (stops before round 0), green here (64 rounds).

Four processors caught with the gdb stub on a stock build: two spinning on
`p->vmap_lock` in `madvise` (`mmap.c:1466`) with interrupts disabled, one holding
it inside `page_invalidate_sync`, one in `fsync` waiting for every processor to
join. A waiter that spins with interrupts off never services the flush IPI.

**Who walks this path:** every `mmap`, `munmap`, `mprotect`, `mremap`, `madvise`,
`msync` and every page fault -- `vmap_lock` is taken by all of them
(`unix_internal.h:833`).

A JVM reaches it on **every** collector, and not through `madvise`: HotSpot
uncommits by mapping `PROT_NONE` over the range with `MAP_FIXED|MAP_ANONYMOUS`
(`os::pd_uncommit_memory`, `os_linux.cpp:3455`) -- G1
(`g1PageBasedVirtualSpace.cpp:203`), Shenandoah (`shenandoahHeap.cpp:1803`),
Parallel and Serial (`virtualspace.cpp:373`), ZGC
(`zPhysicalMemoryBacking_linux.cpp:705`). On nanos that `MAP_FIXED` goes through
`process_remove_range_locked` (`mmap.c:1396`) under this same lock and into the
same `unmap_and_free_phys`. HotSpot's only `madvise(MADV_DONTNEED)` is in
`MutableNUMASpace` (`mutableNUMASpace.cpp:215`), which is inert here.

## 2 — page table lock

**Test:** `test/runtime/vmap_flush_race`. 28 rounds of 64 without this patch, 64 with.

The same shape on `pt_lock`, which the fault takes to map a page and the unmap
takes to remove one. Not argued from symmetry: 28 rounds without it, 64 with.

**Who walks this path:** the page fault and the unmap, for anonymous
(`mmap.c:183`) and file-backed (`:245`) alike. Everything.

## 3 — traverse

**Test:** `test/runtime/vmap_flush_race`, once the two above are in: 43 rounds and a fault at 0x3b without this patch, 64 with.

`pagecache_nodelocked_traverse` hands the first page to the handler without
asking whether it is there, then walks by successor for the length of the range
as though the range were covered. It need not be. The pin handler increments a
refcount through what it is given; refcount is at 0x3c in the record, so a
pointer of all ones writes to 0x3b -- the address of the fault.

**Who walks this path:** `pagecache_nodelocked_traverse` has two callers, and the
only users outside the pagecache are `klib/tmpfs.c:39`, `:98` and `:254` -- so
this is tmpfs-only, and tmpfs is what stands behind `memfd_create`
(`klib/shmem.c:39`). The pin is taken by the write-back, which means the five
second timer (`pagecache.c:1697`) reaches it and not only an explicit sync.

**ZGC only.** A collector with an anonymous heap has no pagecache node and never
comes near this.

## 4 — node lock inside the page table walk

**Test:** none. Reproduction only: concurrent madvise and fsync on a shared mapping while the pagecache scan timer runs.

`pagecache_check_dirty_page` and `pagecache_check_old_page` take the node lock
from inside `traverse_ptes`, which runs holding the page table lock, while the
unmap path takes the two in the opposite order.

**Who walks this path:** two halves, and they are not the same path.
`check_dirty_page` is reached from the five second scan (`pagecache.c:1671`) and
from `pagecache_node_scan` (`:1800`), i.e. from `msync(MS_SYNC)` (`mmap.c:1201`)
and `fsync`; it only ever walks `MAP_SHARED` file mappings. `check_old_page` is
reached from `pagecache_drain` (`:918`) under **memory pressure**
(`init.c:717`), and walks private mappings too (`:951`) -- the `.so` files and
jars a JVM maps, not just its heap.

**On aarch64 the dirty half is dead code**: `pte_is_dirty()` is a stub returning
false (`src/aarch64/page_machine.h:404`), so that body never runs and the node
lock is never asked for there. On arm64 this patch is worth having for the
reclaim half; on x86-64 for both.

## 5 — node lock on the early return

**Test:** none. `test/runtime/tmpfs_punch_race` stopped once at round 8 without this patch, which
looked like the signature -- threads that fault held while the rest runs -- but four repeats without
it and four with it all completed, so that was noise and is not offered as evidence.

`pagecache_release_page` takes the node's lock, looks the page up, and returns
from inside the lock when the lookup finds nothing. The caller is the page fault
path (`mmap.c:264`, its only caller), reached when two processors fault the same
page at once and one of them has to give its reference back. Found by reading
rather than from a failure.

**Who walks this path:** any file-backed mapping, shared **and** private
(`mmap.c:294-297`) -- so every JVM, through `libjvm.so`, its jars and its
modules, and not only a heap kept in a file.

## 6 — page that is already free

**Test:** none that tells the two kernels apart. Removing this patch alone leaves every test in the tree passing; the evidence is the arithmetic above and twelve occurrences in CI.

`realloc_pagelocked` states the contract -- free means no memory and a refcount
of exactly zero -- and the release path deallocates `pp->kvirt` without asking
whether there is one, handing INVALID_ADDRESS to the page heap.
`pageheap_area_from_page` reads `r.start` through the rangemap miss: offset 0x18
on a pointer of all ones is a read of 0x17, and `pageheap_dealloc+0x2c`
disassembles to `mov 0x18(%rax),%rdi`. Twelve occurrences in CI logs, never in
the runs without this series.

## 7 — sync completion

The queued completion only exists when the node is busy and a caller asked for
one (`pagecache.c:1208-1215`), which happens from `sync_node` (`:1276`) --
`fsync`/`fdatasync` (`syscall.c:1383`) -- and from closing a file (`:1304`).
`msync` and the write-back timer pass none.

**Who walks this path:** `fsync` and `fdatasync` on a file. **Not ZGC**:
`zPhysicalMemoryBacking_linux.cpp` contains neither `fsync` nor `msync`, so the
heap file is never synced. In a JVM it is reached through `FileChannel.force`
and `FileDescriptor.sync` (`libnio.so` and `libjava.so` import `fsync`).

**Test:** none. `test/runtime/fsync_completion_race` was written for this path --
a file on the root filesystem so that committing is real I/O and the node stays
busy, one syncing thread per processor so that a completion can queue behind
another -- and it passes on master, on this branch, and on this branch with only
this patch reverted -- at 128K on the default image, at 4M on a 64M one, and at
96M on a 512M one, the last of which recorded 751 completed syncs in a single
round. So neither the size of the work nor the number of callers is the
obstacle: the threads are not serialising, and the queued-completion window
still does not open from a plain multi-threaded fsync. Whether it is reachable
from userspace at all is not established. Recorded rather than dropped, so
that the next attempt starts from what has already failed.

This is the weakest patch of the nine and is sent last: the observations that
prompted it come from before patches 7 and 8 themselves, so it justifies itself
in a circle, and no workload here reproduces it. It is not cosmetic, though --
`assert` is not compiled out in nanos (`runtime.h:69-82`), so the path it guards
is a halt of the guest in production.

## 8 — filesystem call under the node lock

**Test:** none. `test/runtime/tmpfs_commit_race` exercises this path but passes on master too, so it is offered as a non-regression test and not as evidence.

`pagecache_commit_dirty_ranges` calls `pn->fs_write` (`pagecache.c:1061`) holding
the node's spinlock, and the write path takes a mutex that can sleep -- in the
memory filesystem (`klib/tmpfs.c:53`) and on the root filesystem alike
(`tfs.c:796`), both through `filesystem_lock` (`fs.h:151`).

**Who walks this path:** file I/O, with or without a mapping. Not specific to a
heap, and not specific to tmpfs.

## 9 — the punch

**Test:** `test/runtime/tmpfs_punch`. Red on master at the line that requires the block count to drop, green here.

**Who walks this path:** ZGC, literally. `fallocate_punch_hole` with
`FALLOC_FL_PUNCH_HOLE|FALLOC_FL_KEEP_SIZE`
(`zPhysicalMemoryBacking_linux.cpp:552`) on a descriptor from `memfd_create`
(`:211`), sized with `ftruncate` (`:137`) and mapped `MAP_FIXED|MAP_SHARED`
(`:694`), driven by the `ZUncommitter` thread. No other collector: theirs are
anonymous from reservation to uncommit and never call `fallocate`.

Note on the image rather than the code: `memfd_create` needs the `shmem` klib
(`klib/shmem.c:44`), and ZGC is not the default collector -- `-XX:+UseZGC` is
required.

Comes with `test/runtime/tmpfs_punch`, which punches half a file and requires the
block count to drop; on master it fails at that line, and it is the only
assertion in the file that tells the two kernels apart. Measured with a real JVM:
**10.3 GB returned across a thirty minute soak** with three threads writing a
1.5 GB heap, no wrong pages, no faults.
