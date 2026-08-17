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

## La scala misurata — nel testo delle PR 1, 2 e 3

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

**Who walks this path:** `madvise` and `munmap`, on any mapping. Not ZGC, whose
heap is a memfd it punches rather than advises -- but every collector with an
anonymous heap returns memory exactly this way, and so does any program that
does.

## 2 — page table lock

**Test:** `test/runtime/vmap_flush_race`. 28 rounds of 64 without this patch, 64 with.

The same shape on `pt_lock`, which the fault takes to map a page and the unmap
takes to remove one. Not argued from symmetry: 28 rounds without it, 64 with.

**Who walks this path:** the page fault and the unmap. Everything.

## 3 — traverse

**Test:** `test/runtime/vmap_flush_race`, once the two above are in: 43 rounds and a fault at 0x3b without this patch, 64 with.

`pagecache_nodelocked_traverse` hands the first page to the handler without
asking whether it is there, then walks by successor for the length of the range
as though the range were covered. It need not be. The pin handler increments a
refcount through what it is given; refcount is at 0x3c in the record, so a
pointer of all ones writes to 0x3b -- the address of the fault.

**Who walks this path:** the write handler of a memory filesystem pinning its
dirty ranges (`klib/tmpfs.c:39`). A memfd, which is where ZGC keeps its heap.

## 4 — node lock inside the page table walk

**Test:** none. Reproduction only: concurrent madvise and fsync on a shared mapping while the pagecache scan timer runs.

`pagecache_check_dirty_page` and `pagecache_check_old_page` take the node lock
from inside `traverse_ptes`, which runs holding the page table lock, while the
unmap path takes the two in the opposite order.

**Who walks this path:** the periodic scan of shared mappings
(`pagecache_scan_shared_mappings`, `pagecache.c:1671`), over every shared file
mapping in the system -- which includes a ZGC heap.

## 5 — node lock on the early return

**Test:** none. `test/runtime/tmpfs_punch_race` stopped once at round 8 without this patch, which
looked like the signature -- threads that fault held while the rest runs -- but four repeats without
it and four with it all completed, so that was noise and is not offered as evidence.

`pagecache_release_page` takes the node's lock, looks the page up, and returns
from inside the lock when the lookup finds nothing. The caller is the page fault
path (`mmap.c:264`), reached when two processors fault the same page at once and
one of them has to give its reference back. Found by reading rather than from a
failure.

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

## 8 — filesystem call under the node lock

**Test:** none. `test/runtime/tmpfs_commit_race` exercises this path but passes on master too, so it is offered as a non-regression test and not as evidence.

`pagecache_commit_dirty_ranges` calls `pn->fs_write` holding the node's spinlock,
and a memory filesystem's write path takes a mutex that can sleep.

## 9 — the punch

**Test:** `test/runtime/tmpfs_punch`. Red on master at the line that requires the block count to drop, green here.

Comes with `test/runtime/tmpfs_punch`, which punches half a file and requires the
block count to drop; on master it fails at that line, and it is the only
assertion in the file that tells the two kernels apart. Measured with a real JVM:
**10.3 GB returned across a thirty minute soak** with three threads writing a
1.5 GB heap, no wrong pages, no faults.
