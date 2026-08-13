#include <kernel.h>

#define FLUSH_THRESHOLD 32
#define MAX_FLUSH_ENTRIES 1024
#define ENTRIES_SERVICE_THRESHOLD (MAX_FLUSH_ENTRIES/2)

BSS_RO_AFTER_INIT static boolean initialized;
BSS_RO_AFTER_INIT static int flush_ipi;
static volatile word inval_gen;
BSS_RO_AFTER_INIT static queue free_flush_entries;
BSS_RO_AFTER_INIT static flush_entry flush_entries;
static struct list entries;
static int entries_count;
static volatile boolean service_scheduled;
BSS_RO_AFTER_INIT static thunk flush_service;
static struct rw_spinlock flush_lock;

static void queue_flush_service(void);

struct flush_entry {
    struct list l;
    u64 gen;
    struct refcount ref;
    boolean flush;
    volatile boolean wait;
    u32 joined;
    u64 pages[FLUSH_THRESHOLD];
    int npages;
    thunk completion;
    closure_struct(thunk, finish);
};

/* Every entry there is a slot of the one array allocated in init_flush(), and the free queue is
 * the only thing that hands them out, so a pointer that is not one of those slots is not an entry
 * -- whatever it was dequeued from. Checked on both sides of the queue because the two sides say
 * different things: on the way in it is the caller, or the list it was walking, that is wrong; on
 * the way out it is the queue itself. A fault inside the memset in get_page_flush_entry() says
 * neither. */
static inline boolean is_flush_entry(flush_entry f)
{
    return (f >= flush_entries) && (f < flush_entries + MAX_FLUSH_ENTRIES);
}

closure_func_basic(thunk, void, flush_complete)
{
    queue_flush_service();
}

/* Can temporarily drop the reader lock while waiting on a given flush entry. */
static boolean flush_gen_rlocked(word gen, cpuinfo ci, boolean full_flush)
{
    word oldgen = ci->inval_gen;
    ci->inval_gen = gen;
    list_foreach(&entries, l) {
        flush_entry f = struct_from_list(l, flush_entry, l);
        if (f->gen <= oldgen)
            continue;
        if (f->gen > gen)
            break;
        if (!full_flush) {
            if (f->flush) {
                full_flush = true;
            } else {
                for (int i = 0; i < f->npages; i++)
                    invalidate(f->pages[i]);
            }
        }
        if (f->wait) {
            /* To avoid deadlock if another CPU is doing the flush service (for which it needs
             * to acquire the lock as writer), temporarily drop the reader lock while waiting.
             * It is OK to drop the lock while in the middle of list traversal, because:
             * - only entries before the current entry in the list can be deleted
             * - only entries after the current entry in the list can be added
             */
            spin_runlock(&flush_lock);
            fetch_and_add_32(&f->joined, 1);
            while (f->wait)
                kern_pause();
            spin_rlock(&flush_lock);
        }
        refcount_release(&f->ref);
    }
    return full_flush;
}

/* must be called with interrupts off */
static void _flush_handler(void)
{
    cpuinfo ci = current_cpu();
    /* Each generation has at least one page, so if the gen difference is
     * greater than FLUSH_THRESHOLD, just do a full tlb flush */
    boolean full_flush = inval_gen - ci->inval_gen > FLUSH_THRESHOLD;

    spin_rlock(&flush_lock);
    while (ci->inval_gen != inval_gen)
        full_flush = flush_gen_rlocked(inval_gen, ci, full_flush);
    spin_runlock(&flush_lock);

    flush_tlb(full_flush);
}

closure_function(0, 0, void, flush_handler)
{
    _flush_handler();
}

void page_invalidate_flush(void)
{
    if (initialized)
        _flush_handler();
}

void page_invalidate(flush_entry f, u64 p)
{
    if (f && initialized) {
        if (f->flush)
            return;
        f->pages[f->npages++] = p;
        if (f->npages >= FLUSH_THRESHOLD)
            f->flush = true;
    } else {
        invalidate(p);
    }
}

static void service_list(void)
{
    list_foreach(&entries, l) {
        flush_entry f = struct_from_list(l, flush_entry, l);
        if (f->ref.c > 0)
            continue;
        list_delete(&f->l);
        entries_count--;
        thunk completion = f->completion;
        if (completion)
            async_apply(completion);
        assert(is_flush_entry(f));
        assert(enqueue(free_flush_entries, f));
    }
}

closure_function(0, 0, void, do_flush_service)
{
    while (service_scheduled) {
        service_scheduled = false;
        u64 flags = spin_wlock_irq(&flush_lock);
        service_list();
        spin_wunlock_irq(&flush_lock, flags);
    }
}

static void queue_flush_service(void)
{
    if (!service_scheduled) {
        service_scheduled = true;
        async_apply_bh(flush_service);
    }
}

void page_invalidate_sync(flush_entry f, thunk completion, boolean rendezvous)
{
    if (initialized) {
        assert(is_flush_entry(f));      /* before it is read, so a bad one is named here */
        if (f->npages == 0) {
            assert(enqueue(free_flush_entries, f));
            return;
        }
        init_refcount(&f->ref, total_processors,
                      init_closure_func(&f->finish, thunk, flush_complete));
        f->wait = rendezvous;
        if (rendezvous) {
            f->joined = 1;
            f->completion = 0;  /* the completion is invoked during the rendez-vous */
        } else {
            f->completion = completion; /* the completion is invoked asynchronously */
        }

        u64 flags = irq_disable_save();
        spin_wlock(&flush_lock);

        /* The service thunk doesn't always get a chance to run before
         * running out of flush resources, so proactively service the list */
        if (entries_count > ENTRIES_SERVICE_THRESHOLD)
            service_list();

        /* Set flush true on all previous entries to avoid wasted
         * invalidations if this entry causes a flush */
        if (f->flush) {
            list_foreach(&entries, l) {
                flush_entry ff = struct_from_list(l, flush_entry, l);
                if (!ff->flush)
                    ff->flush = true;
            }
        }
        list_push_back(&entries, &f->l);
        entries_count++;
        word prev_gen = fetch_and_add((word *)&inval_gen, 1);
        f->gen = prev_gen + 1;
        spin_wunlock(&flush_lock);

        send_ipi(TARGET_EXCLUSIVE_BROADCAST, flush_ipi);
        if (rendezvous) {
            boolean full_flush = false;
            while (((volatile flush_entry)f)->joined < total_processors) {
                /* Another CPU might be waiting for us to join another rendezvous: to avoid
                 * deadlock, handle any flush entries older than the current entry. */
                spin_rlock(&flush_lock);
                full_flush = flush_gen_rlocked(prev_gen, current_cpu(), full_flush);
                spin_runlock(&flush_lock);
                kern_pause();
            }
            apply(completion);
            f->wait = false;
            if (full_flush)
                flush_tlb(full_flush);
        }
        _flush_handler();
        irq_restore(flags);
    } else {
        flush_tlb(false);
        if (completion)
            async_apply(completion);
    }
}

/* The ring itself cannot invent a value: a producer only ever stores an entry (asserted above),
 * and a slot cannot be given to a producer and a consumer at the same time -- a producer may
 * reserve no further than cons_tail + size - 1, and a consumer reading a slot holds it at or
 * above cons_tail, so the two indices can never meet on one slot. What the ring does not
 * survive is being re-entered on one processor, and that ends in a spin, not in a wrong
 * pointer. So a value that is not an entry was written into the ring by something that does
 * not know it is a ring -- and how many of the slots hold one says which kind of something:
 * one is a stray write, a run of them is memory handed out twice. */
/* The ring turns out to be in use as something else -- a stack, by the contents -- so the
 * question moved from "who wrote this" to "who was given this memory". Answered where an
 * allocation is made rather than when its contents come back looking like a flush entry: the
 * ring is one range, known from boot, and every allocation either overlaps it or does not. Two
 * compares, on a path that is already allocating pages. */
boolean flush_ring_overlaps(u64 start, u64 len)
{
    if (!initialized)
        return false;
    queue q = free_flush_entries;
    u64 rs = u64_from_pointer(q);
    u64 re = u64_from_pointer(q->d) + (U64_FROM_BIT(q->order) * sizeof(void *));
    return (start < re) && (rs < start + len);
}

static void report_bad_flush_entry(flush_entry fe)
{
    queue q = free_flush_entries;
    u64 size = U64_FROM_BIT(q->order);
    u64 foreign = 0, first = size;

    for (u64 i = 0; i < size; i++) {
        flush_entry f = q->d[i];
        if ((f != 0) && (f != INVALID_ADDRESS) && !is_flush_entry(f)) {
            if (foreign++ == 0)
                first = i;
        }
    }
    rputs("flush: the free queue handed out something that is not an entry\n");
    rprintf("  got 0x%lx; the %d entries are [0x%lx, 0x%lx), one is 0x%lx bytes\n",
            u64_from_pointer(fe), MAX_FLUSH_ENTRIES, u64_from_pointer(flush_entries),
            u64_from_pointer(flush_entries + MAX_FLUSH_ENTRIES), sizeof(struct flush_entry));
    rprintf("  ring 0x%lx, %ld slots, prod %d/%d, cons %d/%d\n", u64_from_pointer(q->d), size,
            q->prod_head, q->prod_tail, q->cons_head, q->cons_tail);
    rprintf("  %ld of those slots hold something that is not an entry, the first at %ld\n",
            foreign, first);
    if (first < size)
        rprintf("  around it: 0x%lx 0x%lx 0x%lx\n",
                u64_from_pointer(q->d[(first - 1) & MASK(q->order)]),
                u64_from_pointer(q->d[first]),
                u64_from_pointer(q->d[(first + 1) & MASK(q->order)]));
}

flush_entry get_page_flush_entry(void)
{
    flush_entry fe;

    if (!initialized)
        return 0;

    /* This spins because it must succeed */
    while ((fe = dequeue(free_flush_entries)) == INVALID_ADDRESS) {
        /* Do the flush work to ensure the free queue is not starved by this CPU getting too far
         * behind. */
        u64 flags = irq_disable_save();
        _flush_handler();
        irq_restore(flags);
        kern_pause();
    }

    if (!is_flush_entry(fe))
        report_bad_flush_entry(fe);
    assert(is_flush_entry(fe));
    runtime_memset((void *)fe, 0, sizeof(*fe));
    return fe;
}

void init_flush(heap h)
{
    flush_ipi = allocate_ipi_interrupt();
    register_interrupt(flush_ipi, closure(h, flush_handler), ss("flush ipi"));
    list_init(&entries);
    flush_service = closure(h, do_flush_service);
    free_flush_entries = allocate_queue(h, MAX_FLUSH_ENTRIES + 1);
    flush_entry fa = allocate(h, sizeof(struct flush_entry) * MAX_FLUSH_ENTRIES);
    assert(fa);
    flush_entries = fa;
    for (flush_entry f = fa; f < fa + MAX_FLUSH_ENTRIES; f++)
        assert(enqueue(free_flush_entries, f));
    initialized = true;
}
