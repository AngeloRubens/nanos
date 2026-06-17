/* test/gve/shim/lock.h — spinlock shim.
 *
 * The normal harness build falls through to the real runtime lock.h (which,
 * without KERNEL+SMP_ENABLE, compiles spinlocks to no-ops — fine for the
 * single-threaded functional suite).
 *
 * The ThreadSanitizer concurrency harness defines GVE_HARNESS_SMP and gets
 * REAL atomic spinlocks here, built on the __atomic builtins so TSan sees the
 * acquire/release happens-before and can tell genuine driver races from
 * lock-protected access.  This is what lets `make tsan` actually exercise the
 * driver's ring_mtx / service_lock / global_lock under contention. */
#ifdef GVE_HARNESS_SMP

typedef struct spinlock {
    volatile u64 w;
} *spinlock;

typedef struct rw_spinlock {
    struct spinlock l;
    volatile u64 readers;
} *rw_spinlock;

static inline void spin_lock_init(spinlock l)
{
    __atomic_store_n(&l->w, 0, __ATOMIC_RELAXED);
}
static inline void spin_rw_lock_init(rw_spinlock l)
{
    spin_lock_init(&l->l);
    __atomic_store_n(&l->readers, 0, __ATOMIC_RELAXED);
}

static inline boolean spin_try(spinlock l)
{
    u64 expected = 0;
    return __atomic_compare_exchange_n(&l->w, &expected, 1, false,
                                       __ATOMIC_ACQUIRE, __ATOMIC_RELAXED);
}
static inline void spin_lock(spinlock l)
{
    while (!spin_try(l))
        __builtin_ia32_pause();
}
static inline void spin_unlock(spinlock l)
{
    __atomic_store_n(&l->w, 0, __ATOMIC_RELEASE);
}

/* The concurrency scenarios use only the plain spinlocks above; the rw
 * variants are a conservative exclusive implementation for completeness. */
static inline boolean spin_tryrlock(rw_spinlock l) { return spin_try(&l->l); }
static inline void    spin_rlock(rw_spinlock l)    { spin_lock(&l->l); }
static inline void    spin_runlock(rw_spinlock l)  { spin_unlock(&l->l); }
static inline boolean spin_trywlock(rw_spinlock l) { return spin_try(&l->l); }
static inline void    spin_wlock(rw_spinlock l)    { spin_lock(&l->l); }
static inline void    spin_wunlock(rw_spinlock l)  { spin_unlock(&l->l); }

#else
#include_next <lock.h>
#endif
