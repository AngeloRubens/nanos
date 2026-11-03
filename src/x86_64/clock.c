#include <kernel.h>
#include <apic.h>
#include <io.h>

#define __vdso_dat (&(VVAR_REF(vdso_dat)))

#define PIT_FREQUENCY   1193182ul   /* Hz */
#define PIT_PERIOD_MSB  nanoseconds(256 * BILLION / PIT_FREQUENCY)

BSS_RO_AFTER_INIT clock_now platform_monotonic_now;
BSS_RO_AFTER_INIT clock_timer platform_timer;

void init_clock(void)
{
    /* detect rdtscp */
    u32 regs[4];
    cpuid(0x80000001, 0, regs);
    __vdso_dat->clock_src = VDSO_CLOCK_SYSCALL;
    __vdso_dat->machine.platform_has_rdtscp = (regs[3] & U64_FROM_BIT(27)) != 0;
}

/* error refers to the time (expressed in TSC cycles) it takes to read the PIT counter value. */
static boolean pit_wait_msb(u8 msb, u64 *error, u64 *tsc)
{
    u64 prev_tsc = 0;
    int i = 0;
    for (;;) {
        *tsc = rdtsc();
        *error = *tsc - prev_tsc;
        in8(0x42);  /* LSB, ignored */
        if (in8(0x42) != msb)
            break;
        if (++i > 10000)
            return false;   /* The PIT is not counting as expected. */
        prev_tsc = *tsc;
    }
    return true;
}

/* Calibrates the TSC using PIT channel 2. Both ends of the measurement are the instants at which the
 * most significant byte of the PIT counter changes, so that the time that elapses between loading
 * the counter and the first reading, which is long when accessing an I/O port is slow, is not
 * counted as part of the first period. */
static u64 tsc_calibrate(void)
{
    out8(0x61, (in8(0x61) & ~0x02) | 0x01); /* set gate high, disable speaker */
    out8(0x43, 0xB0);   /* lobyte/hibyte access mode, operating mode 0, binary mode */
    out8(0x42, 0xFF);   /* set initial count LSB */
    out8(0x42, 0xFF);   /* set initial count MSB */
    in8(0x42);          /* LSB, ignored */
    u8 msb = in8(0x42);
    u64 start, error, tsc;
    if (!pit_wait_msb(msb, &error, &start))
        return 0;
    int periods = 0;
    for (int i = 1; i < msb; i++) {
        if (!pit_wait_msb(msb - i, &error, &tsc))
            return 0;
        periods = i;
        if (error < ((tsc - start) >> 11))  /* aim for an error lower than ~500 PPM */
            break;
    }
    if (periods == 0)
        return 0;
    timestamp elapsed = periods * PIT_PERIOD_MSB;
    return (elapsed << 32) / (tsc - start);
}

closure_function(1, 0, timestamp, tsc_now,
                 u64, scaling)
{
    return (((u128)rdtsc()) * bound(scaling)) >> 32;
}

boolean init_tsc_timer(kernel_heaps kh)
{
    u64 tsc_scaling = tsc_calibrate();
    if (tsc_scaling) {
        register_platform_clock_now(closure(heap_general(kh), tsc_now, tsc_scaling),
                                    VDSO_CLOCK_TSC_STABLE, 0);
        thunk percpu_init;
        boolean success = init_lapic_timer(&platform_timer, &percpu_init);
        if (success)
            register_percpu_init(percpu_init);
        return success;
    } else {
        return false;
    }
}
