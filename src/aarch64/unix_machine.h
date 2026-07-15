/* arch-specific syscall definitions */
struct stat {
    /* 0 - 3 */
    u64 st_dev;
    u64 st_ino;
    u32 st_mode;
    u32 st_nlink;
    u32 st_uid;
    u32 st_gid;

    /* 4 - 7 */
    u64 st_rdev;
    u64 pad1;
    s64 st_size;
    s32 st_blksize;
    s32 pad2;

    /* 8 - 11 */
    s64 st_blocks;
    s64 st_atime;
    u64 st_atime_nsec;
    s64 st_mtime;

    /* 12 - 15 */
    u64 st_mtime_nsec;
    s64 st_ctime;
    u64 st_ctime_nsec;
    s32 unused[2];
} __attribute__((packed));

#define O_DIRECTORY     00040000
#define O_NOFOLLOW      00100000
#define O_DIRECT        00200000
#define O_LARGEFILE     00400000

struct epoll_event {
    u32 events;                 /* Epoll events */
    u64 data;
};

/* kernel stuff */

#define SYSCALL_FRAME_ARG0       FRAME_X0
#define SYSCALL_FRAME_ARG1       FRAME_X1
#define SYSCALL_FRAME_ARG2       FRAME_X2
#define SYSCALL_FRAME_ARG3       FRAME_X3
#define SYSCALL_FRAME_ARG4       FRAME_X4
#define SYSCALL_FRAME_ARG5       FRAME_X5
#define SYSCALL_FRAME_RETVAL1    FRAME_X0
#define SYSCALL_FRAME_RETVAL2    FRAME_X1
#define SYSCALL_FRAME_SP         FRAME_SP
#define SYSCALL_FRAME_SP_TOP     FRAME_STACK_TOP
#define SYSCALL_FRAME_PC         FRAME_ELR

#define MINSIGSTKSZ 5120

struct sigcontext {
    u64 fault_address;
    u64 regs[31];
    u64 sp;
    u64 pc;
    u64 pstate;
    u8 reserved[4096] __attribute__((__aligned__(16)));
};

struct _aarch64_ctx {
    u32 magic;
    u32 size;
};

#define FPSIMD_MAGIC 0x46508001

struct fpsimd_context {
    struct _aarch64_ctx head;
    u32 fpsr;
    u32 fpcr;
    u128 vregs[32];
};

struct ucontext {
    unsigned long uc_flags;
    struct ucontext * uc_link;
    stack_t uc_stack;
    sigset_t uc_sigmask;
    u8 pad[1024 / 8 - sizeof(sigset_t)];
    struct sigcontext uc_mcontext;
};

struct rt_sigframe {
    struct siginfo info;
    struct ucontext uc;
};

/* XXX consolidate with sigcontext? */
struct core_regs {
    u64 regs[31];
    u64 sp;
    u64 pc;
    u64 pstate;
};

static inline pageflags pageflags_from_vmflags(u64 vmflags)
{
    pageflags flags = pageflags_user(pageflags_memory());
    if (vmflags & VMAP_FLAG_EXEC)
        flags = pageflags_exec(flags);
    if (vmflags & VMAP_FLAG_WRITABLE)
        flags = pageflags_writable(flags);
    return flags;
}

static inline void set_tls(context_frame f, u64 tls)
{
    f[FRAME_TPIDR_EL0] = tls;
    f[FRAME_TXCTX_FLAGS] |= FRAME_TXCTX_TPIDR_EL0_SAVED;
}

static inline void syscall_restart_arch_setup(context_frame f)
{
    f[FRAME_SAVED_X0] = f[FRAME_X0];
}

static inline void syscall_restart_arch_fixup(context_frame f)
{
    /* rewind to syscall */
    f[FRAME_X0] = f[FRAME_SAVED_X0];
    f[FRAME_ELR] -= 4; /* rewind to syscall; no thumb mode support */
}

void frame_save_fpsimd(context_frame f);
void frame_restore_fpsimd(context_frame f);

static inline void thread_frame_save_fpsimd(context_frame f)
{
    if ((f[FRAME_TXCTX_FLAGS] & FRAME_TXCTX_FPSIMD_SAVED) == 0) {
        f[FRAME_TXCTX_FLAGS] |= FRAME_TXCTX_FPSIMD_SAVED;
        frame_save_fpsimd(f);
    }
}

static inline void thread_frame_restore_fpsimd(context_frame f)
{
    if (f[FRAME_TXCTX_FLAGS] & FRAME_TXCTX_FPSIMD_SAVED) {
        f[FRAME_TXCTX_FLAGS] &= ~FRAME_TXCTX_FPSIMD_SAVED;
        frame_restore_fpsimd(f);
    }
}

static inline void thread_frame_save_tls(context_frame f)
{
    if ((f[FRAME_TXCTX_FLAGS] & FRAME_TXCTX_TPIDR_EL0_SAVED) == 0) {
        f[FRAME_TXCTX_FLAGS] |= FRAME_TXCTX_TPIDR_EL0_SAVED;
        f[FRAME_TPIDR_EL0] = read_psr(TPIDR_EL0);
    }
}

static inline void thread_frame_restore_tls(context_frame f)
{
    if (f[FRAME_TXCTX_FLAGS] & FRAME_TXCTX_TPIDR_EL0_SAVED) {
        f[FRAME_TXCTX_FLAGS] &= ~FRAME_TXCTX_TPIDR_EL0_SAVED;
        write_psr(TPIDR_EL0, f[FRAME_TPIDR_EL0]);
    }
}

/* AT_HWCAP bit positions (Linux arm64 ABI, <asm/hwcap.h>).  glibc's ifunc
   resolvers and the JVM (HotSpot reads getauxval(AT_HWCAP)) gate SIMD/crypto
   feature use on these.  We advertise only features whose instructions run at
   EL0 without trapping AND whose register state is preserved across a context
   switch, i.e. everything living in the V0-V31 / FPSR / FPCR block already
   saved in the exception frame (FP/SIMD are enabled: CPACR_EL1.FPEN=NO_TRAP in
   crt0.S; DC CVAP is enabled: SCTLR_EL1.UCI in page.c).  Each bit is derived
   from the real ID register field, never hardcoded, so a CPU lacking a feature
   never sees it advertised.

   SVE is deliberately NOT advertised: its Z/P/FFR state is not saved on context
   switch (the frame only holds Q0-Q31), so announcing it would silently corrupt
   vector registers across threads.  Pointer authentication (needs key setup),
   user access to ID registers (traps here), and RNDR (MRS may trap) are omitted
   for the same "advertise only what we truly support" reason. */
#define HWCAP_FP        (1 << 0)
#define HWCAP_ASIMD     (1 << 1)
#define HWCAP_AES       (1 << 3)
#define HWCAP_PMULL     (1 << 4)
#define HWCAP_SHA1      (1 << 5)
#define HWCAP_SHA2      (1 << 6)
#define HWCAP_CRC32     (1 << 7)
#define HWCAP_ATOMICS   (1 << 8)
#define HWCAP_FPHP      (1 << 9)
#define HWCAP_ASIMDHP   (1 << 10)
#define HWCAP_ASIMDRDM  (1 << 12)
#define HWCAP_JSCVT     (1 << 13)
#define HWCAP_FCMA      (1 << 14)
#define HWCAP_LRCPC     (1 << 15)
#define HWCAP_DCPOP     (1 << 16)
#define HWCAP_SHA3      (1 << 17)
#define HWCAP_SM3       (1 << 18)
#define HWCAP_SM4       (1 << 19)
#define HWCAP_ASIMDDP   (1 << 20)
#define HWCAP_SHA512    (1 << 21)
#define HWCAP_ASIMDFHM  (1 << 23)
#define HWCAP_ILRCPC    (1 << 26)
#define HWCAP_FLAGM     (1 << 27)
#define HWCAP_SB        (1 << 29)

static inline u64 get_cpu_capabilities(void)
{
    u64 isar0 = read_psr(ID_AA64ISAR0_EL1);
    u64 isar1 = read_psr(ID_AA64ISAR1_EL1);
    u64 pfr0 = read_psr(ID_AA64PFR0_EL1);
    u64 caps = 0;

    /* FP / AdvSIMD: a field of 0xf means "not implemented" (signed sentinel);
       a value of 1 additionally means half-precision (fp16) support. */
    u64 fp = field_from_u64(pfr0, ID_AA64PFR0_EL1_FP);
    if (fp != 0xf) {
        caps |= HWCAP_FP;
        if (fp == 1)
            caps |= HWCAP_FPHP;
    }
    u64 simd = field_from_u64(pfr0, ID_AA64PFR0_EL1_ADVSIMD);
    if (simd != 0xf) {
        caps |= HWCAP_ASIMD;
        if (simd == 1)
            caps |= HWCAP_ASIMDHP;
    }

    /* ID_AA64ISAR0_EL1 */
    u64 aes = field_from_u64(isar0, ID_AA64ISAR0_EL1_AES);
    if (aes >= 1)
        caps |= HWCAP_AES;
    if (aes >= 2)
        caps |= HWCAP_PMULL;
    if (field_from_u64(isar0, ID_AA64ISAR0_EL1_SHA1) >= 1)
        caps |= HWCAP_SHA1;
    u64 sha2 = field_from_u64(isar0, ID_AA64ISAR0_EL1_SHA2);
    if (sha2 >= 1)
        caps |= HWCAP_SHA2;
    if (sha2 >= 2)
        caps |= HWCAP_SHA512;
    if (field_from_u64(isar0, ID_AA64ISAR0_EL1_CRC32) >= 1)
        caps |= HWCAP_CRC32;
    if (field_from_u64(isar0, ID_AA64ISAR0_EL1_ATOMIC) >= ID_AA64ISAR0_EL1_ATOMIC_IMPLEMENTED)
        caps |= HWCAP_ATOMICS;
    if (field_from_u64(isar0, ID_AA64ISAR0_EL1_RDM) >= 1)
        caps |= HWCAP_ASIMDRDM;
    if (field_from_u64(isar0, ID_AA64ISAR0_EL1_SHA3) >= 1)
        caps |= HWCAP_SHA3;
    if (field_from_u64(isar0, ID_AA64ISAR0_EL1_SM3) >= 1)
        caps |= HWCAP_SM3;
    if (field_from_u64(isar0, ID_AA64ISAR0_EL1_SM4) >= 1)
        caps |= HWCAP_SM4;
    if (field_from_u64(isar0, ID_AA64ISAR0_EL1_DP) >= 1)
        caps |= HWCAP_ASIMDDP;
    if (field_from_u64(isar0, ID_AA64ISAR0_EL1_FHM) >= 1)
        caps |= HWCAP_ASIMDFHM;
    if (field_from_u64(isar0, ID_AA64ISAR0_EL1_TS) >= 1)
        caps |= HWCAP_FLAGM;

    /* ID_AA64ISAR1_EL1 */
    if (field_from_u64(isar1, ID_AA64ISAR1_EL1_DPB) >= 1)
        caps |= HWCAP_DCPOP;
    if (field_from_u64(isar1, ID_AA64ISAR1_EL1_JSCVT) >= 1)
        caps |= HWCAP_JSCVT;
    if (field_from_u64(isar1, ID_AA64ISAR1_EL1_FCMA) >= 1)
        caps |= HWCAP_FCMA;
    u64 lrcpc = field_from_u64(isar1, ID_AA64ISAR1_EL1_LRCPC);
    if (lrcpc >= 1)
        caps |= HWCAP_LRCPC;
    if (lrcpc >= 2)
        caps |= HWCAP_ILRCPC;
    if (field_from_u64(isar1, ID_AA64ISAR1_EL1_SB) >= 1)
        caps |= HWCAP_SB;

    return caps;
}
