#!/bin/sh
# Build and run the memory-ordering litmus matrix for the current arch.
#
# What it proves (per litmus.c): the barriers the gve driver uses are not
# cargo-cult — each one's absence is detectable on the architecture that needs
# it, and x86 behaves as the assumed TSO control.
#
#   full build (all barriers)            : never violates           (hard gate)
#   SB / no mfence|dmb-sy (store-load)    : violates on x86 AND arm64
#   MP / no lfence|dmb-ld (read-order)    : violates ONLY on arm64;
#                                           x86 (TSO) stays clean    (hard gate)
#
# The "must-violate" checks are probabilistic (rare reorder windows), so they
# retry; the "must-stay-clean" checks are deterministic.
#
# NOT a driver regression gate: litmus.c is standalone, so it validates the
# ordering CONTRACT and the arch, not the driver's barrier placement (that is
# the static presence check + review).  Needs >= 2 online CPUs (else SKIP).
set -u
arch=$(uname -m)
iters=${ITERS:-5000000}
cc="${CC:-cc} -O2 -pthread"

$cc litmus.c                -o litmus_full  || exit 2
$cc -DLITMUS_NO_MB  litmus.c -o litmus_nomb  || exit 2
$cc -DLITMUS_NO_RMB litmus.c -o litmus_normb || exit 2

run() { ./"$1" "$iters"; echo $?; }   # echoes exit code: 0 clean, 1 violated, 77 skip

# full build: a single run; 77 => skip the whole matrix.
rc=$(run litmus_full | tail -1)
if [ "$rc" = 77 ]; then echo "SKIP: litmus needs >= 2 online CPUs"; exit 0; fi
if [ "$rc" != 0 ]; then echo "FAIL: full build (all barriers) reported a violation"; exit 1; fi
echo "OK: full build clean"

expect_violation() {   # $1 binary, $2 label
    for _ in 1 2 3 4 5 6; do
        [ "$(run "$1" | tail -1)" = 1 ] && { echo "OK: $2 — reordering detected"; return 0; }
    done
    echo "FAIL: $2 — no violation observed (expected one)"; return 1
}
expect_clean() {       # $1 binary, $2 label
    [ "$(run "$1" | tail -1)" = 0 ] && { echo "OK: $2 — clean (control)"; return 0; }
    echo "FAIL: $2 — violated unexpectedly"; return 1
}

# SB store-load: reorders on every arch.
expect_violation litmus_nomb "SB store-load (no full barrier)" || exit 1

# MP read-order: weak arch violates; x86 is the TSO control.
case "$arch" in
    aarch64|arm64) expect_violation litmus_normb "MP read-order (no dmb ld)" || exit 1 ;;
    x86_64|amd64)  expect_clean     litmus_normb "MP read-order (no lfence) [TSO]" || exit 1 ;;
    *) echo "FAIL: unknown arch $arch"; exit 1 ;;
esac

echo "litmus matrix OK on $arch"
