# Standalone host build for the gve DQO datapath harness.
#
# Compiles the real src/gcp/gve/gve_dqo.c (via harness.c) against the nanos
# runtime + the unix_process host simulator, with shim headers for the
# device/kernel glue.  Not wired into the kernel build; run with `make run`.

ROOT := ../..
CONTGEN := $(ROOT)/output/tools/bin/contgen

INC := -Ishim -I. \
       -I$(ROOT)/src/gcp/gve -I$(ROOT)/src/x86_64 -I$(ROOT)/src \
       -I$(ROOT)/src/runtime -I$(ROOT)/src/unix -I$(ROOT)/src/unix_process

CFLAGS := -O2 -std=gnu11 $(INC)

RUNTIME := bitmap buffer crypto/chacha extra_prints format \
	heap/mem_debug heap/freelist heap/id heap/mcache heap/page \
	heap/reserve heap/objcache json management memops merge pqueue queue \
	random range rbtree ringbuf runtime_init sg sha256 string symbol \
	table timer tuple tuple_parser util vector
RUNTIME_SRCS := $(addprefix $(ROOT)/src/runtime/,$(addsuffix .c,$(RUNTIME))) \
	$(ROOT)/src/unix_process/unix_process_runtime.c

# gve_datapath.c (GQI) is a separate TU: it includes gve_priv.h, which (no
# include guard) cannot be pulled twice, so it is NOT #included into
# harness.c the way gve_dqo.c is.  harness.c calls its non-static entry
# points (gve_setup_linkoutput, gve_rx_fill/init, gve_tx_init_gqi).
SRCS := harness.c $(ROOT)/src/gcp/gve/gve_datapath.c \
	$(ROOT)/src/gcp/gve/gve_adminq.c $(ROOT)/src/gcp/gve/gve_main.c \
	$(RUNTIME_SRCS)

# Header dependencies: the shim headers (incl. shim/lock.h) and the driver's
# gve_priv.h/gve.h are #included, not in SRCS, so list them so edits trigger
# a rebuild.
DEPS := $(wildcard shim/*.h) $(wildcard $(ROOT)/src/gcp/gve/*.h)

gve_harness: closure_templates.h $(SRCS) $(DEPS)
	gcc $(CFLAGS) $(SRCS) -lpthread -o $@

# Sanitizer build: AddressSanitizer + UBSan, an independent check of the
# memory/UB behaviour the functional assertions do not cover (use-after-free,
# overflow, bad frees across the ref-ownership and fuzzer paths).
# -fno-sanitize=alignment: the only over-aligned type is struct gve_irq_db
# (aligned(64), "device expects 64-byte alignment").  On hardware it is
# allocated from the page-backed contiguous heap (page-aligned); the host
# harness backs every heap with the general mcache, which is not 64-byte
# aligned, so UBSan's alignment check is a harness artifact, not a driver bug.
gve_harness_san: closure_templates.h $(SRCS) $(DEPS)
	gcc -O1 -g -std=gnu11 -fsanitize=address,undefined -fno-sanitize=alignment \
		-fno-omit-frame-pointer $(INC) $(SRCS) -lpthread -o $@

# Concurrency build: real atomic spinlocks (shim/lock.h under GVE_HARNESS_SMP)
# + ThreadSanitizer, driving the driver's locked state from multiple threads.
# Confirms ring_mtx / service_lock actually serialise it; the lock-free
# runtime queue's internals are suppressed (see tsan.supp).
gve_harness_tsan: closure_templates.h $(SRCS) $(DEPS)
	gcc -O1 -g -std=gnu11 -DGVE_HARNESS_SMP -fsanitize=thread \
		-fno-omit-frame-pointer $(INC) $(SRCS) -lpthread -o $@

closure_templates.h: $(CONTGEN)
	$(CONTGEN) 10 10 > $@

$(CONTGEN):
	$(MAKE) -C $(ROOT)/tools contgen

.PHONY: run sanitize tsan litmus clean
run: gve_harness
	./gve_harness

sanitize: gve_harness_san
	ASAN_OPTIONS=detect_leaks=0 ./gve_harness_san

tsan: gve_harness_tsan
	setarch $$(uname -m) -R env TSAN_OPTIONS=suppressions=tsan.supp ./gve_harness_tsan

# Memory-ordering litmus matrix (standalone, no runtime deps): proves the
# driver's barriers are necessary on the arch that needs them, with x86 as the
# TSO control.  Same matrix on x86 and arm64; see litmus.c / litmus-check.sh.
litmus: litmus.c litmus-check.sh
	./litmus-check.sh

clean:
	rm -f gve_harness gve_harness_san gve_harness_tsan \
		litmus_full litmus_nomb litmus_normb closure_templates.h
