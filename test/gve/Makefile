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
SRCS := harness.c $(ROOT)/src/gcp/gve/gve_datapath.c $(RUNTIME_SRCS)

gve_harness: closure_templates.h $(SRCS)
	gcc $(CFLAGS) $(SRCS) -lpthread -o $@

closure_templates.h: $(CONTGEN)
	$(CONTGEN) 10 10 > $@

$(CONTGEN):
	$(MAKE) -C $(ROOT)/tools contgen

.PHONY: run clean
run: gve_harness
	./gve_harness

clean:
	rm -f gve_harness closure_templates.h
