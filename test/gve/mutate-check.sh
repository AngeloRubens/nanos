#!/bin/sh
# Mutation testing for the gve harness: prove the suite is not vacuous
# (no assertTrue(true)) by injecting realistic bugs into the DRIVER and
# confirming the suite catches each one.
#
# For every mutation: apply it, force a clean rebuild, run `make run`, then
# revert.  A mutation is CAUGHT if the suite fails (the harness exits non-zero
# — a failed assertion, or the mutation not even compiling).  A mutation that
# SURVIVES (suite still reports 0 failures) means the corresponding behaviour
# is not actually verified — the script then exits non-zero.
#
# Mutations span distinct subsystems so a survivor pinpoints which area is
# under-checked.  Run from test/gve:  ./mutate-check.sh
set -u
ROOT=../..
GVE=src/gcp/gve

# Refuse to run with uncommitted driver changes (we revert via git checkout).
if ! git -C "$ROOT" diff --quiet -- "$GVE"; then
    echo "ERROR: $GVE has uncommitted changes; commit or stash first." >&2
    exit 2
fi
revert() { git -C "$ROOT" checkout -- "$GVE" >/dev/null 2>&1; }
trap revert EXIT INT TERM

# label | file | sed expression | scenario expected to fail
MUTATIONS="
DQO doorbell endianness (LE->BE)|gve_dqo.c|s/tx->head & tx->mask);/htobe32(tx->head \\& tx->mask));/g|scenario_doorbell_format
ring-size canonical restore removed|gve_adminq.c|s/adapter->tx_desc_cnt      = adapter->tx_desc_cnt_dev;/adapter->tx_desc_cnt      = adapter->tx_desc_cnt;/|scenario_ring_size_restore
report_event set on every packet|gve_dqo.c|s/>= GVE_TX_MIN_RE_INTERVAL) {/>= 0u) {/|scenario_dqo_report_event
per-CPU TX dispatch hardcoded to queue 0|gve_dqo.c|s#current_cpu()->id % adapter->num_queues#0#|scenario_multiqueue_dispatch
DQO completion gen-bit check inverted|gve_dqo.c|s/if (gen != tx->expected_gen)/if (gen == tx->expected_gen)/|scenario_tx_inorder
RSS indirection table not round-robin|gve_adminq.c|s/htobe32(i % adapter->num_queues)/htobe32(0)/|scenario_multiqueue_rss
TX cleanup budget removed|gve_dqo.c|s/int budget = GVE_TX_CLEAN_BUDGET;/int budget = 100000;/|scenario_tx_cleanup_budget
"

rm -f /tmp/gve_mutate_fail
echo "===== gve mutation testing (each mutation MUST be caught) ====="
echo "$MUTATIONS" | while IFS='|' read -r label file sed_expr scenario; do
    [ -z "$label" ] && continue
    sed -i "$sed_expr" "$ROOT/$GVE/$file"
    make clean >/dev/null 2>&1
    out=$(make run 2>&1); rc=$?
    revert
    fails=$(echo "$out" | grep -c "  FAIL ")
    if [ "$rc" -ne 0 ]; then
        # confirm the targeted scenario is among the failures (when it built)
        if echo "$out" | grep -q "FAIL $scenario:" || ! echo "$out" | grep -q "checks,"; then
            printf "  CAUGHT   %-42s (%s fail)\n" "$label" "$fails"
        else
            printf "  CAUGHT*  %-42s (%s fail, but not %s)\n" "$label" "$fails" "$scenario"
        fi
    else
        printf "  SURVIVED %-42s  <-- VACUOUS TEST\n" "$label"
        echo "VACUOUS" > /tmp/gve_mutate_fail
    fi
done

# the while ran in a subshell (pipe); detect survivors via the marker file
if [ -f /tmp/gve_mutate_fail ]; then
    rm -f /tmp/gve_mutate_fail
    echo "===== FAIL: at least one mutation survived (a test is vacuous) ====="
    exit 1
fi
echo "===== OK: every mutation was caught — the suite is non-vacuous ====="
