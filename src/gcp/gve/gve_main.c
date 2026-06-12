/* gve_main.c — gVNIC driver: PCI probe, device lifecycle, interrupts
 *
 * Google Virtual Ethernet (gVNIC) driver — multi-queue port.
 *
 * Improvements vs src/drivers/gve.c:
 *   1. Integer overflow fix: "head < tx->qpl_head" wrap replaced with
 *      qpl_used accounting (bytes in-flight).
 *   2. GVE_TX_PAD = DEFAULT_CACHELINE_SIZE (symbolic, not hardcoded 64).
 *   3. TX completion watchdog with deferred reset (GVE_TX_WATCHDOG_MS).
 *   4. Multi-queue: up to GVE_MAX_IO_QUEUES TX/RX pairs.
 *   5. GVE_REG_MAX_TX/RX_QUEUES actually read and used.
 *   6. GQI-RDA format negotiation (no QPL bounce copy).
 *   7. DQO format (Andromeda 2.x): generation-bit completion polling,
 *      separate TX-completion and RX-buffer/completion rings; both DQO-RDA
 *      (direct addressing) and DQO-QPL (registered bounce pages).
 *   8. Multi-buffer RX packet reassembly (pbuf_cat to end-of-packet).
 *   9. Device-requested reset (DEVICE_STATUS bit 1, e.g. live migration).
 *  10. RSS configuration (random Toeplitz key + round-robin indirection
 *      table) when the device offers the RSS_CONFIG option.
 *
 * Checksums are computed in software by lwIP (no HW offload), same model
 * as the ENA driver: gVNIC offloads only L4, not the IPv4 header checksum.
 *
 * Nanos-specific simplifications vs Linux driver:
 *   - No DMA map/unmap (single address space, identity mapped).
 *   - No NAPI, no page pool, no RSS tables.
 *   - No scatter-gather DMA: pbuf payload is always physically contiguous.
 *   - TSO not implemented (lwIP always segments TCP at MSS).
 */

#include "gve_priv.h"

/* ------------------------------------------------------------------ */
/* Link status                                                          */
/* ------------------------------------------------------------------ */

closure_func_basic(thunk, void, gve_link_up_task)
{
    gve adapter = struct_from_closure(gve, link_up_task);
    gve_debug("link up task");
    if (!(adapter->flags & (1ULL << GVE_FLAG_ONGOING_RESET)))
        netif_set_link_up(&adapter->ndev.n);
}

closure_func_basic(thunk, void, gve_link_down_task)
{
    gve adapter = struct_from_closure(gve, link_down_task);
    gve_debug("link down task");
    if (!(adapter->flags & (1ULL << GVE_FLAG_ONGOING_RESET)))
        netif_set_link_down(&adapter->ndev.n);
}

closure_func_basic(thunk, void, gve_mgmt_irq)
{
    gve adapter = struct_from_field(closure_self(), gve, mgmt_irq_handler);
    u32 status = pci_bar_read_4(&adapter->reg_bar, GVE_REG_DEVICE_STATUS);
    gve_debug("mgmt irq, status 0x%x", status);
    if (status & GVE_DEVICE_STATUS_RESET) {
        /* Device-requested reset (e.g. live migration); mirrors the official
         * driver's gve_handle_status.  Link state is re-read after reset.
         * Ignored until the adapter is fully initialized: a reset task
         * racing gve_init's queue creation would tear down half-created
         * queues (and after a failed reset the adapter stays down). */
        if (adapter->flags & (1ULL << GVE_FLAG_DEVICE_RUNNING)) {
            msg_err("GVE: device requested reset");
            gve_trigger_reset(adapter);
        }
        return;
    }
    if (status & GVE_DEVICE_STATUS_LINK_STATUS)
        async_apply_bh((thunk)&adapter->link_up_task);
    else
        async_apply_bh((thunk)&adapter->link_down_task);
}

/* ------------------------------------------------------------------ */
/* RX IRQ dispatch (thin trampolines → service closures in datapath)  */
/* ------------------------------------------------------------------ */

closure_func_basic(thunk, void, gve_rx_irq)
{
    gve_debug("GQI RX irq");
    gve_rx_queue rx = struct_from_field(closure_self(),
                                        gve_rx_queue, irq_handler);
    rx->first_interrupt = true;
    async_apply_bh((thunk)&rx->service);
    /* interrupts remain masked until ack in gve_rx_service */
}

closure_func_basic(thunk, void, gve_rx_dqo_irq)
{
    gve_debug("DQO RX irq");
    gve_rx_dqo_queue rx = struct_from_field(closure_self(),
                                            gve_rx_dqo_queue, irq_handler);
    rx->first_interrupt = true;
    async_apply_bh((thunk)&rx->service);
    /* interrupt stays disarmed until the ITR re-arm in gve_rx_dqo_service */
}

/*
 * gve_turnup_rx_dqo — arm the DQO RX interrupts, with the netif up.
 *
 * The analogue of Linux gve_turnup: write the ITR doorbell (enable + 2 us
 * granularity throttling interval) for every RX queue, then kick the
 * service once per queue to pick up any completions that arrived before
 * arming (Linux does the same with a one-off napi_schedule after a memory
 * barrier).  Called at the end of probe and at the end of a successful
 * reset — never at queue creation, where an interrupt taken before the
 * netif is up would be serviced on a secondary CPU and lost.
 */
static void gve_turnup_rx_dqo(gve adapter)
{
    if (!adapter->dqo)
        return;
    for (u32 i = 0; i < adapter->num_queues; i++) {
        gve_rx_dqo_queue rx = &adapter->rx_dqo[i];
        pci_bar_write_4(&adapter->db_bar,
                        be32toh(*rx->irq_db_index) * sizeof(u32),
                        GVE_DQO_ITR_ENABLE |
                        ((GVE_DQO_RX_IRQ_THROTTLE_US / 2) <<
                         GVE_DQO_ITR_INTERVAL_SHIFT));
        memory_barrier();
        /* Called from probe or from the reset BH, never from interrupt
         * context: runqueue. */
        async_apply((thunk)&rx->service);
    }
}

/* ------------------------------------------------------------------ */
/* TX completion watchdog and deferred reset                            */
/* ------------------------------------------------------------------ */

/*
 * gve_reset — deferred reset handler, runs in bottom-half context.
 *
 * Tears down all queues and device resources, then re-configures them.
 * If any step fails, the adapter remains in error state and the operator
 * must restart the image.
 */
closure_func_basic(thunk, void, gve_reset)
{
    gve adapter = struct_from_field(closure_self(), gve, reset_handler);
    struct netif *net_if = &adapter->ndev.n;
    rprintf("GVE: starting adapter reset\n");

    spin_lock(&adapter->global_lock);
    atomic_test_and_set_bit(&adapter->flags, GVE_FLAG_ONGOING_RESET);
    atomic_clear_bit(&adapter->flags, GVE_FLAG_DEVICE_RUNNING);

    /* Clear NETIF_FLAG_UP so that concurrent gve_rx_service / gve_tx_start_xmit
     * BHs on other CPUs exit before touching ring memory.  The flag is restored
     * in the success path below; on failure the adapter is left down. */
    net_if->flags &= ~NETIF_FLAG_UP;
    adapter->adminq_running = true;

    gve_teardown_queues(adapter);
    gve_free_device_resources(adapter);

    if (!gve_cfg_device_resources(adapter)) {
        msg_err("GVE: reset: failed to reconfigure device resources");
        goto done;
    }
    if (adapter->dqo && !gve_get_ptype_map_dqo(adapter)) {
        msg_err("GVE: reset: failed to get DQO ptype map");
        gve_free_device_resources(adapter);
        goto done;
    }
    if (!gve_setup_queues(adapter)) {
        msg_err("GVE: reset: failed to recreate queues");
        gve_free_device_resources(adapter);
        goto done;
    }
    /* Re-apply RSS after the queues exist (fresh random key); best-effort
     * as at init. */
    if (!gve_configure_rss(adapter))
        msg_err("GVE: reset: RSS configuration failed, using device default");

    atomic_test_and_set_bit(&adapter->flags, GVE_FLAG_DEVICE_RUNNING);
    net_if->flags |= NETIF_FLAG_UP;
    gve_turnup_rx_dqo(adapter);
    rprintf("GVE: adapter reset complete\n");
    {
        u32 status = pci_bar_read_4(&adapter->reg_bar, GVE_REG_DEVICE_STATUS);
        if (status & GVE_DEVICE_STATUS_LINK_STATUS)
            async_apply((thunk)&adapter->link_up_task);
        else
            async_apply((thunk)&adapter->link_down_task);
    }
    atomic_clear_bit(&adapter->flags, GVE_FLAG_ONGOING_RESET);
    atomic_clear_bit(&adapter->flags, GVE_FLAG_RESETTING);
    spin_unlock(&adapter->global_lock);
    return;

  done:
    /* Failure: leave RESETTING and ONGOING_RESET set so the watchdog, the
     * RX service interrupt re-arm and further reset triggers all stay
     * inhibited — device resources are freed at this point and the adapter
     * is dead until the image is restarted. */
    spin_unlock(&adapter->global_lock);
}

closure_func_basic(timer_handler, void, gve_watchdog_task,
                   u64 expiry, u64 overruns)
{
    if (overruns == timer_disabled)
        return;

    gve adapter = struct_from_closure(gve, watchdog_task);
    read_barrier();
    if (!(adapter->flags & (1ULL << GVE_FLAG_DEVICE_RUNNING)))
        return;
    if (adapter->flags & ((1ULL << GVE_FLAG_RESETTING) |
                          (1ULL << GVE_FLAG_ONGOING_RESET)))
        return;

    timestamp deadline = milliseconds(GVE_TX_WATCHDOG_MS);
    timestamp now_ts   = now(CLOCK_ID_MONOTONIC);
    boolean   any_stuck = false;

    if (adapter->dqo) {
        /* Rotated loop: TX stuck detection + RX interrupt-miss, GVE_TX_MONITORED_QUEUES
         * per tick (ENA check_missing_comp_in_tx_queue + check_for_rx_interrupt_queue).
         * Full round-trip = ceil(num_queues / GVE_TX_MONITORED_QUEUES) ticks. */
        u32 rot_budget = GVE_TX_MONITORED_QUEUES;
        u32 ri;
        for (ri = adapter->next_monitored_tx_qid;
             ri < adapter->num_queues; ri++) {
            /* Re-check reset flags each iteration (same as the GQI loop):
             * tx_timestamps/miss_times/compl_ring are freed during teardown
             * after ONGOING_RESET is set. */
            read_barrier();
            if (adapter->flags & ((1ULL << GVE_FLAG_RESETTING) |
                                  (1ULL << GVE_FLAG_ONGOING_RESET)))
                return;

            gve_tx_dqo_queue tx = &adapter->tx_dqo[ri];
            if (!tx->stuck) {
                u32 missed_tx = 0;
                /* tx_timestamps is per-tag (one stamp per packet, set at
                 * submit and cleared on retire): scan the whole tag space. */
                for (u32 s = 0; s <= tx->mask; s++) {
                    timestamp ts = tx->tx_timestamps[s];
                    if (!ts)
                        continue;  /* tag not in flight */
                    if (now_ts - ts > deadline) {
                        tx->tx_stats.missing_tx_comp++;
                        missed_tx++;
                    }
                }
                if (missed_tx > GVE_TX_STUCK_THRESHOLD) {
                    msg_err("GVE: DQO TX queue %d: %u stuck completions "
                            "(threshold %u), scheduling reset",
                            ri, missed_tx, GVE_TX_STUCK_THRESHOLD);
                    tx->stuck = true;
                    any_stuck = true;
                }
                /* Miss-reinject timeout: DQO MISS completions that never
                 * get a matching REINJECT within GVE_TX_WATCHDOG_MS indicate
                 * a device stall.  Checked here (watchdog) not in the hot
                 * path (same split as ENA check_missing_comp_in_tx_queue);
                 * the full miss_times scan costs at most mask+1 reads per
                 * monitored queue per tick. */
                for (u32 mi = 0; mi <= tx->mask; mi++) {
                    if (!tx->miss_times[mi])
                        continue;
                    if (now_ts - tx->miss_times[mi] <= deadline)
                        continue;
                    msg_err("GVE: DQO TX queue %d tag %u: miss not "
                            "reinjected after %d ms, scheduling reset",
                            ri, mi, GVE_TX_WATCHDOG_MS);
                    if (tx->pending[mi]) {
                        pbuf_free(tx->pending[mi]);
                        tx->pending[mi] = NULL;
                    }
                    tx->miss_times[mi] = 0;
                    tx->stuck = true;
                    any_stuck = true;
                    break;
                }
            }
            gve_rx_dqo_queue rx = &adapter->rx_dqo[ri];
            if (rx->first_interrupt) {
                rx->first_interrupt = false;
                rx->no_interrupt_event_cnt = 0;
            } else {
                /* CQ-empty guard (mirrors ena_com_cq_empty): if gen bit does
                 * not match expected, no pending completion — no miss possible. */
                u32 slot = rx->compl_head & rx->mask;
                u8 gen = !!(rx->compl_ring[slot].pkt_len_gen & GVE_DQO_RX_GEN);
                if (gen == rx->expected_gen) {  /* CQ not empty */
                    if (++rx->no_interrupt_event_cnt >
                            GVE_MAX_NO_INTERRUPT_ITERATIONS) {
                        msg_err("GVE: DQO RX queue %d: no interrupt after %d "
                                "watchdog ticks, scheduling reset", ri,
                                GVE_MAX_NO_INTERRUPT_ITERATIONS);
                        gve_trigger_reset(adapter);
                    }
                }
            }
            if (--rot_budget == 0) { ri++; break; }
        }
        adapter->next_monitored_tx_qid = ri % adapter->num_queues;

        /* Wakeup: stopped DQO TX queues with pending packets that have not
         * been restarted by new incoming traffic. All queues every tick.
         * Watchdog runs as a BH, not in interrupt context, so these kicks
         * go on the runqueue (as in the ENA watchdog). */
        for (u32 i = 0; i < adapter->num_queues; i++) {
            gve_tx_dqo_queue tx = &adapter->tx_dqo[i];
            if (tx->stuck || tx->running)
                continue;
            u32 free = (tx->mask + 1) - (tx->head - tx->desc_tail);
            if (free >= GVE_TX_RESUME_THRESH)
                async_apply((thunk)&tx->enqueue_task);
        }
        /* Empty-ring detection: if fill posted nothing for several ticks,
         * the device has no buffers and will send no interrupts — deadlock.
         * Force a service run to attempt refill. All queues every tick. */
        for (u32 i = 0; i < adapter->num_queues; i++) {
            gve_rx_dqo_queue rx = &adapter->rx_dqo[i];
            if (rx->empty_rx_queue > 2) {
                rx->rx_stats.empty_rx_ring++;
                rx->empty_rx_queue = 0;
                async_apply((thunk)&rx->service);
            }
        }
    } else {
        /* Rotated loop: TX stuck detection + RX interrupt-miss for GQI,
         * GVE_TX_MONITORED_QUEUES per tick (same ENA pattern as DQO path).
         * Re-check reset flags each iteration: event_counters and q_res are
         * freed during teardown after ONGOING_RESET is set. */
        u32 rot_budget = GVE_TX_MONITORED_QUEUES;
        u32 ri;
        for (ri = adapter->next_monitored_tx_qid;
             ri < adapter->num_queues; ri++) {
            read_barrier();
            if (adapter->flags & ((1ULL << GVE_FLAG_RESETTING) |
                                  (1ULL << GVE_FLAG_ONGOING_RESET)))
                return;

            gve_tx_queue tx = &adapter->tx[ri];
            if (!tx->stuck) {
                u32 missed_tx = 0;
                for (u32 s = tx->tail; s != tx->head; s++) {
                    timestamp ts = tx->tx_timestamps[s & tx->mask];
                    if (!ts)
                        continue;  /* seg descriptor or already retired */
                    if (now_ts - ts > deadline) {
                        tx->tx_stats.missing_tx_comp++;
                        missed_tx++;
                    }
                }
                if (missed_tx > GVE_TX_STUCK_THRESHOLD) {
                    msg_err("GVE: TX queue %d: %u stuck completions "
                            "(threshold %u), scheduling reset",
                            ri, missed_tx, GVE_TX_STUCK_THRESHOLD);
                    tx->stuck = true;
                    any_stuck = true;
                }
            }
            gve_rx_queue rx = &adapter->rx[ri];
            if (rx->first_interrupt) {
                rx->first_interrupt = false;
                rx->no_interrupt_event_cnt = 0;
            } else {
                /* CQ-empty guard: if the event counter has not advanced past
                 * rx->tail, the device has no pending completions — no miss. */
                u32 hw_tail = be32toh(
                    adapter->event_counters[rx->event_counter_idx]);
                if (hw_tail != rx->tail) {  /* CQ not empty */
                    if (++rx->no_interrupt_event_cnt >
                            GVE_MAX_NO_INTERRUPT_ITERATIONS) {
                        msg_err("GVE: RX queue %d: no interrupt after %d "
                                "watchdog ticks, scheduling reset", ri,
                                GVE_MAX_NO_INTERRUPT_ITERATIONS);
                        gve_trigger_reset(adapter);
                    }
                }
            }
            if (--rot_budget == 0) { ri++; break; }
        }
        adapter->next_monitored_tx_qid = ri % adapter->num_queues;

        /* Wakeup: stopped GQI TX queues with pending packets. All queues
         * every tick.  Runqueue, not bhqueue: see the DQO loop above. */
        for (u32 i = 0; i < adapter->num_queues; i++) {
            gve_tx_queue tx = &adapter->tx[i];
            if (tx->stuck || tx->running)
                continue;
            u32 hw_tail = be32toh(
                adapter->event_counters[tx->event_counter_idx]);
            u32 free = adapter->tx_desc_cnt - (tx->head - hw_tail);
            if (free >= GVE_TX_RESUME_THRESH)
                async_apply((thunk)&tx->enqueue_task);
        }
        /* Empty-ring detection. All queues every tick. */
        for (u32 i = 0; i < adapter->num_queues; i++) {
            gve_rx_queue rx = &adapter->rx[i];
            if (rx->empty_rx_queue > 2) {
                rx->rx_stats.empty_rx_ring++;
                rx->empty_rx_queue = 0;
                async_apply((thunk)&rx->service);
            }
        }
    }

    if (any_stuck) {
        adapter->dev_stats.wd_expired++;
        gve_trigger_reset(adapter);
    }
}

/* ------------------------------------------------------------------ */
/* Interrupt setup / teardown                                          */
/* ------------------------------------------------------------------ */

static boolean gve_init_interrupts(gve adapter)
{
    u32 nq     = adapter->num_queues;
    /* TX ntfy slots (0..nq-1) are allocated but have no IRQ handler:
     * TX completion is event-counter driven, not interrupt driven.
     * RX slots (nq..2nq-1) and mgmt slot (2nq) have handlers. */
    int needed = (int)(2 * nq + 1);

    int msix_avail = pci_enable_msix(adapter->pdev);
    if (msix_avail < needed) {
        msg_err("GVE: need %d MSI-X vectors, only %d available",
                needed, msix_avail);
        return false;
    }

    /* Management IRQ at index GVE_IRQ_DB_MGMT(nq) */
    if (pci_setup_msix(adapter->pdev, GVE_IRQ_DB_MGMT(nq),
                       init_closure_func(&adapter->mgmt_irq_handler,
                                         thunk, gve_mgmt_irq),
                       ss("gve_mgmt")) == INVALID_PHYSICAL)
        goto error;

    /* RX IRQs at index GVE_IRQ_DB_RX(nq, i) — dispatch to DQO or GQI */
    for (u32 i = 0; i < nq; i++) {
        thunk irq_fn;
        if (adapter->dqo)
            irq_fn = init_closure_func(&adapter->rx_dqo[i].irq_handler,
                                       thunk, gve_rx_dqo_irq);
        else
            irq_fn = init_closure_func(&adapter->rx[i].irq_handler,
                                       thunk, gve_rx_irq);
        if (pci_setup_msix(adapter->pdev, GVE_IRQ_DB_RX(nq, i),
                           irq_fn, ss("gve_rx")) == INVALID_PHYSICAL)
            goto err_teardown_rx;
    }
    return true;

  err_teardown_rx:
    for (u32 i = 0; i < nq; i++)
        pci_teardown_msix(adapter->pdev, GVE_IRQ_DB_RX(nq, i));
    pci_teardown_msix(adapter->pdev, GVE_IRQ_DB_MGMT(nq));
  error:
    pci_disable_msix(adapter->pdev);
    return false;
}

static void gve_deinit_interrupts(gve adapter)
{
    u32 nq = adapter->num_queues;
    for (u32 i = 0; i < nq; i++)
        pci_teardown_msix(adapter->pdev, GVE_IRQ_DB_RX(nq, i));
    pci_teardown_msix(adapter->pdev, GVE_IRQ_DB_MGMT(nq));
    pci_disable_msix(adapter->pdev);
}

/* ------------------------------------------------------------------ */
/* Adapter init                                                         */
/* ------------------------------------------------------------------ */

static boolean gve_init(gve adapter, tuple config)
{
    adapter->adminq = allocate(adapter->contiguous, PAGESIZE);
    if (adapter->adminq == INVALID_ADDRESS)
        return false;

    pci_bar_init(adapter->pdev, &adapter->reg_bar, GVE_REGISTER_BAR,
                 0, -1);
    pci_bar_init(adapter->pdev, &adapter->db_bar, GVE_DOORBELL_BAR,
                 0, -1);
    pci_enable_io_and_memory(adapter->pdev);

    init_closure_func(&adapter->link_up_task, thunk, gve_link_up_task);
    init_closure_func(&adapter->link_down_task, thunk, gve_link_down_task);
    init_closure_func(&adapter->reset_handler, thunk, gve_reset);
    adapter->flags = 0;
    spin_lock_init(&adapter->global_lock);
    adapter->adminq_head = 0;
    adapter->adminq_mask =
        PAGESIZE / sizeof(struct gve_adminq_command) - 1;
    pci_bar_write_4(&adapter->reg_bar, GVE_REG_ADMINQ_PFN,
                    htobe32(physical_from_virtual(adapter->adminq)
                            >> PAGELOG));
    adapter->adminq_running = true;

    /* Identify the driver to the device before describe (best-effort: older
     * devices that do not support this command simply return an error). */
    if (!gve_verify_driver_compatibility(adapter))
        gve_debug("driver compatibility verification not accepted, continuing");

    if (!gve_describe_device(adapter)) {
        msg_err("GVE: failed to describe device");
        goto err1;
    }

    adapter->num_queues = gve_calc_num_queues(adapter, config);
    rprintf("GVE: using %d TX/RX queue pair(s)\n", adapter->num_queues);

    if (!gve_cfg_device_resources(adapter)) {
        msg_err("GVE: failed to configure device resources");
        goto err1;
    }
    /* DQO requires the driver to fetch the packet-type map before queues. */
    if (adapter->dqo && !gve_get_ptype_map_dqo(adapter)) {
        msg_err("GVE: failed to get DQO ptype map");
        goto err2;
    }
    if (!gve_init_interrupts(adapter)) {
        msg_err("GVE: failed to initialize interrupts");
        goto err2;
    }
    if (!gve_setup_queues(adapter)) {
        msg_err("GVE: failed to set up TX/RX queues");
        goto err3;
    }
    /* RSS references RX queue ids in the indirection table, so it can only
     * be configured once the queues exist (the official driver issues
     * CONFIGURE_RSS via ethtool, always with the interface up).
     * Best-effort: on failure RX steering stays on the device default. */
    if (!gve_configure_rss(adapter))
        msg_err("GVE: RSS configuration failed, using device default");
    atomic_test_and_set_bit(&adapter->flags, GVE_FLAG_DEVICE_RUNNING);

    /* Start TX completion watchdog (fires every GVE_WATCHDOG_INTERVAL_MS). */
    timestamp watchdog_interval = milliseconds(GVE_WATCHDOG_INTERVAL_MS);
    init_timer(&adapter->watchdog_timer);
    register_timer(kernel_timers, &adapter->watchdog_timer,
                   CLOCK_ID_MONOTONIC, watchdog_interval, false,
                   watchdog_interval,
                   init_closure_func(&adapter->watchdog_task,
                                     timer_handler, gve_watchdog_task));
    return true;

  err3:
    gve_deinit_interrupts(adapter);
  err2:
    gve_free_device_resources(adapter);
  err1:
    pci_bar_deinit(&adapter->db_bar);
    pci_bar_deinit(&adapter->reg_bar);
    deallocate(adapter->contiguous, adapter->adminq, PAGESIZE);
    return false;
}

/* ------------------------------------------------------------------ */
/* lwIP netif init callback                                             */
/* ------------------------------------------------------------------ */

static err_t gve_if_init(struct netif *netif)
{
    gve adapter = netif->state;
    netif->name[0] = 'e';
    netif->name[1] = 'n';
    netif->mtu     = adapter->mtu;
    netif->flags   = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP |
                     NETIF_FLAG_UP;
    gve_setup_linkoutput(adapter, netif);
    netif->hwaddr_len = ETH_HWADDR_LEN;

    /* Checksums (IP/TCP/UDP) are generated and verified in software by
     * lwIP: gVNIC offloads only L4, never the IPv4 header checksum, so
     * the safe model is full software checksumming, same as the ENA driver. */

    return ERR_OK;
}

/* ------------------------------------------------------------------ */
/* PCI probe                                                            */
/* ------------------------------------------------------------------ */

closure_function(3, 1, boolean, gve_probe,
        heap, general, heap, contiguous, tuple, config,
        pci_dev d)
{
    if ((pci_get_vendor(d) != PCI_VENDOR_ID_GOOGLE) ||
            (pci_get_device(d) != PCI_DEV_ID_GVNIC))
        return false;
    gve_debug("probing device");
    heap h = bound(general);
    /* allocate_zero: several adapter fields are only conditionally assigned
     * (format booleans in gve_describe_device, rss_supported, watchdog
     * cursor, dev_stats) and rely on a zeroed start. */
    gve adapter = allocate_zero(h, sizeof(struct gve));
    if (adapter == INVALID_ADDRESS)
        return false;
    adapter->general   = h;
    adapter->contiguous = bound(contiguous);
    adapter->pdev      = d;
    if (gve_init(adapter, bound(config))) {
        gve_debug("registering network interface");
        netif_dev_init(&adapter->ndev);
        netif_add(&adapter->ndev.n, 0, 0, 0, adapter,
                  gve_if_init, ethernet_input);
        u32 status = pci_bar_read_4(&adapter->reg_bar, GVE_REG_DEVICE_STATUS);
        if (status & GVE_DEVICE_STATUS_LINK_STATUS)
            netif_set_link_up(&adapter->ndev.n);
        else
            netif_set_link_down(&adapter->ndev.n);
        gve_turnup_rx_dqo(adapter);
        /* Catch a reset the device requested while probe was in progress
         * (the mgmt IRQ ignores it until DEVICE_RUNNING; the official
         * driver re-checks at end of probe for the same reason). */
        if (status & GVE_DEVICE_STATUS_RESET) {
            msg_err("GVE: device requested reset during probe");
            gve_trigger_reset(adapter);
        }
        return true;
    }
    deallocate(h, adapter, sizeof(struct gve));
    return false;
}

void init_gve(kernel_heaps kh)
{
    heap h = heap_locked(kh);
    tuple config = get(get_root_tuple(), sym(gve));
    pci_probe probe = closure(h, gve_probe, h,
                              (heap)heap_linear_backed(kh), config);
    assert(probe != INVALID_ADDRESS);
    register_pci_driver(probe, 0);
}
