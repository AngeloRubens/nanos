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
 *      separate TX-completion and RX-buffer/completion rings.
 *   8. TX checksum offload (GQI and DQO): CHECKSUM_PARTIAL model,
 *      pseudo-header seeded by driver, completed by device.
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
    netif_set_link_up(&adapter->ndev.n);
}

closure_func_basic(thunk, void, gve_link_down_task)
{
    gve adapter = struct_from_closure(gve, link_down_task);
    gve_debug("link down task");
    netif_set_link_down(&adapter->ndev.n);
}

closure_func_basic(thunk, void, gve_mgmt_irq)
{
    gve adapter = struct_from_field(closure_self(), gve, mgmt_irq_handler);
    u32 status = pci_bar_read_4(&adapter->reg_bar, GVE_REG_DEVICE_STATUS);
    gve_debug("mgmt irq, status 0x%x", status);
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
    rprintf("GVE: starting adapter reset\n");

    spin_lock(&adapter->global_lock);
    atomic_test_and_set_bit(&adapter->flags, GVE_FLAG_ONGOING_RESET);

    gve_teardown_queues(adapter);
    gve_free_device_resources(adapter);

    if (!gve_cfg_device_resources(adapter)) {
        msg_err("GVE: reset: failed to reconfigure device resources");
        goto done;
    }
    if (!gve_setup_queues(adapter)) {
        msg_err("GVE: reset: failed to recreate queues");
        gve_free_device_resources(adapter);
        goto done;
    }

    rprintf("GVE: adapter reset complete\n");
    {
        u32 status = pci_bar_read_4(&adapter->reg_bar, GVE_REG_DEVICE_STATUS);
        if (status & GVE_DEVICE_STATUS_LINK_STATUS)
            async_apply_bh((thunk)&adapter->link_up_task);
        else
            async_apply_bh((thunk)&adapter->link_down_task);
    }

  done:
    atomic_clear_bit(&adapter->flags, GVE_FLAG_ONGOING_RESET);
    atomic_clear_bit(&adapter->flags, GVE_FLAG_RESETTING);
    spin_unlock(&adapter->global_lock);
}

closure_func_basic(timer_handler, void, gve_watchdog_task,
                   u64 expiry, u64 overruns)
{
    if (overruns == timer_disabled)
        return;

    gve adapter = struct_from_closure(gve, watchdog_task);
    read_barrier();
    if (adapter->flags & ((1ULL << GVE_FLAG_RESETTING) |
                          (1ULL << GVE_FLAG_ONGOING_RESET)))
        return;

    timestamp deadline = milliseconds(GVE_TX_WATCHDOG_MS);
    timestamp now_ts   = now(CLOCK_ID_MONOTONIC);
    boolean   any_stuck = false;

    if (adapter->dqo) {
        for (u32 i = 0; i < adapter->num_queues; i++) {
            gve_tx_dqo_queue tx = &adapter->tx_dqo[i];
            if (tx->stuck)
                continue;
            /* Pending: head != desc_tail (descriptor slots still in flight) */
            if (tx->head == tx->desc_tail)
                continue;
            if (now_ts - tx->last_completion > deadline) {
                msg_err("GVE: DQO TX queue %d stuck (%d ms), "
                        "scheduling reset", i, GVE_TX_WATCHDOG_MS);
                tx->stuck = true;
                any_stuck = true;
            }
        }
        /* Wakeup: stopped DQO TX queues with pending packets that have not
         * been restarted by new incoming traffic. */
        for (u32 i = 0; i < adapter->num_queues; i++) {
            gve_tx_dqo_queue tx = &adapter->tx_dqo[i];
            if (tx->stuck || tx->running)
                continue;
            u32 free = (tx->mask + 1) - (tx->head - tx->desc_tail);
            if (free >= GVE_TX_RESUME_THRESH)
                async_apply_bh((thunk)&tx->enqueue_task);
        }
        /* Empty-ring detection: if fill posted nothing for several ticks,
         * the device has no buffers and will send no interrupts — deadlock.
         * Force a service run to attempt refill. */
        for (u32 i = 0; i < adapter->num_queues; i++) {
            gve_rx_dqo_queue rx = &adapter->rx_dqo[i];
            if (rx->empty_rx_queue > 2) {
                rx->rx_stats.empty_rx_ring++;
                rx->empty_rx_queue = 0;
                async_apply_bh((thunk)&rx->service);
            }
        }
    } else {
        for (u32 i = 0; i < adapter->num_queues; i++) {
            gve_tx_queue tx = &adapter->tx[i];
            if (tx->stuck)
                continue;
            if (tx->head == tx->tail)
                continue;
            u32 tail = be32toh(
                adapter->event_counters[be32toh(tx->q_res->counter_index)]);
            if (tail != tx->tail) {
                tx->last_completion = now_ts;
            } else if (now_ts - tx->last_completion > deadline) {
                msg_err("GVE: TX queue %d stuck (%d ms), "
                        "scheduling reset", i, GVE_TX_WATCHDOG_MS);
                tx->stuck = true;
                any_stuck = true;
            }
        }
        /* Wakeup: stopped GQI TX queues with pending packets. */
        for (u32 i = 0; i < adapter->num_queues; i++) {
            gve_tx_queue tx = &adapter->tx[i];
            if (tx->stuck || tx->running)
                continue;
            u32 hw_tail = be32toh(
                adapter->event_counters[be32toh(tx->q_res->counter_index)]);
            u32 free = adapter->tx_desc_cnt - (tx->head - hw_tail);
            if (free >= GVE_TX_RESUME_THRESH)
                async_apply_bh((thunk)&tx->enqueue_task);
        }
        for (u32 i = 0; i < adapter->num_queues; i++) {
            gve_rx_queue rx = &adapter->rx[i];
            if (rx->empty_rx_queue > 2) {
                rx->rx_stats.empty_rx_ring++;
                rx->empty_rx_queue = 0;
                async_apply_bh((thunk)&rx->service);
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
    if (!gve_init_interrupts(adapter)) {
        msg_err("GVE: failed to initialize interrupts");
        goto err2;
    }
    if (!gve_setup_queues(adapter)) {
        msg_err("GVE: failed to set up TX/RX queues");
        goto err3;
    }

    /* Start TX completion watchdog (fires every GVE_TX_WATCHDOG_MS/2). */
    timestamp watchdog_interval = milliseconds(GVE_TX_WATCHDOG_MS / 2);
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

    /* Disable SW checksum generation for IP/TCP/UDP: the NIC handles it.
     * Keep generation for ICMP/ICMPv6 (not offloaded) and all RX checks. */
    NETIF_SET_CHECKSUM_CTRL(netif,
        NETIF_CHECKSUM_ENABLE_ALL &
        ~(NETIF_CHECKSUM_GEN_IP | NETIF_CHECKSUM_GEN_UDP |
          NETIF_CHECKSUM_GEN_TCP));

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
    gve adapter = allocate(h, sizeof(struct gve));
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
