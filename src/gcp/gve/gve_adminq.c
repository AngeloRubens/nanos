/* gve_adminq.c — gVNIC admin queue, QPL management, queue create/destroy
 *
 * Covers:
 *   - Admin queue command submission and polling
 *   - Device describe / configure device resources
 *   - QPL (Queue Page List) register / unregister
 *   - TX and RX queue create / destroy
 *   - Queue count negotiation (device caps, MSI-X, CPU, manifest)
 */

#include "gve_priv.h"

/* ------------------------------------------------------------------ */
/* Admin queue helpers                                                  */
/* ------------------------------------------------------------------ */

static boolean gve_adminq_wait(gve adapter, u32 cmd_index)
{
    u32 delay_us = 0;
    int retries = 0;
    do {
        if (delay_us)
            kernel_delay(microseconds(delay_us));
        u32 tail = be32toh(pci_bar_read_4(&adapter->reg_bar,
                                          GVE_REG_ADMINQ_EVT_CNT));
        if (((s32)(tail - cmd_index)) >= 0)
            return true;
        delay_us = delay_us < GVE_ADMINQ_MIN_POLL_US ?
                   GVE_ADMINQ_MIN_POLL_US :
                   MIN(delay_us * 2, GVE_ADMINQ_MAX_POLL_US);
    } while (retries++ < 64);
    msg_err("GVE: admin queue command timed out, marking dead");
    adapter->adminq_running = false;
    return false;
}

static struct gve_adminq_command *gve_adminq_new_cmd(gve adapter)
{
    struct gve_adminq_command *cmd =
        &adapter->adminq[adapter->adminq_head & adapter->adminq_mask];
    zero(cmd, sizeof(*cmd));
    return cmd;
}

static u32 gve_adminq_issue_cmd(gve adapter)
{
    write_barrier();
    pci_bar_write_4(&adapter->reg_bar, GVE_REG_ADMINQ_DOORBELL,
                    htobe32(++adapter->adminq_head));
    return adapter->adminq_head;
}

static boolean gve_adminq_execute_cmd(gve adapter,
                                      struct gve_adminq_command *cmd)
{
    if (!adapter->adminq_running)
        return false;
    u32 index = gve_adminq_issue_cmd(adapter);
    if (!gve_adminq_wait(adapter, index))
        return false;
    read_barrier();
    u32 status = be32toh(cmd->status);
    gve_debug("cmd %d, status %d", be32toh(cmd->opcode), status);
    return (status == GVE_ADMINQ_COMMAND_PASSED);
}

/*
 * gve_verify_driver_compatibility — advertise the driver to the device.
 *
 * Newer gVNIC devices expect the driver to identify itself and declare its
 * capabilities before describe-device.  We declare only the queue formats we
 * implement (GQI-QPL/RDA, DQO-RDA), so the device never offers one we cannot
 * handle.  Best-effort: older devices that do not support this command return
 * an error and we proceed regardless (the original single-queue driver never
 * sent it).  Mirrors the official Google driver.
 */
boolean gve_verify_driver_compatibility(gve adapter)
{
    struct gve_driver_info *info = allocate(adapter->contiguous, PAGESIZE);
    if (info == INVALID_ADDRESS)
        return false;
    zero(info, sizeof(*info));
    info->os_type = 1;      /* Linux-compatible protocol */
    info->driver_capability_flags[0] = htobe64(GVE_DRIVER_CAPABILITY_FLAGS1);
    runtime_memcpy(info->os_version_str1, "nanos", 5);

    struct gve_adminq_command *cmd = gve_adminq_new_cmd(adapter);
    cmd->opcode = htobe32(GVE_ADMINQ_VERIFY_DRIVER_COMPATIBILITY);
    cmd->verify_driver_compat.driver_info_len  = htobe64(sizeof(*info));
    cmd->verify_driver_compat.driver_info_addr =
        htobe64(physical_from_virtual(info));
    boolean success = gve_adminq_execute_cmd(adapter, cmd);
    deallocate(adapter->contiguous, info, PAGESIZE);
    return success;
}

/* ------------------------------------------------------------------ */
/* Device describe / configure                                          */
/* ------------------------------------------------------------------ */

boolean gve_describe_device(gve adapter)
{
    struct gve_device_descriptor *desc =
        allocate(adapter->contiguous, PAGESIZE);
    if (desc == INVALID_ADDRESS)
        return false;
    struct gve_adminq_command *cmd = gve_adminq_new_cmd(adapter);
    cmd->opcode = htobe32(GVE_ADMINQ_DESCRIBE_DEVICE);
    cmd->describe_device.device_descriptor_addr =
        htobe64(physical_from_virtual(desc));
    cmd->describe_device.device_descriptor_version = htobe32(1);
    cmd->describe_device.available_length = htobe32(PAGESIZE);
    boolean success = gve_adminq_execute_cmd(adapter, cmd);
    if (success) {
        u8 *mac = adapter->ndev.n.hwaddr;
        runtime_memcpy(mac, desc->mac, sizeof(desc->mac));
        adapter->mtu               = be16toh(desc->mtu);
        adapter->num_event_counters = be16toh(desc->counters);
        adapter->tx_desc_cnt       = be16toh(desc->tx_queue_entries);
        adapter->rx_desc_cnt       = be16toh(desc->rx_queue_entries);
        adapter->tx_pages_per_qpl  = be16toh(desc->tx_pages_per_qpl);
        adapter->rx_data_slot_cnt  = be16toh(desc->rx_pages_per_qpl);

        /* Stash the device-reported sizes as canonical; gve_setup_queues
         * restores the working sizes to these on every (re)setup attempt. */
        adapter->tx_desc_cnt_dev      = adapter->tx_desc_cnt;
        adapter->rx_desc_cnt_dev      = adapter->rx_desc_cnt;
        adapter->tx_pages_per_qpl_dev = adapter->tx_pages_per_qpl;
        adapter->rx_data_slot_cnt_dev = adapter->rx_data_slot_cnt;

        /* Walk device option list, collecting which formats the device
         * offers, then pick the best by priority (order-independent):
         * DQO-RDA > DQO-QPL > GQI-RDA > GQI-QPL. */
        boolean opt_dqo_rda = false, opt_dqo_qpl = false, opt_gqi_rda = false;
        u16 num_opts   = be16toh(desc->num_device_options);
        u16 total_len  = be16toh(desc->total_length);
        struct gve_device_option *opt =
            (struct gve_device_option *)(desc + 1);
        for (u16 i = 0; i < num_opts; i++) {
            if ((u8 *)(opt + 1) > (u8 *)desc + total_len)
                break;
            u16 id  = be16toh(opt->option_id);
            u16 len = be16toh(opt->option_length);
            if (id == GVE_DEV_OPT_ID_DQO_RDA)
                opt_dqo_rda = true;
            else if (id == GVE_DEV_OPT_ID_DQO_QPL)
                opt_dqo_qpl = true;
            else if (id == GVE_DEV_OPT_ID_GQI_RDA ||
                     id == GVE_DEV_OPT_ID_GQI_RAW_ADDRESSING)
                opt_gqi_rda = true;
            opt = (struct gve_device_option *)((u8 *)(opt + 1) + len);
        }
        if (opt_dqo_rda) {
            adapter->dqo = true;
        } else if (opt_gqi_rda) {
            adapter->raw_addressing = true;
        }   /* else: GQI-QPL fallback (both flags false) */
        /* DQO-QPL addressing verified against Google (physical addresses into
         * a registered QPL, 2048-byte buffers): datapath = DQO-RDA + bounce
         * copy.  Not yet wired into the create/datapath, so detected but not
         * selected (safe GQI-QPL fallback). */
        (void)opt_dqo_qpl;

        const char *fmt =
            adapter->dqo ? (adapter->dqo_qpl ? "DQO-QPL" : "DQO-RDA") :
            adapter->raw_addressing ? "GQI-RDA" : "GQI-QPL";
        rprintf("GVE: MAC %02x:%02x:%02x:%02x:%02x:%02x MTU %d "
                "TX-desc %d RX-desc %d format %s\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                adapter->mtu, adapter->tx_desc_cnt, adapter->rx_desc_cnt, fmt);
    }
    deallocate(adapter->contiguous, desc, PAGESIZE);
    return success;
}

boolean gve_cfg_device_resources(gve adapter)
{
    u32 nq = adapter->num_queues;
    u32 nirq = GVE_IRQ_DB_COUNT(nq);

    u64 evt_cnt_size = MAX(adapter->num_event_counters * sizeof(u32),
                           PAGESIZE);
    adapter->event_counters = allocate(adapter->contiguous, evt_cnt_size);
    if (adapter->event_counters == INVALID_ADDRESS)
        return false;

    u64 irq_db_size = sizeof(struct gve_irq_db) * nirq;
    adapter->irq_db_indices = allocate(adapter->contiguous, irq_db_size);
    if (adapter->irq_db_indices == INVALID_ADDRESS)
        goto err_evt;

    struct gve_adminq_command *cmd = gve_adminq_new_cmd(adapter);
    cmd->opcode = htobe32(GVE_ADMINQ_CONFIGURE_DEVICE_RESOURCES);
    cmd->cfg_dev_resources.counter_array =
        htobe64(physical_from_virtual(adapter->event_counters));
    cmd->cfg_dev_resources.irq_db_addr =
        htobe64(physical_from_virtual(adapter->irq_db_indices));
    cmd->cfg_dev_resources.num_counters =
        htobe32(adapter->num_event_counters);
    cmd->cfg_dev_resources.num_irq_dbs   = htobe32(nirq);
    cmd->cfg_dev_resources.irq_db_stride =
        htobe32(sizeof(*adapter->irq_db_indices));
    cmd->cfg_dev_resources.ntfy_blk_msix_base_idx = htobe32(0);
    cmd->cfg_dev_resources.queue_format =
        adapter->dqo ? (adapter->dqo_qpl ? GVE_DQO_QPL_FORMAT : GVE_DQO_RDA_FORMAT) :
        adapter->raw_addressing ? GVE_GQI_RDA_FORMAT : GVE_GQI_QPL_FORMAT;

    boolean success = gve_adminq_execute_cmd(adapter, cmd);
    if (success) {
        zero(&adapter->hw_stats, sizeof(adapter->hw_stats));
        return true;
    }

    deallocate(adapter->contiguous, adapter->irq_db_indices, irq_db_size);
  err_evt:
    deallocate(adapter->contiguous, adapter->event_counters, evt_cnt_size);
    return false;
}

void gve_free_device_resources(gve adapter)
{
    u32 nq = adapter->num_queues;
    struct gve_adminq_command *cmd = gve_adminq_new_cmd(adapter);
    cmd->opcode = htobe32(GVE_ADMINQ_DECONFIGURE_DEVICE_RESOURCES);
    gve_adminq_execute_cmd(adapter, cmd);
    deallocate(adapter->contiguous, adapter->irq_db_indices,
               sizeof(struct gve_irq_db) * GVE_IRQ_DB_COUNT(nq));
    adapter->irq_db_indices = NULL;
    deallocate(adapter->contiguous, adapter->event_counters,
               MAX(adapter->num_event_counters * sizeof(u32), PAGESIZE));
    adapter->event_counters = NULL;
}

/*
 * gve_get_ptype_map_dqo — fetch the packet-type map (DQO only).
 *
 * The device requires the driver to issue GET_PTYPE_MAP for DQO queue
 * formats (the official Google driver does this at init and aborts on
 * failure).  We compute checksums in software and do not consume the map,
 * so we issue the command only to satisfy the device and discard the
 * result.
 */
boolean gve_get_ptype_map_dqo(gve adapter)
{
    void *ptype_map = allocate(adapter->contiguous, PAGESIZE);
    if (ptype_map == INVALID_ADDRESS)
        return false;
    struct gve_adminq_command *cmd = gve_adminq_new_cmd(adapter);
    cmd->opcode = htobe32(GVE_ADMINQ_GET_PTYPE_MAP);
    cmd->get_ptype_map.ptype_map_len  = htobe64(GVE_PTYPE_MAP_SIZE);
    cmd->get_ptype_map.ptype_map_addr =
        htobe64(physical_from_virtual(ptype_map));
    boolean success = gve_adminq_execute_cmd(adapter, cmd);
    deallocate(adapter->contiguous, ptype_map, PAGESIZE);
    return success;
}

/* ------------------------------------------------------------------ */
/* QPL management                                                       */
/* ------------------------------------------------------------------ */

static void *gve_create_qpl(gve adapter, u16 num_pages, u32 id)
{
    void *qpl_base = allocate(adapter->contiguous,
                              num_pages * PAGESIZE);
    if (qpl_base == INVALID_ADDRESS)
        return qpl_base;
    u64 *page_list = allocate(adapter->contiguous,
                              num_pages * sizeof(*page_list));
    if (page_list == INVALID_ADDRESS)
        goto error;
    u64 page_addr = physical_from_virtual(qpl_base);
    for (u16 page = 0; page < num_pages; page++) {
        page_list[page] = htobe64(page_addr);
        page_addr += PAGESIZE;
    }
    struct gve_adminq_command *cmd = gve_adminq_new_cmd(adapter);
    cmd->opcode = htobe32(GVE_ADMINQ_REGISTER_PAGE_LIST);
    cmd->register_page_list.page_list_id           = htobe32(id);
    cmd->register_page_list.num_pages               = htobe32(num_pages);
    cmd->register_page_list.page_address_list_addr =
        htobe64(physical_from_virtual(page_list));
    cmd->register_page_list.page_size               = htobe64(PAGESIZE);
    boolean success = gve_adminq_execute_cmd(adapter, cmd);
    deallocate(adapter->contiguous, page_list,
               num_pages * sizeof(*page_list));
    if (success)
        return qpl_base;
  error:
    deallocate(adapter->contiguous, qpl_base, num_pages * PAGESIZE);
    return INVALID_ADDRESS;
}

static void gve_destroy_qpl(gve adapter, void *qpl_base,
                             u16 num_pages, u32 id)
{
    struct gve_adminq_command *cmd = gve_adminq_new_cmd(adapter);
    cmd->opcode = htobe32(GVE_ADMINQ_UNREGISTER_PAGE_LIST);
    cmd->unregister_page_list.page_list_id = htobe32(id);
    gve_adminq_execute_cmd(adapter, cmd);
    deallocate(adapter->contiguous, qpl_base, num_pages * PAGESIZE);
}

/* ------------------------------------------------------------------ */
/* Queue create / destroy                                               */
/* ------------------------------------------------------------------ */

/*
 * QPL ID assignment (QPL mode only):
 *   TX queue i  -> QPL id = i
 *   RX queue i  -> QPL id = num_queues + i
 *
 * ntfy_id (= IRQ DB slot index) follows GVE_IRQ_DB_TX/RX macros:
 *   TX ntfy_id = i
 *   RX ntfy_id = num_queues + i
 */

static boolean gve_create_tx_queue(gve adapter, gve_tx_queue tx,
                                    u32 index)
{
    boolean rda = adapter->raw_addressing;

    if (rda) {
        tx->qpl_base = NULL;
        tx->pending  = allocate(adapter->general,
                                adapter->tx_desc_cnt * sizeof(*tx->pending));
        if (tx->pending == INVALID_ADDRESS)
            return false;
        zero(tx->pending, adapter->tx_desc_cnt * sizeof(*tx->pending));
    } else {
        tx->qpl_base = gve_create_qpl(adapter, adapter->tx_pages_per_qpl,
                                      index);
        if (tx->qpl_base == INVALID_ADDRESS)
            return false;
        tx->qpl_allocated = allocate(adapter->general,
                                     adapter->tx_desc_cnt *
                                     sizeof(*tx->qpl_allocated));
        if (tx->qpl_allocated == INVALID_ADDRESS) {
            gve_destroy_qpl(adapter, tx->qpl_base,
                            adapter->tx_pages_per_qpl, index);
            return false;
        }
    }

    tx->desc = allocate(adapter->contiguous,
                        adapter->tx_desc_cnt * sizeof(*tx->desc));
    if (tx->desc == INVALID_ADDRESS)
        goto err_after_mode_alloc;

    tx->q_res = allocate(adapter->contiguous, sizeof(*tx->q_res));
    if (tx->q_res == INVALID_ADDRESS)
        goto err_after_desc;

    tx->tx_timestamps = allocate(adapter->general,
                                  adapter->tx_desc_cnt * sizeof(*tx->tx_timestamps));
    if (tx->tx_timestamps == INVALID_ADDRESS)
        goto err_after_q_res;
    zero(tx->tx_timestamps, adapter->tx_desc_cnt * sizeof(*tx->tx_timestamps));

    tx->br = allocate_queue(adapter->general, GVE_BUF_RING_SIZE);
    if (tx->br == INVALID_ADDRESS)
        goto err_after_timestamps;

    struct gve_adminq_command *cmd = gve_adminq_new_cmd(adapter);
    cmd->opcode = htobe32(GVE_ADMINQ_CREATE_TX_QUEUE);
    cmd->create_tx_queue.queue_id             = htobe32(index);
    cmd->create_tx_queue.queue_resources_addr =
        htobe64(physical_from_virtual(tx->q_res));
    cmd->create_tx_queue.tx_ring_addr         =
        htobe64(physical_from_virtual(tx->desc));
    cmd->create_tx_queue.queue_page_list_id   =
        htobe32(rda ? GVE_RAW_ADDRESSING_QPL_ID : index);
    cmd->create_tx_queue.ntfy_id              =
        htobe32(GVE_IRQ_DB_TX(adapter->num_queues, index));
    cmd->create_tx_queue.tx_ring_size         =
        htobe16(adapter->tx_desc_cnt);

    if (!gve_adminq_execute_cmd(adapter, cmd))
        goto err_after_br;

    tx->mask            = adapter->tx_desc_cnt - 1;
    tx->head = tx->tail = 0;
    if (!rda) {
        tx->qpl_head = 0;
        tx->qpl_used = 0;
        tx->qpl_size = adapter->tx_pages_per_qpl * PAGESIZE;
    }
    tx->adapter           = adapter;
    tx->stuck             = false;
    tx->running           = true;
    tx->event_counter_idx = be32toh(tx->q_res->counter_index);
    tx->db_idx            = be32toh(tx->q_res->db_index);
    zero(&tx->tx_stats, sizeof(tx->tx_stats));
    gve_tx_init_gqi(tx);
    return true;

  err_after_br:
    deallocate_queue(tx->br);
  err_after_timestamps:
    deallocate(adapter->general, tx->tx_timestamps,
               adapter->tx_desc_cnt * sizeof(*tx->tx_timestamps));
  err_after_q_res:
    deallocate(adapter->contiguous, tx->q_res, sizeof(*tx->q_res));
  err_after_desc:
    deallocate(adapter->contiguous, tx->desc,
               adapter->tx_desc_cnt * sizeof(*tx->desc));
  err_after_mode_alloc:
    if (rda) {
        deallocate(adapter->general, tx->pending,
                   adapter->tx_desc_cnt * sizeof(*tx->pending));
    } else {
        deallocate(adapter->general, tx->qpl_allocated,
                   adapter->tx_desc_cnt * sizeof(*tx->qpl_allocated));
        gve_destroy_qpl(adapter, tx->qpl_base,
                        adapter->tx_pages_per_qpl, index);
    }
    return false;
}

static void gve_destroy_tx_queue(gve adapter, gve_tx_queue tx,
                                  u32 index)
{
    struct gve_adminq_command *cmd = gve_adminq_new_cmd(adapter);
    cmd->opcode = htobe32(GVE_ADMINQ_DESTROY_TX_QUEUE);
    cmd->destroy_tx_queue.queue_id = htobe32(index);
    gve_adminq_execute_cmd(adapter, cmd);

    /* Hold ring_mtx to exclude any in-progress gve_tx_start_xmit_gqi BH
     * running on another CPU.  NETIF_FLAG_UP has been cleared by gve_reset
     * before we reach here, so no new xmit BH will enqueue after this
     * point; any already running will complete and release the lock. */
    spin_lock(&tx->ring_mtx);

    /* Drain software TX queue: free any pbufs not yet sent. */
    struct pbuf *p;
    while ((p = dequeue(tx->br)) != INVALID_ADDRESS)
        pbuf_free(p);
    deallocate_queue(tx->br);
    tx->br = NULL;

    deallocate(adapter->contiguous, tx->q_res, sizeof(*tx->q_res));
    tx->q_res = NULL;
    deallocate(adapter->general, tx->tx_timestamps,
               adapter->tx_desc_cnt * sizeof(*tx->tx_timestamps));
    tx->tx_timestamps = NULL;
    deallocate(adapter->contiguous, tx->desc,
               adapter->tx_desc_cnt * sizeof(*tx->desc));
    tx->desc = NULL;
    if (adapter->raw_addressing) {
        for (u32 i = 0; i <= tx->mask; i++)
            if (tx->pending[i])
                pbuf_free(tx->pending[i]);
        deallocate(adapter->general, tx->pending,
                   adapter->tx_desc_cnt * sizeof(*tx->pending));
        tx->pending = NULL;
    } else {
        deallocate(adapter->general, tx->qpl_allocated,
                   adapter->tx_desc_cnt * sizeof(*tx->qpl_allocated));
        tx->qpl_allocated = NULL;
        gve_destroy_qpl(adapter, tx->qpl_base,
                        adapter->tx_pages_per_qpl, index);
        tx->qpl_base = NULL;
    }

    spin_unlock(&tx->ring_mtx);
}

static boolean gve_create_rx_queue(gve adapter, gve_rx_queue rx,
                                    u32 index)
{
    boolean rda = adapter->raw_addressing;
    u16 num_pages = adapter->rx_data_slot_cnt;
    u32 nq = adapter->num_queues;
    u32 qpl_id;

    if (rda) {
        rx->qpl_base = allocate(adapter->contiguous,
                                num_pages * PAGESIZE);
        if (rx->qpl_base == INVALID_ADDRESS)
            return false;
        rx->rda_base_phys = physical_from_virtual(rx->qpl_base);
        qpl_id = GVE_RAW_ADDRESSING_QPL_ID;
    } else {
        qpl_id = nq + index;
        rx->qpl_base = gve_create_qpl(adapter, num_pages, qpl_id);
        if (rx->qpl_base == INVALID_ADDRESS)
            return false;
        rx->rda_base_phys = 0;
    }

    rx->qpl_available = rx->qpl_count = num_pages;

    rx->pbufs = allocate(adapter->general,
                         rx->qpl_count * sizeof(*rx->pbufs));
    if (rx->pbufs == INVALID_ADDRESS)
        goto err_after_qpl;

    rx->desc = allocate(adapter->contiguous,
                        adapter->rx_desc_cnt * sizeof(*rx->desc));
    if (rx->desc == INVALID_ADDRESS)
        goto err_after_pbufs;

    /* The completion-descriptor ring (rx->desc, rx_desc_cnt entries) and the
     * data ring (rx->data, rx_data_slot_cnt entries) are both indexed with
     * rx->mask (= rx_desc_cnt - 1) in the hot path.  This relies on the
     * GQI invariant rx_queue_entries == rx_pages_per_qpl: GVE reports equal
     * values for both, so the single mask is valid for both rings.  If a
     * device ever reported rx_data_slot_cnt < rx_desc_cnt, rx->data would be
     * indexed out of bounds. */
    rx->data = allocate(adapter->contiguous,
                        adapter->rx_data_slot_cnt * sizeof(*rx->data));
    if (rx->data == INVALID_ADDRESS)
        goto err_after_desc;

    rx->q_res = allocate(adapter->contiguous, sizeof(*rx->q_res));
    if (rx->q_res == INVALID_ADDRESS)
        goto err_after_data;

    struct gve_adminq_command *cmd = gve_adminq_new_cmd(adapter);
    cmd->opcode = htobe32(GVE_ADMINQ_CREATE_RX_QUEUE);
    cmd->create_rx_queue.queue_id =
        cmd->create_rx_queue.index = htobe32(index);
    cmd->create_rx_queue.ntfy_id  = htobe32(GVE_IRQ_DB_RX(nq, index));
    cmd->create_rx_queue.queue_resources_addr =
        htobe64(physical_from_virtual(rx->q_res));
    cmd->create_rx_queue.rx_desc_ring_addr =
        htobe64(physical_from_virtual(rx->desc));
    cmd->create_rx_queue.rx_data_ring_addr =
        htobe64(physical_from_virtual(rx->data));
    cmd->create_rx_queue.queue_page_list_id  = htobe32(qpl_id);
    cmd->create_rx_queue.rx_ring_size        = htobe16(adapter->rx_desc_cnt);
    cmd->create_rx_queue.packet_buffer_size  = htobe16(PAGESIZE / 2);
    cmd->create_rx_queue.rx_buff_ring_size   =
        htobe16(rda ? 0 : adapter->rx_data_slot_cnt);

    if (!gve_adminq_execute_cmd(adapter, cmd))
        goto err_after_q_res;

    rx->mask = adapter->rx_desc_cnt - 1;
    rx->head = rx->tail = 0;
    rx->qpl_head = 0;
    for (u32 i = 0; i < rx->qpl_count; i++) {
        struct pbuf *pb = &rx->pbufs[i];
        pb->next          = NULL;
        pb->type_internal = PBUF_REF;
        pb->flags         = 0;
        pb->ref           = 1;
        pb->if_idx        = NETIF_NO_INDEX;
    }
    rx->irq_db_index =
        &adapter->irq_db_indices[GVE_IRQ_DB_RX(nq, index)].index;
    rx->adapter = adapter;
    zero(&rx->rx_stats, sizeof(rx->rx_stats));

    /* Init closures and fill initial RX buffers (gve_datapath.c). */
    gve_rx_init(rx);
    rx->first_interrupt        = false;
    rx->no_interrupt_event_cnt = 0;
    rx->empty_rx_queue         = 0;
    rx->event_counter_idx      = be32toh(rx->q_res->counter_index);
    rx->db_idx                 = be32toh(rx->q_res->db_index);
    rx->idx                    = index;
    rx->ctx_head               = NULL;
    gve_rx_fill(rx);
    return true;

  err_after_q_res:
    deallocate(adapter->contiguous, rx->q_res, sizeof(*rx->q_res));
  err_after_data:
    deallocate(adapter->contiguous, rx->data,
               adapter->rx_data_slot_cnt * sizeof(*rx->data));
  err_after_desc:
    deallocate(adapter->contiguous, rx->desc,
               adapter->rx_desc_cnt * sizeof(*rx->desc));
  err_after_pbufs:
    deallocate(adapter->general, rx->pbufs,
               rx->qpl_count * sizeof(*rx->pbufs));
  err_after_qpl:
    if (rda)
        deallocate(adapter->contiguous, rx->qpl_base,
                   num_pages * PAGESIZE);
    else
        gve_destroy_qpl(adapter, rx->qpl_base, num_pages, qpl_id);
    return false;
}

static void gve_destroy_rx_queue(gve adapter, gve_rx_queue rx,
                                  u32 index)
{
    u32 nq = adapter->num_queues;
    struct gve_adminq_command *cmd = gve_adminq_new_cmd(adapter);
    cmd->opcode = htobe32(GVE_ADMINQ_DESTROY_RX_QUEUE);
    cmd->destroy_rx_queue.queue_id = htobe32(index);
    gve_adminq_execute_cmd(adapter, cmd);

    /* Free a partially-assembled multi-buffer packet, if any. */
    if (rx->ctx_head) {
        pbuf_free(rx->ctx_head);
        rx->ctx_head = NULL;
    }

    /* Wait for any in-flight RX pbufs held by lwIP to be released.
     * NETIF_FLAG_UP was cleared by gve_reset before this point; any
     * gve_rx_service BH that passed the UP check will complete its
     * current pass and exit on the next check.  Bound the wait: lwIP
     * may hold refs in a TCP receive buffer, but reset is more important
     * than a clean drain.  After 1000 iterations log and proceed. */
    for (u32 i = 0; i < rx->qpl_count; i++) {
        struct pbuf *pb = &rx->pbufs[i];
        for (int retries = 1000; pb->ref > 1 && retries > 0; retries--)
            read_barrier();
        if (pb->ref > 1)
            msg_err("GVE: RX teardown: pbuf[%u] still held (ref %d), "
                    "proceeding", i, (int)pb->ref);
    }

    deallocate(adapter->contiguous, rx->q_res, sizeof(*rx->q_res));
    rx->q_res = NULL;
    deallocate(adapter->contiguous, rx->data,
               adapter->rx_data_slot_cnt * sizeof(*rx->data));
    rx->data = NULL;
    deallocate(adapter->contiguous, rx->desc,
               adapter->rx_desc_cnt * sizeof(*rx->desc));
    rx->desc = NULL;
    deallocate(adapter->general, rx->pbufs,
               rx->qpl_count * sizeof(*rx->pbufs));
    rx->pbufs = NULL;
    if (adapter->raw_addressing) {
        deallocate(adapter->contiguous, rx->qpl_base,
                   adapter->rx_data_slot_cnt * PAGESIZE);
    } else {
        gve_destroy_qpl(adapter, rx->qpl_base,
                        adapter->rx_data_slot_cnt, nq + index);
    }
    rx->qpl_base = NULL;
}

/* ------------------------------------------------------------------ */
/* DQO queue create / destroy                                           */
/* ------------------------------------------------------------------ */

static boolean gve_create_tx_queue_dqo(gve adapter,
                                        gve_tx_dqo_queue tx, u32 index)
{
    u16 desc_cnt  = adapter->tx_desc_cnt;

    tx->desc = allocate(adapter->contiguous,
                        desc_cnt * sizeof(*tx->desc));
    if (tx->desc == INVALID_ADDRESS)
        return false;

    tx->compl = allocate(adapter->contiguous,
                         desc_cnt * sizeof(*tx->compl));
    if (tx->compl == INVALID_ADDRESS)
        goto err_compl;
    zero(tx->compl, desc_cnt * sizeof(*tx->compl));

    tx->pending = allocate(adapter->general,
                           desc_cnt * sizeof(*tx->pending));
    if (tx->pending == INVALID_ADDRESS)
        goto err_pending;
    zero(tx->pending, desc_cnt * sizeof(*tx->pending));

    tx->seg_counts = allocate(adapter->general,
                              desc_cnt * sizeof(*tx->seg_counts));
    if (tx->seg_counts == INVALID_ADDRESS)
        goto err_seg_counts;
    zero(tx->seg_counts, desc_cnt * sizeof(*tx->seg_counts));

    tx->miss_times = allocate(adapter->general,
                              desc_cnt * sizeof(*tx->miss_times));
    if (tx->miss_times == INVALID_ADDRESS)
        goto err_miss;
    zero(tx->miss_times, desc_cnt * sizeof(*tx->miss_times));

    tx->q_res = allocate(adapter->contiguous, sizeof(*tx->q_res));
    if (tx->q_res == INVALID_ADDRESS)
        goto err_q_res;

    tx->tx_timestamps = allocate(adapter->general,
                                  desc_cnt * sizeof(*tx->tx_timestamps));
    if (tx->tx_timestamps == INVALID_ADDRESS)
        goto err_after_q_res;
    zero(tx->tx_timestamps, desc_cnt * sizeof(*tx->tx_timestamps));

    tx->br = allocate_queue(adapter->general, GVE_BUF_RING_SIZE);
    if (tx->br == INVALID_ADDRESS)
        goto err_timestamps;

    struct gve_adminq_command *cmd = gve_adminq_new_cmd(adapter);
    cmd->opcode = htobe32(GVE_ADMINQ_CREATE_TX_QUEUE);
    cmd->create_tx_queue.queue_id             = htobe32(index);
    cmd->create_tx_queue.queue_resources_addr =
        htobe64(physical_from_virtual(tx->q_res));
    cmd->create_tx_queue.tx_ring_addr         =
        htobe64(physical_from_virtual(tx->desc));
    cmd->create_tx_queue.queue_page_list_id   =
        htobe32(GVE_RAW_ADDRESSING_QPL_ID);
    cmd->create_tx_queue.ntfy_id              =
        htobe32(GVE_IRQ_DB_TX(adapter->num_queues, index));
    cmd->create_tx_queue.tx_ring_size         = htobe16(desc_cnt);
    cmd->create_tx_queue.tx_comp_ring_addr    =
        htobe64(physical_from_virtual(tx->compl));
    cmd->create_tx_queue.tx_comp_ring_size    = htobe16(desc_cnt);

    if (!gve_adminq_execute_cmd(adapter, cmd))
        goto err_cmd;

    tx->mask             = desc_cnt - 1;
    tx->head             = 0;
    tx->desc_tail        = 0;
    tx->compl_head       = 0;
    tx->expected_gen     = 1;  /* device writes gen=1 on first pass */
    tx->adapter          = adapter;
    tx->stuck            = false;
    tx->running          = true;
    tx->pending_misses   = 0;
    tx->last_re_idx      = 0;
    tx->db_idx           = be32toh(tx->q_res->db_index);
    zero(&tx->tx_stats, sizeof(tx->tx_stats));
    gve_tx_init_dqo(tx);
    return true;

  err_cmd:
    deallocate_queue(tx->br);
  err_timestamps:
    deallocate(adapter->general, tx->tx_timestamps,
               desc_cnt * sizeof(*tx->tx_timestamps));
  err_after_q_res:
    deallocate(adapter->contiguous, tx->q_res, sizeof(*tx->q_res));
  err_q_res:
    deallocate(adapter->general, tx->miss_times,
               desc_cnt * sizeof(*tx->miss_times));
  err_miss:
    deallocate(adapter->general, tx->seg_counts,
               desc_cnt * sizeof(*tx->seg_counts));
  err_seg_counts:
    deallocate(adapter->general, tx->pending,
               desc_cnt * sizeof(*tx->pending));
  err_pending:
    deallocate(adapter->contiguous, tx->compl,
               desc_cnt * sizeof(*tx->compl));
  err_compl:
    deallocate(adapter->contiguous, tx->desc,
               desc_cnt * sizeof(*tx->desc));
    return false;
}

static void gve_destroy_tx_queue_dqo(gve adapter,
                                      gve_tx_dqo_queue tx, u32 index)
{
    struct gve_adminq_command *cmd = gve_adminq_new_cmd(adapter);
    cmd->opcode = htobe32(GVE_ADMINQ_DESTROY_TX_QUEUE);
    cmd->destroy_tx_queue.queue_id = htobe32(index);
    gve_adminq_execute_cmd(adapter, cmd);

    /* Hold ring_mtx to exclude any in-progress gve_tx_start_xmit_dqo BH. */
    spin_lock(&tx->ring_mtx);

    /* Drain software TX queue: free any pbufs not yet sent. */
    struct pbuf *p;
    while ((p = dequeue(tx->br)) != INVALID_ADDRESS)
        pbuf_free(p);
    deallocate_queue(tx->br);
    tx->br = NULL;

    u16 desc_cnt = adapter->tx_desc_cnt;
    /* Free any in-flight pbufs. */
    for (u32 i = 0; i <= tx->mask; i++)
        if (tx->pending[i])
            pbuf_free(tx->pending[i]);
    deallocate(adapter->contiguous, tx->q_res, sizeof(*tx->q_res));
    tx->q_res = NULL;
    deallocate(adapter->general,    tx->tx_timestamps,
               desc_cnt * sizeof(*tx->tx_timestamps));
    tx->tx_timestamps = NULL;
    deallocate(adapter->general,    tx->miss_times,
               desc_cnt * sizeof(*tx->miss_times));
    tx->miss_times = NULL;
    deallocate(adapter->general,    tx->seg_counts,
               desc_cnt * sizeof(*tx->seg_counts));
    tx->seg_counts = NULL;
    deallocate(adapter->general,    tx->pending,
               desc_cnt * sizeof(*tx->pending));
    tx->pending = NULL;
    deallocate(adapter->contiguous, tx->compl,
               desc_cnt * sizeof(*tx->compl));
    tx->compl = NULL;
    deallocate(adapter->contiguous, tx->desc,
               desc_cnt * sizeof(*tx->desc));
    tx->desc = NULL;

    spin_unlock(&tx->ring_mtx);
}

static boolean gve_create_rx_queue_dqo(gve adapter,
                                        gve_rx_dqo_queue rx, u32 index)
{
    u32 num_bufs  = adapter->rx_data_slot_cnt;
    u32 nq        = adapter->num_queues;

    rx->pbufs = allocate(adapter->general,
                         num_bufs * sizeof(*rx->pbufs));
    if (rx->pbufs == INVALID_ADDRESS)
        return false;
    zero(rx->pbufs, num_bufs * sizeof(*rx->pbufs));

    rx->free_ids = allocate(adapter->general,
                            num_bufs * sizeof(*rx->free_ids));
    if (rx->free_ids == INVALID_ADDRESS)
        goto err_free_ids;
    for (u32 i = 0; i < num_bufs; i++)
        rx->free_ids[i] = (u16)i;

    rx->buf_ring = allocate(adapter->contiguous,
                            num_bufs * sizeof(*rx->buf_ring));
    if (rx->buf_ring == INVALID_ADDRESS)
        goto err_buf_ring;
    zero(rx->buf_ring, num_bufs * sizeof(*rx->buf_ring));

    rx->compl_ring = allocate(adapter->contiguous,
                              num_bufs * sizeof(*rx->compl_ring));
    if (rx->compl_ring == INVALID_ADDRESS)
        goto err_compl_ring;
    zero(rx->compl_ring, num_bufs * sizeof(*rx->compl_ring));

    rx->q_res = allocate(adapter->contiguous, sizeof(*rx->q_res));
    if (rx->q_res == INVALID_ADDRESS)
        goto err_q_res;

    struct gve_adminq_command *cmd = gve_adminq_new_cmd(adapter);
    cmd->opcode = htobe32(GVE_ADMINQ_CREATE_RX_QUEUE);
    cmd->create_rx_queue.queue_id =
        cmd->create_rx_queue.index = htobe32(index);
    cmd->create_rx_queue.ntfy_id  = htobe32(GVE_IRQ_DB_RX(nq, index));
    cmd->create_rx_queue.queue_resources_addr =
        htobe64(physical_from_virtual(rx->q_res));
    cmd->create_rx_queue.rx_desc_ring_addr =   /* completion ring */
        htobe64(physical_from_virtual(rx->compl_ring));
    cmd->create_rx_queue.rx_data_ring_addr =   /* buffer ring */
        htobe64(physical_from_virtual(rx->buf_ring));
    cmd->create_rx_queue.queue_page_list_id  =
        htobe32(GVE_RAW_ADDRESSING_QPL_ID);
    cmd->create_rx_queue.rx_ring_size        = htobe16(num_bufs);
    cmd->create_rx_queue.packet_buffer_size  = htobe16(GVE_DQO_BUF_SIZE);
    cmd->create_rx_queue.rx_buff_ring_size   = htobe16(num_bufs);

    if (!gve_adminq_execute_cmd(adapter, cmd))
        goto err_cmd;

    rx->mask          = num_bufs - 1;
    rx->num_bufs      = num_bufs;
    rx->buf_head      = 0;
    rx->compl_head    = 0;
    rx->expected_gen  = 1;
    rx->next_to_use   = 0;
    rx->next_to_clean = num_bufs;  /* free list starts full: all IDs available */
    rx->adapter       = adapter;
    rx->irq_db_index  =
        &adapter->irq_db_indices[GVE_IRQ_DB_RX(nq, index)].index;
    rx->db_idx        = be32toh(rx->q_res->db_index);
    zero(&rx->rx_stats, sizeof(rx->rx_stats));

    /* Init service closure and fill initial buffers (gve_dqo.c). */
    gve_rx_dqo_init(rx);
    rx->first_interrupt        = false;
    rx->no_interrupt_event_cnt = 0;
    rx->empty_rx_queue         = 0;
    rx->idx                    = index;
    rx->ctx_head               = NULL;
    gve_rx_dqo_fill(rx);
    return true;

  err_cmd:
    deallocate(adapter->contiguous, rx->q_res, sizeof(*rx->q_res));
  err_q_res:
    deallocate(adapter->contiguous, rx->compl_ring,
               num_bufs * sizeof(*rx->compl_ring));
  err_compl_ring:
    deallocate(adapter->contiguous, rx->buf_ring,
               num_bufs * sizeof(*rx->buf_ring));
  err_buf_ring:
    deallocate(adapter->general, rx->free_ids,
               num_bufs * sizeof(*rx->free_ids));
  err_free_ids:
    deallocate(adapter->general, rx->pbufs,
               num_bufs * sizeof(*rx->pbufs));
    return false;
}

static void gve_destroy_rx_queue_dqo(gve adapter,
                                      gve_rx_dqo_queue rx, u32 index)
{
    u32 num_bufs = rx->num_bufs;
    struct gve_adminq_command *cmd = gve_adminq_new_cmd(adapter);
    cmd->opcode = htobe32(GVE_ADMINQ_DESTROY_RX_QUEUE);
    cmd->destroy_rx_queue.queue_id = htobe32(index);
    gve_adminq_execute_cmd(adapter, cmd);

    /* Free a partially-assembled multi-buffer packet, if any. */
    if (rx->ctx_head) {
        pbuf_free(rx->ctx_head);
        rx->ctx_head = NULL;
    }

    /* Free any pbufs still posted to the device.  NETIF_FLAG_UP was
     * cleared before teardown so no new completions arrive; each slot
     * is either NULL (already completed and returned) or holds the only
     * reference to the pbuf (the device's reference is gone). */
    for (u32 i = 0; i < num_bufs; i++)
        if (rx->pbufs[i])
            pbuf_free(rx->pbufs[i]);

    deallocate(adapter->contiguous, rx->q_res, sizeof(*rx->q_res));
    rx->q_res = NULL;
    deallocate(adapter->contiguous, rx->compl_ring,
               num_bufs * sizeof(*rx->compl_ring));
    rx->compl_ring = NULL;
    deallocate(adapter->contiguous, rx->buf_ring,
               num_bufs * sizeof(*rx->buf_ring));
    rx->buf_ring = NULL;
    deallocate(adapter->general,    rx->free_ids,
               num_bufs * sizeof(*rx->free_ids));
    rx->free_ids = NULL;
    deallocate(adapter->general,    rx->pbufs,
               num_bufs * sizeof(*rx->pbufs));
    rx->pbufs = NULL;
}

/* ------------------------------------------------------------------ */
/* Queue count negotiation and setup                                    */
/* ------------------------------------------------------------------ */

/*
 * gve_calc_num_queues: determine how many TX/RX queue pairs to create.
 *
 * Priority (lowest wins):
 *   1. GVE_MAX_IO_QUEUES compile-time cap
 *   2. Device-reported max TX / RX queues (register BAR)
 *   3. Available MSI-X vectors (2 per queue + 1 mgmt)
 *   4. CPU count
 *   5. "io-queues" manifest option
 */
u32 gve_calc_num_queues(gve adapter, tuple config)
{
    u32 dev_max_tx = be32toh(pci_bar_read_4(&adapter->reg_bar,
                                             GVE_REG_MAX_TX_QUEUES));
    u32 dev_max_rx = be32toh(pci_bar_read_4(&adapter->reg_bar,
                                             GVE_REG_MAX_RX_QUEUES));
    u32 dev_max = MIN(dev_max_tx, dev_max_rx);
    if (dev_max == 0)
        dev_max = 1;    /* legacy firmware that reports 0 = single queue */

    u32 nq = MIN(GVE_MAX_IO_QUEUES, dev_max);

    /* MSI-X constraint: need 2*nq + 1 vectors. */
    int msix_avail = pci_get_msix_count(adapter->pdev);
    if (msix_avail > 1)
        nq = MIN(nq, (u32)((msix_avail - 1) / 2));

    /* CPU constraint. */
    nq = MIN(nq, (u32)total_processors);

    /* Manifest override. */
    u64 manifest_val = 0;
    if (config && get_u64(config, sym_this("io-queues"), &manifest_val)
            && manifest_val > 0)
        nq = MIN(nq, (u32)manifest_val);

    if (nq == 0)
        nq = 1;

    return nq;
}

static boolean gve_try_setup_queues(gve adapter)
{
    u32 nq = adapter->num_queues;
    u32 i;

    if (adapter->dqo) {
        for (i = 0; i < nq; i++) {
            if (!gve_create_tx_queue_dqo(adapter, &adapter->tx_dqo[i], i))
                goto err_tx_dqo;
        }
        for (i = 0; i < nq; i++) {
            if (!gve_create_rx_queue_dqo(adapter, &adapter->rx_dqo[i], i))
                goto err_rx_dqo;
        }
        return true;

      err_rx_dqo:
        for (u32 j = 0; j < i; j++)
            gve_destroy_rx_queue_dqo(adapter, &adapter->rx_dqo[j], j);
        i = nq;
      err_tx_dqo:
        for (u32 j = 0; j < i; j++)
            gve_destroy_tx_queue_dqo(adapter, &adapter->tx_dqo[j], j);
        return false;
    }

    for (i = 0; i < nq; i++) {
        if (!gve_create_tx_queue(adapter, &adapter->tx[i], i))
            goto err_tx;
    }
    for (i = 0; i < nq; i++) {
        if (!gve_create_rx_queue(adapter, &adapter->rx[i], i))
            goto err_rx;
    }
    return true;

  err_rx:
    for (u32 j = 0; j < i; j++)
        gve_destroy_rx_queue(adapter, &adapter->rx[j], j);
    i = nq;
  err_tx:
    for (u32 j = 0; j < i; j++)
        gve_destroy_tx_queue(adapter, &adapter->tx[j], j);
    return false;
}

/*
 * gve_setup_queues — create all TX/RX queue pairs with ring-size backoff.
 *
 * If contiguous memory allocation fails, halve tx_desc_cnt / rx_desc_cnt
 * (and the QPL page counts for GQI-QPL mode) and retry until we reach
 * GVE_MIN_RING_SIZE.  This matches the ENA driver pattern and avoids
 * probe failure under memory pressure at boot or after reset.
 *
 * The working sizes are reset to the device-reported (canonical) values
 * first, so that a backoff from an earlier setup (e.g. before a reset)
 * does not leave the rings permanently small once memory is available
 * again.  Mirrors ENA set_io_rings_size (ena.c:1042).
 */
boolean gve_setup_queues(gve adapter)
{
    adapter->tx_desc_cnt      = adapter->tx_desc_cnt_dev;
    adapter->rx_desc_cnt      = adapter->rx_desc_cnt_dev;
    adapter->tx_pages_per_qpl = adapter->tx_pages_per_qpl_dev;
    adapter->rx_data_slot_cnt = adapter->rx_data_slot_cnt_dev;

    for (;;) {
        if (gve_try_setup_queues(adapter))
            return true;

        if (adapter->tx_desc_cnt <= GVE_MIN_RING_SIZE ||
            adapter->rx_desc_cnt <= GVE_MIN_RING_SIZE) {
            msg_err("GVE: queue alloc failed at minimum ring size "
                    "(TX %u RX %u slots); giving up",
                    adapter->tx_desc_cnt, adapter->rx_desc_cnt);
            return false;
        }

        adapter->tx_desc_cnt      /= 2;
        adapter->rx_desc_cnt      /= 2;
        adapter->tx_pages_per_qpl /= 2;
        adapter->rx_data_slot_cnt /= 2;
        rprintf("GVE: ring alloc failed, retrying with "
                "TX %u / RX %u slots\n",
                adapter->tx_desc_cnt, adapter->rx_desc_cnt);
    }
}

void gve_teardown_queues(gve adapter)
{
    u32 nq = adapter->num_queues;
    if (adapter->dqo) {
        for (u32 i = 0; i < nq; i++)
            gve_destroy_rx_queue_dqo(adapter, &adapter->rx_dqo[i], i);
        for (u32 i = 0; i < nq; i++)
            gve_destroy_tx_queue_dqo(adapter, &adapter->tx_dqo[i], i);
    } else {
        for (u32 i = 0; i < nq; i++)
            gve_destroy_rx_queue(adapter, &adapter->rx[i], i);
        for (u32 i = 0; i < nq; i++)
            gve_destroy_tx_queue(adapter, &adapter->tx[i], i);
    }
}
