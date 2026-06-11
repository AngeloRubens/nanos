/* Google Virtual Ethernet (gVNIC) driver — multi-queue port
 *
 * Ported from src/drivers/gve.c; adds:
 *   - multi-queue TX/RX (up to GVE_MAX_IO_QUEUES, limited by device,
 *     MSI-X vectors, CPU count, and the "io-queues" manifest option)
 *   - io-queues manifest support (same pattern as virtio_net)
 *   - queue format negotiation: DQO-RDA > DQO-QPL > GQI-RDA > GQI-QPL
 *   - DQO format (Andromeda 2.x): generation-bit completion polling,
 *     separate TX-completion and RX-buffer/completion rings, in both
 *     RDA and QPL (bounce) addressing
 *   - multi-buffer RX packet reassembly (pbuf_cat to end-of-packet)
 *   - device-requested reset handling (e.g. live migration)
 *   - SO_INCOMING_NAPI_ID: per-queue napi_id tagged on received packets
 *   - TX QPL integer-overflow fix (was: head < qpl_head wrap)
 *   - TX completion timeout / watchdog + deferred reset (prevents
 *     silent TX hang)
 *   - ring-size backoff under memory pressure, restored to device
 *     sizes on each (re)setup
 *   - cacheline padding uses DEFAULT_CACHELINE_SIZE
 */
void init_gve(kernel_heaps kh);
