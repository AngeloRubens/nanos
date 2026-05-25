/* Google Virtual Ethernet (gVNIC) driver — multi-queue port
 *
 * Ported from src/drivers/gve.c; adds:
 *   - multi-queue TX/RX (up to GVE_MAX_IO_QUEUES, limited by device,
 *     MSI-X vectors, CPU count, and the "io-queues" manifest option)
 *   - io-queues manifest support (same pattern as virtio_net)
 *   - TX QPL integer-overflow fix (was: head < qpl_head wrap)
 *   - TX completion timeout / watchdog (prevents silent TX hang)
 *   - cacheline padding uses DEFAULT_CACHELINE_SIZE
 */
void init_gve(kernel_heaps kh);
