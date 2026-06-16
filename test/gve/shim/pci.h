/* Host shim for <pci.h>: the PCI BAR is backed by the device model's
 * register/doorbell space (see harness.c). */
#ifndef GVE_HARNESS_SHIM_PCI_H
#define GVE_HARNESS_SHIM_PCI_H

#include <harness_runtime.h>

typedef struct pci_dev *pci_dev;

struct pci_bar {
    void *vaddr;     /* harness: points at the device model's register space */
    u64   size;
};

u32  pci_bar_read_4(struct pci_bar *b, u64 offset);
void pci_bar_write_4(struct pci_bar *b, u64 offset, u32 val);
int  pci_get_msix_count(pci_dev d);

/* For the full-lifecycle scenario, which links gve_main.c and drives
 * init_gve -> probe -> setup.  register_pci_driver records the probe so the
 * harness can apply it; interrupt setup is stubbed (the harness never takes
 * a real MSI-X). */
closure_type(pci_probe, boolean, pci_dev dev);
closure_type(pci_remove, void, pci_dev dev);
void register_pci_driver(pci_probe p, pci_remove remove);
int  pci_enable_msix(pci_dev dev);
u64  pci_setup_msix_aff(pci_dev dev, int msi_slot, thunk h, sstring name,
                        range cpu_affinity);
#define pci_setup_msix(dev, slot, handler, name) \
    pci_setup_msix_aff(dev, slot, handler, name, irange(0, 0))
void pci_teardown_msix(pci_dev dev, int msi_slot);
void pci_disable_msix(pci_dev dev);
void pci_bar_init(pci_dev dev, struct pci_bar *b, int bar, bytes offset,
                  bytes length);
void pci_bar_deinit(struct pci_bar *b);
void pci_enable_io_and_memory(pci_dev dev);
u16  pci_get_vendor(pci_dev dev);
u16  pci_get_device(pci_dev dev);

#endif
