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

#endif
