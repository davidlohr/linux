/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCIe Unordered IO (UIO) support.
 *
 * UIO (PCIe 6.x, sec 6.34) lets a requester issue memory requests whose
 * completions carry no ordering guarantees; ordering of dependent writes
 * (data before flag/doorbell) is enforced by the requester hardware by
 * counting UIO completions. The kernel discovers, validates, routes and
 * revokes UIO transport - it never orders or fences it.
 *
 * Copyright (C) 2026 Davidlohr Bueso <dave@stgolabs.net>
 */

#ifndef _LINUX_PCI_UIO_H
#define _LINUX_PCI_UIO_H

#include <linux/pci.h>

#ifdef CONFIG_PCI_UIO
/*
 * Single device predicates. Building blocks only: possessing a
 * capability grants nothing. All UIO enablement is owned by the
 * route object (pci_uio_route_get() and friends).
 */
static inline bool pci_uio_requester_capable(struct pci_dev *pdev)
{
	return pdev->uio_req_capable;
}

static inline bool pci_uio_completer_capable(struct pci_dev *pdev)
{
	return pdev->uio_cpl_capable;
}

bool pci_uio_routing_capable(struct pci_dev *pdev);
#else /* CONFIG_PCI_UIO */
static inline bool pci_uio_requester_capable(struct pci_dev *pdev)
{
	return false;
}

static inline bool pci_uio_completer_capable(struct pci_dev *pdev)
{
	return false;
}

static inline bool pci_uio_routing_capable(struct pci_dev *pdev)
{
	return false;
}
#endif /* CONFIG_PCI_UIO */

#endif /* _LINUX_PCI_UIO_H */
