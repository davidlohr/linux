// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe Unordered IO (UIO) support (PCIe 6.x, sec 6.34).
 *
 * UIO transactions carry no fabric ordering guarantees; the requester
 * hardware enforces dependent ordering (data before flag/doorbell) by
 * accounting UIO completions. The kernel's job is capability discovery,
 * transport validation, routing (VC) programming and revocation - it
 * never orders, fences or waits for UIO traffic.
 *
 * The config space surface (per the UIO ECN, final 2023-03-16):
 *
 *  - Device 3 Extended Capability: DevCap3 UIO Mem RdWr Completer (bit
 *    10) and Requester (bit 11) Supported; DevCtl3 UIO Requester Enable
 *    (bit 7, effective only with Bus Master Enable) and UIO Request
 *    256B Boundary Disable (bit 8). There is no completer enable and no
 *    outstanding-request status anywhere.
 *
 *  - Streamlined Virtual Channel (SVC) Extended Capability (0x35),
 *    required on every port that routes UIO. UIO TLPs travel only on
 *    VCs whose SVC resource selects a UIO protocol; if UIO is
 *    supported, VC3 must be UIO capable, and a second UIO VC must be
 *    VC4. SVC and VC/MFVC are mutually exclusive, arbitrated by the
 *    one-way (per boot) Use VC/MFVC status bit.
 *
 * Copyright (C) 2026 Davidlohr Bueso <dave@stgolabs.net>
 */

#define dev_fmt(fmt) "UIO: " fmt

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/pci.h>
#include <linux/pci-uio.h>
#include <linux/seq_file.h>

#include "pci.h"

/**
 * pci_uio_init - discover UIO capabilities of a function
 * @pdev: PCI device to initialize
 *
 * Called from pci_init_capabilities(). Parses the UIO role capabilities
 * from Device Capabilities 3 and locates the SVC capability. Discovery
 * only: no role is enabled here. Role enablement is exclusively owned
 * by UIO route lifetime.
 */
void pci_uio_init(struct pci_dev *pdev)
{
	u32 cap, ctl;

	if (!pci_is_pcie(pdev))
		return;

	if (pdev->dev3_cap) {
		pci_read_config_dword(pdev, pdev->dev3_cap + PCI_DEV3_CAP,
				      &cap);
		pdev->uio_cpl_capable = !!(cap & PCI_DEV3_CAP_UIO_MEM_CPL);
		pdev->uio_req_capable = !!(cap & PCI_DEV3_CAP_UIO_MEM_REQ);
	}

	/*
	 * Requester emission permission is route-object owned. Clear any
	 * leftover enable (firmware handoff, kexec): an enabled requester
	 * with no validated route invites TLPs the fabric cannot deliver.
	 */
	if (pdev->uio_req_capable) {
		pci_read_config_dword(pdev, pdev->dev3_cap + PCI_DEV3_CTL,
				      &ctl);
		if (ctl & PCI_DEV3_CTL_UIO_REQ_EN) {
			ctl &= ~PCI_DEV3_CTL_UIO_REQ_EN;
			pci_write_config_dword(pdev,
					       pdev->dev3_cap + PCI_DEV3_CTL,
					       ctl);
		}
	}

	/*
	 * SVC placement rules: Upstream Ports implement it only in
	 * function 0, and SR-IOV VFs must not implement it at all.
	 */
	if (!pdev->is_virtfn)
		pdev->svc_cap = pci_find_ext_capability(pdev,
							PCI_EXT_CAP_ID_SVC);
}

/* Number of SVC VC resources (extended count is on top of VC0) */
static int pci_svc_nr_resources(struct pci_dev *pdev)
{
	u32 cap1;

	pci_read_config_dword(pdev, pdev->svc_cap + PCI_SVC_PORT_CAP1, &cap1);
	return FIELD_GET(PCI_SVC_PORT_CAP1_EVCC, cap1) + 1;
}

/*
 * pci_uio_svc_resource - find the SVC VC resource usable for UIO
 * @pdev: port to scan
 *
 * Returns the index of the first SVC VC resource whose Protocols
 * Supported field permits UIO (UIO-only, or UIO/restricted non-UIO),
 * or -ENODEV. Per the mandatory TC/VC assignments this is VC3 on
 * conformant hardware (a second UIO VC, if any, is VC4).
 */
int pci_uio_svc_resource(struct pci_dev *pdev)
{
	int i, nres;
	u32 rcap;

	if (!pdev->svc_cap)
		return -ENODEV;

	nres = pci_svc_nr_resources(pdev);
	for (i = 0; i < nres; i++) {
		pci_read_config_dword(pdev,
				      pdev->svc_cap + PCI_SVC_RES_CAP(i),
				      &rcap);
		switch (FIELD_GET(PCI_SVC_RES_CAP_PROTOCOLS, rcap)) {
		case PCI_SVC_PROTOCOL_UIO:
		case PCI_SVC_PROTOCOL_UIO_OR_NON:
			return i;
		}
	}
	return -ENODEV;
}

/**
 * pci_uio_routing_capable - can this port route UIO traffic?
 * @pdev: the port to interrogate
 *
 * True when the port implements SVC with at least one VC resource
 * whose supported protocols permit UIO. Capability only - says
 * nothing about the VC being configured or enabled.
 */
bool pci_uio_routing_capable(struct pci_dev *pdev)
{
	return pci_uio_svc_resource(pdev) >= 0;
}
EXPORT_SYMBOL_GPL(pci_uio_routing_capable);

/* Diagnostics only, never a production control surface */
static int pci_uio_capabilities_show(struct seq_file *m, void *unused)
{
	struct pci_dev *pdev = NULL;

	for_each_pci_dev(pdev) {
		bool req = pci_uio_requester_capable(pdev);
		bool cpl = pci_uio_completer_capable(pdev);
		bool routing = pci_uio_routing_capable(pdev);

		if (!req && !cpl && !routing)
			continue;

		seq_printf(m, "%s:%s%s%s\n", pci_name(pdev),
			   req ? " requester" : "",
			   cpl ? " completer" : "",
			   routing ? " routing" : "");
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(pci_uio_capabilities);

static struct dentry *pci_uio_debugfs;

static int __init pci_uio_debugfs_init(void)
{
	pci_uio_debugfs = debugfs_create_dir("pci_uio", NULL);
	debugfs_create_file("capabilities", 0444, pci_uio_debugfs, NULL,
			    &pci_uio_capabilities_fops);
	return 0;
}
late_initcall(pci_uio_debugfs_init);
