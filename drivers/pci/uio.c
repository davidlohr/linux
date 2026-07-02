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
#include <linux/dma-map-ops.h>
#include <linux/iopoll.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/pci-ats.h>
#include <linux/pci-p2pdma.h>
#include <linux/pci-uio.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/xarray.h>

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

/**
 * pci_uio_vc_ownership - who owns UIO VC/SVC programming below a bridge
 * @bridge: host bridge to interrogate
 *
 * Ownership is a per host bridge deployment property. The ECN's
 * multi-host guidance anticipates fabrics where a Fabric Manager has
 * preconfigured TC/VC assignments that system software must preserve
 * (PLATFORM ownership: the kernel verifies and only flips the requester
 * enable); single-host deployments let the OS program SVC. Negotiation
 * via _OSC is not yet defined for UIO, so ownership currently defaults
 * to OS unless the platform withdraws it (fail closed on no bridge).
 */
enum pci_uio_vc_owner pci_uio_vc_ownership(struct pci_host_bridge *bridge)
{
	if (!bridge)
		return PCI_UIO_VC_OWNER_NONE;

	return bridge->native_svc ? PCI_UIO_VC_OWNER_OS :
				    PCI_UIO_VC_OWNER_PLATFORM;
}
EXPORT_SYMBOL_GPL(pci_uio_vc_ownership);

/*
 * Route infrastructure.
 *
 * pci_uio_lock protects the route list, the per-device enable counts
 * and all role state transitions. Enablement ordering is normative:
 * completer side and routing hops before the requester on commit,
 * requester first on teardown - a requester left enabled past its
 * routing emits TLPs the fabric cannot deliver; a completer disabled
 * under live mappings produces completions nobody is counting.
 */
static DEFINE_MUTEX(pci_uio_lock);
static LIST_HEAD(pci_uio_routes);

/* Per-device role enable reference counts */
struct pci_uio_dev_state {
	int vc_users;	/* SVC UIO VC programmed/verified */
	u8 vc_tc;	/* the TC the enabled UIO VC maps */
	int req_users;	/* DevCtl3 UIO Requester Enable */
	int hdm_users;	/* CXL port DVSEC UIO To HDM Enable */
};

static DEFINE_XARRAY(pci_uio_dev_states);

static struct pci_uio_dev_state *pci_uio_dev_state(struct pci_dev *pdev)
{
	struct pci_uio_dev_state *state;

	state = xa_load(&pci_uio_dev_states, (unsigned long)pdev);
	if (state)
		return state;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return NULL;

	if (xa_err(xa_store(&pci_uio_dev_states, (unsigned long)pdev, state,
			    GFP_KERNEL))) {
		kfree(state);
		return NULL;
	}
	return state;
}

/* The link between @pdev and its upstream bridge is in flit mode */
static bool pci_uio_link_flit(struct pci_dev *pdev)
{
	struct pci_dev *bridge = pci_upstream_bridge(pdev);
	u16 lnksta2 = 0;

	if (!bridge)
		return true;	/* no PCIe link above (root bus) */

	pcie_capability_read_word(bridge, PCI_EXP_LNKSTA2, &lnksta2);
	return lnksta2 & PCI_EXP_LNKSTA2_FLIT;
}

/*
 * SVC bring-up on one port, under OS ownership.
 *
 * Adopting SVC is one-way per boot: clearing the Use VC/MFVC status bit
 * (RW1C) clears every VC/MFVC VC Enable, enables SVC VC0, and stays
 * clear until the next Conventional Reset. It is therefore done here,
 * when a route actually commits, never as a probing side effect.
 */
static int pci_uio_svc_program(struct pci_dev *pdev, u8 tc)
{
	int res = pci_uio_svc_resource(pdev);
	u16 svc = pdev->svc_cap;
	u32 val;

	if (res < 0)
		return -EOPNOTSUPP;

	pci_read_config_dword(pdev, svc + PCI_SVC_PORT_STATUS, &val);
	if (val & PCI_SVC_PORT_STATUS_USE_VC_MFVC) {
		pci_write_config_dword(pdev, svc + PCI_SVC_PORT_STATUS,
				       PCI_SVC_PORT_STATUS_USE_VC_MFVC);
		pci_read_config_dword(pdev, svc + PCI_SVC_PORT_STATUS, &val);
		if (val & PCI_SVC_PORT_STATUS_USE_VC_MFVC) {
			pci_dbg(pdev, "SVC adoption failed\n");
			return -EOPNOTSUPP;
		}
	}

	pci_read_config_dword(pdev, svc + PCI_SVC_RES_CTRL(res), &val);
	val &= ~(PCI_SVC_RES_CTRL_TCVC_MAP | PCI_SVC_RES_CTRL_PROTOCOL);
	val |= FIELD_PREP(PCI_SVC_RES_CTRL_TCVC_MAP, BIT(tc));
	val |= FIELD_PREP(PCI_SVC_RES_CTRL_PROTOCOL, PCI_SVC_PROTOCOL_UIO);
	val |= PCI_SVC_RES_CTRL_VC_ENABLE;
	pci_write_config_dword(pdev, svc + PCI_SVC_RES_CTRL(res), val);

	pci_read_config_dword(pdev, svc + PCI_SVC_PORT_CTRL, &val);
	val |= PCI_SVC_PORT_CTRL_VC_EN_COMPLETED;
	pci_write_config_dword(pdev, svc + PCI_SVC_PORT_CTRL, val);

	return 0;
}

/*
 * FC negotiation for a VC completes only once BOTH Link partners have
 * enabled it (sec 7.9.29.7/.8), so waiting is a separate phase, run
 * only after every port on the path has been programmed. Polling a
 * port whose partner is not yet enabled would only ever time out.
 */
static int pci_uio_svc_wait(struct pci_dev *pdev)
{
	int res = pci_uio_svc_resource(pdev);
	u32 val;

	if (res < 0)
		return -EOPNOTSUPP;

	for (int i = 0; i < 100; i++) {
		pci_read_config_dword(pdev,
				      pdev->svc_cap + PCI_SVC_RES_STATUS(res),
				      &val);
		if (!(val & PCI_SVC_RES_STATUS_NEGO_PENDING))
			return 0;
		fsleep(1000);
	}

	pci_dbg(pdev, "UIO VC negotiation timed out\n");
	return -ETIMEDOUT;
}

/* Verify firmware's SVC programming without touching it */
static int pci_uio_svc_verify(struct pci_dev *pdev, u8 tc)
{
	int res = pci_uio_svc_resource(pdev);
	u32 val;

	if (res < 0)
		return -EOPNOTSUPP;

	pci_read_config_dword(pdev, pdev->svc_cap + PCI_SVC_PORT_STATUS, &val);
	if (val & PCI_SVC_PORT_STATUS_USE_VC_MFVC)
		return -EOPNOTSUPP;

	pci_read_config_dword(pdev, pdev->svc_cap + PCI_SVC_RES_CTRL(res),
			      &val);
	switch (FIELD_GET(PCI_SVC_RES_CTRL_PROTOCOL, val)) {
	case PCI_SVC_PROTOCOL_UIO:
	case PCI_SVC_PROTOCOL_UIO_OR_NON:
		break;
	default:
		return -EOPNOTSUPP;
	}
	if (!(val & PCI_SVC_RES_CTRL_VC_ENABLE))
		return -EOPNOTSUPP;
	if (!(FIELD_GET(PCI_SVC_RES_CTRL_TCVC_MAP, val) & BIT(tc)))
		return -EOPNOTSUPP;

	return 0;
}

static int pci_uio_svc_enable(struct pci_dev *pdev, enum pci_uio_vc_owner owner,
			      u8 tc)
{
	struct pci_uio_dev_state *state = pci_uio_dev_state(pdev);
	int rc;

	if (!state)
		return -ENOMEM;

	if (state->vc_users) {
		/*
		 * The enabled UIO VC maps exactly one TC; a route asking
		 * for a different one cannot share this port's VC state.
		 */
		if (state->vc_tc != tc)
			return -EBUSY;
		state->vc_users++;
		return 0;
	}

	if (owner == PCI_UIO_VC_OWNER_OS)
		rc = pci_uio_svc_program(pdev, tc);
	else
		rc = pci_uio_svc_verify(pdev, tc);
	if (rc)
		return rc;

	state->vc_tc = tc;
	state->vc_users = 1;
	return 0;
}

static void pci_uio_svc_disable(struct pci_dev *pdev,
				enum pci_uio_vc_owner owner)
{
	struct pci_uio_dev_state *state = pci_uio_dev_state(pdev);
	int res;
	u32 val;

	if (!state || WARN_ON(!state->vc_users))
		return;

	if (--state->vc_users)
		return;

	if (owner != PCI_UIO_VC_OWNER_OS || pci_dev_is_disconnected(pdev))
		return;

	res = pci_uio_svc_resource(pdev);
	if (res < 0)
		return;

	pci_read_config_dword(pdev, pdev->svc_cap + PCI_SVC_RES_CTRL(res),
			      &val);
	val &= ~PCI_SVC_RES_CTRL_VC_ENABLE;
	pci_write_config_dword(pdev, pdev->svc_cap + PCI_SVC_RES_CTRL(res),
			       val);
}

/*
 * CXL in-fabric UIO-to-HDM containment gate. Decoder-matched UIO
 * forwarding explicitly bypasses ACS (including egress control
 * vectors); the architected control is the UIO To HDM Enable bit in
 * the port's CXL Extensions DVSEC (default 0 => Completer Abort).
 */
static int pci_uio_hdm_gate_set(struct pci_dev *pdev, bool enable)
{
	u16 dvsec;
	u32 ctrl;

	dvsec = pci_find_dvsec_capability(pdev, PCI_VENDOR_ID_CXL,
					  PCI_DVSEC_CXL_PORT);
	if (!dvsec)
		return -EOPNOTSUPP;

	pci_read_config_dword(pdev, dvsec + PCI_DVSEC_CXL_PORT_CTL, &ctrl);
	if (enable)
		ctrl |= PCI_DVSEC_CXL_PORT_CTL_UIO_HDM_EN;
	else
		ctrl &= ~PCI_DVSEC_CXL_PORT_CTL_UIO_HDM_EN;
	pci_write_config_dword(pdev, dvsec + PCI_DVSEC_CXL_PORT_CTL, ctrl);

	return 0;
}

/*
 * The UIO To HDM Enable bit is RW only in Switch Downstream Ports;
 * everywhere else (including Root Ports, which forward peer UIO
 * subject to host-specific controls instead) it is RsvdP and
 * software must not set it (CXL r4.0 Table 8-32).
 */
static bool pci_uio_hop_is_dsp(struct pci_dev *pdev)
{
	return pci_pcie_type(pdev) == PCI_EXP_TYPE_DOWNSTREAM;
}

/*
 * A hop's UIO VC is programmed only when one of its Links is on the
 * traversed path: VC enablement is per-Link state owned by BOTH Link
 * partners (PCIe 6.4 sec 7.9.29.7/.8), and enabling a VC whose partner
 * is never configured leaves VC Negotiation Pending set forever.
 *
 * Downstream-type hops (Switch Downstream Ports, Root Ports) always
 * face a traversed Link - the one to the path's child below them; the
 * endpoints own the other side. An Upstream Port's external Link is
 * traversed only when the path continues above its switch, i.e. its
 * parent bridge is also a hop; a USP that tops an in-fabric path only
 * crosses the switch-internal bus and must be left alone.
 */
static bool pci_uio_hop_uses_link(struct pci_dev **hops, unsigned int n,
				  struct pci_dev *hop)
{
	struct pci_dev *parent;
	unsigned int i;

	if (pci_pcie_type(hop) != PCI_EXP_TYPE_UPSTREAM)
		return true;

	parent = pci_upstream_bridge(hop);
	for (i = 0; i < n; i++)
		if (hops[i] == parent)
			return true;
	return false;
}

static int pci_uio_hdm_gate_enable(struct pci_dev *pdev)
{
	struct pci_uio_dev_state *state = pci_uio_dev_state(pdev);
	int rc;

	if (!state)
		return -ENOMEM;

	if (state->hdm_users) {
		state->hdm_users++;
		return 0;
	}

	rc = pci_uio_hdm_gate_set(pdev, true);
	if (rc)
		return rc;

	state->hdm_users = 1;
	return 0;
}

static void pci_uio_hdm_gate_disable(struct pci_dev *pdev)
{
	struct pci_uio_dev_state *state = pci_uio_dev_state(pdev);

	if (!state || !state->hdm_users)
		return;

	if (--state->hdm_users)
		return;

	if (!pci_dev_is_disconnected(pdev))
		pci_uio_hdm_gate_set(pdev, false);
}

/* Requester emission permission: DevCtl3[7], effective only with BME */
static int pci_uio_requester_enable(struct pci_dev *pdev)
{
	struct pci_uio_dev_state *state = pci_uio_dev_state(pdev);
	u32 ctl;

	if (!state)
		return -ENOMEM;

	if (state->req_users) {
		state->req_users++;
		return 0;
	}

	pci_read_config_dword(pdev, pdev->dev3_cap + PCI_DEV3_CTL, &ctl);
	ctl |= PCI_DEV3_CTL_UIO_REQ_EN;
	pci_write_config_dword(pdev, pdev->dev3_cap + PCI_DEV3_CTL, ctl);
	pdev->uio_req_enabled = 1;

	if (!pdev->is_busmaster)
		pci_info(pdev, "UIO requester enabled without Bus Master Enable\n");

	state->req_users = 1;
	return 0;
}

static void pci_uio_requester_disable(struct pci_dev *pdev)
{
	struct pci_uio_dev_state *state = pci_uio_dev_state(pdev);
	u32 ctl;

	if (!state || WARN_ON(!state->req_users))
		return;

	if (--state->req_users)
		return;

	pdev->uio_req_enabled = 0;
	if (pci_dev_is_disconnected(pdev))
		return;

	pci_read_config_dword(pdev, pdev->dev3_cap + PCI_DEV3_CTL, &ctl);
	ctl &= ~PCI_DEV3_CTL_UIO_REQ_EN;
	pci_write_config_dword(pdev, pdev->dev3_cap + PCI_DEV3_CTL, ctl);
}

/*
 * Tree topology path finding: lowest common ancestor of @a and @b via
 * upstream walks, mirroring the p2pdma distance calculation. No
 * references are taken on returned bridges: the caller holds
 * references on the child devices, which pin the upstream chain.
 */
static struct pci_dev *pci_uio_find_lca(struct pci_dev *a, struct pci_dev *b)
{
	struct pci_dev *iter_a, *iter_b;

	for (iter_a = pci_upstream_bridge(a); iter_a;
	     iter_a = pci_upstream_bridge(iter_a)) {
		for (iter_b = pci_upstream_bridge(b); iter_b;
		     iter_b = pci_upstream_bridge(iter_b)) {
			if (iter_a == iter_b)
				return iter_a;
		}
	}
	return NULL;
}

struct pci_uio_path {
	struct pci_dev **hops;
	unsigned int nr_hops;
	unsigned int max_hops;
	unsigned long flags;
};

static bool pci_uio_path_contains(struct pci_uio_path *path,
				  struct pci_dev *pdev)
{
	unsigned int i;

	for (i = 0; i < path->nr_hops; i++)
		if (path->hops[i] == pdev)
			return true;
	return false;
}

static int pci_uio_path_add(struct pci_uio_path *path, struct pci_dev *pdev)
{
	if (pci_uio_path_contains(path, pdev))
		return 0;

	if (path->nr_hops == path->max_hops) {
		struct pci_dev **hops;

		path->max_hops = max(16U, path->max_hops * 2);
		hops = krealloc_array(path->hops, path->max_hops,
				      sizeof(*hops), GFP_KERNEL);
		if (!hops)
			return -ENOMEM;
		path->hops = hops;
	}
	path->hops[path->nr_hops++] = pci_dev_get(pdev);
	return 0;
}

/*
 * Add every bridge from @child (exclusive) up to @lca (inclusive).
 */
static int pci_uio_path_add_chain(struct pci_uio_path *path,
				  struct pci_dev *child, struct pci_dev *lca)
{
	struct pci_dev *iter;
	int rc;

	for (iter = pci_upstream_bridge(child); iter;
	     iter = pci_upstream_bridge(iter)) {
		rc = pci_uio_path_add(path, iter);
		if (rc)
			return rc;
		if (iter == lca)
			return 0;
	}
	return -ENODEV;
}

static void pci_uio_path_free(struct pci_uio_path *path)
{
	unsigned int i;

	for (i = 0; i < path->nr_hops; i++)
		pci_dev_put(path->hops[i]);
	kfree(path->hops);
	path->hops = NULL;
	path->nr_hops = 0;
}

/*
 * Validate the (requester, target) pair and accumulate the traversed
 * bridges. All-or-nothing across the target set is the caller's
 * structure: any target failing fails the route.
 */
static int pci_uio_path_validate(struct pci_uio_path *path,
				 struct pci_dev *requester,
				 struct pci_dev *target, bool cxl_hdm)
{
	struct pci_dev *lca, *iter;
	int rc;

	lca = pci_uio_find_lca(requester, target);
	if (!lca) {
		pci_dbg(requester, "UIO: no in-hierarchy path to %s\n",
			pci_name(target));
		return -EOPNOTSUPP;
	}

	/*
	 * Cross root port peer traffic is forwarded by the RC "subject
	 * to host-specific access controls" - host mediated policy this
	 * implementation does not claim yet. Fail closed.
	 */
	if (pci_pcie_type(lca) == PCI_EXP_TYPE_ROOT_PORT)
		path->flags |= PCI_UIO_ROUTE_THRU_RP;
	else
		path->flags |= PCI_UIO_ROUTE_IN_FABRIC;

	rc = pci_uio_path_add_chain(path, requester, lca);
	if (rc)
		return rc;
	rc = pci_uio_path_add_chain(path, target, lca);
	if (rc)
		return rc;

	/*
	 * Every traversed link is in flit mode. The hop directly below
	 * an Upstream Port LCA connects over the switch's internal bus,
	 * not a link (sampling "the link above" there would read the
	 * USP's own upstream link, which an in-fabric route never
	 * traverses) - skip it.
	 */
	for (iter = requester; iter != lca;
	     iter = pci_upstream_bridge(iter)) {
		if (pci_upstream_bridge(iter) == lca &&
		    pci_pcie_type(lca) == PCI_EXP_TYPE_UPSTREAM)
			break;
		if (!pci_uio_link_flit(iter)) {
			pci_dbg(requester, "UIO: %s upstream link not in flit mode\n",
				pci_name(iter));
			return -EOPNOTSUPP;
		}
	}
	for (iter = target; iter != lca; iter = pci_upstream_bridge(iter)) {
		if (pci_upstream_bridge(iter) == lca &&
		    pci_pcie_type(lca) == PCI_EXP_TYPE_UPSTREAM)
			break;
		if (!pci_uio_link_flit(iter)) {
			pci_dbg(requester, "UIO: %s upstream link not in flit mode\n",
				pci_name(iter));
			return -EOPNOTSUPP;
		}
	}

	/*
	 * Containment gate. For plain PCIe paths this is ACS: any
	 * P2P redirect on the path sends the traffic through the host
	 * bridge, which is a different transfer plan, not this route.
	 * For CXL UIO-to-HDM this is explicitly NOT ACS (forwarding
	 * bypasses it): the UIO To HDM Enable port gate is programmed
	 * at commit instead, and the requester must be ATS capable
	 * because HDM decoders match translated addresses only.
	 */
	if (!cxl_hdm) {
		for (iter = pci_upstream_bridge(requester); iter != lca;
		     iter = pci_upstream_bridge(iter)) {
			if (pci_bridge_has_acs_redir(iter))
				return -EOPNOTSUPP;
		}
		for (iter = pci_upstream_bridge(target); iter != lca;
		     iter = pci_upstream_bridge(iter)) {
			if (pci_bridge_has_acs_redir(iter))
				return -EOPNOTSUPP;
		}
	} else if (!pci_ats_supported(requester)) {
		pci_dbg(requester, "UIO: HDM target requires ATS capable requester\n");
		return -EOPNOTSUPP;
	}

	return 0;
}

static void pci_uio_route_limits(struct pci_uio_route *route,
				 u32 interleave_granularity)
{
	struct pci_uio_limits *l = &route->limits;
	unsigned int i;
	int mps = pcie_get_mps(route->requester);

	for (i = 0; i < route->nr_hops; i++)
		mps = min(mps, pcie_get_mps(route->hops[i]));
	for (i = 0; i < route->nr_targets; i++)
		mps = min(mps, pcie_get_mps(route->targets[i]));

	l->max_write_size = mps;
	l->max_read_size = pcie_get_readrq(route->requester);
	l->boundary = 256;
	if (interleave_granularity)
		l->boundary = min(l->boundary, interleave_granularity);
	l->update_granule = 64;
	l->large_uio = false;
}

/* Drop all roles this route holds, in requester-first order */
static void pci_uio_route_drop_roles(struct pci_uio_route *route,
				     enum pci_uio_vc_owner owner)
{
	unsigned int i;

	lockdep_assert_held(&pci_uio_lock);

	if (!route->roles_active)
		return;
	route->roles_active = false;

	pci_uio_requester_disable(route->requester);
	pci_uio_svc_disable(route->requester, owner);

	for (i = 0; i < route->nr_hops; i++) {
		if (pci_uio_hop_uses_link(route->hops, route->nr_hops,
					  route->hops[i]))
			pci_uio_svc_disable(route->hops[i], owner);
		if (route->provider->type == P2PDMA_PROVIDER_CXL_HDM &&
		    pci_uio_hop_is_dsp(route->hops[i]))
			pci_uio_hdm_gate_disable(route->hops[i]);
	}

	for (i = 0; i < route->nr_targets; i++)
		pci_uio_svc_disable(route->targets[i], owner);

	dma_debug_uio_range_del(&route->requester->dev,
				route->provider->base + route->offset,
				route->len);
}

static void pci_uio_route_revoke_locked(struct pci_uio_route *route)
{
	struct pci_host_bridge *bridge =
		pci_find_host_bridge(route->requester->bus);

	lockdep_assert_held(&pci_uio_lock);

	if (route->revoked)
		return;

	route->generation++;
	WRITE_ONCE(route->revoked, true);

	if (route->ops && route->ops->quiesce)
		route->ops->quiesce(route, route->ops_priv);

	pci_uio_route_drop_roles(route, pci_uio_vc_ownership(bridge));

	if (route->ops && route->ops->revoke)
		route->ops->revoke(route, route->ops_priv);
}

/**
 * pci_uio_route_get - validate and commit a UIO transport binding
 * @requester: device that will emit UIO requests
 * @provider: peer memory provider terminating them
 * @offset: subrange start within @provider
 * @len: subrange length
 * @req: acquisition parameters, see &struct pci_uio_route_req
 * @route: on success, the committed route (reference held)
 *
 * Validates then commits, all-or-nothing with unwind: requester role
 * capability, provider completer declaration, per-target paths (flit
 * mode on every link, SVC routing capability and UIO VC state on every
 * hop, the containment gate for the path type), then enables roles
 * leaf-to-requester.
 *
 * Sleeps; callable from probe/ioctl context only.
 *
 * Return: 0 on success, -EOPNOTSUPP when any capability, flit or VC
 * requirement is missing (the failing hop is named via pci_dbg),
 * -ENODEV when an actor is going away, -ENOMEM.
 */
int pci_uio_route_get(struct pci_dev *requester,
		      struct p2pdma_provider *provider,
		      phys_addr_t offset, resource_size_t len,
		      const struct pci_uio_route_req *req,
		      struct pci_uio_route **route)
{
	static const struct pci_uio_route_req default_req;
	struct pci_host_bridge *bridge;
	struct p2p_target_set *tset = NULL;
	struct pci_uio_path path = {};
	struct pci_uio_route *r;
	enum pci_uio_vc_owner owner;
	u32 granularity = 0;
	unsigned int i, enabled_hops = 0, enabled_tgts = 0, enabled_gates = 0;
	u8 tc;
	int rc;

	*route = NULL;
	if (!req)
		req = &default_req;
	if (req->policy == PCI_UIO_FORBIDDEN)
		return -EINVAL;
	if (req->tc > 7 || req->vc > 7)
		return -EINVAL;
	tc = req->tc ?: 3;	/* default UIO TC/VC assignment: TC3/VC3 */

	if (!len || offset > provider->size || len > provider->size - offset)
		return -EINVAL;

	if (!pci_uio_requester_capable(requester))
		return -EOPNOTSUPP;
	if (!(provider->flags & P2PDMA_PROVIDER_UIO_COMPLETER))
		return -EOPNOTSUPP;

	bridge = pci_find_host_bridge(requester->bus);
	owner = pci_uio_vc_ownership(bridge);
	if (owner == PCI_UIO_VC_OWNER_NONE)
		return -EOPNOTSUPP;

	/*
	 * Obtain the interleave target set: a route is a binding to
	 * every endpoint the subrange decodes to, or to none.
	 */
	if (provider->range_validate) {
		rc = provider->range_validate(provider, &requester->dev,
					      offset, len, &tset);
		if (rc)
			return rc;
		granularity = tset->interleave_granularity;
	}

	r = kzalloc(sizeof(*r), GFP_KERNEL);
	if (!r) {
		rc = -ENOMEM;
		goto free_tset;
	}

	kref_init(&r->ref);
	r->requester = pci_dev_get(requester);
	r->provider = provider;
	/*
	 * A revoked route may linger (holder yet to put it) after the
	 * provider's device is torn down; pin the owner so provider
	 * memory stays valid for diagnostics and teardown bookkeeping.
	 */
	get_device(provider->owner);
	r->offset = offset;
	r->len = len;
	r->tc = tc;
	r->ops = req->ops;
	r->ops_priv = req->ops_priv;

	/* Pin the target set */
	if (tset) {
		r->targets = kcalloc(tset->nr_targets, sizeof(*r->targets),
				     GFP_KERNEL);
		if (!r->targets) {
			rc = -ENOMEM;
			goto free_route;
		}
		r->nr_targets = tset->nr_targets;
		for (i = 0; i < r->nr_targets; i++)
			r->targets[i] = pci_dev_get(tset->targets[i]);
	} else {
		if (!dev_is_pci(provider->owner)) {
			rc = -EINVAL;
			goto free_route;
		}
		r->targets = kcalloc(1, sizeof(*r->targets), GFP_KERNEL);
		if (!r->targets) {
			rc = -ENOMEM;
			goto free_route;
		}
		r->nr_targets = 1;
		r->targets[0] = pci_dev_get(to_pci_dev(provider->owner));
	}

	/* Validate every path; collect the union of traversed hops */
	for (i = 0; i < r->nr_targets; i++) {
		rc = pci_uio_path_validate(&path, requester, r->targets[i],
				provider->type == P2PDMA_PROVIDER_CXL_HDM);
		if (rc)
			goto free_path;
	}

	/* Every hop must be able to route UIO before anything is enabled */
	for (i = 0; i < path.nr_hops; i++) {
		if (!pci_uio_routing_capable(path.hops[i])) {
			pci_dbg(requester, "UIO: %s cannot route UIO\n",
				pci_name(path.hops[i]));
			rc = -EOPNOTSUPP;
			goto free_path;
		}
	}

	/*
	 * Endpoint conformance. VC enablement is a per-Link property of
	 * both Link partners, and Ports containing Functions that
	 * generate or complete TC3 UIO traffic must implement SVC to
	 * map the TC (sec 2.5, 7.9.29) - so the requester and every
	 * target need a UIO-protocol VC resource of their own. (SVC of
	 * an Upstream Port lives in Function 0 and applies port-wide;
	 * multi-function requesters are expected to expose it there.)
	 * UIO Completers use 14-bit Tags exclusively; the DevCap3
	 * 14-Bit Tag Completer Supported bit MUST@FLIT be Set, so this
	 * check only screens out nonconformant hardware (sec 2.2.6.2,
	 * 7.7.9.2).
	 */
	if (pci_uio_svc_resource(requester) < 0) {
		pci_dbg(requester, "UIO: requester lacks a UIO VC\n");
		rc = -EOPNOTSUPP;
		goto free_path;
	}
	for (i = 0; i < r->nr_targets; i++) {
		struct pci_dev *t = r->targets[i];
		u32 cap3 = 0;

		if (!pci_uio_completer_capable(t) ||
		    pci_uio_svc_resource(t) < 0) {
			pci_dbg(requester, "UIO: %s cannot complete UIO\n",
				pci_name(t));
			rc = -EOPNOTSUPP;
			goto free_path;
		}
		pci_read_config_dword(t, t->dev3_cap + PCI_DEV3_CAP, &cap3);
		if (!(cap3 & PCI_DEV3_CAP_14BIT_TAG_CPL)) {
			pci_dbg(requester, "UIO: %s lacks 14-bit Tag Completer support\n",
				pci_name(t));
			rc = -EOPNOTSUPP;
			goto free_path;
		}
	}

	r->flags = path.flags;
	r->nr_hops = path.nr_hops;
	r->hops = path.hops;

	mutex_lock(&pci_uio_lock);

	/* Commit: completer side first, then routing, then requester */
	for (; enabled_tgts < r->nr_targets; enabled_tgts++) {
		rc = pci_uio_svc_enable(r->targets[enabled_tgts], owner, tc);
		if (rc) {
			pci_dbg(requester, "UIO: %s VC setup failed: %d\n",
				pci_name(r->targets[enabled_tgts]), rc);
			goto unwind_tgts;
		}
	}

	if (provider->type == P2PDMA_PROVIDER_CXL_HDM) {
		for (i = 0; i < r->nr_hops; i++) {
			if (!pci_uio_hop_is_dsp(r->hops[i]))
				continue;
			rc = pci_uio_hdm_gate_enable(r->hops[i]);
			if (rc)
				goto unwind_hdm;
			enabled_gates++;
		}
	}

	for (; enabled_hops < r->nr_hops; enabled_hops++) {
		if (!pci_uio_hop_uses_link(r->hops, r->nr_hops,
					   r->hops[enabled_hops]))
			continue;
		rc = pci_uio_svc_enable(r->hops[enabled_hops], owner, tc);
		if (rc) {
			pci_dbg(requester, "UIO: %s VC setup failed: %d\n",
				pci_name(r->hops[enabled_hops]), rc);
			goto unwind_svc;
		}
	}

	rc = pci_uio_svc_enable(requester, owner, tc);
	if (rc)
		goto unwind_svc;

	/*
	 * Every port on the path now has the UIO VC enabled; only from
	 * this point can FC negotiation complete on each Link (it needs
	 * both partners). Wait it out before granting emission.
	 */
	for (i = 0; i < r->nr_targets; i++) {
		rc = pci_uio_svc_wait(r->targets[i]);
		if (rc)
			goto unwind_req_svc;
	}
	for (i = 0; i < r->nr_hops; i++) {
		if (!pci_uio_hop_uses_link(r->hops, r->nr_hops, r->hops[i]))
			continue;
		rc = pci_uio_svc_wait(r->hops[i]);
		if (rc)
			goto unwind_req_svc;
	}
	rc = pci_uio_svc_wait(requester);
	if (rc)
		goto unwind_req_svc;

	rc = pci_uio_requester_enable(requester);
	if (rc)
		goto unwind_req_svc;

	/*
	 * VC identity across the path: report the requested VC or the
	 * architected default (VC3 backs the first UIO VC).
	 */
	r->vc = req->vc ?: 3;
	r->roles_active = true;
	pci_uio_route_limits(r, granularity);

	list_add(&r->node, &pci_uio_routes);
	/*
	 * Register debug coverage before dropping the lock: once the
	 * route is list-visible, a concurrent revocation would delete
	 * a registration that does not exist yet and never retry.
	 */
	dma_debug_uio_range_add(&requester->dev, provider->base + offset, len);
	/*
	 * A removal that ran before this route became list-visible could
	 * not have revoked it; re-check now that it can.
	 */
	for (i = 0; i < r->nr_targets; i++)
		if (pci_dev_is_disconnected(r->targets[i]))
			break;
	if (i < r->nr_targets || pci_dev_is_disconnected(requester))
		pci_uio_route_revoke_locked(r);
	mutex_unlock(&pci_uio_lock);

	p2p_target_set_put(tset);
	*route = r;
	return 0;

unwind_req_svc:
	pci_uio_svc_disable(requester, owner);
unwind_svc:
	while (enabled_hops--)
		if (pci_uio_hop_uses_link(r->hops, r->nr_hops,
					  r->hops[enabled_hops]))
			pci_uio_svc_disable(r->hops[enabled_hops], owner);
unwind_hdm:
	/*
	 * Disable only the gates this route enabled (they were taken in
	 * hops[] order): a blanket walk would steal reference counts,
	 * and eventually the live hardware gate, from sibling routes
	 * sharing a downstream port.
	 */
	for (i = 0; enabled_gates && i < r->nr_hops; i++)
		if (pci_uio_hop_is_dsp(r->hops[i])) {
			pci_uio_hdm_gate_disable(r->hops[i]);
			enabled_gates--;
		}
unwind_tgts:
	while (enabled_tgts--)
		pci_uio_svc_disable(r->targets[enabled_tgts], owner);
	mutex_unlock(&pci_uio_lock);
	path.hops = r->hops;
	path.nr_hops = r->nr_hops;
free_path:
	pci_uio_path_free(&path);
	r->hops = NULL;
	r->nr_hops = 0;
free_route:
	for (i = 0; i < r->nr_targets; i++)
		pci_dev_put(r->targets[i]);
	kfree(r->targets);
	put_device(r->provider->owner);
	pci_dev_put(r->requester);
	kfree(r);
free_tset:
	p2p_target_set_put(tset);
	return rc;
}
EXPORT_SYMBOL_GPL(pci_uio_route_get);

static void pci_uio_route_release(struct kref *ref)
{
	struct pci_uio_route *route = container_of(ref, struct pci_uio_route,
						   ref);
	struct pci_host_bridge *bridge =
		pci_find_host_bridge(route->requester->bus);
	unsigned int i;

	lockdep_assert_held(&pci_uio_lock);

	pci_uio_route_drop_roles(route, pci_uio_vc_ownership(bridge));
	list_del(&route->node);

	for (i = 0; i < route->nr_hops; i++)
		pci_dev_put(route->hops[i]);
	kfree(route->hops);
	for (i = 0; i < route->nr_targets; i++)
		pci_dev_put(route->targets[i]);
	kfree(route->targets);
	put_device(route->provider->owner);
	pci_dev_put(route->requester);
	kfree(route);
}

/**
 * pci_uio_route_put - drop a route reference
 * @route: route to release
 *
 * The final put drops the requester role, then routing/completer-side
 * roles leaf-ward, for roles no surviving route needs. The caller must
 * have quiesced its engines and unmapped all DMA mappings created
 * under the route beforehand.
 */
void pci_uio_route_put(struct pci_uio_route *route)
{
	if (!route)
		return;

	mutex_lock(&pci_uio_lock);
	kref_put(&route->ref, pci_uio_route_release);
	mutex_unlock(&pci_uio_lock);
}
EXPORT_SYMBOL_GPL(pci_uio_route_put);

static bool pci_uio_route_involves(struct pci_uio_route *route,
				   struct pci_dev *pdev)
{
	unsigned int i;

	if (route->requester == pdev)
		return true;
	for (i = 0; i < route->nr_hops; i++)
		if (route->hops[i] == pdev)
			return true;
	for (i = 0; i < route->nr_targets; i++)
		if (route->targets[i] == pdev)
			return true;
	return false;
}

/**
 * pci_uio_route_revoke_dev - kill every route involving a device
 * @pdev: device being reset, contained or removed
 *
 * Fabric events (DPC, AER recovery, FLR, hot remove) enter here. Marks
 * every route traversing @pdev revoked, bumps generations, invokes the
 * holder's quiesce+revoke ops and drops the roles no surviving route
 * needs. References drain via pci_uio_route_put().
 */
void pci_uio_route_revoke_dev(struct pci_dev *pdev)
{
	struct pci_uio_route *route;

	if (list_empty(&pci_uio_routes))
		return;

	mutex_lock(&pci_uio_lock);
	list_for_each_entry(route, &pci_uio_routes, node)
		if (pci_uio_route_involves(route, pdev))
			pci_uio_route_revoke_locked(route);
	mutex_unlock(&pci_uio_lock);
}
EXPORT_SYMBOL_GPL(pci_uio_route_revoke_dev);

static int pci_uio_revoke_walk(struct pci_dev *pdev, void *unused)
{
	struct pci_uio_route *route;

	list_for_each_entry(route, &pci_uio_routes, node)
		if (pci_uio_route_involves(route, pdev))
			pci_uio_route_revoke_locked(route);
	return 0;
}

/**
 * pci_uio_destroy_dev - final device teardown, reap per-device state
 * @pdev: device being destroyed
 *
 * Distinct from revocation: FLR/reset revoke routes but the device
 * survives and its enable-count state stays reusable. Only final
 * destruction reaps the state entry; every route involving @pdev was
 * revoked (counts driven to zero) at pci_stop_dev() time.
 */
void pci_uio_destroy_dev(struct pci_dev *pdev)
{
	struct pci_uio_dev_state *state;

	state = xa_erase(&pci_uio_dev_states, (unsigned long)pdev);
	kfree(state);
}

/**
 * pci_uio_route_revoke_subtree - kill routes touching a bus subtree
 * @bridge: bridge whose subordinate hierarchy is affected
 *
 * Error containment (DPC/AER recovery) affects everything below the
 * recovering bridge, including routes that never traverse the bridge
 * itself.
 */
void pci_uio_route_revoke_subtree(struct pci_dev *bridge)
{
	if (list_empty(&pci_uio_routes))
		return;

	mutex_lock(&pci_uio_lock);
	pci_uio_revoke_walk(bridge, NULL);
	if (bridge->subordinate)
		pci_walk_bus(bridge->subordinate, pci_uio_revoke_walk, NULL);
	mutex_unlock(&pci_uio_lock);
}

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

static int pci_uio_routes_show(struct seq_file *m, void *unused)
{
	struct pci_uio_route *route;
	unsigned int i;

	mutex_lock(&pci_uio_lock);
	list_for_each_entry(route, &pci_uio_routes, node) {
		seq_printf(m, "%s -> %s [%#llx +%#llx] tc%u vc%u gen%lu%s%s%s\n",
			   pci_name(route->requester),
			   dev_name(route->provider->owner),
			   (unsigned long long)route->offset,
			   (unsigned long long)route->len,
			   route->tc, route->vc, route->generation,
			   route->flags & PCI_UIO_ROUTE_IN_FABRIC ?
				" in-fabric" : "",
			   route->flags & PCI_UIO_ROUTE_THRU_RP ?
				" thru-rp" : "",
			   route->revoked ? " REVOKED" : "");
		for (i = 0; i < route->nr_hops; i++)
			seq_printf(m, "  hop %s\n",
				   pci_name(route->hops[i]));
	}
	mutex_unlock(&pci_uio_lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(pci_uio_routes);

static int pci_uio_vc_ownership_show(struct seq_file *m, void *unused)
{
	struct pci_host_bridge *prev = NULL;
	struct pci_dev *pdev = NULL;

	for_each_pci_dev(pdev) {
		struct pci_host_bridge *bridge;

		if (!pci_is_root_bus(pdev->bus))
			continue;
		bridge = pci_find_host_bridge(pdev->bus);
		if (bridge == prev)
			continue;
		prev = bridge;
		switch (pci_uio_vc_ownership(bridge)) {
		case PCI_UIO_VC_OWNER_OS:
			seq_printf(m, "%s: os\n", dev_name(&bridge->dev));
			break;
		case PCI_UIO_VC_OWNER_PLATFORM:
			seq_printf(m, "%s: platform\n",
				   dev_name(&bridge->dev));
			break;
		default:
			seq_printf(m, "%s: none\n", dev_name(&bridge->dev));
			break;
		}
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(pci_uio_vc_ownership);

static struct dentry *pci_uio_debugfs;

static int __init pci_uio_debugfs_init(void)
{
	pci_uio_debugfs = debugfs_create_dir("pci_uio", NULL);
	debugfs_create_file("capabilities", 0444, pci_uio_debugfs, NULL,
			    &pci_uio_capabilities_fops);
	debugfs_create_file("routes", 0444, pci_uio_debugfs, NULL,
			    &pci_uio_routes_fops);
	debugfs_create_file("vc_ownership", 0444, pci_uio_debugfs, NULL,
			    &pci_uio_vc_ownership_fops);
	return 0;
}
late_initcall(pci_uio_debugfs_init);
