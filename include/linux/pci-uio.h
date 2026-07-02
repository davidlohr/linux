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

#include <linux/kref.h>
#include <linux/pci.h>

struct p2pdma_provider;

/**
 * enum pci_uio_policy - consumer stance on UIO transport
 * @PCI_UIO_FORBIDDEN: never attempt UIO; classify ordered transport only.
 * @PCI_UIO_PREFERRED: attempt UIO; on failure fall back to a NEW ordered
 *	mapping decision. The fallback is a different transfer plan (for
 *	CXL HDM it may change the address form to host-mediated), never a
 *	silently degraded UIO plan.
 * @PCI_UIO_REQUIRED: no route means hard failure. No ordered result is
 *	offered. Correct default for CXL HDM-DB Direct P2P, where the
 *	host-mediated alternative defeats the point.
 */
enum pci_uio_policy {
	PCI_UIO_FORBIDDEN = 0,
	PCI_UIO_PREFERRED,
	PCI_UIO_REQUIRED,
};

/**
 * enum pci_uio_vc_owner - who programs UIO VC/SVC state
 * @PCI_UIO_VC_OWNER_PLATFORM: firmware programs VC/TC; kernel verifies.
 * @PCI_UIO_VC_OWNER_OS: kernel may program SVC.
 * @PCI_UIO_VC_OWNER_NONE: unclaimed; no route ever validates.
 */
enum pci_uio_vc_owner {
	PCI_UIO_VC_OWNER_PLATFORM,
	PCI_UIO_VC_OWNER_OS,
	PCI_UIO_VC_OWNER_NONE,
};

/**
 * struct pci_uio_limits - negotiated per-route request shaping
 * @max_write_size: largest UIO Memory Write payload on this route.
 * @max_read_size: largest UIO Memory Read request on this route.
 * @boundary: no UIO Request may cross this naturally aligned boundary.
 *	256B while DevCtl3 256B Boundary Disable is clear (always, in
 *	this implementation); further clamped to the target interleave
 *	granularity - a request straddling an interleave boundary is
 *	forwarded to the start-address owner and Completer Aborted.
 * @update_granule: 64B, normative: per-64B-block all-or-nothing
 *	visibility, consistent for all readers of a block.
 * @large_uio: mirrors DevCtl3[8]; never set by this implementation.
 */
struct pci_uio_limits {
	u32	max_write_size;
	u32	max_read_size;
	u32	boundary;
	u32	update_granule;
	bool	large_uio;
};

struct pci_uio_route;

/**
 * struct pci_uio_route_ops - revocation contract with the route holder
 * @quiesce: stop new submissions and drain all outstanding UIO
 *	completions for traffic on this route. Dependent flag/doorbell
 *	writes for failed batches must be suppressed. May sleep. Must be
 *	idempotent. Called with the UIO core lock held; must not call
 *	back into pci_uio_route_get()/put().
 * @revoke: the route is gone (fabric event, provider teardown, admin
 *	action). All DMA mappings covered by it must be treated as
 *	invalid. Called after a successful quiesce.
 *
 * Optional for trusted in-kernel consumers whose teardown already
 * quiesces their engines; mandatory when the requester is driven by
 * userspace (the kernel cannot trust userspace to stop DMA - DMABUF
 * exporters map move_notify onto these).
 */
struct pci_uio_route_ops {
	int  (*quiesce)(struct pci_uio_route *route, void *priv);
	void (*revoke)(struct pci_uio_route *route, void *priv);
};

/**
 * struct pci_uio_route_req - parameters for route acquisition
 * @policy: see &enum pci_uio_policy. FORBIDDEN never acquires.
 * @tc: requested traffic class, 0 selects the default (TC3).
 * @vc: requested VC ID, 0 selects the platform/hardware choice.
 * @ops: revocation callbacks, see &struct pci_uio_route_ops.
 * @ops_priv: passed to @ops callbacks.
 */
struct pci_uio_route_req {
	enum pci_uio_policy		policy;
	u8				tc;
	u8				vc;
	const struct pci_uio_route_ops	*ops;
	void				*ops_priv;
};

/**
 * struct pci_uio_route - a validated, revocable UIO transport binding
 *
 * The central UIO object: refcounted proof that a (requester, provider
 * subrange) pair holds validated UIO transport, with every role enable
 * on the path owned by this object's lifetime. UIO is a property of
 * (requester, path, target set, range) - never of a single device.
 *
 * @ref: reference count.
 * @requester: the device emitting UIO requests.
 * @provider: the peer memory provider terminating them.
 * @offset: validated subrange start within @provider.
 * @len: validated subrange length.
 * @tc: traffic class UIO requests must carry on this route.
 * @vc: virtual channel backing @tc across the path.
 * @limits: negotiated path minimum request shaping, see
 *	&struct pci_uio_limits.
 * @generation: bumped on fabric events; snapshot and compare for cheap
 *	staleness checks.
 * @flags: PCI_UIO_ROUTE_* topology properties.
 * @revoked: route is dead; no new work may be submitted.
 */
struct pci_uio_route {
	struct kref			 ref;
	struct pci_dev			*requester;
	struct p2pdma_provider		*provider;
	phys_addr_t			 offset;
	resource_size_t			 len;
	u8				 tc;
	u8				 vc;
	struct pci_uio_limits		 limits;
	unsigned long			 generation;
	unsigned long			 flags;
#define PCI_UIO_ROUTE_IN_FABRIC		BIT(0)	/* never crosses host bridge */
#define PCI_UIO_ROUTE_THRU_RP		BIT(1)	/* traverses RP/SoC segment */
#define PCI_UIO_ROUTE_MULTIPATH		BIT(2)	/* reserved: non-tree fabrics */
	bool				 revoked;

	/* internal, all pci_uio_lock protected */
	struct list_head		 node;
	const struct pci_uio_route_ops	*ops;
	void				*ops_priv;
	bool				 roles_active;
	unsigned int			 nr_hops;
	struct pci_dev			**hops;
	unsigned int			 nr_targets;
	struct pci_dev			**targets;
};

#ifdef CONFIG_PCI_UIO
/*
 * Single device predicates. Building blocks only: possessing a
 * capability grants nothing. All UIO enablement is owned by the
 * route object.
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

enum pci_uio_vc_owner pci_uio_vc_ownership(struct pci_host_bridge *bridge);

int pci_uio_route_get(struct pci_dev *requester,
		      struct p2pdma_provider *provider,
		      phys_addr_t offset, resource_size_t len,
		      const struct pci_uio_route_req *req,
		      struct pci_uio_route **route);
void pci_uio_route_put(struct pci_uio_route *route);

/* Cheap, hot-path safe */
static inline bool pci_uio_route_valid(struct pci_uio_route *route)
{
	return route && !READ_ONCE(route->revoked);
}
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

static inline enum pci_uio_vc_owner
pci_uio_vc_ownership(struct pci_host_bridge *bridge)
{
	return PCI_UIO_VC_OWNER_NONE;
}

static inline int pci_uio_route_get(struct pci_dev *requester,
				    struct p2pdma_provider *provider,
				    phys_addr_t offset, resource_size_t len,
				    const struct pci_uio_route_req *req,
				    struct pci_uio_route **route)
{
	return -EOPNOTSUPP;
}

static inline void pci_uio_route_put(struct pci_uio_route *route)
{
}

static inline bool pci_uio_route_valid(struct pci_uio_route *route)
{
	return false;
}
#endif /* CONFIG_PCI_UIO */

#endif /* _LINUX_PCI_UIO_H */
