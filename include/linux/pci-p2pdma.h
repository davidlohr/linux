/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCI Peer 2 Peer DMA support.
 *
 * Copyright (c) 2016-2018, Logan Gunthorpe
 * Copyright (c) 2016-2017, Microsemi Corporation
 * Copyright (c) 2017, Christoph Hellwig
 * Copyright (c) 2018, Eideticom Inc.
 */

#ifndef _LINUX_PCI_P2PDMA_H
#define _LINUX_PCI_P2PDMA_H

#include <linux/dma-mapping.h>
#include <linux/pci.h>

struct block_device;
struct scatterlist;
struct pci_uio_route;
struct pci_uio_route_req;

/**
 * enum p2pdma_provider_type - what kind of memory a provider exposes
 *
 * @P2PDMA_PROVIDER_PCI_BAR_MMIO: PCI BAR backed MMIO. Never mapped
 *	cacheable by the CPU; DMA mappings of it carry DMA_ATTR_MMIO.
 * @P2PDMA_PROVIDER_CXL_HDM: a committed CXL region (SPA decoded,
 *	potentially CPU cacheable, host managed coherency). Peers emit
 *	host physical addresses - switch HDM/FAST decoders route by HPA
 *	- so @bus_offset is always 0 and DMA_ATTR_MMIO is never implied.
 *
 * The type determines mapping attribute derivation and consumers' CPU
 * access rules. It is data, not behavior: everything address-related
 * remains base + offset.
 */
enum p2pdma_provider_type {
	P2PDMA_PROVIDER_PCI_BAR_MMIO = 0,
	P2PDMA_PROVIDER_CXL_HDM,
};

/**
 * struct p2p_target_set - endpoints a provider subrange decodes to
 * @nr_targets: number of entries in @targets
 * @interleave_granularity: bytes of contiguous HPA per target
 * @targets: every PCI endpoint the range decodes to
 *
 * Allocated by a provider's @range_validate hook (single allocation)
 * with a reference held on every target (the hook resolves them under
 * its own locking; the caller pins them before that lock drops).
 * Release with p2p_target_set_put().
 */
struct p2p_target_set {
	unsigned int nr_targets;
	u32 interleave_granularity;
	struct pci_dev *targets[] __counted_by(nr_targets);
};

/**
 * struct p2pdma_provider
 *
 * A p2pdma provider is a range of peer memory address space available
 * to the CPU.
 * @owner: Device to which this provider belongs.
 * @type: memory exposure model, see &enum p2pdma_provider_type.
 * @base: CPU physical base address (BAR start, or region HPA).
 * @size: size of the provided range in bytes.
 * @bus_offset: Bus offset for p2p communication (0 for CXL_HDM).
 * @flags: P2PDMA_PROVIDER_* properties of the whole range.
 * @range_validate: optional subrange/target-set validation. NULL means
 *	the whole [base, base + size) range is uniformly valid with a
 *	single implicit target (the owner). Interleaved CXL regions back
 *	this with interleave decode; on success *@targets returns the
 *	kmalloc()ed endpoint set for the queried subrange.
 */
struct p2pdma_provider {
	struct device *owner;
	enum p2pdma_provider_type type;
	phys_addr_t base;
	resource_size_t size;
	u64 bus_offset;
	unsigned long flags;
#define P2PDMA_PROVIDER_UIO_COMPLETER	BIT(0)
	int (*range_validate)(struct p2pdma_provider *provider,
			      struct device *client,
			      phys_addr_t offset, resource_size_t len,
			      struct p2p_target_set **targets);
};

static inline bool p2pdma_provider_is_mmio(const struct p2pdma_provider *p)
{
	return p->type == P2PDMA_PROVIDER_PCI_BAR_MMIO;
}

/* Drop the target references and free a range_validate() result */
static inline void p2p_target_set_put(struct p2p_target_set *tset)
{
	unsigned int i;

	if (!tset)
		return;
	for (i = 0; i < tset->nr_targets; i++)
		pci_dev_put(tset->targets[i]);
	kfree(tset);
}

enum pci_p2pdma_map_type {
	/*
	 * PCI_P2PDMA_MAP_UNKNOWN: Used internally as an initial state before
	 * the mapping type has been calculated. Exported routines for the API
	 * will never return this value.
	 */
	PCI_P2PDMA_MAP_UNKNOWN = 0,

	/*
	 * Not a PCI P2PDMA transfer.
	 */
	PCI_P2PDMA_MAP_NONE,

	/*
	 * PCI_P2PDMA_MAP_NOT_SUPPORTED: Indicates the transaction will
	 * traverse the host bridge and the host bridge is not in the
	 * allowlist. DMA Mapping routines should return an error when
	 * this is returned.
	 */
	PCI_P2PDMA_MAP_NOT_SUPPORTED,

	/*
	 * PCI_P2PDMA_MAP_BUS_ADDR: Indicates that two devices can talk to
	 * each other directly through a PCI switch and the transaction will
	 * not traverse the host bridge. Such a mapping should program
	 * the DMA engine with PCI bus addresses.
	 */
	PCI_P2PDMA_MAP_BUS_ADDR,

	/*
	 * PCI_P2PDMA_MAP_THRU_HOST_BRIDGE: Indicates two devices can talk
	 * to each other, but the transaction traverses a host bridge on the
	 * allowlist. In this case, a normal mapping either with CPU physical
	 * addresses (in the case of dma-direct) or IOVA addresses (in the
	 * case of IOMMUs) should be used to program the DMA engine.
	 */
	PCI_P2PDMA_MAP_THRU_HOST_BRIDGE,
};

/**
 * struct pci_p2pdma_map_info - transfer plan for a provider subrange
 * @type: address-form classification for programming the DMA engine
 * @xport_flags: transport properties orthogonal to the address form
 * @uio_route: reference-held UIO route backing PCI_P2PDMA_XPORT_UIO,
 *	or NULL. The caller owns the reference and must
 *	pci_uio_route_put() it after the final unmap of mappings
 *	created under this plan.
 *
 * Produced by pci_p2pdma_map_info(). Unlike the page/map_state path,
 * this interface is range-aware: for interleaved providers eligibility
 * and classification are properties of the queried subrange (which
 * endpoints it decodes to), not of the provider as a whole.
 *
 * Address form and transport ordering are orthogonal: UIO is not a new
 * map type but a transport flag, so existing consumers of the map type
 * enum never see new states.
 */
struct pci_p2pdma_map_info {
	enum pci_p2pdma_map_type type;
	unsigned int xport_flags;
#define PCI_P2PDMA_XPORT_UIO	BIT(0)
	struct pci_uio_route *uio_route;
};

/**
 * p2pdma_map_attrs - derive DMA attributes for a peer transfer plan
 * @info: transfer plan from pci_p2pdma_map_info()
 * @provider: the provider the plan was computed against
 *
 * Centralized so no consumer hand-rolls it wrong: DMA_ATTR_MMIO is a
 * property of the provider's exposure model (BAR MMIO), never of "the
 * transfer is P2P" - CXL HDM may be host cacheable and must keep cache
 * maintenance. DMA_ATTR_UIO reflects a held UIO route only.
 */
static inline unsigned long
p2pdma_map_attrs(const struct pci_p2pdma_map_info *info,
		 const struct p2pdma_provider *provider)
{
	unsigned long attrs = 0;

	if (p2pdma_provider_is_mmio(provider))
		attrs |= DMA_ATTR_MMIO;
	if (info->uio_route)
		attrs |= DMA_ATTR_UIO;
	return attrs;
}

#ifdef CONFIG_PCI_P2PDMA
int pcim_p2pdma_init(struct pci_dev *pdev);
struct p2pdma_provider *pcim_p2pdma_provider(struct pci_dev *pdev, int bar);
int pci_p2pdma_add_resource(struct pci_dev *pdev, int bar, size_t size,
		u64 offset);
int pci_p2pdma_distance_many(struct pci_dev *provider, struct device **clients,
			     int num_clients, bool verbose);
struct pci_dev *pci_p2pmem_find_many(struct device **clients, int num_clients);
void *pci_alloc_p2pmem(struct pci_dev *pdev, size_t size);
void pci_free_p2pmem(struct pci_dev *pdev, void *addr, size_t size);
pci_bus_addr_t pci_p2pmem_virt_to_bus(struct pci_dev *pdev, void *addr);
struct scatterlist *pci_p2pmem_alloc_sgl(struct pci_dev *pdev,
					 unsigned int *nents, u32 length);
void pci_p2pmem_free_sgl(struct pci_dev *pdev, struct scatterlist *sgl);
void pci_p2pmem_publish(struct pci_dev *pdev, bool publish);
int pci_p2pdma_enable_store(const char *page, struct pci_dev **p2p_dev,
			    bool *use_p2pdma);
ssize_t pci_p2pdma_enable_show(char *page, struct pci_dev *p2p_dev,
			       bool use_p2pdma);
enum pci_p2pdma_map_type pci_p2pdma_map_type(struct p2pdma_provider *provider,
					     struct device *dev);
int pci_p2pdma_map_info(struct p2pdma_provider *provider, struct device *client,
			phys_addr_t offset, resource_size_t len,
			const struct pci_uio_route_req *uio_req,
			struct pci_p2pdma_map_info *info);
#else /* CONFIG_PCI_P2PDMA */
static inline int pcim_p2pdma_init(struct pci_dev *pdev)
{
	return -EOPNOTSUPP;
}
static inline struct p2pdma_provider *pcim_p2pdma_provider(struct pci_dev *pdev,
							   int bar)
{
	return NULL;
}
static inline int pci_p2pdma_add_resource(struct pci_dev *pdev, int bar,
		size_t size, u64 offset)
{
	return -EOPNOTSUPP;
}
static inline int pci_p2pdma_distance_many(struct pci_dev *provider,
	struct device **clients, int num_clients, bool verbose)
{
	return -1;
}
static inline struct pci_dev *pci_p2pmem_find_many(struct device **clients,
						   int num_clients)
{
	return NULL;
}
static inline void *pci_alloc_p2pmem(struct pci_dev *pdev, size_t size)
{
	return NULL;
}
static inline void pci_free_p2pmem(struct pci_dev *pdev, void *addr,
		size_t size)
{
}
static inline pci_bus_addr_t pci_p2pmem_virt_to_bus(struct pci_dev *pdev,
						    void *addr)
{
	return 0;
}
static inline struct scatterlist *pci_p2pmem_alloc_sgl(struct pci_dev *pdev,
		unsigned int *nents, u32 length)
{
	return NULL;
}
static inline void pci_p2pmem_free_sgl(struct pci_dev *pdev,
		struct scatterlist *sgl)
{
}
static inline void pci_p2pmem_publish(struct pci_dev *pdev, bool publish)
{
}
static inline int pci_p2pdma_enable_store(const char *page,
		struct pci_dev **p2p_dev, bool *use_p2pdma)
{
	*use_p2pdma = false;
	return 0;
}
static inline ssize_t pci_p2pdma_enable_show(char *page,
		struct pci_dev *p2p_dev, bool use_p2pdma)
{
	return sprintf(page, "none\n");
}
static inline enum pci_p2pdma_map_type
pci_p2pdma_map_type(struct p2pdma_provider *provider, struct device *dev)
{
	return PCI_P2PDMA_MAP_NOT_SUPPORTED;
}
static inline int pci_p2pdma_map_info(struct p2pdma_provider *provider,
				      struct device *client,
				      phys_addr_t offset, resource_size_t len,
				      const struct pci_uio_route_req *uio_req,
				      struct pci_p2pdma_map_info *info)
{
	return -EOPNOTSUPP;
}
#endif /* CONFIG_PCI_P2PDMA */


static inline int pci_p2pdma_distance(struct pci_dev *provider,
	struct device *client, bool verbose)
{
	return pci_p2pdma_distance_many(provider, &client, 1, verbose);
}

static inline struct pci_dev *pci_p2pmem_find(struct device *client)
{
	return pci_p2pmem_find_many(&client, 1);
}

struct pci_p2pdma_map_state {
	struct p2pdma_provider *mem;
	enum pci_p2pdma_map_type map;
};


/* helper for pci_p2pdma_state(), do not use directly */
void __pci_p2pdma_update_state(struct pci_p2pdma_map_state *state,
		struct device *dev, struct page *page);

/**
 * pci_p2pdma_state - check the P2P transfer state of a page
 * @state:	P2P state structure
 * @dev:	device to transfer to/from
 * @page:	page to map
 *
 * Check if @page is a PCI P2PDMA page, and if yes of what kind.  Returns the
 * map type, and updates @state with all information needed for a P2P transfer.
 */
static inline enum pci_p2pdma_map_type
pci_p2pdma_state(struct pci_p2pdma_map_state *state, struct device *dev,
		struct page *page)
{
	if (IS_ENABLED(CONFIG_PCI_P2PDMA) && is_pci_p2pdma_page(page)) {
		__pci_p2pdma_update_state(state, dev, page);
		return state->map;
	}
	return PCI_P2PDMA_MAP_NONE;
}

/**
 * pci_p2pdma_bus_addr_map - Translate a physical address to a bus address
 *			     for a PCI_P2PDMA_MAP_BUS_ADDR transfer.
 * @provider:	P2P provider structure
 * @paddr:	physical address to map
 *
 * Map a physically contiguous PCI_P2PDMA_MAP_BUS_ADDR transfer.
 */
static inline dma_addr_t
pci_p2pdma_bus_addr_map(struct p2pdma_provider *provider, phys_addr_t paddr)
{
	return paddr + provider->bus_offset;
}

#endif /* _LINUX_PCI_P2P_H */
