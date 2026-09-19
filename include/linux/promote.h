/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_PROMOTE_H
#define _LINUX_PROMOTE_H

#include <linux/errno.h>
#include <linux/types.h>

/*
 * Promotion for hotness sources that COUNT accesses in hardware over a
 * bounded region, rather than sampling individual pages.
 *
 * Such a source (a CXL Hotness Monitoring Unit, for example) reports a
 * bounded, already-aggregated list once per integration epoch. It does not
 * need the kernel to remember anything between epochs, so nothing here
 * persists: no per-page metadata, no per-section allocation.
 *
 * Extents are ranked by access DENSITY - accesses per page per second - not
 * by raw count. A large lukewarm region and a small scorching one can carry
 * the same count, and promoting the former would displace useful top-tier
 * memory to chase a subrange. Density also makes weights comparable between
 * devices with different unit sizes and different epoch lengths.
 *
 * A device counts over its own unit of address space, and interleaving can
 * scatter one unit across several physically contiguous runs. Each run is
 * its own extent, but the count was taken over the whole unit, so density
 * is the count over the pages of the unit (@nr_counted), not of the run
 * (@nr_pages). Attributing the whole count to every run would rank an
 * interleaved device above an identical non-interleaved one by the number
 * of runs its units split into.
 */

enum promote_src {
	PROMOTE_SRC_CXL_HMU,
	PROMOTE_SRC_MAX,
};

struct promote_extent {
	unsigned long	pfn;		/* first pfn of a contiguous run */
	unsigned long	nr_pages;	/* run length in base pages */
	unsigned long	nr_counted;	/* pages @weight was counted over; nr_pages if 0 or smaller */
	u64		weight;		/* raw device count for the unit */
	u64		density;	/* out: accesses per page per second, 16 fractional bits */
	u32		seq;		/* out: arrival order, ranking tiebreak */
	u32		pad;
};

struct promote_batch {
	struct promote_extent	*extents;	/* caller owned, sorted in place */
	unsigned int		nr_extents;
	unsigned int		epoch_ms;	/* integration interval, > 0 */
	unsigned int		src;
	bool			ranked;		/* weights comparable in batch */
	bool			truncated;	/* device dropped entries */
};

/* Accounting for one worker pass; exported through vmstat, not the caller. */
struct promote_stats {
	unsigned int	nr_extents;
	unsigned long	nr_pages_considered;
	unsigned long	nr_pages_eligible;
	unsigned long	nr_pages_budgeted;
	unsigned long	nr_pages_promoted;
	unsigned long	nr_pages_dropped_budget;
	u64		cut_density;	/* of the last extent admitted, same units */
};

/*
 * Queue a batch for asynchronous promotion and return. The caller keeps
 * ownership of @b->extents; they are copied into a bounded per-node queue,
 * so this may be called from a device's threaded interrupt handler without
 * holding it for the duration of the migration.
 *
 * Returns the number of extents queued, which is less than b->nr_extents if
 * the queue was full.
 */
#ifdef CONFIG_HOT_PAGE_PROMOTION
int promote_queue_extents(struct promote_batch *b);
#else
static inline int promote_queue_extents(struct promote_batch *b)
{
	return -EOPNOTSUPP;
}
#endif

#endif /* _LINUX_PROMOTE_H */
