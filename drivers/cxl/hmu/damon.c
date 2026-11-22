// SPDX-License-Identifier: GPL-2.0-only
/*
 * CXL Hot-range Monitoring Unit: a hot range source for DAMOS.
 *
 * Each instance is registered with DAMON as a hot range source named after
 * the instance device. A DAMOS migration scheme of a physical address
 * space context that names it has the context's kdamond do the rest: it
 * starts the source, fetches from it just before it applies the scheme,
 * and stops it. Nothing here knows of a DAMON context or scheme.
 *
 * A fetch takes a bounded batch of entries off the device hotlist, hottest
 * first where the device says how hot, and hands each unit over as
 * host-physical ranges until DAMON refuses one. The hotlist is the only
 * queue: what is not fetched stays in the device. Everything about a page,
 * from looking it up to isolating and migrating it, is done in mm/damon;
 * this file handles physical ranges and nothing else.
 *
 * DAMON serializes the three callbacks of a source and calls none once
 * damos_unregister_source() has returned, so there is no lock here.
 */
#include <linux/damon.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/memory_hotplug.h>
#include <linux/module.h>
#include <linux/nodemask.h>
#include <linux/numa.h>
#include <linux/pgtable.h>
#include <linux/range.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/sort.h>

#include "../cxlmem.h"
#include "hmu.h"
#include "trace.h"

/* Entries per fetch: what is ranked together, and what one refusal can drop */
#define CXL_HMU_DAMON_BATCH		32

/* How long a device that failed to be programmed is left alone */
#define CXL_HMU_DAMON_RETRY		(10 * HZ)

/**
 * struct cxl_hmu_damon - an instance as a DAMOS hot range source
 * @src: what DAMON knows it by; the name is that of the instance device
 * @inst: the instance
 * @start_err: what programming the device last failed with, or 0
 * @retry_at: when a start may program the device again after @start_err
 * @entries: the batch of a fetch
 */
struct cxl_hmu_damon {
	struct damos_source src;
	struct cxl_hmu_inst *inst;
	int start_err;
	unsigned long retry_at;
	u64 entries[CXL_HMU_DAMON_BATCH];
};

/* What DAMON gave a fetch to hand ranges to, and how many it was handed */
struct cxl_hmu_damon_taker {
	damos_source_add_fn add;
	void *arg;
	unsigned int nr_ranges;
};

/*
 * The node the instance's memory is on: that of the first DPA of its range
 * that a region maps. -ENXIO if none does, -EOPNOTSUPP if the only regions
 * that do use an addressing that cannot be translated, -ENODEV if cxl_mem
 * is not bound to the memory device, and -EBUSY if a CXL lock was not
 * free: this is called by kdamond, which waits for none of them.
 */
static int cxl_hmu_damon_node(struct cxl_hmu_inst *inst)
{
	const struct cxl_hmu_config *cfg = &inst->live;
	struct cxl_memdev *cxlmd = inst->block->hmu->cxlmd;
	u64 dpa = (u64)cfg->range_base * SZ_256M;
	u64 end = dpa + (u64)cfg->range_num * SZ_256M;
	int rc = -ENXIO;
	u64 done;

	for (; dpa < end; dpa += done) {
		struct range hpa;
		int n, nid;

		n = cxl_memdev_dpa_to_hpa_ranges_nowait(cxlmd, dpa, end - dpa,
							&hpa, 1, &done);
		if (n == -EOPNOTSUPP) {
			rc = n;
			continue;
		}
		if (n < 0)
			return n;
		if (!n)
			continue;
		nid = phys_to_target_node(hpa.start);
		if (nid == NUMA_NO_NODE)
			nid = memory_add_physaddr_to_nid(hpa.start);
		return nid == NUMA_NO_NODE ? -ENXIO : nid;
	}
	return rc;
}

/*
 * Take the instance, with the configuration staged in its attributes, and
 * start it without an interrupt: it is polled by cxl_hmu_damon_fetch().
 *
 * A unit larger than a PMD is refused. One entry then stands for more than
 * the largest folio there is, with one count for all of it, and handing it
 * over would migrate everything beneath it in address order. Such a unit
 * is for perf. So is an instance whose memory is on a node without online
 * memory: there is no page to act on.
 *
 * DAMON calls this again at every apply interval of the scheme for as long
 * as it fails, so a failure is quiet, and nothing is written to the device
 * before the last two steps. Those can fail in the device, after waiting
 * for it with interrupts off, and would fail the same way an interval
 * later: such an error is remembered and returned, with the instance not
 * claimed and the device not touched, until CXL_HMU_DAMON_RETRY has passed.
 */
static int cxl_hmu_damon_start(struct damos_source *src)
{
	struct cxl_hmu_damon *d = container_of(src, struct cxl_hmu_damon, src);
	struct cxl_hmu_inst *inst = d->inst;
	int nid, rc;

	if (d->start_err && time_before(jiffies, d->retry_at))
		return d->start_err;
	d->start_err = 0;

	rc = cxl_hmu_claim(inst, "damon", NULL);
	if (rc)
		return rc;
	if (BIT_ULL(inst->live.unit_shift) > PMD_SIZE) {
		dev_dbg(&inst->dev, "unit size %llu is above %llu\n",
			BIT_ULL(inst->live.unit_shift), (u64)PMD_SIZE);
		rc = -EINVAL;
		goto release;
	}
	nid = cxl_hmu_damon_node(inst);
	if (nid < 0) {
		rc = nid;
		goto release;
	}
	if (!node_state(nid, N_MEMORY)) {
		dev_dbg(&inst->dev, "node %d has no online memory\n", nid);
		rc = -ENXIO;
		goto release;
	}
	/* A threshold that does not fit the counter is refused quietly here */
	rc = cxl_hmu_check_threshold(inst);
	if (!rc)
		rc = cxl_hmu_start(inst, 0);
	if (!rc)
		return 0;
	if (rc == -EIO || rc == -ETIMEDOUT) {
		d->start_err = rc;
		d->retry_at = jiffies + CXL_HMU_DAMON_RETRY;
	}
release:
	cxl_hmu_release(inst);
	return rc;
}

/* Disable the instance and give it up */
static void cxl_hmu_damon_stop(struct damos_source *src)
{
	struct cxl_hmu_damon *d = container_of(src, struct cxl_hmu_damon, src);

	cxl_hmu_release(d->inst);
}

/* Hottest first; the count is the low live_width bits of an entry */
static int cxl_hmu_count_cmp(const void *a, const void *b, const void *priv)
{
	u64 mask = *(const u64 *)priv;
	u64 ca = *(const u64 *)a & mask, cb = *(const u64 *)b & mask;

	return ca < cb ? 1 : ca > cb ? -1 : 0;
}

/* DAMON takes ranges, not counts: the order they come in is the ranking */
static int cxl_hmu_damon_add(u64 hpa, u64 len, u32 nr_accesses, void *data)
{
	struct cxl_hmu_damon_taker *taker = data;

	taker->nr_ranges++;
	return taker->add(taker->arg, hpa, len);
}

/*
 * Take one batch of entries off the hotlist and hand the unit of each to
 * @add, one host-physical range at a time, until it returns non-zero. In
 * epoch mode the entries are handed over by count, the highest first, so
 * that a scheme's quota is spent on the hottest of the batch; in always-on
 * mode an entry has no count and the order is the device's.
 *
 * Entries are consumed when they are read, so those of the batch that
 * were not handed over whole when @add refused, the refused one included,
 * are gone: they are counted in the instance's "dropped" attribute, not
 * kept, since a second queue here would go stale behind the device's. A
 * unit that is still hot is reported again. The batch is small so that
 * little is lost, and DAMON asks for another while it takes more. An
 * entry whose unit has no range to hand over (no region maps it, or what
 * does is not System RAM) is dropped as well.
 *
 * kdamond may be holding the folios @add isolated from earlier ranges,
 * which is why the walk waits for no CXL lock: -EBUSY from it ends the
 * fetching for now, as a refusal does. -ENODEV says cxl_mem is not bound
 * to the memory device, after which no unit translates; DAMON then stops
 * the source, and starts it again once it can be.
 */
static int cxl_hmu_damon_fetch(struct damos_source *src,
			       damos_source_add_fn add, void *arg)
{
	struct cxl_hmu_damon *d = container_of(src, struct cxl_hmu_damon, src);
	struct cxl_hmu_damon_taker taker = { .add = add, .arg = arg };
	struct cxl_hmu_inst *inst = d->inst;
	u64 mask = BIT_ULL(inst->live_width) - 1;
	unsigned int flags, n, i, skipped = 0;
	int rc = 0;

	n = cxl_hmu_drain(inst, d->entries, CXL_HMU_DAMON_BATCH, &flags);
	if (inst->live.mode == CXL_HMU_MODE_EPOCH)
		sort_r(d->entries, n, sizeof(*d->entries), cxl_hmu_count_cmp,
		       NULL, &mask);
	for (i = 0; i < n; i++) {
		taker.nr_ranges = 0;
		rc = cxl_hmu_for_each_hpa_range(inst, d->entries[i],
						cxl_hmu_damon_add, &taker);
		if (rc)
			break;
		if (!taker.nr_ranges)
			skipped++;
	}
	if (n - i + skipped)
		cxl_hmu_account_dropped(inst, n - i + skipped);
	trace_cxl_hmu_fetch(inst, n, i - skipped, n - i + skipped,
			    flags & CXL_HMU_DRAIN_LOST, rc);
	if (rc)
		return rc;
	return flags & CXL_HMU_DRAIN_MORE ? 1 : 0;
}

/*
 * With the instance device added, so that it has its name and is in the
 * block's table: DAMON may start the source as soon as this returns.
 */
int cxl_hmu_damon_register(struct cxl_hmu_inst *inst)
{
	struct cxl_hmu_damon *d;
	int rc;

	d = kzalloc_obj(*d);
	if (!d)
		return -ENOMEM;
	d->inst = inst;
	d->src.name = dev_name(&inst->dev);
	d->src.owner = THIS_MODULE;
	d->src.start = cxl_hmu_damon_start;
	d->src.stop = cxl_hmu_damon_stop;
	d->src.fetch = cxl_hmu_damon_fetch;
	rc = damos_register_source(&d->src);
	if (rc) {
		kfree(d);
		return rc;
	}
	inst->damon = d;
	return 0;
}

/*
 * At unbind. Waits for a fetch in flight, and stops the source if a scheme
 * has it started: on return the instance is released and DAMON calls
 * nothing here. The caller must not hold the block lock, which the stop
 * takes.
 */
void cxl_hmu_damon_unregister(struct cxl_hmu_inst *inst)
{
	struct cxl_hmu_damon *d = inst->damon;

	damos_unregister_source(&d->src);
	inst->damon = NULL;
	kfree(d);
}
