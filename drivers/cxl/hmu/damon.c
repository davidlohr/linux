// SPDX-License-Identifier: GPL-2.0-only
/*
 * CXL Hot-range Monitoring Unit: hand hot units to DAMOS.
 *
 * The consumer is a DAMON API user. It owns a DAMON context with no
 * monitoring targets and one DAMOS_MIGRATE_HOT scheme; the folios of
 * every hot unit the hardware reports are pushed to that scheme with
 * damos_add_paddr_range(), and kdamond migrates them to the target node
 * under the scheme's quota. Nothing is sampled: the hardware applied the
 * hotness threshold.
 */
#include <linux/damon.h>
#include <linux/device.h>
#include <linux/kstrtox.h>
#include <linux/limits.h>
#include <linux/mutex.h>
#include <linux/nodemask.h>
#include <linux/numa.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/sysfs.h>
#include <linux/time64.h>
#include <linux/topology.h>
#include <cxl/hmu.h>

#include "../cxlmem.h"
#include "hmu.h"

#define CXL_HMU_DAMON_INTERVAL_US	(100 * USEC_PER_MSEC)

struct cxl_hmu_damon {
	struct mutex lock;		/* orders the sysfs stores */
	struct damon_ctx *ctx;		/* non-NULL while started */
	struct damos *scheme;
	struct damos_stat stat;		/* copy of the scheme's at stop */
	int target_node;		/* NUMA_NO_NODE: pick at start */
	int nid;			/* the node in use while started */
	bool stopping;			/* the final drain is not worth isolating for */
	unsigned long quota_bytes;
	unsigned long quota_ms;
	unsigned long quota_reset_ms;
};

static const struct cxl_hmu_consumer cxl_hmu_damon_consumer;

/*
 * A push is refused once the scheme's quota for this interval is used up,
 * or while kdamond is not applying; the rest of the batch would be refused
 * too, so stop walking it rather than isolate and put back every unit.
 */
static int cxl_hmu_damon_extent(u64 hpa, u64 len, u32 nr_accesses, void *data)
{
	struct cxl_hmu_damon *d = data;

	return damos_add_paddr_range(d->scheme, hpa, len);
}

static void cxl_hmu_damon_hotlist(struct cxl_hmu_inst *inst, const u64 *entries,
				  unsigned int nr, u8 counter_width, bool overflowed)
{
	struct cxl_hmu_damon *d = cxl_hmu_consumer_data(inst);
	unsigned int i;

	/* A stop waits for this batch; do not make it wait for a refused one */
	for (i = 0; i < nr && !READ_ONCE(d->stopping); i++)
		if (cxl_hmu_for_each_extent(inst, entries[i], counter_width,
					    cxl_hmu_damon_extent, d))
			break;
}

/* The node the instance's memory is on: the first DPA of its range that a region maps */
static int cxl_hmu_damon_device_node(struct cxl_hmu_inst *inst)
{
	const struct cxl_hmu_config *cfg = cxl_hmu_config(inst);
	struct cxl_memdev *cxlmd = inst->block->cxlmd;
	u64 dpa = (u64)cfg->range_base * SZ_256M;
	u64 end = dpa + (u64)cfg->range_num * SZ_256M;

	for (; dpa < end; dpa += SZ_256M) {
		u64 hpa = cxl_memdev_dpa_to_hpa(cxlmd, dpa);
		int nid;

		if (hpa == ULLONG_MAX)
			continue;
		nid = phys_to_target_node(hpa);
		if (nid == NUMA_NO_NODE)
			nid = memory_add_physaddr_to_nid(hpa);
		return nid;
	}
	return NUMA_NO_NODE;
}

/*
 * The node to promote to. A CXL node described only by a CFMWS has no SLIT
 * row and is at the remote distance from everyone, so the distance is
 * measured from the host bridge's node instead, which firmware does
 * describe: the nearest node with CPUs and memory to that, which is the
 * bridge's own node when it has both.
 */
static int cxl_hmu_damon_pick_target(struct cxl_hmu_inst *inst, int source)
{
	int from = dev_to_node(inst->block->dev.parent);
	int n, best = NUMA_NO_NODE, best_dist = INT_MAX;

	if (from == NUMA_NO_NODE)
		from = source;
	for_each_node_state(n, N_CPU) {
		if (!node_state(n, N_MEMORY) || n == source ||
		    node_distance(from, n) >= best_dist)
			continue;
		best = n;
		best_dist = node_distance(from, n);
	}
	return best == NUMA_NO_NODE ? -ENXIO : best;
}

static int cxl_hmu_damon_start(struct cxl_hmu_inst *inst, struct cxl_hmu_damon *d)
{
	/* Never matches a region: this scheme only takes pushed folios */
	struct damos_access_pattern never = { .min_sz_region = 1, .max_sz_region = 0 };
	struct damos_quota quota = {
		.sz = d->quota_bytes,
		.ms = d->quota_ms,
		.reset_interval = d->quota_reset_ms,
	};
	struct damos_watermarks wmarks = { .metric = DAMOS_WMARK_NONE };
	struct damon_attrs attrs;
	int source, nid, rc;

	source = cxl_hmu_damon_device_node(inst);
	if (source == NUMA_NO_NODE)
		return -ENXIO;	/* no region maps the instance's range */
	nid = d->target_node;
	if (nid == NUMA_NO_NODE)
		nid = cxl_hmu_damon_pick_target(inst, source);
	if (nid < 0)
		return nid;
	if (nid == source) {
		dev_err(&inst->dev, "target node %d is the instance's own\n", nid);
		return -EINVAL;
	}

	d->ctx = damon_new_ctx();
	if (!d->ctx)
		return -ENOMEM;
	rc = damon_select_ops(d->ctx, DAMON_OPS_PADDR);
	if (rc)
		goto destroy;
	/* No targets to sample; the interval is how often the queue is applied */
	attrs = d->ctx->attrs;
	attrs.sample_interval = CXL_HMU_DAMON_INTERVAL_US;
	attrs.aggr_interval = CXL_HMU_DAMON_INTERVAL_US;
	rc = damon_set_attrs(d->ctx, &attrs);
	if (rc)
		goto destroy;
	d->scheme = damon_new_scheme(&never, DAMOS_MIGRATE_HOT, 0, &quota,
				     &wmarks, nid);
	if (!d->scheme) {
		rc = -ENOMEM;
		goto destroy;
	}
	damon_set_schemes(d->ctx, &d->scheme, 1);
	/*
	 * Not exclusive, so an administrator's /sys/kernel/mm/damon contexts
	 * keep running; the exclusive in-kernel users (DAMON_STAT by default,
	 * DAMON_RECLAIM, DAMON_LRU_SORT) and this consumer exclude each other.
	 */
	rc = damon_start(&d->ctx, 1, false);
	if (rc) {
		if (rc == -EBUSY)
			dev_err(&inst->dev, "an exclusive DAMON context (damon_stat, reclaim, lru_sort) is running\n");
		goto destroy;
	}
	d->nid = nid;
	d->stopping = false;
	rc = cxl_hmu_start(inst, &cxl_hmu_damon_consumer, cxl_hmu_config(inst));
	if (rc) {
		damon_stop(&d->ctx, 1);
		goto destroy;
	}
	return 0;
destroy:
	damon_destroy_ctx(d->ctx);
	d->ctx = NULL;
	d->scheme = NULL;
	return rc;
}

static void cxl_hmu_damon_stop(struct cxl_hmu_inst *inst, struct cxl_hmu_damon *d)
{
	if (!d->ctx)
		return;
	WRITE_ONCE(d->stopping, true);	/* the final drain is put back anyway */
	cxl_hmu_stop(inst);		/* no hotlist callback runs after this */
	damon_stop(&d->ctx, 1);		/* queued folios go back to the LRU */
	d->stat = d->scheme->stat;
	damon_destroy_ctx(d->ctx);
	d->ctx = NULL;
	d->scheme = NULL;
}

static ssize_t start_show(struct device *dev, struct device_attribute *a, char *buf)
{
	struct cxl_hmu_damon *d = cxl_hmu_consumer_data(to_cxl_hmu_inst(dev));
	bool on;

	mutex_lock(&d->lock);
	on = d->ctx && damon_is_running(d->ctx);
	mutex_unlock(&d->lock);
	return sysfs_emit(buf, "%d\n", on);
}

static ssize_t start_store(struct device *dev, struct device_attribute *a,
			   const char *buf, size_t len)
{
	struct cxl_hmu_inst *inst = to_cxl_hmu_inst(dev);
	struct cxl_hmu_damon *d = cxl_hmu_consumer_data(inst);
	bool on;
	int rc = 0;

	if (kstrtobool(buf, &on))
		return -EINVAL;
	mutex_lock(&d->lock);
	if (on && !d->ctx)
		rc = cxl_hmu_damon_start(inst, d);
	else if (!on)
		cxl_hmu_damon_stop(inst, d);
	mutex_unlock(&d->lock);
	return rc ? rc : len;
}
static DEVICE_ATTR_RW(start);

/* While started, the node in use, so a chosen default is visible */
static ssize_t target_node_show(struct device *dev, struct device_attribute *a,
				char *buf)
{
	struct cxl_hmu_damon *d = cxl_hmu_consumer_data(to_cxl_hmu_inst(dev));
	int nid;

	mutex_lock(&d->lock);
	nid = d->ctx ? d->nid : d->target_node;
	mutex_unlock(&d->lock);
	return sysfs_emit(buf, "%d\n", nid);
}

static ssize_t target_node_store(struct device *dev, struct device_attribute *a,
				 const char *buf, size_t len)
{
	struct cxl_hmu_damon *d = cxl_hmu_consumer_data(to_cxl_hmu_inst(dev));
	int nid, rc = 0;

	if (kstrtoint(buf, 0, &nid))
		return -EINVAL;
	if (nid != NUMA_NO_NODE && (nid < 0 || nid >= MAX_NUMNODES ||
				    !node_state(nid, N_MEMORY)))
		return -EINVAL;
	mutex_lock(&d->lock);
	if (d->ctx)
		rc = -EBUSY;
	else
		d->target_node = nid;
	mutex_unlock(&d->lock);
	return rc ? rc : len;
}
static DEVICE_ATTR_RW(target_node);

#define CXL_HMU_DAMON_ULONG_ATTR(name, field, min)			\
static ssize_t name##_show(struct device *dev,				\
			   struct device_attribute *a, char *buf)	\
{									\
	struct cxl_hmu_damon *d =					\
		cxl_hmu_consumer_data(to_cxl_hmu_inst(dev));		\
	unsigned long v;						\
									\
	mutex_lock(&d->lock);						\
	v = d->field;							\
	mutex_unlock(&d->lock);						\
	return sysfs_emit(buf, "%lu\n", v);				\
}									\
static ssize_t name##_store(struct device *dev,				\
			    struct device_attribute *a,			\
			    const char *buf, size_t len)		\
{									\
	struct cxl_hmu_damon *d =					\
		cxl_hmu_consumer_data(to_cxl_hmu_inst(dev));		\
	unsigned long v;						\
	int rc = 0;							\
									\
	if (kstrtoul(buf, 0, &v) || v < (min))				\
		return -EINVAL;						\
	mutex_lock(&d->lock);						\
	if (d->ctx)							\
		rc = -EBUSY;						\
	else								\
		d->field = v;						\
	mutex_unlock(&d->lock);						\
	return rc ? rc : len;						\
}									\
static DEVICE_ATTR_RW(name)

CXL_HMU_DAMON_ULONG_ATTR(quota_bytes, quota_bytes, 0);
CXL_HMU_DAMON_ULONG_ATTR(quota_ms, quota_ms, 0);
/* A zero window would reset the quota at every apply */
CXL_HMU_DAMON_ULONG_ATTR(quota_reset_ms, quota_reset_ms, 1);

#define CXL_HMU_DAMON_STAT_ATTR(name)					\
static ssize_t name##_show(struct device *dev,				\
			   struct device_attribute *a, char *buf)	\
{									\
	struct cxl_hmu_damon *d =					\
		cxl_hmu_consumer_data(to_cxl_hmu_inst(dev));		\
	unsigned long v;						\
									\
	mutex_lock(&d->lock);						\
	v = d->ctx ? READ_ONCE(d->scheme->stat.name) : d->stat.name;	\
	mutex_unlock(&d->lock);						\
	return sysfs_emit(buf, "%lu\n", v);				\
}									\
static DEVICE_ATTR_RO(name)

CXL_HMU_DAMON_STAT_ATTR(sz_tried);
CXL_HMU_DAMON_STAT_ATTR(sz_applied);
CXL_HMU_DAMON_STAT_ATTR(qt_exceeds);

static struct attribute *cxl_hmu_damon_attrs[] = {
	&dev_attr_start.attr,
	&dev_attr_target_node.attr,
	&dev_attr_quota_bytes.attr,
	&dev_attr_quota_ms.attr,
	&dev_attr_quota_reset_ms.attr,
	&dev_attr_sz_tried.attr,
	&dev_attr_sz_applied.attr,
	&dev_attr_qt_exceeds.attr,
	NULL,
};

static const struct attribute_group cxl_hmu_damon_group = {
	.name = "damon",
	.attrs = cxl_hmu_damon_attrs,
};

static int cxl_hmu_damon_attach(struct cxl_hmu_inst *inst)
{
	struct cxl_hmu_damon *d;
	int rc;

	d = kzalloc_obj(*d);
	if (!d)
		return -ENOMEM;
	mutex_init(&d->lock);
	d->target_node = NUMA_NO_NODE;
	d->quota_reset_ms = 1000;
	cxl_hmu_set_consumer_data(inst, d);
	rc = device_add_group(&inst->dev, &cxl_hmu_damon_group);
	if (rc) {
		cxl_hmu_set_consumer_data(inst, NULL);
		kfree(d);
	}
	return rc;
}

/* Called with the instance stopped, or stopping it: either way no callback is left */
static void cxl_hmu_damon_detach(struct cxl_hmu_inst *inst)
{
	struct cxl_hmu_damon *d = cxl_hmu_consumer_data(inst);

	device_remove_group(&inst->dev, &cxl_hmu_damon_group);
	mutex_lock(&d->lock);
	cxl_hmu_damon_stop(inst, d);
	mutex_unlock(&d->lock);
	cxl_hmu_set_consumer_data(inst, NULL);
	mutex_destroy(&d->lock);
	kfree(d);
}

static const struct cxl_hmu_consumer cxl_hmu_damon_consumer = {
	.name = "damon",
	.owner = THIS_MODULE,
	.attach = cxl_hmu_damon_attach,
	.detach = cxl_hmu_damon_detach,
	.hotlist = cxl_hmu_damon_hotlist,
};

int cxl_hmu_damon_init(void)
{
	return cxl_hmu_register_consumer(&cxl_hmu_damon_consumer);
}

void cxl_hmu_damon_exit(void)
{
	cxl_hmu_unregister_consumer(&cxl_hmu_damon_consumer);
}
