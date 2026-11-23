// SPDX-License-Identifier: GPL-2.0-only
/*
 * CXL Hot-range Monitoring Unit: a client that makes every instance a hot
 * range source for DAMOS.
 *
 * Each instance is registered with DAMON as a hot range source named after
 * the instance device. A DAMOS migration scheme of a physical address
 * space context that names it has the context's kdamond do the rest: it
 * starts the source, fetches from it just before it applies the scheme,
 * and stops it. Nothing here knows of a DAMON context or scheme, nor of a
 * page: the instance hands over HPA ranges, and mm/damon does the rest.
 *
 * DAMON serializes the three callbacks of a source and calls none once
 * damos_unregister_source() has returned, so there is no lock here.
 */
#include <linux/damon.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/pgtable.h>
#include <linux/slab.h>

#include <cxl/hmu.h>

/*
 * How long an instance whose staged configuration does not fit, or whose
 * device failed to be programmed or is gone, is left alone
 */
#define CXL_HMU_DAMON_RETRY		(10 * HZ)

/**
 * struct cxl_hmu_damon - an instance as a DAMOS hot range source
 * @src: what DAMON knows it by; the name is that of the instance device
 * @inst: the instance
 * @start_err: what the last start or fetch failed with, if it is worth
 *	waiting for, or 0
 * @retry_at: when a start may program the device again after @start_err
 */
struct cxl_hmu_damon {
	struct damos_source src;
	struct cxl_hmu_inst *inst;
	int start_err;
	unsigned long retry_at;
};

/* What DAMON gave a fetch to hand ranges to */
struct cxl_hmu_damon_taker {
	damos_source_add_fn add;
	void *arg;
};

static struct cxl_hmu_client cxl_hmu_damon_client;

static void cxl_hmu_damon_backoff(struct cxl_hmu_damon *d, int err)
{
	d->start_err = err;
	d->retry_at = jiffies + CXL_HMU_DAMON_RETRY;
}

/*
 * Take the instance, with the configuration staged in its attributes, and
 * start it polled: cxl_hmu_damon_fetch() reads it. A unit larger than a
 * PMD is refused by the claim, before the device is touched (see
 * cxl_hmu_damon_init()).
 *
 * DAMON calls this again at every apply interval of the scheme for as long
 * as it fails. A staged configuration that does not fit (-EINVAL) stays so
 * until the attributes are written, a device that cannot be programmed
 * would fail the same way an interval later, after waiting for it, and a
 * memory device that cxl_mem left has nothing to translate: such an error
 * is remembered and returned, with the instance not claimed and the device
 * not touched, until CXL_HMU_DAMON_RETRY has passed.
 */
static int cxl_hmu_damon_start(struct damos_source *src)
{
	struct cxl_hmu_damon *d = container_of(src, struct cxl_hmu_damon, src);
	struct cxl_hmu_inst *inst = d->inst;
	int rc;

	if (d->start_err && time_before(jiffies, d->retry_at))
		return d->start_err;
	d->start_err = 0;

	rc = cxl_hmu_claim(inst, &cxl_hmu_damon_client);
	if (rc == -EINVAL || rc == -EIO || rc == -ETIMEDOUT)
		cxl_hmu_damon_backoff(d, rc);
	return rc;
}

/* Disable the instance and give it up */
static void cxl_hmu_damon_stop(struct damos_source *src)
{
	struct cxl_hmu_damon *d = container_of(src, struct cxl_hmu_damon, src);

	cxl_hmu_release(d->inst);
}

/* DAMON takes ranges, not counts: the order they come in is the ranking */
static int cxl_hmu_damon_add(void *arg, u64 hpa, u64 len, u32 count)
{
	struct cxl_hmu_damon_taker *t = arg;

	return t->add(t->arg, hpa, len);
}

/*
 * One read of up to CXL_HMU_READ_MAX entries, hottest first in epoch mode,
 * each unit handed over as HPA ranges until DAMON refuses one; what is not
 * handed over is counted in the instance's "dropped" attribute. DAMON asks
 * for another batch while it takes more and the hotlist has some.
 *
 * kdamond may be holding the folios @add isolated from earlier ranges,
 * which is why the read waits for no CXL lock: -EBUSY ends the fetching
 * for now, as a refusal does. -ENODEV says cxl_mem is not bound to the
 * memory device, after which no unit translates; DAMON then stops the
 * source, and the next start waits for CXL_HMU_DAMON_RETRY.
 */
static int cxl_hmu_damon_fetch(struct damos_source *src,
			       damos_source_add_fn add, void *arg)
{
	struct cxl_hmu_damon *d = container_of(src, struct cxl_hmu_damon, src);
	struct cxl_hmu_damon_taker t = { .add = add, .arg = arg };
	unsigned int flags;
	int rc;

	rc = cxl_hmu_read_ranges(d->inst, CXL_HMU_READ_MAX, cxl_hmu_damon_add,
				 &t, &flags);
	if (rc == -ENODEV)
		cxl_hmu_damon_backoff(d, rc);
	if (rc < 0)
		return rc;
	return flags & CXL_HMU_READ_MORE ? 1 : 0;
}

/* DAMON may start the source as soon as it is registered */
static int cxl_hmu_damon_inst_add(struct cxl_hmu_inst *inst, void **data)
{
	struct cxl_hmu_damon *d;
	int rc;

	d = kzalloc_obj(*d);
	if (!d)
		return -ENOMEM;
	d->inst = inst;
	d->src.name = dev_name(cxl_hmu_dev(inst));
	d->src.owner = THIS_MODULE;
	d->src.start = cxl_hmu_damon_start;
	d->src.stop = cxl_hmu_damon_stop;
	d->src.fetch = cxl_hmu_damon_fetch;
	rc = damos_register_source(&d->src);
	if (rc) {
		kfree(d);
		return rc;
	}
	*data = d;
	return 0;
}

/*
 * Waits for a fetch in flight, and stops the source if a scheme has it
 * started: on return the instance is released and DAMON calls nothing
 * here.
 */
static void cxl_hmu_damon_inst_remove(struct cxl_hmu_inst *inst, void *data)
{
	struct cxl_hmu_damon *d = data;

	damos_unregister_source(&d->src);
	kfree(d);
}

static struct cxl_hmu_client cxl_hmu_damon_client = {
	.name = "damon",
	.add = cxl_hmu_damon_inst_add,
	.remove = cxl_hmu_damon_inst_remove,
};

static int __init cxl_hmu_damon_init(void)
{
	/*
	 * A unit larger than a PMD stands for more than the largest folio
	 * there is, with one count for all of it, and handing it over would
	 * migrate everything beneath it in address order. Such a unit is for
	 * perf. Set here: PMD_SIZE is not a constant on every architecture.
	 */
	cxl_hmu_damon_client.max_unit_size = PMD_SIZE;
	return cxl_hmu_register_client(&cxl_hmu_damon_client);
}

static void __exit cxl_hmu_damon_exit(void)
{
	cxl_hmu_unregister_client(&cxl_hmu_damon_client);
}

module_init(cxl_hmu_damon_init);
module_exit(cxl_hmu_damon_exit);
MODULE_AUTHOR("Davidlohr Bueso <dave@stgolabs.net>");
MODULE_DESCRIPTION("CXL Hot-range Monitoring Unit DAMOS hot range sources");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("CXL_HMU");
