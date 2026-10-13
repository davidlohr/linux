// SPDX-License-Identifier: GPL-2.0-only
/*
 * CXL Hot-range Monitoring Unit: instance devices, hardware control and
 * the consumer interface.
 *
 * A CHMU register block holds up to eight instances, each with its own
 * configuration, hotlist and interrupt (CXL r4.0 Section 8.2.8). Each is
 * registered as a child device of the block and driven by one consumer at
 * a time, chosen through the device's "consumer" attribute. The core owns
 * the capability decode, the configuration and its checks, the start and
 * stop sequences and the hotlist drain; a consumer only decides what a
 * drained hotlist means.
 */
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/kstrtox.h>
#include <linux/list.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include "../cxl.h"
#include "../cxlmem.h"
#include "../cxlpci.h"
#include "hmu.h"


/* Table 8-192: the epoch scale encodings, in microseconds */
static const u32 cxl_hmu_epoch_scale_us[] = {
	[CHMU_EPOCH_SCALE_100US] = 100,
	[CHMU_EPOCH_SCALE_1MS] = 1000,
	[CHMU_EPOCH_SCALE_10MS] = 10000,
	[CHMU_EPOCH_SCALE_100MS] = 100000,
	[CHMU_EPOCH_SCALE_1S] = 1000000,
};
#define CXL_HMU_EPOCH_MULT_MAX	4095

u64 cxl_hmu_epoch_us(u8 scale, u16 mult)
{
	if (scale < CHMU_EPOCH_SCALE_100US || scale >= ARRAY_SIZE(cxl_hmu_epoch_scale_us))
		return 0;
	return (u64)cxl_hmu_epoch_scale_us[scale] * mult;
}

/*
 * The scale and multiplier for an epoch of @target_us, clamped to what the
 * instance supports; the finest scale that encodes it. @mult is 0 if
 * nothing does.
 */
void cxl_hmu_pick_epoch(const struct cxl_hmu_inst *inst, u64 target_us, u8 *scale, u16 *mult)
{
	const struct cxl_hmu_caps *c = &inst->caps;
	u64 want = target_us;
	int s;

	if (c->epoch_min_us && want < c->epoch_min_us)
		want = c->epoch_min_us;
	if (c->epoch_max_us && want > c->epoch_max_us)
		want = c->epoch_max_us;
	for (s = CHMU_EPOCH_SCALE_100US; s < ARRAY_SIZE(cxl_hmu_epoch_scale_us); s++) {
		u64 m = want / cxl_hmu_epoch_scale_us[s];

		if (m >= 1 && m <= CXL_HMU_EPOCH_MULT_MAX) {
			*scale = s;
			*mult = m;
			return;
		}
	}
	*scale = 0;
	*mult = 0;
}

static void cxl_hmu_decode_caps(struct cxl_hmu_inst *inst)
{
	struct cxl_memdev_state *mds = to_cxl_memdev_state(inst->block->cxlmd->cxlds);
	struct cxl_hmu_caps *c = &inst->caps;
	u64 cap0 = readq(inst->base + CHMU_INST0_CAP0_REG);
	u64 cap1 = readq(inst->base + CHMU_INST0_CAP1_REG);
	u64 hl = readq(inst->base + CHMU_INST0_HOTLIST_OFFSET_REG);
	u64 bm = readq(inst->base + CHMU_INST0_RANGE_BITMAP_OFFSET_REG);

	c->msi = FIELD_GET(CHMU_INST0_CAP0_MSI_N_MSK, cap0);
	c->irq_overflow = FIELD_GET(CHMU_INST0_CAP0_OVRFLW_CAP, cap0);
	c->irq_level = FIELD_GET(CHMU_INST0_CAP0_FILLTHRESH_CAP, cap0);
	c->epoch_type = FIELD_GET(CHMU_INST0_CAP0_EPOCH_TYPE_MSK, cap0);
	c->access_types = FIELD_GET(CHMU_INST0_CAP0_TRACK_MSK, cap0);
	c->epoch_min_us = cxl_hmu_epoch_us(FIELD_GET(CHMU_INST0_CAP0_EPOCH_MIN_SCALE_MSK, cap0),
					   FIELD_GET(CHMU_INST0_CAP0_EPOCH_MIN_MULT_MSK, cap0));
	c->epoch_max_us = cxl_hmu_epoch_us(FIELD_GET(CHMU_INST0_CAP0_EPOCH_MAX_SCALE_MSK, cap0),
					   FIELD_GET(CHMU_INST0_CAP0_EPOCH_MAX_MULT_MSK, cap0));
	c->hotlist_size = FIELD_GET(CHMU_INST0_CAP0_HOTLIST_SIZE_MSK, cap0);
	c->unit_sizes = FIELD_GET(CHMU_INST0_CAP1_UNIT_SIZE_MSK, cap1);
	c->ds_factors = FIELD_GET(CHMU_INST0_CAP1_DOWNSAMP_MSK, cap1);
	c->epoch = FIELD_GET(CHMU_INST0_CAP1_EPOCH_SUP, cap1);
	c->always_on = FIELD_GET(CHMU_INST0_CAP1_ALWAYS_ON_SUP, cap1);
	c->randomized_ds = FIELD_GET(CHMU_INST0_CAP1_RAND_DOWNSAMP_SUP, cap1);
	c->addr_overlap = FIELD_GET(CHMU_INST0_CAP1_ADDR_OVERLAP_SUP, cap1);
	c->postponed_overflow = FIELD_GET(CHMU_INST0_CAP1_POSTPONED_ON_OVRFLOW_SUP, cap1);
	c->range_bits = (hl - bm) * 8;
	/*
	 * The bitmap covers the device capacity in 256 MiB units; anything
	 * beyond that is padding the device may treat as reserved.
	 */
	c->capacity_ranges = min_t(u64, DIV_ROUND_UP_ULL(mds->total_bytes, SZ_256M),
				   c->range_bits);
}

int cxl_hmu_validate_config(struct cxl_hmu_inst *inst, const struct cxl_hmu_config *cfg)
{
	const struct cxl_hmu_caps *c = &inst->caps;
	u64 epoch = cxl_hmu_epoch_us(cfg->epoch_scale, cfg->epoch_mult);

	if (cfg->mode == CXL_HMU_MODE_EPOCH ? !c->epoch : !c->always_on)
		return -EOPNOTSUPP;
	if (cfg->unit_shift < 8 || cfg->unit_shift - 8 >= 32 ||
	    !(c->unit_sizes & BIT(cfg->unit_shift - 8)))
		return -EOPNOTSUPP;
	if (cfg->access_type < CHMU_INST0_CFG0_WHAT_NONTEE_R ||
	    cfg->access_type > CHMU_INST0_CFG0_WHAT_RW ||
	    !(c->access_types & BIT(cfg->access_type - 1)))
		return -EOPNOTSUPP;
	if (cfg->ds_shift >= 16 || !(c->ds_factors & BIT(cfg->ds_shift)))
		return -EOPNOTSUPP;
	if (cfg->randomized_ds && !c->randomized_ds)
		return -EOPNOTSUPP;
	if (!cfg->epoch_mult || epoch < c->epoch_min_us || epoch > c->epoch_max_us)
		return -EINVAL;
	if (!cfg->range_num || (u64)cfg->range_base + cfg->range_num > c->capacity_ranges)
		return -EINVAL;
	/* A range smaller than one unit cannot hold one */
	if ((u64)cfg->range_num * SZ_256M < BIT_ULL(cfg->unit_shift))
		return -EINVAL;
	if (inst->counter_width && cfg->hotness_threshold >= BIT_ULL(inst->counter_width))
		return -EINVAL;
	return 0;
}

/*
 * Program the unit size into the disabled instance and read back the
 * counter width the device chose for it (r4.0 8.2.8: configuration is
 * writable while disabled; the width depends on the unit size).
 */
int cxl_hmu_set_unit_size(struct cxl_hmu_inst *inst, u8 unit_shift)
{
	unsigned long flags;
	u64 val;
	u8 width;

	raw_spin_lock_irqsave(&inst->lock, flags);
	if (inst->active) {
		raw_spin_unlock_irqrestore(&inst->lock, flags);
		return -EBUSY;
	}
	val = readq(inst->base + CHMU_INST0_CFG1_REG);
	val &= ~CHMU_INST0_CFG1_UNIT_SIZE_MSK;
	val |= FIELD_PREP(CHMU_INST0_CFG1_UNIT_SIZE_MSK, unit_shift);
	writeq(val, inst->base + CHMU_INST0_CFG1_REG);
	width = FIELD_GET(CHMU_INST0_STATUS_COUNTER_WIDTH_MSK,
			  readq(inst->base + CHMU_INST0_STATUS_REG));
	if (width && width < 64) {
		inst->counter_width = width;
		inst->config.unit_shift = unit_shift;
	}
	raw_spin_unlock_irqrestore(&inst->lock, flags);
	return width && width < 64 ? 0 : -EIO;
}

/*
 * Unless this instance and every enabled sibling in the block advertise
 * Address Overlap, their range bitmaps must be disjoint. The spec defines
 * only the set case of the capability; this is the safe reading of its
 * absence, scoped to one register block as the spec scopes the bit. A
 * driver policy: it compares against enabled siblings only, since a
 * stopped one keeps its bitmap programmed. Called with the block lock
 * held, under which siblings enable and disable.
 */
static int cxl_hmu_check_overlap(struct cxl_hmu_inst *inst, const struct cxl_hmu_config *cfg)
{
	u64 a0 = cfg->range_base, a1 = a0 + cfg->range_num;
	unsigned int i;

	lockdep_assert_held(&inst->block->lock);
	for (i = 0; i < inst->block->num_inst; i++) {
		struct cxl_hmu_inst *sib = inst->block->inst[i];
		u64 b0, b1;

		if (!sib || sib == inst || !sib->active)
			continue;
		if (inst->caps.addr_overlap && sib->caps.addr_overlap)
			continue;
		b0 = sib->live.range_base;
		b1 = b0 + sib->live.range_num;
		if (a0 < b1 && b0 < a1) {
			dev_err(&inst->dev, "range overlaps enabled %s and Address Overlap is not supported\n",
				dev_name(&sib->dev));
			return -EBUSY;
		}
	}
	return 0;
}

static int cxl_hmu_wait_idle(struct cxl_hmu_inst *inst)
{
	u64 status;

	/* Timeouts are a guess at what real hardware needs: 0.1 s */
	return readq_poll_timeout_atomic(inst->base + CHMU_INST0_STATUS_REG, status,
			FIELD_GET(CHMU_INST0_STATUS_OP_INPROG_MSK, status) == 0,
			10, 100000);
}

/* Clear the RW1C status bits, which inhibit the interrupt until cleared */
static void cxl_hmu_ack(struct cxl_hmu_inst *inst)
{
	u64 status = readq(inst->base + CHMU_INST0_STATUS_REG);

	writeq(status & (CHMU_INST0_STATUS_OVRFLW | CHMU_INST0_STATUS_FILLTHRESH),
	       inst->base + CHMU_INST0_STATUS_REG);
}

static int cxl_hmu_disable_locked(struct cxl_hmu_inst *inst)
{
	u64 val = readq(inst->base + CHMU_INST0_CFG0_REG);

	val &= ~CHMU_INST0_CFG0_ENABLE;
	writeq(val, inst->base + CHMU_INST0_CFG0_REG);
	return cxl_hmu_wait_idle(inst);
}

/*
 * Program @cfg and enable the instance. Called with the instance and
 * block locks held, so the ownership check, the overlap check and the
 * enable are one step for everyone else.
 */
static int cxl_hmu_hw_start_locked(struct cxl_hmu_inst *inst, const struct cxl_hmu_config *cfg)
{
	const struct cxl_hmu_caps *caps = &inst->caps;
	u32 nwords = DIV_ROUND_UP(caps->capacity_ranges, 64), w;
	u64 val, bitmap_base;
	u8 width;
	int rc;

	/* Reset here disrupts sampling with -F, should we avoid doing so? */
	writeq(FIELD_PREP(CHMU_INST0_CFG0_RESET_COUNTERS, 1),
	       inst->base + CHMU_INST0_CFG0_REG);
	rc = cxl_hmu_wait_idle(inst);
	if (rc) {
		dev_dbg(&inst->dev, "reset timed out\n");
		return rc;
	}

	val = FIELD_PREP(CHMU_INST0_CFG1_UNIT_SIZE_MSK, cfg->unit_shift) |
		FIELD_PREP(CHMU_INST0_CFG1_DS_FACTOR_MSK, cfg->ds_shift) |
		FIELD_PREP(CHMU_INST0_CFG1_MODE_MSK, cfg->mode) |
		FIELD_PREP(CHMU_INST0_CFG1_EPOCH_SCALE_MSK, cfg->epoch_scale) |
		FIELD_PREP(CHMU_INST0_CFG1_EPOCH_MULT_MSK, cfg->epoch_mult);
	writeq(val, inst->base + CHMU_INST0_CFG1_REG);

	/*
	 * The range bitmap: one bit per 256 MiB of DPA. Every word that
	 * covers the device capacity is written, the configured range set
	 * and the rest clear, since a reset leaves the configuration alone
	 * and a previous session's bits would otherwise stay in force. The
	 * words past the capacity are padding the device may treat as
	 * reserved, so no write lands there.
	 */
	bitmap_base = readq(inst->base + CHMU_INST0_RANGE_BITMAP_OFFSET_REG);
	for (w = 0; w < nwords; w++) {
		u64 lo = (u64)w * 64, hi = lo + 64;
		u64 b0 = max_t(u64, lo, cfg->range_base);
		u64 b1 = min_t(u64, hi, (u64)cfg->range_base + cfg->range_num);

		val = 0;
		if (b0 < b1)
			val = GENMASK_ULL(b1 - lo - 1, b0 - lo);
		writeq(val, inst->base + bitmap_base + w * 8);
	}

	/* Notification at half the hotlist, if the device can raise it */
	if (caps->irq_level)
		writeq(FIELD_PREP(CHMU_INST0_CFG2_FILLTHRESH_THRESHOLD_MSK,
				  caps->hotlist_size / 2),
		       inst->base + CHMU_INST0_CFG2_REG);

	/* The counter width follows the unit size; the threshold must fit it */
	width = FIELD_GET(CHMU_INST0_STATUS_COUNTER_WIDTH_MSK,
			  readq(inst->base + CHMU_INST0_STATUS_REG));
	if (!width || width >= 64)
		return -EIO;
	inst->counter_width = width;
	if (cfg->hotness_threshold >= BIT_ULL(width)) {
		dev_err(&inst->dev, "threshold %u exceeds %u-bit counter\n",
			cfg->hotness_threshold, width);
		return -EINVAL;
	}

	/*
	 * Drop whatever a previous session, the firmware or a kexec'd kernel
	 * left in the hotlist: it was produced under another configuration.
	 * Then clear the status bits so the first interrupt is not inhibited.
	 */
	writew(readw(inst->base + CHMU_INST0_TAIL_REG), inst->base + CHMU_INST0_HEAD_REG);
	cxl_hmu_ack(inst);

	val = FIELD_PREP(CHMU_INST0_CFG0_WHAT_MSK, cfg->access_type) |
		FIELD_PREP(CHMU_INST0_CFG0_RAND_DOWNSAMP_EN, cfg->randomized_ds ? 1 : 0) |
		FIELD_PREP(CHMU_INST0_CFG0_OVRFLW_INT_EN, caps->irq_overflow ? 1 : 0) |
		FIELD_PREP(CHMU_INST0_CFG0_FILLTHRESH_INT_EN, caps->irq_level ? 1 : 0) |
		FIELD_PREP(CHMU_INST0_CFG0_ENABLE, 1) |
		FIELD_PREP(CHMU_INST0_CFG0_HOTNESS_THRESH_MSK, cfg->hotness_threshold);
	writeq(val, inst->base + CHMU_INST0_CFG0_REG);

	rc = cxl_hmu_wait_idle(inst);
	if (rc) {
		dev_info(&inst->dev, "enable timed out\n");
		cxl_hmu_disable_locked(inst);
		return rc;
	}
	inst->live = *cfg;
	inst->live_width = width;
	inst->config = *cfg;
	inst->overflows = 0;
	return 0;
}

/*
 * Disable the instance if @c owns it and it is running; true if it was.
 * Entries still in the hotlist stay there for the caller to drain; units
 * the device never inserted are lost, as the spec says. Does not wait for
 * a delivery in progress: an atomic consumer is serialized by its own
 * lock, a threaded one by cxl_hmu_stop().
 */
bool cxl_hmu_hw_stop(struct cxl_hmu_inst *inst, const struct cxl_hmu_consumer *c)
{
	unsigned long flags;
	bool was_active;

	raw_spin_lock_irqsave(&inst->lock, flags);
	raw_spin_lock(&inst->block->lock);
	was_active = inst->active && inst->consumer == c;
	if (was_active) {
		if (cxl_hmu_disable_locked(inst))
			dev_info(&inst->dev, "disable timed out\n");
		inst->active = false;
	}
	raw_spin_unlock(&inst->block->lock);
	raw_spin_unlock_irqrestore(&inst->lock, flags);
	return was_active;
}

/* With the instance lock held */
static unsigned int __cxl_hmu_drain(struct cxl_hmu_inst *inst, u64 *entries,
				    unsigned int max, bool *overflowed)
{
	u64 hl = readq(inst->base + CHMU_INST0_HOTLIST_OFFSET_REG);
	u64 status = readq(inst->base + CHMU_INST0_STATUS_REG);
	u16 top = inst->caps.hotlist_size;
	u16 head = readw(inst->base + CHMU_INST0_HEAD_REG);
	u16 tail = readw(inst->base + CHMU_INST0_TAIL_REG);
	unsigned int n = 0;

	*overflowed = FIELD_GET(CHMU_INST0_STATUS_OVRFLW, status);
	if (*overflowed)
		inst->overflows++;
	while (head != tail && n < max) {
		entries[n++] = readq(inst->base + hl + head * 8);
		head = (head + 1) % top;
	}
	writew(head, inst->base + CHMU_INST0_HEAD_REG);
	/*
	 * Acknowledge only after the head has moved: the RW1C bits inhibit
	 * further interrupts until cleared, and clearing them with the list
	 * still full would let the next hot unit re-raise an overflow at
	 * once. The spec sets no order; this is the one that loses nothing.
	 */
	writeq(status & (CHMU_INST0_STATUS_OVRFLW | CHMU_INST0_STATUS_FILLTHRESH),
	       inst->base + CHMU_INST0_STATUS_REG);
	return n;
}

/*
 * Copy the hotlist from head to tail into @entries, advance head and
 * acknowledge the status bits. Returns the number of entries copied; the
 * counter width is the running session's, which the entries were produced
 * under.
 */
int cxl_hmu_drain(struct cxl_hmu_inst *inst, u64 *entries, unsigned int max,
		  u8 *counter_width, bool *overflowed)
{
	unsigned long flags;
	unsigned int n;

	raw_spin_lock_irqsave(&inst->lock, flags);
	n = __cxl_hmu_drain(inst, entries, max, overflowed);
	*counter_width = inst->live_width;
	raw_spin_unlock_irqrestore(&inst->lock, flags);
	return n;
}

/*
 * The vector may be shared between instances (the spec only recommends
 * against it), so each handler checks its own status first. The handler
 * and whether it is atomic are read under the lock together with active,
 * so a consumer change cannot route a delivery to the wrong one.
 */
static irqreturn_t cxl_hmu_irq(int irq, void *data)
{
	struct cxl_hmu_inst *inst = data;
	cxl_hmu_hotlist_fn fn;
	bool overflowed;
	u64 status;
	u8 width;
	int n;

	raw_spin_lock(&inst->lock);
	status = readq(inst->base + CHMU_INST0_STATUS_REG);
	if (!FIELD_GET(CHMU_INST0_STATUS_OVRFLW, status) &&
	    !FIELD_GET(CHMU_INST0_STATUS_FILLTHRESH, status)) {
		raw_spin_unlock(&inst->lock);
		return IRQ_NONE;
	}
	if (!inst->active || !inst->hotlist) {
		/* A retired session's interrupt: clear it, it inhibits the next */
		cxl_hmu_ack(inst);
		raw_spin_unlock(&inst->lock);
		return IRQ_HANDLED;
	}
	if (!inst->hotlist_atomic) {
		raw_spin_unlock(&inst->lock);
		return IRQ_WAKE_THREAD;
	}
	fn = inst->hotlist;
	n = __cxl_hmu_drain(inst, inst->entries, inst->caps.hotlist_size, &overflowed);
	width = inst->live_width;
	raw_spin_unlock(&inst->lock);
	fn(inst, inst->entries, n, width, overflowed);
	return IRQ_HANDLED;
}

static irqreturn_t cxl_hmu_irq_thread(int irq, void *data)
{
	struct cxl_hmu_inst *inst = data;
	cxl_hmu_hotlist_fn fn;
	bool overflowed;
	u8 width;
	int n;

	/*
	 * The drain lock is what a stop waits on, so a consumer is never
	 * called after cxl_hmu_stop() has taken it.
	 */
	mutex_lock(&inst->drain_lock);
	raw_spin_lock_irq(&inst->lock);
	if (!inst->active || !inst->hotlist || inst->hotlist_atomic) {
		cxl_hmu_ack(inst);
		raw_spin_unlock_irq(&inst->lock);
		mutex_unlock(&inst->drain_lock);
		return IRQ_HANDLED;
	}
	fn = inst->hotlist;
	n = __cxl_hmu_drain(inst, inst->entries, inst->caps.hotlist_size, &overflowed);
	width = inst->live_width;
	raw_spin_unlock_irq(&inst->lock);
	fn(inst, inst->entries, n, width, overflowed);
	mutex_unlock(&inst->drain_lock);
	return IRQ_HANDLED;
}

/* Consumers and ownership */

struct cxl_hmu_consumer_entry {
	const struct cxl_hmu_consumer *c;
	struct list_head list;
};

static LIST_HEAD(cxl_hmu_consumers);
static LIST_HEAD(cxl_hmu_insts);
/*
 * The registry, the instance list, consumer switches and config stores.
 * Taken before a consumer's own locks (a switch calls detach) and before
 * an instance's drain lock; never from a consumer's callbacks.
 */
static DEFINE_MUTEX(cxl_hmu_consumers_lock);

static const struct cxl_hmu_consumer *cxl_hmu_find_consumer(const char *name)
{
	struct cxl_hmu_consumer_entry *e;

	lockdep_assert_held(&cxl_hmu_consumers_lock);
	list_for_each_entry(e, &cxl_hmu_consumers, list)
		if (sysfs_streq(name, e->c->name))
			return e->c;
	return NULL;
}

int cxl_hmu_register_consumer(const struct cxl_hmu_consumer *c)
{
	struct cxl_hmu_consumer_entry *e;

	if (!c->name || !c->hotlist || !strcmp(c->name, "none"))
		return -EINVAL;
	e = kzalloc_obj(*e);
	if (!e)
		return -ENOMEM;
	e->c = c;
	guard(mutex)(&cxl_hmu_consumers_lock);
	if (cxl_hmu_find_consumer(c->name)) {
		kfree(e);
		return -EEXIST;
	}
	list_add_tail(&e->list, &cxl_hmu_consumers);
	return 0;
}
EXPORT_SYMBOL_NS_GPL(cxl_hmu_register_consumer, "CXL");

/* A consumer in this module must not pin it, or it could never unload */
static bool cxl_hmu_consumer_get(const struct cxl_hmu_consumer *c)
{
	return c->owner == THIS_MODULE || try_module_get(c->owner);
}

static void cxl_hmu_consumer_put(const struct cxl_hmu_consumer *c)
{
	if (c->owner != THIS_MODULE)
		module_put(c->owner);
}

/*
 * Take the instance away from its owner. Refused while the owner has it
 * running, unless @force, which is for an instance that is going away:
 * the owner is then stopped on its behalf. The owner loses the instance
 * before it is told, and cxl_hmu_start() checks ownership in the same
 * critical section that enables, so it cannot restart it.
 */
static int cxl_hmu_detach_locked(struct cxl_hmu_inst *inst, bool force)
{
	const struct cxl_hmu_consumer *old = inst->consumer;
	unsigned long flags;

	lockdep_assert_held(&cxl_hmu_consumers_lock);
	if (!old)
		return 0;
	raw_spin_lock_irqsave(&inst->lock, flags);
	if (inst->active && !force) {
		raw_spin_unlock_irqrestore(&inst->lock, flags);
		return -EBUSY;
	}
	inst->consumer = NULL;
	raw_spin_unlock_irqrestore(&inst->lock, flags);
	if (force)
		cxl_hmu_stop(inst);
	if (old->detach)
		old->detach(inst);
	cxl_hmu_consumer_put(old);
	return 0;
}

static int cxl_hmu_attach_locked(struct cxl_hmu_inst *inst, const struct cxl_hmu_consumer *new)
{
	unsigned long flags;
	int rc;

	lockdep_assert_held(&cxl_hmu_consumers_lock);
	if (!cxl_hmu_consumer_get(new))
		return -ENODEV;
	/* The owner first, so what attach sets up can be used at once */
	raw_spin_lock_irqsave(&inst->lock, flags);
	inst->consumer = new;
	raw_spin_unlock_irqrestore(&inst->lock, flags);
	if (new->attach) {
		rc = new->attach(inst);
		if (rc) {
			raw_spin_lock_irqsave(&inst->lock, flags);
			inst->consumer = NULL;
			raw_spin_unlock_irqrestore(&inst->lock, flags);
			cxl_hmu_consumer_put(new);
			return rc;
		}
	}
	return 0;
}

int cxl_hmu_set_consumer(struct cxl_hmu_inst *inst, const char *name)
{
	const struct cxl_hmu_consumer *new = NULL;
	int rc;

	guard(mutex)(&cxl_hmu_consumers_lock);
	if (inst->dead)
		return -ENODEV;
	if (!sysfs_streq(name, "none")) {
		new = cxl_hmu_find_consumer(name);
		if (!new)
			return -ENOENT;
	}
	if (new == inst->consumer)
		return 0;
	/* Nothing can be delivered without an interrupt; a polling mode could lift this */
	if (new && inst->irq < 0)
		return -EOPNOTSUPP;
	rc = cxl_hmu_detach_locked(inst, false);
	if (rc)
		return rc;
	if (!new)
		return 0;
	return cxl_hmu_attach_locked(inst, new);
}

/*
 * For a consumer that programs its own unit size ahead of a start, from
 * process context: serialized with switches and stores, and refused unless
 * @c owns the instance.
 */
int cxl_hmu_consumer_set_unit_size(struct cxl_hmu_inst *inst,
				   const struct cxl_hmu_consumer *c, u8 unit_shift)
{
	guard(mutex)(&cxl_hmu_consumers_lock);
	guard(mutex)(&inst->drain_lock);
	if (inst->dead)
		return -ENODEV;
	if (inst->consumer != c)
		return -EPERM;
	return cxl_hmu_set_unit_size(inst, unit_shift);
}
EXPORT_SYMBOL_NS_GPL(cxl_hmu_consumer_set_unit_size, "CXL");

int cxl_hmu_start(struct cxl_hmu_inst *inst, const struct cxl_hmu_consumer *c,
		  const struct cxl_hmu_config *cfg)
{
	unsigned long flags;
	int rc;

	/*
	 * A consumer that may sleep starts from process context, where the
	 * start is serialized with config stores, which hold the drain lock;
	 * ownership is checked under the instance lock below, so a switch
	 * needs no other lock here (and taking the consumers lock would
	 * invert a consumer's own lock against a switch). An atomic one
	 * (perf) starts with interrupts off and relies on its configuration
	 * being read-only in sysfs while it owns the instance.
	 */
	if (!c->atomic)
		mutex_lock(&inst->drain_lock);
	rc = cxl_hmu_validate_config(inst, cfg);
	if (rc)
		goto out;
	raw_spin_lock_irqsave(&inst->lock, flags);
	raw_spin_lock(&inst->block->lock);
	if (inst->dead)
		rc = -ENODEV;
	else if (inst->consumer != c)
		rc = -EPERM;
	else if (inst->active)
		rc = -EBUSY;
	else
		rc = cxl_hmu_check_overlap(inst, cfg);
	if (!rc)
		rc = cxl_hmu_hw_start_locked(inst, cfg);
	if (!rc) {
		inst->hotlist = c->hotlist;
		inst->hotlist_atomic = c->atomic;
		inst->active = true;
	}
	raw_spin_unlock(&inst->block->lock);
	raw_spin_unlock_irqrestore(&inst->lock, flags);
out:
	if (!c->atomic)
		mutex_unlock(&inst->drain_lock);
	return rc;
}
EXPORT_SYMBOL_NS_GPL(cxl_hmu_start, "CXL");

/*
 * For a consumer whose handler may sleep: disable, wait out a delivery in
 * progress, deliver what the device still holds. An atomic consumer stops
 * from the context that excludes its handler and drains there itself.
 */
void cxl_hmu_stop(struct cxl_hmu_inst *inst)
{
	cxl_hmu_hotlist_fn fn = NULL;
	unsigned long flags;
	bool overflowed;
	u8 width;
	int n;

	raw_spin_lock_irqsave(&inst->lock, flags);
	raw_spin_lock(&inst->block->lock);
	if (inst->active) {
		if (cxl_hmu_disable_locked(inst))
			dev_info(&inst->dev, "disable timed out\n");
		inst->active = false;
		if (!inst->hotlist_atomic)
			fn = inst->hotlist;
	}
	inst->hotlist = NULL;
	raw_spin_unlock(&inst->block->lock);
	raw_spin_unlock_irqrestore(&inst->lock, flags);

	if (inst->irq >= 0)
		synchronize_irq(inst->irq);
	mutex_lock(&inst->drain_lock);
	if (fn) {
		n = cxl_hmu_drain(inst, inst->entries, inst->caps.hotlist_size,
				  &width, &overflowed);
		fn(inst, inst->entries, n, width, overflowed);
	}
	mutex_unlock(&inst->drain_lock);
}
EXPORT_SYMBOL_NS_GPL(cxl_hmu_stop, "CXL");

void cxl_hmu_unregister_consumer(const struct cxl_hmu_consumer *c)
{
	struct cxl_hmu_consumer_entry *e, *tmp;
	struct cxl_hmu_inst *inst;

	guard(mutex)(&cxl_hmu_consumers_lock);
	list_for_each_entry(inst, &cxl_hmu_insts, list)
		if (inst->consumer == c)
			cxl_hmu_detach_locked(inst, true);
	list_for_each_entry_safe(e, tmp, &cxl_hmu_consumers, list) {
		if (e->c == c) {
			list_del(&e->list);
			kfree(e);
		}
	}
}
EXPORT_SYMBOL_NS_GPL(cxl_hmu_unregister_consumer, "CXL");

const struct cxl_hmu_caps *cxl_hmu_caps(const struct cxl_hmu_inst *inst)
{
	return &inst->caps;
}
EXPORT_SYMBOL_NS_GPL(cxl_hmu_caps, "CXL");

const struct cxl_hmu_config *cxl_hmu_config(const struct cxl_hmu_inst *inst)
{
	return &inst->config;
}
EXPORT_SYMBOL_NS_GPL(cxl_hmu_config, "CXL");

void *cxl_hmu_consumer_data(struct cxl_hmu_inst *inst)
{
	return inst->consumer_data;
}
EXPORT_SYMBOL_NS_GPL(cxl_hmu_consumer_data, "CXL");

void cxl_hmu_set_consumer_data(struct cxl_hmu_inst *inst, void *data)
{
	inst->consumer_data = data;
}
EXPORT_SYMBOL_NS_GPL(cxl_hmu_set_consumer_data, "CXL");

/* Sysfs */

static const char * const cxl_hmu_mode_names[] = { "epoch", "always_on" };
/* Index n is the CFG0 WHAT encoding n + 1 */
static const char * const cxl_hmu_access_names[] = {
	"nontee_r", "nontee_w", "nontee_rw", "r", "w", "rw",
};

static const char *cxl_hmu_access_name(u8 what)
{
	if (what < CHMU_INST0_CFG0_WHAT_NONTEE_R || what > CHMU_INST0_CFG0_WHAT_RW)
		return "none";
	return cxl_hmu_access_names[what - 1];
}

static ssize_t consumer_show(struct device *dev, struct device_attribute *a, char *buf)
{
	struct cxl_hmu_inst *inst = to_cxl_hmu_inst(dev);
	const struct cxl_hmu_consumer *c;

	guard(mutex)(&cxl_hmu_consumers_lock);
	c = inst->consumer;
	return sysfs_emit(buf, "%s\n", c ? c->name : "none");
}

static ssize_t consumer_store(struct device *dev, struct device_attribute *a,
			      const char *buf, size_t len)
{
	int rc = cxl_hmu_set_consumer(to_cxl_hmu_inst(dev), buf);

	return rc ? rc : len;
}
static DEVICE_ATTR_RW(consumer);

/* perf programs its own configuration from the event; it is read-only then */
static bool cxl_hmu_config_writable(struct cxl_hmu_inst *inst)
{
	return !inst->consumer || strcmp(inst->consumer->name, "perf");
}

/*
 * A store takes the consumers lock, against switches and a start that may
 * sleep, and the drain lock, against a delivery or the final drain of a
 * stop decoding under the configuration it would change.
 */
#define CXL_HMU_CFG_ATTR(_name, _fmt, _val, _parse)				\
static ssize_t _name##_show(struct device *dev, struct device_attribute *a,	\
			    char *buf)						\
{										\
	struct cxl_hmu_inst *inst = to_cxl_hmu_inst(dev);			\
										\
	return sysfs_emit(buf, _fmt "\n", _val);				\
}										\
static ssize_t _name##_store(struct device *dev, struct device_attribute *a,	\
			     const char *buf, size_t len)			\
{										\
	struct cxl_hmu_inst *inst = to_cxl_hmu_inst(dev);			\
	struct cxl_hmu_config cfg, old;						\
	int rc;									\
										\
	guard(mutex)(&cxl_hmu_consumers_lock);					\
	guard(mutex)(&inst->drain_lock);					\
	if (inst->dead)								\
		return -ENODEV;							\
	if (!cxl_hmu_config_writable(inst))					\
		return -EPERM;							\
	if (READ_ONCE(inst->active))						\
		return -EBUSY;							\
	old = inst->config;							\
	cfg = old;								\
	rc = _parse(inst, &cfg, buf);						\
	if (rc)									\
		return rc;							\
	rc = cxl_hmu_validate_config(inst, &cfg);				\
	if (rc) {								\
		if (cfg.unit_shift != old.unit_shift &&				\
		    cxl_hmu_set_unit_size(inst, old.unit_shift))		\
			dev_warn(dev, "could not restore the unit size\n");	\
		return rc;							\
	}									\
	inst->config = cfg;							\
	return len;								\
}										\
static DEVICE_ATTR_RW(_name)

static int parse_mode(struct cxl_hmu_inst *inst, struct cxl_hmu_config *cfg, const char *buf)
{
	int i = sysfs_match_string(cxl_hmu_mode_names, buf);

	if (i < 0)
		return i;
	cfg->mode = i;
	return 0;
}
CXL_HMU_CFG_ATTR(mode, "%s", cxl_hmu_mode_names[inst->config.mode], parse_mode);

static int parse_unit_size(struct cxl_hmu_inst *inst, struct cxl_hmu_config *cfg, const char *buf)
{
	unsigned long v;
	int rc;

	if (kstrtoul(buf, 0, &v) || !is_power_of_2(v) || v < 256 || v >= BIT_ULL(40))
		return -EINVAL;
	if (!(inst->caps.unit_sizes & BIT(ilog2(v) - 8)))
		return -EOPNOTSUPP;
	/* Programs the disabled instance and refreshes counter_width */
	rc = cxl_hmu_set_unit_size(inst, ilog2(v));
	if (rc)
		return rc;
	cfg->unit_shift = ilog2(v);
	return 0;
}
CXL_HMU_CFG_ATTR(unit_size, "%lu", 1UL << inst->config.unit_shift, parse_unit_size);

static int parse_hotness_threshold(struct cxl_hmu_inst *inst, struct cxl_hmu_config *cfg,
				   const char *buf)
{
	u32 v;

	if (kstrtou32(buf, 0, &v))
		return -EINVAL;
	/* The width for the current unit_size */
	if (inst->counter_width && v >= BIT_ULL(inst->counter_width))
		return -ERANGE;
	cfg->hotness_threshold = v;
	return 0;
}
CXL_HMU_CFG_ATTR(hotness_threshold, "%u", inst->config.hotness_threshold, parse_hotness_threshold);

static int parse_epoch_ms(struct cxl_hmu_inst *inst, struct cxl_hmu_config *cfg, const char *buf)
{
	unsigned long v;

	if (kstrtoul(buf, 0, &v) || !v)
		return -EINVAL;
	if (v > ULONG_MAX / USEC_PER_MSEC)
		v = ULONG_MAX / USEC_PER_MSEC;
	cxl_hmu_pick_epoch(inst, (u64)v * USEC_PER_MSEC, &cfg->epoch_scale, &cfg->epoch_mult);
	return cfg->epoch_mult ? 0 : -ERANGE;
}
CXL_HMU_CFG_ATTR(epoch_ms, "%llu",
		 cxl_hmu_epoch_us(inst->config.epoch_scale,
				  inst->config.epoch_mult) / USEC_PER_MSEC,
		 parse_epoch_ms);

static int parse_access_type(struct cxl_hmu_inst *inst, struct cxl_hmu_config *cfg, const char *buf)
{
	int i = sysfs_match_string(cxl_hmu_access_names, buf);

	if (i < 0)
		return i;
	cfg->access_type = i + 1;
	return 0;
}
CXL_HMU_CFG_ATTR(access_type, "%s", cxl_hmu_access_name(inst->config.access_type),
		 parse_access_type);

static int parse_downsampling_factor(struct cxl_hmu_inst *inst, struct cxl_hmu_config *cfg,
				     const char *buf)
{
	unsigned long v;

	if (kstrtoul(buf, 0, &v) || !is_power_of_2(v) || v >= BIT(16))
		return -EINVAL;
	if (!(inst->caps.ds_factors & BIT(ilog2(v))))
		return -EOPNOTSUPP;
	cfg->ds_shift = ilog2(v);
	return 0;
}
CXL_HMU_CFG_ATTR(downsampling_factor, "%u", 1U << inst->config.ds_shift, parse_downsampling_factor);

static int parse_randomized_downsampling(struct cxl_hmu_inst *inst, struct cxl_hmu_config *cfg,
					 const char *buf)
{
	return kstrtobool(buf, &cfg->randomized_ds);
}
CXL_HMU_CFG_ATTR(randomized_downsampling, "%d", inst->config.randomized_ds,
		 parse_randomized_downsampling);

static int parse_range_base(struct cxl_hmu_inst *inst, struct cxl_hmu_config *cfg, const char *buf)
{
	return kstrtou32(buf, 0, &cfg->range_base);
}
CXL_HMU_CFG_ATTR(range_base, "%u", inst->config.range_base, parse_range_base);

static int parse_range_size(struct cxl_hmu_inst *inst, struct cxl_hmu_config *cfg, const char *buf)
{
	return kstrtou32(buf, 0, &cfg->range_num);
}
CXL_HMU_CFG_ATTR(range_size, "%u", inst->config.range_num, parse_range_size);

#define CXL_HMU_RO_ATTR(_name, _fmt, _val)					\
static ssize_t _name##_show(struct device *dev, struct device_attribute *a,	\
			    char *buf)						\
{										\
	struct cxl_hmu_inst *inst = to_cxl_hmu_inst(dev);			\
										\
	return sysfs_emit(buf, _fmt "\n", _val);				\
}										\
static DEVICE_ATTR_RO(_name)

static ssize_t supported_modes_show(struct device *dev, struct device_attribute *a, char *buf)
{
	const struct cxl_hmu_caps *c = &to_cxl_hmu_inst(dev)->caps;

	return sysfs_emit(buf, "%s%s%s\n", c->epoch ? "epoch" : "",
			  c->epoch && c->always_on ? " " : "",
			  c->always_on ? "always_on" : "");
}
static DEVICE_ATTR_RO(supported_modes);

static ssize_t supported_unit_sizes_show(struct device *dev, struct device_attribute *a,
					 char *buf)
{
	const struct cxl_hmu_caps *c = &to_cxl_hmu_inst(dev)->caps;
	unsigned long bits = c->unit_sizes;
	ssize_t len = 0;
	unsigned int n;

	for_each_set_bit(n, &bits, 32)
		len += sysfs_emit_at(buf, len, "%s%lu", len ? " " : "", 256UL << n);
	return len + sysfs_emit_at(buf, len, "\n");
}
static DEVICE_ATTR_RO(supported_unit_sizes);

static ssize_t supported_access_types_show(struct device *dev, struct device_attribute *a,
					   char *buf)
{
	const struct cxl_hmu_caps *c = &to_cxl_hmu_inst(dev)->caps;
	unsigned long bits = c->access_types;
	ssize_t len = 0;
	unsigned int n;

	for_each_set_bit(n, &bits, ARRAY_SIZE(cxl_hmu_access_names))
		len += sysfs_emit_at(buf, len, "%s%s", len ? " " : "", cxl_hmu_access_names[n]);
	return len + sysfs_emit_at(buf, len, "\n");
}
static DEVICE_ATTR_RO(supported_access_types);

static ssize_t supported_downsampling_show(struct device *dev, struct device_attribute *a,
					   char *buf)
{
	const struct cxl_hmu_caps *c = &to_cxl_hmu_inst(dev)->caps;
	unsigned long bits = c->ds_factors;
	ssize_t len = 0;
	unsigned int n;

	for_each_set_bit(n, &bits, 16)
		len += sysfs_emit_at(buf, len, "%s%u", len ? " " : "", 1U << n);
	return len + sysfs_emit_at(buf, len, "\n");
}
static DEVICE_ATTR_RO(supported_downsampling);

static ssize_t interrupts_show(struct device *dev, struct device_attribute *a, char *buf)
{
	const struct cxl_hmu_caps *c = &to_cxl_hmu_inst(dev)->caps;

	return sysfs_emit(buf, "%s%s%s\n", c->irq_overflow ? "overflow" : "",
			  c->irq_overflow && c->irq_level ? " " : "",
			  c->irq_level ? "level" : "");
}
static DEVICE_ATTR_RO(interrupts);

static ssize_t epoch_range_ms_show(struct device *dev, struct device_attribute *a, char *buf)
{
	const struct cxl_hmu_caps *c = &to_cxl_hmu_inst(dev)->caps;

	return sysfs_emit(buf, "%llu %llu\n", c->epoch_min_us / USEC_PER_MSEC,
			  c->epoch_max_us / USEC_PER_MSEC);
}
static DEVICE_ATTR_RO(epoch_range_ms);

CXL_HMU_RO_ATTR(hotlist_size, "%u", inst->caps.hotlist_size);
CXL_HMU_RO_ATTR(capacity_ranges, "%u", inst->caps.capacity_ranges);
CXL_HMU_RO_ATTR(epoch_type, "%s", inst->caps.epoch_type == CHMU_INST0_CAP0_EPOCH_TYPE_PERCNT ?
		"per_counter" : "global");
CXL_HMU_RO_ATTR(address_overlap, "%d", inst->caps.addr_overlap);
CXL_HMU_RO_ATTR(postponed_overflow, "%d", inst->caps.postponed_overflow);
CXL_HMU_RO_ATTR(counter_width, "%u", inst->counter_width);
CXL_HMU_RO_ATTR(active, "%d", READ_ONCE(inst->active));
CXL_HMU_RO_ATTR(overflows, "%lu", READ_ONCE(inst->overflows));

static struct attribute *cxl_hmu_inst_attrs[] = {
	&dev_attr_consumer.attr,
	&dev_attr_mode.attr,
	&dev_attr_unit_size.attr,
	&dev_attr_hotness_threshold.attr,
	&dev_attr_epoch_ms.attr,
	&dev_attr_access_type.attr,
	&dev_attr_downsampling_factor.attr,
	&dev_attr_randomized_downsampling.attr,
	&dev_attr_range_base.attr,
	&dev_attr_range_size.attr,
	&dev_attr_supported_modes.attr,
	&dev_attr_supported_unit_sizes.attr,
	&dev_attr_supported_access_types.attr,
	&dev_attr_supported_downsampling.attr,
	&dev_attr_epoch_range_ms.attr,
	&dev_attr_hotlist_size.attr,
	&dev_attr_capacity_ranges.attr,
	&dev_attr_epoch_type.attr,
	&dev_attr_address_overlap.attr,
	&dev_attr_postponed_overflow.attr,
	&dev_attr_interrupts.attr,
	&dev_attr_counter_width.attr,
	&dev_attr_active.attr,
	&dev_attr_overflows.attr,
	NULL,
};

static const struct attribute_group cxl_hmu_inst_group = {
	.attrs = cxl_hmu_inst_attrs,
};

static const struct attribute_group *cxl_hmu_inst_groups[] = {
	&cxl_hmu_inst_group,
	NULL,
};

/* Instance devices */

static void cxl_hmu_inst_release(struct device *dev)
{
	kfree(to_cxl_hmu_inst(dev));
}

static const struct device_type cxl_hmu_inst_type = {
	.name = "cxl_hmu_instance",
	.release = cxl_hmu_inst_release,
	.groups = cxl_hmu_inst_groups,
};

bool cxl_hmu_dev_is_inst(struct device *dev)
{
	return dev->type == &cxl_hmu_inst_type;
}

/* What a kernel consumer starts with unless it configures otherwise */
static void cxl_hmu_default_config(struct cxl_hmu_inst *inst)
{
	const struct cxl_hmu_caps *c = &inst->caps;
	struct cxl_hmu_config *cfg = &inst->config;

	cfg->mode = c->epoch ? CXL_HMU_MODE_EPOCH : CXL_HMU_MODE_ALWAYS_ON;
	cfg->hotness_threshold = 1;
	cxl_hmu_pick_epoch(inst, 1000 * USEC_PER_MSEC, &cfg->epoch_scale, &cfg->epoch_mult);
	/* Reads and writes, TEE or not; else non-TEE reads and writes; else what it offers */
	if (c->access_types & BIT(CHMU_INST0_CFG0_WHAT_RW - 1))
		cfg->access_type = CHMU_INST0_CFG0_WHAT_RW;
	else if (c->access_types & BIT(CHMU_INST0_CFG0_WHAT_NONTEE_RW - 1))
		cfg->access_type = CHMU_INST0_CFG0_WHAT_NONTEE_RW;
	else if (c->access_types)
		cfg->access_type = __ffs(c->access_types) + 1;
	cfg->ds_shift = c->ds_factors && !(c->ds_factors & BIT(0)) ? __ffs(c->ds_factors) : 0;
	cfg->randomized_ds = false;
	cfg->range_base = 0;
	cfg->range_num = c->capacity_ranges;
}

static void cxl_hmu_inst_remove(void *data)
{
	struct cxl_hmu_inst *inst = data;

	/* The overlap check walks the table under the block lock */
	scoped_guard(raw_spinlock_irqsave, &inst->block->lock)
		inst->block->inst[inst->index] = NULL;
	device_unregister(&inst->dev);
}

static void cxl_hmu_free_irq(void *data)
{
	struct cxl_hmu_inst *inst = data;

	free_irq(inst->irq, inst);
}

/*
 * First to run at unbind, after the PMU has closed its sessions: no
 * consumer can be chosen or start from here on, and the owner, if any, is
 * stopped and detached on its behalf.
 */
static void cxl_hmu_inst_quiesce(void *data)
{
	struct cxl_hmu_inst *inst = data;
	unsigned long flags;

	guard(mutex)(&cxl_hmu_consumers_lock);
	raw_spin_lock_irqsave(&inst->lock, flags);
	inst->dead = true;
	raw_spin_unlock_irqrestore(&inst->lock, flags);
	cxl_hmu_detach_locked(inst, true);
	list_del(&inst->list);
}

int cxl_hmu_inst_add(struct cxl_hmu *block, unsigned int index, unsigned int inst_len)
{
	struct pci_dev *pdev = to_pci_dev(block->dev.parent);
	struct cxl_hmu_inst *inst;
	int irq, rc;

	if (index >= CXL_HMU_MAX_INST)
		return -EINVAL;
	inst = kzalloc_obj(*inst);
	if (!inst)
		return -ENOMEM;
	inst->block = block;
	inst->index = index;
	inst->base = block->base + 0x10 + inst_len * index;
	inst->irq = -1;
	inst->dead = true;	/* until it can be quiesced */
	raw_spin_lock_init(&inst->lock);
	mutex_init(&inst->drain_lock);
	INIT_LIST_HEAD(&inst->list);
	cxl_hmu_decode_caps(inst);
	cxl_hmu_default_config(inst);

	/* Whatever left it enabled, the configuration is ours to write now */
	if (FIELD_GET(CHMU_INST0_STATUS_ENABLED, readq(inst->base + CHMU_INST0_STATUS_REG)) &&
	    cxl_hmu_disable_locked(inst))
		dev_warn(&block->dev, "instance %u would not disable\n", index);

	device_initialize(&inst->dev);
	device_set_pm_not_required(&inst->dev);
	inst->dev.parent = &block->dev;
	inst->dev.type = &cxl_hmu_inst_type;
	rc = dev_set_name(&inst->dev, "%s.%u", dev_name(&block->dev), index);
	if (rc)
		goto err;
	rc = device_add(&inst->dev);
	if (rc)
		goto err;
	block->inst[index] = inst;
	if (index >= block->num_inst)
		block->num_inst = index + 1;
	rc = devm_add_action_or_reset(&block->dev, cxl_hmu_inst_remove, inst);
	if (rc)
		return rc;

	inst->entries = devm_kcalloc(&block->dev, inst->caps.hotlist_size,
				     sizeof(*inst->entries), GFP_KERNEL);
	if (!inst->entries)
		return -ENOMEM;

	/* 4 KiB or the smallest unit, so the counter width is known from the start */
	if (inst->caps.unit_sizes) {
		u8 shift = inst->caps.unit_sizes & BIT(4) ? 12 : 8 + __ffs(inst->caps.unit_sizes);

		rc = cxl_hmu_set_unit_size(inst, shift);
		if (rc)
			dev_warn(&inst->dev, "no counter width for the default unit size\n");
	}

	/*
	 * The Interrupt Message Number is defined for an instance with the
	 * overflow interrupt; it is used for a level-crossing-only instance
	 * too, as the spec gives such an instance no other vector. Shared,
	 * since two instances may report the same number; not threaded by
	 * force, since an atomic consumer's delivery is a hard interrupt by
	 * design.
	 */
	if (inst->caps.irq_overflow || inst->caps.irq_level) {
		irq = pdev->msi_enabled || pdev->msix_enabled ?
		      pci_irq_vector(pdev, inst->caps.msi) : -ENXIO;
		if (irq < 0) {
			/* No consumer can use it; the device still shows its capabilities */
			dev_warn(&inst->dev, "no vector for interrupt message number %u (%d): consumers refused\n",
				 inst->caps.msi, irq);
			goto no_irq;
		}
		rc = request_threaded_irq(irq, cxl_hmu_irq, cxl_hmu_irq_thread,
					  IRQF_SHARED | IRQF_NO_THREAD | IRQF_NOBALANCING,
					  dev_name(&inst->dev), inst);
		if (rc)
			return rc;
		inst->irq = irq;
		rc = devm_add_action_or_reset(&block->dev, cxl_hmu_free_irq, inst);
		if (rc)
			return rc;
	}
no_irq:
	/* Registered last, so it runs first at unbind, before the interrupt goes */
	rc = devm_add_action_or_reset(&block->dev, cxl_hmu_inst_quiesce, inst);
	if (rc)
		return rc;
	scoped_guard(mutex, &cxl_hmu_consumers_lock) {
		list_add_tail(&inst->list, &cxl_hmu_insts);
		inst->dead = false;
	}
	return 0;
err:
	put_device(&inst->dev);
	return rc;
}

/* The driver */

static int cxl_hmu_probe(struct device *dev)
{
	struct cxl_hmu *block = to_cxl_hmu(dev);
	unsigned int num_inst = FIELD_GET(CHMU_COMMON_CAP0_NUMINST_MSK,
					  readq(block->base + CHMU_COMMON_CAP0_REG));
	unsigned int inst_len = FIELD_GET(CHMU_COMMON_CAP1_INSTLEN_MSK,
					  readq(block->base + CHMU_COMMON_CAP1_REG));
	unsigned int i;
	int rc;

	for (i = 0; i < num_inst; i++) {
		rc = cxl_hmu_inst_add(block, i, inst_len);
		if (rc)
			return rc;
	}
	for (i = 0; i < block->num_inst; i++) {
		struct cxl_hmu_inst *inst = block->inst[i];

		/* Its devm actions run before the quiesce: it closes its sessions first */
		rc = cxl_hmu_perf_probe_inst(inst);
		if (rc)
			return rc;
		/* perf by default where it has a PMU, so the perf tool sees no change */
		rc = cxl_hmu_set_consumer(inst, inst->drvdata ? "perf" : "none");
		if (rc)
			dev_warn(&inst->dev, "no default consumer: %d\n", rc);
	}
	return 0;
}

static struct cxl_driver cxl_hmu_driver = {
	.name = "cxl_hmu",
	.probe = cxl_hmu_probe,
	.id = CXL_DEVICE_HMU,
};

static __init int cxl_hmu_init(void)
{
	int rc;

	rc = cxl_hmu_perf_init();
	if (rc)
		return rc;
	rc = cxl_driver_register(&cxl_hmu_driver);
	if (rc)
		cxl_hmu_perf_exit();
	return rc;
}

static __exit void cxl_hmu_exit(void)
{
	cxl_driver_unregister(&cxl_hmu_driver);
	cxl_hmu_perf_exit();
}

MODULE_AUTHOR("Jonathan Cameron <Jonathan.Cameron@huawei.com>");
MODULE_AUTHOR("Davidlohr Bueso <dave@stgolabs.net>");
MODULE_DESCRIPTION("CXL Hot-range Monitoring Unit driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("CXL");
module_init(cxl_hmu_init);
module_exit(cxl_hmu_exit);
MODULE_ALIAS_CXL(CXL_DEVICE_HMU);
