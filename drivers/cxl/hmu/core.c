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
 * Disable the instance if it is running; true if it was. Entries still in
 * the hotlist stay there for the caller to drain; units the device never
 * inserted are lost, as the spec says. Does not wait for a delivery in
 * progress: an atomic handler is serialized by its caller's lock.
 */
bool cxl_hmu_hw_stop(struct cxl_hmu_inst *inst)
{
	unsigned long flags;
	bool was_active;

	raw_spin_lock_irqsave(&inst->lock, flags);
	raw_spin_lock(&inst->block->lock);
	was_active = inst->active;
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
	 * called after a stop has taken it.
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

/*
 * Program @cfg, enable the instance and install @fn as the handler of its
 * hotlist, atomic or not; one critical section with the overlap check.
 */
int cxl_hmu_hw_start(struct cxl_hmu_inst *inst, const struct cxl_hmu_config *cfg,
		     cxl_hmu_hotlist_fn fn, bool atomic)
{
	unsigned long flags;
	int rc;

	rc = cxl_hmu_validate_config(inst, cfg);
	if (rc)
		return rc;
	raw_spin_lock_irqsave(&inst->lock, flags);
	raw_spin_lock(&inst->block->lock);
	if (inst->active)
		rc = -EBUSY;
	else
		rc = cxl_hmu_check_overlap(inst, cfg);
	if (!rc)
		rc = cxl_hmu_hw_start_locked(inst, cfg);
	if (!rc) {
		inst->hotlist = fn;
		inst->hotlist_atomic = atomic;
		inst->active = true;
	}
	raw_spin_unlock(&inst->block->lock);
	raw_spin_unlock_irqrestore(&inst->lock, flags);
	return rc;
}

/* Instance devices */

static void cxl_hmu_inst_release(struct device *dev)
{
	kfree(to_cxl_hmu_inst(dev));
}

static const struct device_type cxl_hmu_inst_type = {
	.name = "cxl_hmu_instance",
	.release = cxl_hmu_inst_release,
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
	raw_spin_lock_init(&inst->lock);
	mutex_init(&inst->drain_lock);
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

		rc = cxl_hmu_perf_probe_inst(inst);
		if (rc)
			return rc;
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
