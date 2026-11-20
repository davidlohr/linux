// SPDX-License-Identifier: GPL-2.0-only
/*
 * CXL Hot-range Monitoring Unit: instance devices and hardware control.
 *
 * A CHMU register block holds up to eight instances, each with its own
 * configuration, hotlist and interrupt (CXL r4.0 Section 8.2.8). Each is
 * registered as a child device of the block. This file is a library over
 * one instance: claim it, start it, drain its hotlist a bounded number of
 * entries at a time, stop it and release it. It delivers nothing and
 * requests no interrupt. The device hotlist is the only queue, and whoever
 * holds the instance decides when to read it: the perf PMU from an
 * interrupt it requests itself, an in-kernel user by polling.
 *
 * Ownership is claim-on-use, first come, first served. The perf PMU holds
 * an instance from event_init to event->destroy, an in-kernel user from
 * its start to its stop. There is no reservation.
 *
 * Configuration comes with the claim: perf's is decoded from the event's
 * attributes. A claim copies its configuration to inst->live and a start
 * programs that copy.
 *
 * Locking, outermost first:
 *
 *   block->lock (mutex)  the owner and the claimed range of every instance
 *                        of the block, so the overlap check of a claim
 *                        sees a stable set of siblings.
 *   inst->lock (raw)     the registers of the instance, enabled and the
 *                        counter width of the session. Raw because perf
 *                        starts, stops and drains with interrupts off, on
 *                        PREEMPT_RT too. Nothing is taken under it.
 *
 * inst->owner and inst->live are written under both, so either is enough
 * to read them. A front-end's own lock is taken before both (perf's raw
 * lock before inst->lock only: perf claims and releases without it).
 */
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/slab.h>

#include "../cxl.h"
#include "../cxlmem.h"
#include "../cxlpci.h"
#include "hmu.h"

#define CREATE_TRACE_POINTS
#include "trace.h"

/* Table 8-192: the epoch scale encodings, in microseconds */
static const u32 cxl_hmu_epoch_scale_us[] = {
	[CHMU_EPOCH_SCALE_100US] = 100,
	[CHMU_EPOCH_SCALE_1MS] = 1000,
	[CHMU_EPOCH_SCALE_10MS] = 10000,
	[CHMU_EPOCH_SCALE_100MS] = 100000,
	[CHMU_EPOCH_SCALE_1S] = 1000000,
};
#define CXL_HMU_EPOCH_MULT_MAX	4095

static u64 cxl_hmu_epoch_us(u8 scale, u16 mult)
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
	struct cxl_memdev_state *mds = to_cxl_memdev_state(inst->block->hmu->cxlmd->cxlds);
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

/* Against the capabilities only: the counter width is not known before a start */
static int cxl_hmu_validate_config(struct cxl_hmu_inst *inst, const struct cxl_hmu_config *cfg)
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
	return 0;
}

/*
 * Unless this instance and a held sibling in the block both advertise
 * Address Overlap, their ranges must be disjoint. The spec defines only
 * the set case of the capability; this is the safe reading of its absence,
 * scoped to one register block as the spec scopes the bit. A driver policy:
 * it compares against the siblings that are held, running or not, since
 * the holder may start at any time, with interrupts off, where the block
 * cannot be searched.
 */
static int cxl_hmu_check_overlap(struct cxl_hmu_inst *inst, const struct cxl_hmu_config *cfg)
{
	struct cxl_hmu_block *block = inst->block;
	u64 a0 = cfg->range_base, a1 = a0 + cfg->range_num;
	unsigned int i;

	lockdep_assert_held(&block->lock);
	for (i = 0; i < block->num_inst; i++) {
		struct cxl_hmu_inst *sib = block->inst[i];
		u64 b0, b1;

		if (!sib || sib == inst || !sib->owner)
			continue;
		if (inst->caps.addr_overlap && sib->caps.addr_overlap)
			continue;
		b0 = sib->live.range_base;
		b1 = b0 + sib->live.range_num;
		if (a0 < b1 && b0 < a1) {
			dev_dbg(&inst->dev, "range overlaps that of %s, held by %s, and Address Overlap is not supported\n",
				dev_name(&sib->dev), sib->owner);
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

static int cxl_hmu_disable(struct cxl_hmu_inst *inst)
{
	u64 val = readq(inst->base + CHMU_INST0_CFG0_REG);

	val &= ~CHMU_INST0_CFG0_ENABLE;
	writeq(val, inst->base + CHMU_INST0_CFG0_REG);
	return cxl_hmu_wait_idle(inst);
}

/* The counter width for the unit size in CFG1, 0 if it makes no sense */
static u8 cxl_hmu_read_width(struct cxl_hmu_inst *inst)
{
	u8 width = FIELD_GET(CHMU_INST0_STATUS_COUNTER_WIDTH_MSK,
			     readq(inst->base + CHMU_INST0_STATUS_REG));

	return width < 64 ? width : 0;
}

/*
 * Program @cfg and enable the instance, with the instance lock held.
 *
 * @irq says who will read the hotlist. A holder that requested the
 * instance's vector gets every interrupt the instance has: the hotlist
 * overflow and the fill level crossing. One that polls gets none, so the
 * vector, which other instances and cxl_pci may share, stays quiet for it.
 * The fill level is programmed either way; the status bits are expected to
 * latch with their interrupts disabled, and a poll reads the overflow one.
 */
static int cxl_hmu_program(struct cxl_hmu_inst *inst, const struct cxl_hmu_config *cfg,
			   bool irq)
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

	/*
	 * The counter width follows the unit size, so it can only be read
	 * now, and the threshold must fit it.
	 */
	width = cxl_hmu_read_width(inst);
	if (!width)
		return -EIO;
	if (cfg->hotness_threshold >= BIT_ULL(width)) {
		dev_err_ratelimited(&inst->dev, "threshold %u exceeds the %u-bit counter of a %llu byte unit\n",
				    cfg->hotness_threshold, width, BIT_ULL(cfg->unit_shift));
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
		FIELD_PREP(CHMU_INST0_CFG0_OVRFLW_INT_EN, irq && caps->irq_overflow ? 1 : 0) |
		FIELD_PREP(CHMU_INST0_CFG0_FILLTHRESH_INT_EN, irq && caps->irq_level ? 1 : 0) |
		FIELD_PREP(CHMU_INST0_CFG0_ENABLE, 1) |
		FIELD_PREP(CHMU_INST0_CFG0_HOTNESS_THRESH_MSK, cfg->hotness_threshold);
	writeq(val, inst->base + CHMU_INST0_CFG0_REG);

	rc = cxl_hmu_wait_idle(inst);
	if (rc) {
		dev_info_ratelimited(&inst->dev, "enable timed out\n");
		cxl_hmu_disable(inst);
		return rc;
	}
	inst->live_width = width;
	inst->overflows = 0;
	inst->dropped = 0;
	return 0;
}

int __cxl_hmu_claim(struct cxl_hmu_inst *inst, const char *who,
		    const struct cxl_hmu_config *cfg)
{
	int rc;

	guard(mutex)(&inst->block->lock);
	if (inst->owner)
		return -EBUSY;
	rc = cxl_hmu_validate_config(inst, cfg);
	if (rc)
		return rc;
	rc = cxl_hmu_check_overlap(inst, cfg);
	if (rc)
		return rc;
	guard(raw_spinlock_irqsave)(&inst->lock);
	inst->live = *cfg;
	inst->owner = who;
	return 0;
}

int cxl_hmu_check_threshold(struct cxl_hmu_inst *inst)
{
	u64 val;
	u8 width;

	guard(raw_spinlock_irqsave)(&inst->lock);
	if (!inst->owner)
		return -EPERM;
	if (inst->enabled)
		return -EBUSY;
	val = readq(inst->base + CHMU_INST0_CFG1_REG);
	val &= ~CHMU_INST0_CFG1_UNIT_SIZE_MSK;
	val |= FIELD_PREP(CHMU_INST0_CFG1_UNIT_SIZE_MSK, inst->live.unit_shift);
	writeq(val, inst->base + CHMU_INST0_CFG1_REG);
	width = cxl_hmu_read_width(inst);
	if (!width)
		return -EIO;
	if (inst->live.hotness_threshold >= BIT_ULL(width)) {
		dev_dbg(&inst->dev, "threshold %u exceeds the %u-bit counter of a %llu byte unit\n",
			inst->live.hotness_threshold, width,
			BIT_ULL(inst->live.unit_shift));
		return -EINVAL;
	}
	return 0;
}

void cxl_hmu_release(struct cxl_hmu_inst *inst)
{
	guard(mutex)(&inst->block->lock);
	cxl_hmu_stop(inst);
	guard(raw_spinlock_irqsave)(&inst->lock);
	inst->owner = NULL;
}

int __cxl_hmu_start(struct cxl_hmu_inst *inst, unsigned int flags)
{
	int rc;

	guard(raw_spinlock_irqsave)(&inst->lock);
	if (!inst->owner)
		return -EPERM;
	if (inst->enabled)
		return -EBUSY;
	rc = cxl_hmu_program(inst, &inst->live, flags & CXL_HMU_START_IRQ);
	if (rc)
		return rc;
	inst->enabled = true;
	return 0;
}

/*
 * Units the device has counted but not yet inserted are lost, as the spec
 * says. The interrupt enables stay as programmed: only a start writes the
 * configuration, and it clears the status first.
 */
void cxl_hmu_stop(struct cxl_hmu_inst *inst)
{
	guard(raw_spinlock_irqsave)(&inst->lock);
	if (!inst->enabled)
		return;
	if (cxl_hmu_disable(inst))
		dev_info(&inst->dev, "disable timed out\n");
	inst->enabled = false;
}

unsigned int cxl_hmu_drain(struct cxl_hmu_inst *inst, u64 *entries,
			   unsigned int max, unsigned int *flags)
{
	u16 top = inst->caps.hotlist_size;
	unsigned int n = 0;
	u16 head, tail;
	u64 hl, status;
	bool overflowed;

	guard(raw_spinlock_irqsave)(&inst->lock);
	hl = readq(inst->base + CHMU_INST0_HOTLIST_OFFSET_REG);
	status = readq(inst->base + CHMU_INST0_STATUS_REG);
	head = readw(inst->base + CHMU_INST0_HEAD_REG);
	tail = readw(inst->base + CHMU_INST0_TAIL_REG);

	*flags = 0;
	overflowed = FIELD_GET(CHMU_INST0_STATUS_OVRFLW, status);
	if (overflowed) {
		inst->overflows++;
		if (!inst->caps.postponed_overflow)
			*flags |= CXL_HMU_DRAIN_LOST;
	}
	while (head != tail && n < max) {
		if (entries)
			entries[n] = readq(inst->base + hl + head * 8);
		n++;
		head = (head + 1) % top;
	}
	if (!entries)
		inst->dropped += n;
	if (head != tail)
		*flags |= CXL_HMU_DRAIN_MORE;
	writew(head, inst->base + CHMU_INST0_HEAD_REG);
	/*
	 * Acknowledge only after the head has moved: the RW1C bits inhibit
	 * further interrupts until cleared, and clearing them with the list
	 * still full would let the next hot unit re-raise an overflow at
	 * once. The spec sets no order; this is the one that loses nothing.
	 * Only the bits read above are cleared, so one set since is kept.
	 */
	writeq(status & (CHMU_INST0_STATUS_OVRFLW | CHMU_INST0_STATUS_FILLTHRESH),
	       inst->base + CHMU_INST0_STATUS_REG);
	trace_cxl_hmu_hotlist(inst, n, inst->live_width, overflowed);
	return n;
}

bool cxl_hmu_pending(struct cxl_hmu_inst *inst)
{
	return readq(inst->base + CHMU_INST0_STATUS_REG) &
		(CHMU_INST0_STATUS_OVRFLW | CHMU_INST0_STATUS_FILLTHRESH);
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

/*
 * Unbind. The actions that unregister the instance's PMU and free the
 * interrupt it requested were added after this one, so they have run:
 * every perf event is closed, which released a claim perf held, and none
 * can be opened.
 */
static void cxl_hmu_inst_remove(void *data)
{
	struct cxl_hmu_inst *inst = data;
	struct cxl_hmu_block *block = inst->block;

	scoped_guard(mutex, &block->lock) {
		WARN_ON_ONCE(inst->owner);
		block->inst[inst->index] = NULL;
	}
	device_unregister(&inst->dev);
}

static int cxl_hmu_inst_add(struct cxl_hmu_block *block, unsigned int index,
			    unsigned int inst_len)
{
	struct device *dev = &block->hmu->dev;
	struct pci_dev *pdev = to_pci_dev(dev->parent);
	struct cxl_hmu_inst *inst;
	int irq, rc;

	if (index >= CXL_HMU_MAX_INST)
		return -EINVAL;
	inst = kzalloc_obj(*inst);
	if (!inst)
		return -ENOMEM;
	inst->block = block;
	inst->index = index;
	inst->base = block->hmu->base + 0x10 + inst_len * index;
	inst->irq = -1;
	raw_spin_lock_init(&inst->lock);
	cxl_hmu_decode_caps(inst);

	/* Whatever left it enabled, the configuration is ours to write now */
	if (FIELD_GET(CHMU_INST0_STATUS_ENABLED, readq(inst->base + CHMU_INST0_STATUS_REG)) &&
	    cxl_hmu_disable(inst))
		dev_warn(dev, "instance %u would not disable\n", index);

	device_initialize(&inst->dev);
	device_set_pm_not_required(&inst->dev);
	inst->dev.parent = dev;
	inst->dev.type = &cxl_hmu_inst_type;
	rc = dev_set_name(&inst->dev, "%s.%u", dev_name(dev), index);
	if (rc)
		goto err;
	/* Under the lock: a claim checks the siblings in the table */
	scoped_guard(mutex, &block->lock) {
		block->inst[index] = inst;
		block->num_inst = max(block->num_inst, index + 1);
	}
	rc = device_add(&inst->dev);
	if (rc) {
		scoped_guard(mutex, &block->lock)
			block->inst[index] = NULL;
		goto err;
	}
	rc = devm_add_action_or_reset(dev, cxl_hmu_inst_remove, inst);
	if (rc)
		return rc;

	/*
	 * The Interrupt Message Number is defined for an instance with the
	 * overflow interrupt; it is used for a level-crossing-only instance
	 * too, as the spec gives such an instance no other vector. Only
	 * looked up here: whoever wants the interrupt requests it, shared,
	 * since two instances may report the same number. Without a vector
	 * the instance can still be polled.
	 */
	if (inst->caps.irq_overflow || inst->caps.irq_level) {
		irq = pdev->msi_enabled || pdev->msix_enabled ?
		      pci_irq_vector(pdev, inst->caps.msi) : -ENXIO;
		if (irq < 0)
			dev_warn(&inst->dev, "no vector for interrupt message number %u (%d): no interrupt, no PMU\n",
				 inst->caps.msi, irq);
		else
			inst->irq = irq;
	}
	return 0;
err:
	put_device(&inst->dev);
	return rc;
}

/* The driver */

static int cxl_hmu_probe(struct device *dev)
{
	struct cxl_hmu *hmu = to_cxl_hmu(dev);
	unsigned int num_inst = FIELD_GET(CHMU_COMMON_CAP0_NUMINST_MSK,
					  readq(hmu->base + CHMU_COMMON_CAP0_REG));
	unsigned int inst_len = FIELD_GET(CHMU_COMMON_CAP1_INSTLEN_MSK,
					  readq(hmu->base + CHMU_COMMON_CAP1_REG));
	struct cxl_hmu_block *block;
	unsigned int i;
	int rc;

	block = devm_kzalloc_obj(dev, *block);
	if (!block)
		return -ENOMEM;
	block->hmu = hmu;
	rc = devm_mutex_init(dev, &block->lock);
	if (rc)
		return rc;

	for (i = 0; i < num_inst; i++) {
		rc = cxl_hmu_inst_add(block, i, inst_len);
		if (rc)
			return rc;
	}
	/*
	 * After every instance, so that at unbind every PMU is gone before
	 * the first instance is: see cxl_hmu_inst_remove().
	 */
	for (i = 0; i < block->num_inst; i++) {
		rc = cxl_hmu_perf_probe_inst(block->inst[i]);
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
