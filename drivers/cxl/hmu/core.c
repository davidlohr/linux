// SPDX-License-Identifier: GPL-2.0-only
/*
 * CXL Hot-range Monitoring Unit: instance devices and hardware control.
 *
 * A CHMU register block holds up to eight instances, each with its own
 * configuration, hotlist and interrupt (CXL r4.0 Section 8.2.8). Each is
 * registered as a child device of the block. This file is a library over
 * one instance: claim it, start it, drain its hotlist a bounded number of
 * entries at a time, stop it, release it, and translate an entry to host
 * physical ranges. The device hotlist is the only queue, and whoever holds
 * the instance decides when to read it: the perf PMU from an interrupt it
 * requests itself, an in-kernel client by polling or from a notify.
 *
 * In-kernel clients use the interface of include/cxl/hmu.h and nothing
 * else. The core knows none of them: a client registers, is told of every
 * instance and claims the ones it wants. perf is part of this module and
 * uses the library directly (see hmu.h).
 *
 * Ownership is claim-on-use, first come, first served. The perf PMU holds
 * an instance from event_init to event->destroy, a client from its claim
 * to its release, which also start and stop the instance. There is no
 * reservation.
 *
 * Configuration has one writer per holder. perf's is decoded from the
 * event's attributes and handed to the claim. A client's is the one
 * staged in sysfs, which only the sysfs stores write. A claim copies its
 * configuration to inst->live and a start programs that copy, so no
 * session changes what the next client starts with.
 *
 * Locking, outermost first:
 *
 *   cxl_hmu_clients_lock (mutex)  the registered clients, the probed
 *                        blocks and the binds of every instance. A
 *                        client's add() and remove() run under it, so it
 *                        is taken before whatever they take; nothing else
 *                        of the client interface takes it.
 *   block->lock (mutex)  the owner, the staged configuration and the
 *                        claimed range of every instance of the block, so
 *                        the overlap check of a claim sees a stable set of
 *                        siblings.
 *   inst->lock (raw)     the registers of the instance, enabled and the
 *                        counter width of the session. Raw because perf
 *                        starts, stops and drains with interrupts off, on
 *                        PREEMPT_RT too. Nothing is taken under it.
 *
 * inst->owner and inst->live are written under both, so either is enough
 * to read them. A holder's own lock is taken before both (perf's raw lock
 * before inst->lock only: perf claims and releases without it). The HPA
 * walk takes neither: it tries the memdev lock and cxl_rwsem in the region
 * core and reads what is fixed while its caller holds the claim.
 */
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/kstrtox.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/range.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/string.h>
#include <linux/sysfs.h>

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

/* A unit above @max_unit_size, if not 0, is -EINVAL */
static int cxl_hmu_take(struct cxl_hmu_inst *inst, const char *who,
			const struct cxl_hmu_config *cfg, u64 max_unit_size)
{
	int rc;

	guard(mutex)(&inst->block->lock);
	if (inst->owner)
		return -EBUSY;
	if (!cfg)
		cfg = &inst->config;
	rc = cxl_hmu_validate_config(inst, cfg);
	if (rc)
		return rc;
	if (max_unit_size && BIT_ULL(cfg->unit_shift) > max_unit_size) {
		dev_dbg(&inst->dev, "unit size %llu is above %llu\n",
			BIT_ULL(cfg->unit_shift), max_unit_size);
		return -EINVAL;
	}
	rc = cxl_hmu_check_overlap(inst, cfg);
	if (rc)
		return rc;
	guard(raw_spinlock_irqsave)(&inst->lock);
	inst->live = *cfg;
	inst->owner = who;
	return 0;
}

int __cxl_hmu_claim(struct cxl_hmu_inst *inst, const char *who,
		    const struct cxl_hmu_config *cfg)
{
	return cxl_hmu_take(inst, who, cfg, 0);
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

/**
 * cxl_hmu_release() - Stop an instance and give it up
 * @inst: the instance, held by the caller
 *
 * Disables the instance if it runs and, if it was claimed with a notify,
 * frees the interrupt: the notify function is not running when this
 * returns, and is not called again. The instance is then free for the
 * next claim. What its hotlist held is lost.
 *
 * Tear down in this order: first stop the client's own reader, that is
 * disable or cancel, and wait for, the work a notify queues or a poll
 * runs, so that no cxl_hmu_read_ranges() is in flight and none starts;
 * then release. A release or a read after the release would act on the
 * next holder's session.
 *
 * Context: process context, may sleep. For the holder only: an instance
 * that nobody holds is warned about and left alone.
 */
void cxl_hmu_release(struct cxl_hmu_inst *inst)
{
	guard(mutex)(&inst->block->lock);
	if (WARN_ON_ONCE(!inst->owner))
		return;
	cxl_hmu_stop(inst);
	/* The handler does nothing for a stopped instance; this waits for one running */
	if (inst->notify) {
		free_irq(inst->irq, inst);
		inst->notify = NULL;
		inst->notify_arg = NULL;
	}
	guard(raw_spinlock_irqsave)(&inst->lock);
	inst->owner = NULL;
}
EXPORT_SYMBOL_NS_GPL(cxl_hmu_release, "CXL_HMU");

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

void cxl_hmu_account_dropped(struct cxl_hmu_inst *inst, unsigned int nr)
{
	guard(raw_spinlock_irqsave)(&inst->lock);
	inst->dropped += nr;
}

bool cxl_hmu_pending(struct cxl_hmu_inst *inst)
{
	return readq(inst->base + CHMU_INST0_STATUS_REG) &
		(CHMU_INST0_STATUS_OVRFLW | CHMU_INST0_STATUS_FILLTHRESH);
}

/* One page of HPA ranges: a 2 MiB unit over 8 KiB interleave granules */
#define CXL_HMU_WALK_RANGES	(PAGE_SIZE / sizeof(struct range))

/* One read: where its ranges go, and the walk's buffer */
struct cxl_hmu_reader {
	cxl_hmu_range_fn	fn;
	void			*arg;
	unsigned int		nr_ranges;	/* handed over for the current entry */
	struct range		*hpa;		/* CXL_HMU_WALK_RANGES */
};

/*
 * Hand one HPA range to the reader, unless none of it is System RAM: a
 * region that is device-dax, or that nothing has claimed, has no page the
 * reader could act on, and walking its page frames to find that out is the
 * expensive way. That is all this check is for. It says nothing of whether
 * the memory is online, and by the time the reader's function runs the
 * answer may be stale, so that function must still validate every page
 * frame it touches, which is also what skips the offline blocks and the
 * holes of a range that is only partly System RAM.
 */
static int cxl_hmu_hpa_range(struct cxl_hmu_inst *inst, u64 unit_id,
			     const struct range *hpa, u32 count,
			     struct cxl_hmu_reader *r)
{
	if (region_intersects(hpa->start, range_len(hpa), IORESOURCE_SYSTEM_RAM,
			      IORES_DESC_NONE) == REGION_DISJOINT)
		return 0;

	trace_cxl_hmu_hpa_range(inst, unit_id, hpa->start, range_len(hpa), count);
	r->nr_ranges++;
	return r->fn(r->arg, hpa->start, range_len(hpa), count);
}

/*
 * A unit is a contiguous range of device physical address; on an
 * interleaved region it is spread over several HPA ranges. Walk it by
 * range. The region core translates the unit a batch of HPA ranges at a
 * time, taking its locks once per batch: page granular, adjacent ranges
 * merged, DPA that no committed region maps left out. A granule or a unit
 * smaller than a page puts the whole page in play, once, so an HPA range
 * can be larger than the unit. Only those that are at least partly System
 * RAM are handed over. The count handed over is the entry's in epoch mode;
 * in always-on mode the entry's counter is not valid and the configured
 * threshold, which the unit is known to have reached, is handed over
 * instead.
 *
 * The reader may keep what it is handed until after the walk, pages it
 * took off the LRU for one, so no CXL lock is waited for between two
 * ranges: whoever holds the memory device's lock may be removing the very
 * memory and waiting for those pages. See
 * cxl_memdev_dpa_to_hpa_ranges_nowait().
 *
 * Returns the first non-zero return of the reader's function, which ends
 * the walk, -ENODEV if cxl_mem is not bound to the memory device, in which
 * case no unit will translate until it is, or -EBUSY if a lock was not
 * free, which says nothing about the unit. May sleep.
 */
static int cxl_hmu_for_each_hpa_range(struct cxl_hmu_inst *inst, u64 entry,
				      struct cxl_hmu_reader *r)
{
	u8 counter_width = inst->live_width;
	u64 unit_id = entry >> counter_width;
	u64 count = entry & (BIT_ULL(counter_width) - 1);
	u64 dpa = unit_id << inst->live.unit_shift;
	u64 dpa_end = dpa + BIT_ULL(inst->live.unit_shift);
	struct range *hpa = r->hpa;
	struct range run;
	bool have_run = false;
	u32 nr;
	int rc;

	nr = inst->live.mode == CXL_HMU_MODE_EPOCH ? min_t(u64, count, U32_MAX)
						   : inst->live.hotness_threshold;

	while (dpa < dpa_end) {
		u64 done;
		int i, n;

		n = cxl_memdev_dpa_to_hpa_ranges_nowait(inst->block->hmu->cxlmd,
							dpa, dpa_end - dpa, hpa,
							CXL_HMU_WALK_RANGES,
							&done);
		/* A region whose addressing cannot be translated is a hole here */
		if (n < 0 && n != -EOPNOTSUPP)
			return n;
		dpa += done;

		/* The last HPA range of a batch may continue in the next */
		for (i = 0; i < n; i++) {
			if (have_run && hpa[i].start >= run.start &&
			    hpa[i].start <= run.end + 1) {
				run.end = max(run.end, hpa[i].end);
				continue;
			}
			if (have_run) {
				rc = cxl_hmu_hpa_range(inst, unit_id, &run, nr, r);
				if (rc)
					return rc;
			}
			run = hpa[i];
			have_run = true;
		}
		cond_resched();
	}
	if (have_run)
		return cxl_hmu_hpa_range(inst, unit_id, &run, nr, r);
	return 0;
}

/* Hottest first; the count is the low live_width bits of an entry */
static int cxl_hmu_count_cmp(const void *a, const void *b, const void *priv)
{
	u64 mask = *(const u64 *)priv;
	u64 ca = *(const u64 *)a & mask, cb = *(const u64 *)b & mask;

	return ca < cb ? 1 : ca > cb ? -1 : 0;
}

/**
 * cxl_hmu_read_ranges() - Read hot units off an instance as HPA ranges
 * @inst: the instance, held by the caller
 * @max: the most hotlist entries to take, at most %CXL_HMU_READ_MAX
 * @fn: called with @arg for each HPA range, with the HPA, the length in
 *	bytes and the count; returns 0 to go on, or a negative errno that
 *	ends the read
 * @arg: passed to @fn
 * @flags: set to %CXL_HMU_READ_MORE if entries are left in the hotlist and
 *	%CXL_HMU_READ_LOST if the device lost hot units to a hotlist overflow
 *	since the last read; always written
 *
 * Takes up to @max entries off the head of the hotlist. They are consumed:
 * a unit that stays hot is reported again. In epoch mode they are handed
 * over by count, the highest first, so whatever @fn takes before it stops
 * is the hottest of the batch; in always-on mode in the device's order,
 * and the count is the configured threshold, since the entry has none.
 *
 * The unit of each entry is handed over one HPA range at a time, page
 * granular and merged, and only where it is at least partly System RAM:
 * @fn must still validate every page frame. @fn may keep what it is
 * handed, isolated folios for one, past the call, which is why the read
 * waits for no CXL lock.
 *
 * What is not handed over whole, from the entry that ended the read to
 * the end of the batch, and every entry whose unit had no range to hand
 * over, is counted in the instance's "dropped" attribute.
 *
 * The read acknowledges the interrupt status bits it found, including
 * when it then ends in an error. A client claimed with a notify may not
 * be notified again until the hotlist crosses the fill level or
 * overflows anew, so after a read that ends in an error, or with
 * %CXL_HMU_READ_MORE set, it must schedule the next read itself.
 *
 * Context: process context, may sleep. For the holder, which serializes
 * its own calls.
 *
 * Return: the number of entries whose unit was handed over whole, or what
 * ended the read: the error @fn returned, -EBUSY if a CXL lock was not
 * free, -ENODEV if cxl_mem is not bound to the memory device, -ENOMEM,
 * -EINVAL for a @max of 0, or -EPERM (with a warning) if nobody holds the
 * instance.
 */
int cxl_hmu_read_ranges(struct cxl_hmu_inst *inst, unsigned int max,
			cxl_hmu_range_fn fn, void *arg, unsigned int *flags)
{
	u64 mask = BIT_ULL(inst->live_width) - 1;
	struct cxl_hmu_reader r = { .fn = fn, .arg = arg };
	u64 entries[CXL_HMU_READ_MAX];
	unsigned int dflags, n, i, skipped = 0;
	int rc = 0;

	*flags = 0;
	if (WARN_ON_ONCE(!READ_ONCE(inst->owner)))
		return -EPERM;
	if (!max)
		return -EINVAL;
	max = min(max, CXL_HMU_READ_MAX);

	/* Before the drain, so that a failure loses no entry */
	struct range *hpa __free(kfree) =
		kmalloc_array(CXL_HMU_WALK_RANGES, sizeof(*hpa), GFP_KERNEL);
	if (!hpa)
		return -ENOMEM;
	r.hpa = hpa;

	n = cxl_hmu_drain(inst, entries, max, &dflags);
	if (dflags & CXL_HMU_DRAIN_MORE)
		*flags |= CXL_HMU_READ_MORE;
	if (dflags & CXL_HMU_DRAIN_LOST)
		*flags |= CXL_HMU_READ_LOST;
	if (inst->live.mode == CXL_HMU_MODE_EPOCH)
		sort_r(entries, n, sizeof(*entries), cxl_hmu_count_cmp, NULL,
		       &mask);
	for (i = 0; i < n; i++) {
		r.nr_ranges = 0;
		rc = cxl_hmu_for_each_hpa_range(inst, entries[i], &r);
		if (rc)
			break;
		if (!r.nr_ranges)
			skipped++;
	}
	if (n - i + skipped)
		cxl_hmu_account_dropped(inst, n - i + skipped);
	trace_cxl_hmu_read(inst, n, i - skipped, n - i + skipped,
			   dflags & CXL_HMU_DRAIN_LOST, rc);
	if (rc)
		return rc;
	return i - skipped;
}
EXPORT_SYMBOL_NS_GPL(cxl_hmu_read_ranges, "CXL_HMU");

/* Clients */

static DEFINE_MUTEX(cxl_hmu_clients_lock);
static LIST_HEAD(cxl_hmu_clients);	/* registered */
static LIST_HEAD(cxl_hmu_blocks);	/* probed, until unbind */

/* A client bound to an instance: its add() accepted it */
struct cxl_hmu_bind {
	struct list_head	node;		/* inst->binds */
	struct cxl_hmu_client	*client;
	void			*data;		/* what add() stored */
};

static void cxl_hmu_bind(struct cxl_hmu_client *client, struct cxl_hmu_inst *inst)
{
	struct cxl_hmu_bind *b;
	void *data = NULL;
	int rc;

	lockdep_assert_held(&cxl_hmu_clients_lock);
	b = kzalloc_obj(*b);
	if (!b) {
		rc = -ENOMEM;
		goto out;
	}
	rc = client->add(inst, &data);
	if (rc) {
		kfree(b);
		goto out;
	}
	b->client = client;
	b->data = data;
	list_add_tail(&b->node, &inst->binds);
	return;
out:
	dev_dbg(&inst->dev, "not bound to client %s: %d\n", client->name, rc);
}

/*
 * remove() ends the client's use of the instance, a claim included. A
 * claim it left is released here, so that no client name outlives its
 * client as an owner.
 */
static void cxl_hmu_unbind(struct cxl_hmu_inst *inst, struct cxl_hmu_bind *b)
{
	bool held;

	lockdep_assert_held(&cxl_hmu_clients_lock);
	b->client->remove(inst, b->data);
	scoped_guard(mutex, &inst->block->lock)
		held = inst->owner == b->client->name;
	if (WARN_ON_ONCE(held))
		cxl_hmu_release(inst);
	list_del(&b->node);
	kfree(b);
}

/**
 * cxl_hmu_register_client() - Register an in-kernel client of CHMU instances
 * @client: the client, with its name and both callbacks set
 *
 * Calls @client->add() for every instance there is, and from then on for
 * every instance that is probed, until cxl_hmu_unregister_client(). An
 * instance whose add() fails is not bound to the client, and the client
 * is not told of it again; the failure is not returned.
 *
 * Context: process context, may sleep.
 *
 * Return: 0, -EINVAL if a name or a callback is missing or the name is
 * reserved, -EEXIST if a registered client has the name.
 */
int cxl_hmu_register_client(struct cxl_hmu_client *client)
{
	struct cxl_hmu_client *c;
	struct cxl_hmu_block *block;
	unsigned int i;

	if (!client->name || !client->name[0] || !client->add || !client->remove)
		return -EINVAL;
	/* What "owner" shows for perf and for a free instance */
	if (!strcmp(client->name, "perf") || !strcmp(client->name, "none"))
		return -EINVAL;

	guard(mutex)(&cxl_hmu_clients_lock);
	list_for_each_entry(c, &cxl_hmu_clients, node)
		if (!strcmp(c->name, client->name))
			return -EEXIST;
	list_add_tail(&client->node, &cxl_hmu_clients);
	list_for_each_entry(block, &cxl_hmu_blocks, node)
		for (i = 0; i < block->num_inst; i++)
			if (block->inst[i])
				cxl_hmu_bind(client, block->inst[i]);
	return 0;
}
EXPORT_SYMBOL_NS_GPL(cxl_hmu_register_client, "CXL_HMU");

/**
 * cxl_hmu_unregister_client() - Unregister an in-kernel client
 * @client: a registered client
 *
 * Calls @client->remove() for every instance bound to it. On return the
 * core holds nothing of the client's and calls none of its callbacks.
 *
 * Context: process context, may sleep. Must not be called with a lock
 * held that @client->remove() takes.
 */
void cxl_hmu_unregister_client(struct cxl_hmu_client *client)
{
	struct cxl_hmu_block *block;
	struct cxl_hmu_bind *b, *tmp;
	unsigned int i;

	guard(mutex)(&cxl_hmu_clients_lock);
	list_del(&client->node);
	list_for_each_entry(block, &cxl_hmu_blocks, node) {
		for (i = 0; i < block->num_inst; i++) {
			struct cxl_hmu_inst *inst = block->inst[i];

			if (!inst)
				continue;
			list_for_each_entry_safe(b, tmp, &inst->binds, node)
				if (b->client == client)
					cxl_hmu_unbind(inst, b);
		}
	}
}
EXPORT_SYMBOL_NS_GPL(cxl_hmu_unregister_client, "CXL_HMU");

/**
 * cxl_hmu_dev() - The device of an instance
 * @inst: an instance the caller is bound to
 *
 * Its name is the instance's, for example "hmu_mem0.0.0", and its sysfs
 * directory holds the configuration a claim takes.
 *
 * Context: any, between the client's add() and remove() for @inst.
 *
 * Return: the device, valid until the client's remove() for @inst.
 */
struct device *cxl_hmu_dev(struct cxl_hmu_inst *inst)
{
	return &inst->dev;
}
EXPORT_SYMBOL_NS_GPL(cxl_hmu_dev, "CXL_HMU");

/* A notify: the instance is a client's, running, and has something to read */
static irqreturn_t cxl_hmu_notify_irq(int irq, void *data)
{
	struct cxl_hmu_inst *inst = data;

	scoped_guard(raw_spinlock, &inst->lock)
		if (!inst->enabled || !cxl_hmu_pending(inst))
			return IRQ_NONE;
	/*
	 * Not acknowledged: the status bits hold off another interrupt until
	 * the client's next read has moved the head and acknowledged them.
	 */
	inst->notify(inst->notify_arg);
	return IRQ_HANDLED;
}

static int cxl_hmu_claim_start(struct cxl_hmu_inst *inst,
			       const struct cxl_hmu_client *client,
			       cxl_hmu_notify_fn fn, void *arg)
{
	int rc;

	rc = cxl_hmu_take(inst, client->name, NULL, client->max_unit_size);
	if (rc)
		return rc;
	/* A threshold that does not fit the counter is refused quietly here */
	rc = cxl_hmu_check_threshold(inst);
	if (rc)
		goto release;
	if (fn) {
		/* Before the request: the vector may be shared and taken at once */
		inst->notify = fn;
		inst->notify_arg = arg;
		rc = request_irq(inst->irq, cxl_hmu_notify_irq,
				 IRQF_SHARED | IRQF_NO_THREAD,
				 dev_name(&inst->dev), inst);
		if (rc) {
			dev_dbg(&inst->dev, "interrupt %d not available: %d\n",
				inst->irq, rc);
			inst->notify = NULL;
			inst->notify_arg = NULL;
			goto release;
		}
	}
	rc = __cxl_hmu_start(inst, fn ? CXL_HMU_START_IRQ : 0);
	if (!rc)
		return 0;
release:
	cxl_hmu_release(inst);
	return rc;
}

/**
 * cxl_hmu_claim() - Take an instance and start it, polled
 * @inst: an instance the client is bound to
 * @client: the client; its name is the instance's owner until the release
 *
 * Claims @inst with the configuration staged in its sysfs files, programs
 * it, empties its hotlist and enables it with its interrupts off: the
 * client reads it with cxl_hmu_read_ranges() when it chooses. The claim is
 * first come, first served, between clients and perf alike.
 *
 * Context: process context, may sleep, between the client's add() and
 * remove() for @inst. Busy-waits for the device for up to 0.3 s.
 *
 * Return: 0 with the instance held and running. On error nothing is held:
 * -EBUSY if the instance is held, or its range overlaps that of a held
 * sibling and both do not support Address Overlap; -EOPNOTSUPP or -EINVAL
 * if the staged configuration does not fit the instance, a threshold that
 * does not fit the counter included; -EINVAL, before the device is
 * touched, if the staged unit size is above @client->max_unit_size; -EIO
 * or -ETIMEDOUT if the device does not follow.
 */
int cxl_hmu_claim(struct cxl_hmu_inst *inst, const struct cxl_hmu_client *client)
{
	return cxl_hmu_claim_start(inst, client, NULL, NULL);
}
EXPORT_SYMBOL_NS_GPL(cxl_hmu_claim, "CXL_HMU");

/**
 * cxl_hmu_claim_notify() - Take an instance and start it, with a notify
 * @inst: an instance the client is bound to
 * @client: the client; its name is the instance's owner until the release
 * @fn: called with @arg in hard interrupt context when the hotlist fills
 *	to half or overflows; it may only queue work
 * @arg: passed to @fn
 *
 * As cxl_hmu_claim(), but the instance's interrupt is requested, shared,
 * before it is enabled with its overflow and fill level interrupts on.
 * The interrupt is not acknowledged until the next read moves the head.
 * A read acknowledges it whatever it then returns, and the device raises
 * it again only when the hotlist crosses the fill level or overflows
 * anew: after a read that ends in an error, or with %CXL_HMU_READ_MORE
 * set, the client must schedule the next read itself rather than wait
 * for a notify. cxl_hmu_release() frees the interrupt.
 *
 * Context: as cxl_hmu_claim().
 *
 * Return: as cxl_hmu_claim(), and -EOPNOTSUPP if the instance has no
 * interrupt vector or no interrupt, or the error of request_irq() (-EBUSY
 * if the vector cannot be shared). On error nothing is held and no
 * interrupt is requested.
 */
int cxl_hmu_claim_notify(struct cxl_hmu_inst *inst,
			 const struct cxl_hmu_client *client,
			 cxl_hmu_notify_fn fn, void *arg)
{
	if (!fn)
		return -EINVAL;
	if (inst->irq < 0 || !(inst->caps.irq_overflow || inst->caps.irq_level))
		return -EOPNOTSUPP;
	return cxl_hmu_claim_start(inst, client, fn, arg);
}
EXPORT_SYMBOL_NS_GPL(cxl_hmu_claim_notify, "CXL_HMU");

/**
 * cxl_hmu_unit_size() - The unit size an instance was claimed with
 * @inst: the instance, held by the caller
 *
 * Context: any. For the holder: the value is fixed until the release.
 *
 * Return: the size in bytes of the unit one hotlist entry stands for.
 */
u64 cxl_hmu_unit_size(struct cxl_hmu_inst *inst)
{
	return BIT_ULL(inst->live.unit_shift);
}
EXPORT_SYMBOL_NS_GPL(cxl_hmu_unit_size, "CXL_HMU");

/* Sysfs */

/* Index n is the CFG0 WHAT encoding n + 1 */
static const char * const cxl_hmu_access_names[] = {
	"nontee_r", "nontee_w", "nontee_rw", "r", "w", "rw",
};

static ssize_t owner_show(struct device *dev, struct device_attribute *a, char *buf)
{
	struct cxl_hmu_inst *inst = to_cxl_hmu_inst(dev);

	guard(mutex)(&inst->block->lock);
	return sysfs_emit(buf, "%s\n", inst->owner ?: "none");
}
static DEVICE_ATTR_RO(owner);

/*
 * The staged configuration: what an in-kernel holder claims with. Only
 * these stores write it. A store is checked against the capabilities and
 * takes effect at the next claim; a holder keeps what it claimed with.
 *
 * Five files, the ones an in-kernel holder has a reason to be told: how
 * large a unit is and how many accesses make it hot, how often the device
 * reports, and which part of the device to watch. The mode, the access
 * type and the down-sampling stay at cxl_hmu_default_config(). None of the
 * five shares a name with a perf format attribute unless it also shares
 * its meaning and its unit.
 */
#define CXL_HMU_CFG_ATTR(_name, _fmt, _val, _parse)				\
static ssize_t _name##_show(struct device *dev, struct device_attribute *a,	\
			    char *buf)						\
{										\
	struct cxl_hmu_inst *inst = to_cxl_hmu_inst(dev);			\
										\
	guard(mutex)(&inst->block->lock);					\
	return sysfs_emit(buf, _fmt "\n", _val);				\
}										\
static ssize_t _name##_store(struct device *dev, struct device_attribute *a,	\
			     const char *buf, size_t len)			\
{										\
	struct cxl_hmu_inst *inst = to_cxl_hmu_inst(dev);			\
	struct cxl_hmu_config cfg;						\
	int rc;									\
										\
	guard(mutex)(&inst->block->lock);					\
	cfg = inst->config;							\
	rc = _parse(inst, &cfg, buf);						\
	if (rc)									\
		return rc;							\
	rc = cxl_hmu_validate_config(inst, &cfg);				\
	if (rc)									\
		return rc;							\
	inst->config = cfg;							\
	return len;								\
}										\
static DEVICE_ATTR_RW(_name)

static int parse_unit_size(struct cxl_hmu_inst *inst, struct cxl_hmu_config *cfg, const char *buf)
{
	unsigned long v;

	if (kstrtoul(buf, 0, &v) || !is_power_of_2(v) || v < 256 || v >= BIT_ULL(40))
		return -EINVAL;
	cfg->unit_shift = ilog2(v);
	return 0;
}
CXL_HMU_CFG_ATTR(unit_size, "%lu", 1UL << inst->config.unit_shift, parse_unit_size);

/* Its bound, the counter width, is known at a start, which is where it is checked */
static int parse_hotness_threshold(struct cxl_hmu_inst *inst, struct cxl_hmu_config *cfg,
				   const char *buf)
{
	return kstrtou32(buf, 0, &cfg->hotness_threshold) ? -EINVAL : 0;
}
CXL_HMU_CFG_ATTR(hotness_threshold, "%u", inst->config.hotness_threshold, parse_hotness_threshold);

/* Microseconds: the finest scale of Table 8-192 is 100 us */
static int parse_epoch_us(struct cxl_hmu_inst *inst, struct cxl_hmu_config *cfg, const char *buf)
{
	u64 v;

	if (kstrtou64(buf, 0, &v) || !v)
		return -EINVAL;
	cxl_hmu_pick_epoch(inst, v, &cfg->epoch_scale, &cfg->epoch_mult);
	return cfg->epoch_mult ? 0 : -ERANGE;
}
CXL_HMU_CFG_ATTR(epoch_us, "%llu",
		 cxl_hmu_epoch_us(inst->config.epoch_scale, inst->config.epoch_mult),
		 parse_epoch_us);

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

static ssize_t epoch_range_us_show(struct device *dev, struct device_attribute *a, char *buf)
{
	const struct cxl_hmu_caps *c = &to_cxl_hmu_inst(dev)->caps;

	return sysfs_emit(buf, "%llu %llu\n", c->epoch_min_us, c->epoch_max_us);
}
static DEVICE_ATTR_RO(epoch_range_us);

CXL_HMU_RO_ATTR(hotlist_size, "%u", inst->caps.hotlist_size);
CXL_HMU_RO_ATTR(capacity_ranges, "%u", inst->caps.capacity_ranges);
/* Not "epoch_type": that is perf's name for epoch against always-on */
CXL_HMU_RO_ATTR(epoch_scope, "%s", inst->caps.epoch_type == CHMU_INST0_CAP0_EPOCH_TYPE_PERCNT ?
		"per_counter" : "global");
CXL_HMU_RO_ATTR(address_overlap, "%d", inst->caps.addr_overlap);
CXL_HMU_RO_ATTR(postponed_overflow, "%d", inst->caps.postponed_overflow);
CXL_HMU_RO_ATTR(active, "%d", READ_ONCE(inst->enabled));
CXL_HMU_RO_ATTR(overflows, "%lu", READ_ONCE(inst->overflows));
CXL_HMU_RO_ATTR(dropped, "%lu", READ_ONCE(inst->dropped));

static struct attribute *cxl_hmu_inst_attrs[] = {
	&dev_attr_owner.attr,
	&dev_attr_unit_size.attr,
	&dev_attr_hotness_threshold.attr,
	&dev_attr_epoch_us.attr,
	&dev_attr_range_base.attr,
	&dev_attr_range_size.attr,
	&dev_attr_supported_modes.attr,
	&dev_attr_supported_unit_sizes.attr,
	&dev_attr_supported_access_types.attr,
	&dev_attr_supported_downsampling.attr,
	&dev_attr_epoch_range_us.attr,
	&dev_attr_epoch_scope.attr,
	&dev_attr_hotlist_size.attr,
	&dev_attr_capacity_ranges.attr,
	&dev_attr_address_overlap.attr,
	&dev_attr_postponed_overflow.attr,
	&dev_attr_interrupts.attr,
	&dev_attr_active.attr,
	&dev_attr_overflows.attr,
	&dev_attr_dropped.attr,
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

/* What an in-kernel holder starts with unless sysfs stages otherwise */
static void cxl_hmu_default_config(struct cxl_hmu_inst *inst)
{
	const struct cxl_hmu_caps *c = &inst->caps;
	struct cxl_hmu_config *cfg = &inst->config;

	cfg->mode = c->epoch ? CXL_HMU_MODE_EPOCH : CXL_HMU_MODE_ALWAYS_ON;
	/* 4 KiB, else the smallest unit */
	if (c->unit_sizes)
		cfg->unit_shift = c->unit_sizes & BIT(4) ? 12 : 8 + __ffs(c->unit_sizes);
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

/*
 * Unbind. The clients were unbound first (cxl_hmu_unbind_clients()), and
 * each released what it held. The actions that unregister the instance's
 * PMU and free the interrupt it requested were added after this one, so
 * they have run too: every perf event is closed, which released a claim
 * perf held, and none can be opened. So no claim is left and none can
 * succeed.
 */
static void cxl_hmu_inst_remove(void *data)
{
	struct cxl_hmu_inst *inst = data;
	struct cxl_hmu_block *block = inst->block;

	WARN_ON_ONCE(!list_empty(&inst->binds));
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
	INIT_LIST_HEAD(&inst->binds);
	cxl_hmu_decode_caps(inst);
	cxl_hmu_default_config(inst);

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
	/* In the table before its attributes can be written: a claim checks siblings */
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
	 * looked up here: perf, or the claim of a client that wants a notify,
	 * requests it, shared, since two instances may report the same
	 * number. Without a vector the instance can still be polled.
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

/*
 * Unbind, first: once the block is off the list no client is told of it,
 * and once every bound client's remove() has returned none uses it.
 */
static void cxl_hmu_unbind_clients(void *data)
{
	struct cxl_hmu_block *block = data;
	struct cxl_hmu_bind *b, *tmp;
	unsigned int i;

	guard(mutex)(&cxl_hmu_clients_lock);
	list_del(&block->node);
	for (i = 0; i < block->num_inst; i++) {
		struct cxl_hmu_inst *inst = block->inst[i];

		if (!inst)
			continue;
		list_for_each_entry_safe(b, tmp, &inst->binds, node)
			cxl_hmu_unbind(inst, b);
	}
}

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

	/* Last, so that at unbind the clients go first */
	scoped_guard(mutex, &cxl_hmu_clients_lock) {
		struct cxl_hmu_client *client;

		list_add_tail(&block->node, &cxl_hmu_blocks);
		list_for_each_entry(client, &cxl_hmu_clients, node)
			for (i = 0; i < block->num_inst; i++)
				cxl_hmu_bind(client, block->inst[i]);
	}
	return devm_add_action_or_reset(dev, cxl_hmu_unbind_clients, block);
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
