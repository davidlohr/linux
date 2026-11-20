/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * CXL Hot-range Monitoring Unit: an instance of a CHMU register block,
 * shared between the instance core and the front-ends built on it.
 * CXL r4.0 Section 8.2.8 (CHMU Register Interface).
 */
#ifndef CXL_HMU_INST_H
#define CXL_HMU_INST_H
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "../hmu.h"

/* Register offsets within instance */
#define CHMU_INST0_CAP0_REG				0x00
#define   CHMU_INST0_CAP0_MSI_N_MSK			GENMASK(3, 0)
#define   CHMU_INST0_CAP0_OVRFLW_CAP			BIT(4)
#define   CHMU_INST0_CAP0_FILLTHRESH_CAP		BIT(5)
#define   CHMU_INST0_CAP0_EPOCH_TYPE_MSK		GENMASK(7, 6)
#define     CHMU_INST0_CAP0_EPOCH_TYPE_GLOBAL		0
#define     CHMU_INST0_CAP0_EPOCH_TYPE_PERCNT		1
#define   CHMU_INST0_CAP0_TRACK_MSK			GENMASK(13, 8)
#define   CHMU_INST0_CAP0_TRACK_NONTEE_R		BIT(8)
#define   CHMU_INST0_CAP0_TRACK_NONTEE_W		BIT(9)
#define   CHMU_INST0_CAP0_TRACK_NONTEE_RW		BIT(10)
#define   CHMU_INST0_CAP0_TRACK_R			BIT(11)
#define   CHMU_INST0_CAP0_TRACK_W			BIT(12)
#define   CHMU_INST0_CAP0_TRACK_RW			BIT(13)
/* Epoch defined as scale * multiplier */
#define   CHMU_INST0_CAP0_EPOCH_MAX_SCALE_MSK		GENMASK(19, 16)
#define     CHMU_EPOCH_SCALE_100US			1
#define     CHMU_EPOCH_SCALE_1MS			2
#define     CHMU_EPOCH_SCALE_10MS			3
#define     CHMU_EPOCH_SCALE_100MS			4
#define     CHMU_EPOCH_SCALE_1S			5
#define   CHMU_INST0_CAP0_EPOCH_MAX_MULT_MSK		GENMASK(31, 20)
#define   CHMU_INST0_CAP0_EPOCH_MIN_SCALE_MSK		GENMASK_ULL(35, 32)
#define   CHMU_INST0_CAP0_EPOCH_MIN_MULT_MSK		GENMASK_ULL(47, 36)
#define   CHMU_INST0_CAP0_HOTLIST_SIZE_MSK		GENMASK_ULL(63, 48)
#define CHMU_INST0_CAP1_REG				0x08
/* Power of 2 * 256 bits */
#define   CHMU_INST0_CAP1_UNIT_SIZE_MSK			GENMASK(31, 0)
/* Power of 2 */
#define   CHMU_INST0_CAP1_DOWNSAMP_MSK			GENMASK_ULL(47, 32)
#define   CHMU_INST0_CAP1_EPOCH_SUP			BIT_ULL(48)
#define   CHMU_INST0_CAP1_ALWAYS_ON_SUP			BIT_ULL(49)
#define   CHMU_INST0_CAP1_RAND_DOWNSAMP_SUP		BIT_ULL(50)
#define   CHMU_INST0_CAP1_ADDR_OVERLAP_SUP		BIT_ULL(51)
#define   CHMU_INST0_CAP1_POSTPONED_ON_OVRFLOW_SUP	BIT_ULL(52)

/*
 * In CXL r3.2 all defined as part of single giant CAP register.
 * Where a whole 64 bits is in one field just name after the field.
 */
#define CHMU_INST0_RANGE_BITMAP_OFFSET_REG		0x10
#define CHMU_INST0_HOTLIST_OFFSET_REG			0x18

#define CHMU_INST0_CFG0_REG				0x40
#define   CHMU_INST0_CFG0_WHAT_MSK			GENMASK(7, 0)
#define      CHMU_INST0_CFG0_WHAT_NONTEE_R		1
#define      CHMU_INST0_CFG0_WHAT_NONTEE_W		2
#define      CHMU_INST0_CFG0_WHAT_NONTEE_RW		3
#define      CHMU_INST0_CFG0_WHAT_R			4
#define      CHMU_INST0_CFG0_WHAT_W			5
#define      CHMU_INST0_CFG0_WHAT_RW			6
#define   CHMU_INST0_CFG0_RAND_DOWNSAMP_EN		BIT(8)
#define   CHMU_INST0_CFG0_OVRFLW_INT_EN			BIT(9)
#define   CHMU_INST0_CFG0_FILLTHRESH_INT_EN		BIT(10)
#define   CHMU_INST0_CFG0_ENABLE			BIT(16)
#define   CHMU_INST0_CFG0_RESET_COUNTERS		BIT(17)
#define   CHMU_INST0_CFG0_HOTNESS_THRESH_MSK		GENMASK_ULL(63, 32)
#define CHMU_INST0_CFG1_REG				0x48
#define   CHMU_INST0_CFG1_UNIT_SIZE_MSK			GENMASK(31, 0)
#define   CHMU_INST0_CFG1_DS_FACTOR_MSK			GENMASK(35, 32)
#define   CHMU_INST0_CFG1_MODE_MSK			GENMASK_ULL(47, 40)
#define   CHMU_INST0_CFG1_EPOCH_SCALE_MSK		GENMASK_ULL(51, 48)
#define   CHMU_INST0_CFG1_EPOCH_MULT_MSK		GENMASK_ULL(63, 52)
#define CHMU_INST0_CFG2_REG				0x50
#define   CHMU_INST0_CFG2_FILLTHRESH_THRESHOLD_MSK	GENMASK(15, 0)

#define CHMU_INST0_STATUS_REG				0x60
#define   CHMU_INST0_STATUS_ENABLED			BIT(0)
#define   CHMU_INST0_STATUS_OP_INPROG_MSK		GENMASK(31, 16)
#define     CHMU_INST0_STATUS_OP_INPROG_NONE		0
#define     CHMU_INST0_STATUS_OP_INPROG_ENABLE		1
#define     CHMU_INST0_STATUS_OP_INPROG_DISABLE		2
#define     CHMU_INST0_STATUS_OP_INPROG_RESET		3
#define   CHMU_INST0_STATUS_COUNTER_WIDTH_MSK		GENMASK_ULL(39, 32)
#define   CHMU_INST0_STATUS_OVRFLW			BIT_ULL(40)
#define   CHMU_INST0_STATUS_FILLTHRESH			BIT_ULL(41)

/* 2 byte registers */
#define CHMU_INST0_HEAD_REG				0x68
#define CHMU_INST0_TAIL_REG				0x6A

/* CXL r4.0 Table 8-190: NUMINST is four bits, up to 8 instances */
#define CXL_HMU_MAX_INST 8

struct cxl_hmu_inst;

enum cxl_hmu_mode { CXL_HMU_MODE_EPOCH, CXL_HMU_MODE_ALWAYS_ON };

/**
 * struct cxl_hmu_config - what a start programs into an instance
 * @mode: epoch (a unit is reported when its count crosses the threshold
 *	within an epoch, with the count) or always-on (a unit is reported once
 *	its count crosses the threshold, and the count is not valid)
 * @unit_shift: log2 of the unit size in bytes, one of the supported sizes
 * @hotness_threshold: count at which a unit is hot; below 1 << counter width
 * @epoch_scale: epoch length encoding, scale (1: 100 us .. 5: 1 s)
 * @epoch_mult: epoch length encoding, multiplier
 * @access_type: the M2S requests counted, in the CFG0 WHAT encoding
 * @ds_shift: log2 of the down-sampling factor
 * @randomized_ds: randomize the down-sampling
 * @range_base: first 256 MiB unit of device physical address to track
 * @range_num: number of 256 MiB units to track
 */
struct cxl_hmu_config {
	enum cxl_hmu_mode	mode;
	u8			unit_shift;
	u32			hotness_threshold;
	u8			epoch_scale;
	u16			epoch_mult;
	u8			access_type;
	u8			ds_shift;
	bool			randomized_ds;
	u32			range_base, range_num;
};

/**
 * struct cxl_hmu_caps - what an instance advertises
 * @epoch: epoch mode supported
 * @always_on: always-on mode supported
 * @randomized_ds: randomized down-sampling supported
 * @irq_overflow: raises an interrupt on hotlist overflow
 * @irq_level: raises an interrupt when the hotlist fill level is crossed
 * @addr_overlap: may track a range that overlaps a sibling instance's
 * @postponed_overflow: outstanding hot units survive a hotlist overflow
 * @epoch_type: 0 for a global epoch, 1 for a per-counter one
 * @unit_sizes: bitmap, bit n means a unit of 256 << n bytes
 * @ds_factors: bitmap, bit n means a down-sampling factor of 1 << n
 * @access_types: bitmap of the supported CFG0 WHAT encodings, bit n for n + 1
 * @epoch_min_us: shortest epoch
 * @epoch_max_us: longest epoch
 * @hotlist_size: entries in the hotlist
 * @range_bits: bits in the range bitmap register
 * @capacity_ranges: the device capacity in 256 MiB units; the bitmap's
 *	valid length and the bound of any configured range
 * @msi: the Interrupt Message Number
 */
struct cxl_hmu_caps {
	bool			epoch, always_on, randomized_ds;
	bool			irq_overflow, irq_level;
	bool			addr_overlap;
	bool			postponed_overflow;
	u8			epoch_type;
	u32			unit_sizes;
	u16			ds_factors;
	u8			access_types;
	u64			epoch_min_us, epoch_max_us;
	u16			hotlist_size;
	u32			range_bits;
	u32			capacity_ranges;
	u8			msi;
};

/*
 * The driver's state for one register block. The instances are in a table
 * so a claim can check its range against the siblings that are held.
 */
struct cxl_hmu_block {
	struct cxl_hmu		*hmu;
	/*
	 * The owner, the staged configuration and the claimed range of every
	 * instance of the block. Taken before an instance's lock.
	 */
	struct mutex		lock;
	unsigned int		num_inst;
	struct cxl_hmu_inst	*inst[CXL_HMU_MAX_INST];
};

struct cxl_hmu_inst {
	struct device		dev;
	struct cxl_hmu_block	*block;
	void __iomem		*base;		/* block base + 0x10 + INSTLEN * index */
	unsigned int		index;
	/*
	 * The vector of the instance's Interrupt Message Number, or -1. The
	 * core requests nothing on it: a holder that wants the interrupt
	 * requests it, shared, and starts with CXL_HMU_START_IRQ.
	 */
	int			irq;
	struct cxl_hmu_caps	caps;		/* decoded once at probe */
	/*
	 * The instance's registers, enabled and the session snapshot. Raw:
	 * perf starts, stops and drains with interrupts off.
	 */
	raw_spinlock_t		lock;
	/*
	 * Who holds the instance, as the "owner" attribute shows it; NULL
	 * when free. Written under the block lock and the instance lock.
	 */
	const char		*owner;
	struct cxl_hmu_config	config;		/* staged in sysfs; the block lock */
	/*
	 * What the holder claimed with and a start programs. Written by the
	 * claim under both locks, fixed until the release.
	 */
	struct cxl_hmu_config	live;
	u8			live_width;	/* the counter width of the last start */
	bool			enabled;	/* hardware enabled */
	unsigned long		overflows;	/* hotlist overflows since the last start */
	unsigned long		dropped;	/* entries the holder discarded since then */
};
#define to_cxl_hmu_inst(d) container_of(d, struct cxl_hmu_inst, dev)

void cxl_hmu_pick_epoch(const struct cxl_hmu_inst *inst, u64 target_us, u8 *scale, u16 *mult);

/*
 * The instance library. An instance has one holder at a time, first come,
 * first served.
 *
 * cxl_hmu_claim() takes the instance for @who with @cfg, or with the
 * configuration staged in sysfs if @cfg is NULL: -EBUSY if it is held or
 * if the range overlaps that of a held sibling and both do not support
 * Address Overlap, -EOPNOTSUPP or -EINVAL if the instance cannot do @cfg.
 * It writes no register. cxl_hmu_release() stops the instance if it runs
 * and gives it up. Both may sleep.
 *
 * cxl_hmu_check_threshold() is for a holder that wants to know before its
 * first start, where an errno may have nowhere to go, whether the claimed
 * threshold fits the counter: it programs the claimed unit size into the
 * disabled instance, which is what makes the device show the width, and
 * nothing else. -EINVAL if the threshold does not fit, -EIO if the device
 * shows no width, -EBUSY if the instance is enabled. A start checks again.
 *
 * The rest is for the holder only, which serializes its own calls, and may
 * be called with interrupts off.
 *
 * cxl_hmu_start() programs the claimed configuration, empties the hotlist
 * and enables: -EBUSY if already enabled, -EINVAL if the threshold does not
 * fit the counter width the device has for the unit size, -EIO or
 * -ETIMEDOUT if the device does not follow. With CXL_HMU_START_IRQ every
 * interrupt the instance has is enabled, for a holder that requested
 * inst->irq; without it none is, for a holder that polls.
 * cxl_hmu_stop() disables. What the hotlist still holds can be drained
 * until the next start, which discards it.
 */
#define CXL_HMU_START_IRQ	BIT(0)

int cxl_hmu_claim(struct cxl_hmu_inst *inst, const char *who,
		  const struct cxl_hmu_config *cfg);
void cxl_hmu_release(struct cxl_hmu_inst *inst);
int cxl_hmu_check_threshold(struct cxl_hmu_inst *inst);
int cxl_hmu_start(struct cxl_hmu_inst *inst, unsigned int flags);
void cxl_hmu_stop(struct cxl_hmu_inst *inst);

/*
 * cxl_hmu_drain() copies at most @max entries from the head of the hotlist
 * to @entries, advances the head by as many and acknowledges the status
 * bits it read. An entry is the unit number in the upper bits and the count
 * in the low inst->live_width bits. With @entries NULL the entries are
 * discarded instead, for a holder that has nowhere to put them, and counted
 * in the instance's "dropped", so that what the kernel threw away can be
 * told from what the device lost. Returns the number taken; @flags says
 * what else there is to know.
 *
 * cxl_hmu_pending() is true if the instance has an interrupt status bit
 * set, for the handler of a shared vector.
 */
/*
 * The hotlist overflowed and hot units were lost for it. Not set for an
 * instance that postpones the units it could not insert: its overflow
 * delays them and loses none. Either kind is counted in "overflows".
 */
#define CXL_HMU_DRAIN_LOST	BIT(0)
#define CXL_HMU_DRAIN_MORE	BIT(1)	/* entries are left in the hotlist */

unsigned int cxl_hmu_drain(struct cxl_hmu_inst *inst, u64 *entries,
			   unsigned int max, unsigned int *flags);
bool cxl_hmu_pending(struct cxl_hmu_inst *inst);

/*
 * Walk the hot unit of a drained entry by contiguous host-physical range.
 * fn() is called with the HPA, the length of the run, and the entry's count
 * (or the threshold in always-on mode); it returns 0 to continue. For the
 * holder, for an entry of its current session. May sleep, but waits for no
 * CXL lock, so fn() may hold on to pages: -EBUSY if one was not free.
 */
typedef int (*cxl_hmu_hpa_range_fn)(u64 hpa, u64 len, u32 nr_accesses,
				    void *data);
int cxl_hmu_for_each_hpa_range(struct cxl_hmu_inst *inst, u64 entry,
			       cxl_hmu_hpa_range_fn fn, void *data);

#if IS_ENABLED(CONFIG_CXL_HMU_PERF)
int cxl_hmu_perf_init(void);
void cxl_hmu_perf_exit(void);
int cxl_hmu_perf_probe_inst(struct cxl_hmu_inst *inst);
#else
static inline int cxl_hmu_perf_init(void) { return 0; }
static inline void cxl_hmu_perf_exit(void) { }
static inline int cxl_hmu_perf_probe_inst(struct cxl_hmu_inst *inst) { return 0; }
#endif

#endif
