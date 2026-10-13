/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * CXL Hot-range Monitoring Unit: an instance of a CHMU register block,
 * shared between the instance core and its consumers.
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
 *	valid extent and the bound of any configured range
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

struct cxl_hmu_inst;

/*
 * Delivery of a drained hotlist. @entries holds @nr raw hotlist entries
 * (unit number in the upper bits, count in the low @counter_width bits);
 * @overflowed says the device reported a hotlist overflow since the
 * previous drain, so entries may have been lost before these.
 */
typedef void (*cxl_hmu_hotlist_fn)(struct cxl_hmu_inst *inst, const u64 *entries,
				   unsigned int nr, u8 counter_width, bool overflowed);

struct cxl_hmu_inst {
	struct device		dev;
	struct cxl_hmu		*block;
	void __iomem		*base;		/* block base + 0x10 + INSTLEN * index */
	unsigned int		index;
	int			irq;		/* -1 without an interrupt capability */
	struct cxl_hmu_caps	caps;		/* decoded once at probe */
	/*
	 * Hardware RMW, active, owner, the handler and the drain. The block's
	 * lock nests inside it around an enable or disable, so the overlap
	 * check and the state it reads from siblings are consistent.
	 */
	raw_spinlock_t		lock;
	struct mutex		drain_lock;	/* the threaded drain and its delivery */
	struct cxl_hmu_config	config;		/* what the next start programs */
	struct cxl_hmu_config	live;		/* what the running session was started with */
	u8			live_width;	/* the counter width of that session */
	bool			active;		/* hardware enabled */
	u8			counter_width;	/* valid once a unit size is set */
	u64			*entries;	/* drain scratch, hotlist_size entries */
	unsigned long		overflows;	/* hotlist overflows since the last start */
	/*
	 * The handler installed by cxl_hmu_hw_start() under the lock together
	 * with active. An atomic one runs from the hard interrupt and must
	 * not sleep; otherwise it runs from the interrupt thread.
	 */
	cxl_hmu_hotlist_fn	hotlist;
	bool			hotlist_atomic;
	void			*drvdata;	/* the perf PMU of this instance, if built */
};
#define to_cxl_hmu_inst(d) container_of(d, struct cxl_hmu_inst, dev)

int cxl_hmu_inst_add(struct cxl_hmu *block, unsigned int index, unsigned int inst_len);
bool cxl_hmu_dev_is_inst(struct device *dev);

int cxl_hmu_perf_init(void);
void cxl_hmu_perf_exit(void);
int cxl_hmu_perf_probe_inst(struct cxl_hmu_inst *inst);

u64 cxl_hmu_epoch_us(u8 scale, u16 mult);
void cxl_hmu_pick_epoch(const struct cxl_hmu_inst *inst, u64 target_us, u8 *scale, u16 *mult);
int cxl_hmu_validate_config(struct cxl_hmu_inst *inst, const struct cxl_hmu_config *cfg);
int cxl_hmu_set_unit_size(struct cxl_hmu_inst *inst, u8 unit_shift);
int cxl_hmu_hw_start(struct cxl_hmu_inst *inst, const struct cxl_hmu_config *cfg,
		     cxl_hmu_hotlist_fn fn, bool atomic);
bool cxl_hmu_hw_stop(struct cxl_hmu_inst *inst);
int cxl_hmu_drain(struct cxl_hmu_inst *inst, u64 *entries, unsigned int max,
		  u8 *counter_width, bool *overflowed);

#endif
