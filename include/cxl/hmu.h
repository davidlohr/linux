/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * CXL Hot-range Monitoring Unit (CHMU, CXL r4.0 Section 8.2.8): the
 * in-kernel consumer interface. An instance of a CHMU is driven by one
 * consumer at a time, chosen per instance through sysfs; the consumer
 * decides what a drained hotlist means.
 */
#ifndef __CXL_HMU_H__
#define __CXL_HMU_H__
#include <linux/types.h>

struct module;
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

/**
 * struct cxl_hmu_consumer - a user of CHMU instances
 * @name: what an administrator writes to an instance's "consumer" attribute
 * @owner: the module; pinned while the consumer owns an instance, unless it
 *	is this one
 * @atomic: @hotlist must not sleep; it is then called from the hard
 *	interrupt on the CPU the consumer made it affine to. Otherwise it is
 *	called from the interrupt thread and may sleep.
 * @attach: the consumer was selected for @inst; may set up and fail
 * @detach: the consumer is being deselected; the instance is stopped
 * @hotlist: a drained hotlist: @nr raw entries, each the unit number in
 *	the upper bits and the count in the low @counter_width bits;
 *	@overflowed says the hotlist overflowed since the previous drain, so
 *	units that became hot while it was full may have been lost (unless
 *	the device postpones their insertion)
 */
struct cxl_hmu_consumer {
	const char	*name;
	struct module	*owner;
	bool		atomic;
	int  (*attach)(struct cxl_hmu_inst *inst);
	void (*detach)(struct cxl_hmu_inst *inst);
	void (*hotlist)(struct cxl_hmu_inst *inst, const u64 *entries,
			unsigned int nr, u8 counter_width, bool overflowed);
};

int cxl_hmu_register_consumer(const struct cxl_hmu_consumer *c);
void cxl_hmu_unregister_consumer(const struct cxl_hmu_consumer *c);

/*
 * By the owning consumer only. cxl_hmu_start() validates @cfg against
 * the instance's capabilities and enables the hardware; -EPERM from a
 * non-owner, -EBUSY while active or when the range overlaps an active
 * sibling's, -ENODEV while the instance is going away. A consumer that may
 * sleep calls it from process context. cxl_hmu_stop() returns only after
 * the hardware is disabled, any delivery in progress has finished and the
 * entries still in the hotlist have been delivered; an atomic consumer
 * stops the hardware itself (see the perf consumer) instead.
 */
int cxl_hmu_start(struct cxl_hmu_inst *inst, const struct cxl_hmu_consumer *c,
		  const struct cxl_hmu_config *cfg);
void cxl_hmu_stop(struct cxl_hmu_inst *inst);
const struct cxl_hmu_caps *cxl_hmu_caps(const struct cxl_hmu_inst *inst);
const struct cxl_hmu_config *cxl_hmu_config(const struct cxl_hmu_inst *inst);
void *cxl_hmu_consumer_data(struct cxl_hmu_inst *inst);
void cxl_hmu_set_consumer_data(struct cxl_hmu_inst *inst, void *data);

#endif
