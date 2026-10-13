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


struct cxl_hmu_inst {
	struct device		dev;
	struct cxl_hmu		*block;
	void __iomem		*base;		/* block base + 0x10 + INSTLEN * index */
	unsigned int		index;
	int			irq;
	raw_spinlock_t		lock;
	void			*drvdata;	/* the perf consumer's per-instance state */
};
#define to_cxl_hmu_inst(d) container_of(d, struct cxl_hmu_inst, dev)

int cxl_hmu_inst_add(struct cxl_hmu *block, unsigned int index, unsigned int inst_len);
bool cxl_hmu_dev_is_inst(struct device *dev);

#endif
