/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright(c) 2024 Huawei
 * CXL Specification rev 3.2 Setion 8.2.8 (CHMU Register Interface)
 */
#ifndef CXL_HMU_H
#define CXL_HMU_H
#include <linux/device.h>

/* CXL r4.0 Table 8-190: NUMINST is four bits, up to 8 instances */
#define CXL_HMU_MAX_INST 8

struct cxl_hmu_inst;
struct cxl_memdev;

struct cxl_hmu {
	struct device dev;
	struct cxl_memdev *cxlmd;	/* the memdev whose DPA the block monitors */
	void __iomem *base;
	int assoc_id;
	int index;
	unsigned int num_inst;
	struct cxl_hmu_inst *inst[CXL_HMU_MAX_INST];	/* the registered instances */
	raw_spinlock_t lock;	/* enables and disables across the instances */
};

#define to_cxl_hmu(dev) container_of(dev, struct cxl_hmu, dev)
struct cxl_hmu_regs;
int devm_cxl_hmu_add(struct device *parent, struct cxl_hmu_regs *regs,
		     int assoc_id, int idx);

#endif
