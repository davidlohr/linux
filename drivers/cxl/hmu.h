/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright(c) 2024 Huawei
 * CXL Specification rev 3.2 Setion 8.2.8 (CHMU Register Interface)
 */
#ifndef CXL_HMU_H
#define CXL_HMU_H
#include <linux/device.h>

struct cxl_memdev;

/* A CHMU register block as cxl_core registers it; the driver's state is its own */
struct cxl_hmu {
	struct device dev;
	struct cxl_memdev *cxlmd;	/* the memdev whose DPA the block monitors */
	void __iomem *base;
	int assoc_id;
	int index;
};

#define to_cxl_hmu(dev) container_of(dev, struct cxl_hmu, dev)
struct cxl_hmu_regs;
int devm_cxl_hmu_add(struct cxl_memdev *cxlmd, struct cxl_hmu_regs *regs,
		     int idx);

#endif
