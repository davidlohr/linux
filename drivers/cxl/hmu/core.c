// SPDX-License-Identifier: GPL-2.0-only
/*
 * CXL Hot-range Monitoring Unit: instance devices.
 *
 * A CHMU register block holds up to eight instances, each with its own
 * configuration, hotlist and interrupt (CXL r4.0 Section 8.2.8). Each is
 * registered as a child device of the block.
 */
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "../cxl.h"
#include "../cxlpci.h"
#include "hmu.h"

/* Instance devices */

static void cxl_hmu_inst_release(struct device *dev)
{
	kfree(to_cxl_hmu_inst(dev));
}

static const struct device_type cxl_hmu_inst_type = {
	.name = "cxl_hmu_instance",
	.release = cxl_hmu_inst_release,
};

static void cxl_hmu_inst_remove(void *data)
{
	struct cxl_hmu_inst *inst = data;
	struct cxl_hmu_block *block = inst->block;

	block->inst[inst->index] = NULL;
	device_unregister(&inst->dev);
}

static int cxl_hmu_inst_add(struct cxl_hmu_block *block, unsigned int index,
			    unsigned int inst_len)
{
	struct device *dev = &block->hmu->dev;
	struct cxl_hmu_inst *inst;
	int rc;

	if (index >= CXL_HMU_MAX_INST)
		return -EINVAL;
	inst = kzalloc_obj(*inst);
	if (!inst)
		return -ENOMEM;
	inst->block = block;
	inst->index = index;
	inst->base = block->hmu->base + 0x10 + inst_len * index;

	device_initialize(&inst->dev);
	device_set_pm_not_required(&inst->dev);
	inst->dev.parent = dev;
	inst->dev.type = &cxl_hmu_inst_type;
	rc = dev_set_name(&inst->dev, "%s.%u", dev_name(dev), index);
	if (rc)
		goto err;
	block->inst[index] = inst;
	block->num_inst = max(block->num_inst, index + 1);
	rc = device_add(&inst->dev);
	if (rc) {
		block->inst[index] = NULL;
		goto err;
	}
	return devm_add_action_or_reset(dev, cxl_hmu_inst_remove, inst);
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

	for (i = 0; i < num_inst; i++) {
		rc = cxl_hmu_inst_add(block, i, inst_len);
		if (rc)
			return rc;
	}
	/*
	 * After every instance, so that at unbind every PMU is gone before
	 * the first instance is.
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
