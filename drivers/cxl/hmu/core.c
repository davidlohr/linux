// SPDX-License-Identifier: GPL-2.0-only
/*
 * CXL Hot-range Monitoring Unit: instance devices.
 *
 * A CHMU register block holds up to eight instances, each with its own
 * configuration, hotlist and interrupt (CXL r4.0 Section 8.2.8). Each is
 * registered as a child device of the block so that ownership and
 * configuration can be expressed per instance.
 */
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/slab.h>

#include "hmu.h"

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

static void cxl_hmu_inst_remove(void *dev)
{
	device_unregister(dev);
}

int cxl_hmu_inst_add(struct cxl_hmu *block, unsigned int index, unsigned int inst_len)
{
	struct pci_dev *pdev = to_pci_dev(block->dev.parent);
	struct cxl_hmu_inst *inst;
	int msg_num, rc;

	inst = kzalloc_obj(*inst);
	if (!inst)
		return -ENOMEM;
	inst->block = block;
	inst->index = index;
	inst->base = block->base + 0x10 + inst_len * index;
	raw_spin_lock_init(&inst->lock);
	msg_num = FIELD_GET(CHMU_INST0_CAP0_MSI_N_MSK,
			    readq(inst->base + CHMU_INST0_CAP0_REG));
	rc = pci_irq_vector(pdev, msg_num);
	if (rc < 0) {
		kfree(inst);
		return rc;
	}
	inst->irq = rc;

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
	return devm_add_action_or_reset(&block->dev, cxl_hmu_inst_remove, &inst->dev);
err:
	put_device(&inst->dev);
	return rc;
}
