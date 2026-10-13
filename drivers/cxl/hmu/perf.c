// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for CXL Hotness Monitoring unit
 *
 * Based on hisi_ptt.c (Author Yicong Yang <yangyicong@hisilicon.com>)
 * Copyright (c) 2022-2024 HiSilicon Technologies Co., Ltd.
 *
 * TODO:
 * - Add capabilities attributes to help userspace know what can be set.
 * - Find out if timeouts are appropriate for real hardware. Currently
 *   assuming 0.1 seconds is enough for anything.
 */
#include <linux/dev_printk.h>
#include <linux/perf_event.h>
#include <linux/bitfield.h>
#include <linux/spinlock.h>
#include <linux/cleanup.h>
#include <linux/smp.h>
#include <linux/vmalloc.h>
#include <linux/device.h>
#include <linux/iopoll.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/math.h>
#include <linux/pci.h>
#include <linux/slab.h>

#include "../cxl.h"
#include "../hmu.h"
#include "hmu.h"

/* CFG attribute bit mappings */
#define CXL_HMU_ATTR_CONFIG_EPOCH_TYPE_MASK GENMASK(1, 0)
#define CXL_HMU_ATTR_CONFIG_ACCESS_TYPE_MASK GENMASK(9, 2)
#define CXL_HMU_ATTR_CONFIG_EPOCH_SCALE_MASK GENMASK(13, 10)
#define CXL_HMU_ATTR_CONFIG_EPOCH_MULT_MASK GENMASK(25, 14)
#define CXL_HMU_ATTR_CONFIG_RANDOM_DS_MASK BIT(26)
#define CXL_HMU_ATTR_CONFIG_DS_FACTOR_MASK GENMASK_ULL(34, 27)

#define CXL_HMU_ATTR_CONFIG1_HOTNESS_THRESH_MASK GENMASK(31, 0)
#define CXL_HMU_ATTR_CONFIG1_HOTNESS_GRANUAL_MASK GENMASK_ULL(63, 32)

/* In multiples of 256MiB */
#define CXL_HMU_ATTR_CONFIG2_DPA_BASE_MASK GENMASK(31, 0)
#define CXL_HMU_ATTR_CONFIG2_DPA_SIZE_MASK GENMASK_ULL(63, 32)

static int cxl_hmu_cpuhp_state_num;

struct cxl_hmu_info {
	struct pmu pmu;
	struct perf_output_handle handle;
	struct cxl_hmu_inst *inst;
	struct hlist_node node;
	int on_cpu;
	/*
	 * A hotlist delivered on another CPU (a shared vector another instance
	 * pointed elsewhere, a failed affinity change) is handed to the
	 * event's CPU, where the AUX buffer must be written from.
	 */
	call_single_data_t csd;
	u64 *pending;		/* twice the hotlist, for a batch and a late one */
	unsigned int nr_pending;
	u8 pending_width;
	bool pending_overflowed;

	/* Protect the pmu state; the device state has the instance lock */
	raw_spinlock_t lock;
};

#define pmu_to_cxl_hmu(p) container_of(p, struct cxl_hmu_info, pmu)

/* destriptor for the aux buffer */
struct cxl_hmu_buf {
	size_t length;
	int nr_pages;
	void *base;
};

static ssize_t cpumask_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct cxl_hmu_info *hmu = pmu_to_cxl_hmu((struct pmu *)dev_get_drvdata(dev));

	return cpumap_print_to_pagebuf(true, buf, cpumask_of(hmu->on_cpu));
}
static DEVICE_ATTR_RO(cpumask);

static struct attribute *cxl_hmu_cpumask_attrs[] = {
	&dev_attr_cpumask.attr,
	NULL
};

static const struct attribute_group cxl_hmu_cpumask_attr_group = {
	.attrs = cxl_hmu_cpumask_attrs,
};

/* Sized fields to future proof based on space in spec */
PMU_FORMAT_ATTR(epoch_type, "config:0-1"); /* 2 bits to future proof */
PMU_FORMAT_ATTR(access_type, "config:2-9");
PMU_FORMAT_ATTR(epoch_scale, "config:10-13");
PMU_FORMAT_ATTR(epoch_multiplier, "config:14-25");
PMU_FORMAT_ATTR(randomized_downsampling, "config:26-26");
PMU_FORMAT_ATTR(downsampling_factor, "config:27-34");

PMU_FORMAT_ATTR(hotness_threshold, "config1:0-31");
PMU_FORMAT_ATTR(hotness_granual, "config1:32-63");

/* RFC this is a bitmap can we control it better? */
PMU_FORMAT_ATTR(range_base, "config2:0-31");
PMU_FORMAT_ATTR(range_size, "config2:32-63");
static struct attribute *cxl_hmu_format_attrs[] = {
	&format_attr_epoch_type.attr,
	&format_attr_access_type.attr,
	&format_attr_epoch_scale.attr,
	&format_attr_epoch_multiplier.attr,
	&format_attr_randomized_downsampling.attr,
	&format_attr_downsampling_factor.attr,
	&format_attr_hotness_threshold.attr,
	&format_attr_hotness_granual.attr,
	&format_attr_range_base.attr,
	&format_attr_range_size.attr,
	NULL
};

static struct attribute_group cxl_hmu_format_attr_group = {
	.name = "format",
	.attrs = cxl_hmu_format_attrs,
};

static const struct attribute_group *cxl_hmu_groups[] = {
	&cxl_hmu_cpumask_attr_group,
	&cxl_hmu_format_attr_group,
	NULL
};

static void cxl_hmu_event_destroy(struct perf_event *event)
{
	kfree(event->pmu_private);
	event->pmu_private = NULL;
}

/*
 * The configuration is the event's, decoded once here and programmed when
 * the event starts, so another open on the PMU cannot change it.
 */
static int cxl_hmu_event_init(struct perf_event *event)
{
	struct cxl_hmu_info *hmu = pmu_to_cxl_hmu(event->pmu);
	struct cxl_hmu_inst *inst = hmu->inst;
	const struct cxl_hmu_caps *caps = &inst->caps;
	struct device *dev = event->pmu->dev;
	struct cxl_hmu_config *cfg;
	u64 gran;
	int rc;

	if (event->attr.type != hmu->pmu.type)
		return -ENOENT;

	/* Hotlist entries carry raw device physical addresses */
	if (!perfmon_capable())
		return -EACCES;

	if (event->cpu < 0) {
		dev_info(dev, "Per-task mode not supported\n");
		return -EOPNOTSUPP;
	}

	if (event->attach_state & PERF_ATTACH_TASK)
		return -EOPNOTSUPP;

	cfg = kzalloc_obj(*cfg);
	if (!cfg)
		return -ENOMEM;
	event->pmu_private = cfg;
	event->destroy = cxl_hmu_event_destroy;

	switch (FIELD_GET(CXL_HMU_ATTR_CONFIG_EPOCH_TYPE_MASK,
		event->attr.config)) {
	case 0:
		cfg->mode = CXL_HMU_MODE_EPOCH;
		break;
	case 1:
		cfg->mode = CXL_HMU_MODE_ALWAYS_ON;
		break;
	default:
		dev_dbg(dev, "Tried for a non existent type\n");
		return -EINVAL;
	}
	cfg->randomized_ds = FIELD_GET(CXL_HMU_ATTR_CONFIG_RANDOM_DS_MASK,
				       event->attr.config);
	cfg->access_type = FIELD_GET(CXL_HMU_ATTR_CONFIG_ACCESS_TYPE_MASK,
				     event->attr.config);
	cfg->hotness_threshold = FIELD_GET(CXL_HMU_ATTR_CONFIG1_HOTNESS_THRESH_MASK,
					   event->attr.config1);

	/* The unit size as log2; default to the smallest supported unit */
	gran = FIELD_GET(CXL_HMU_ATTR_CONFIG1_HOTNESS_GRANUAL_MASK, event->attr.config1);
	if (gran > 63)
		return -EINVAL;
	cfg->unit_shift = gran;
	if (cfg->unit_shift == 0 && caps->unit_sizes)
		cfg->unit_shift = 8 + __ffs(caps->unit_sizes);

	/* Default of 0 means the smallest supported down-sampling factor */
	cfg->ds_shift = FIELD_GET(CXL_HMU_ATTR_CONFIG_DS_FACTOR_MASK,
				  event->attr.config);
	if (cfg->ds_shift == 0 && caps->ds_factors && !(caps->ds_factors & BIT(0)))
		cfg->ds_shift = __ffs(caps->ds_factors);

	cfg->epoch_scale = FIELD_GET(CXL_HMU_ATTR_CONFIG_EPOCH_SCALE_MASK,
				     event->attr.config);
	cfg->epoch_mult = FIELD_GET(CXL_HMU_ATTR_CONFIG_EPOCH_MULT_MASK,
				    event->attr.config);
	/* Default to the shortest epoch the device supports */
	if (cfg->epoch_mult == 0 && cfg->epoch_scale == 0)
		cxl_hmu_pick_epoch(inst, caps->epoch_min_us, &cfg->epoch_scale,
				   &cfg->epoch_mult);

	cfg->range_base = FIELD_GET(CXL_HMU_ATTR_CONFIG2_DPA_BASE_MASK,
				    event->attr.config2);
	cfg->range_num = FIELD_GET(CXL_HMU_ATTR_CONFIG2_DPA_SIZE_MASK,
				   event->attr.config2);
	/* Default of 'everything': the device capacity */
	if (cfg->range_num == 0)
		cfg->range_num = caps->capacity_ranges;

	/*
	 * The counter width, and so the threshold bound, follows the unit
	 * size, which can be programmed into the disabled instance now.
	 */
	rc = cxl_hmu_set_unit_size(inst, cfg->unit_shift);
	if (rc)
		return rc;
	rc = cxl_hmu_validate_config(inst, cfg);
	if (rc) {
		dev_dbg(dev, "configuration rejected: %d\n", rc);
		return rc;
	}
	return 0;
}

/* 16 bytes of header - arbitrary choice! */
#define CHMU_HEADER0_SIZE_MASK GENMASK(15, 0)
#define CHMU_HEADER0_COUNT_WIDTH GENMASK(23, 16)
#define CHMU_HEADER1_MAGIC 0xDEADBEEF

static void cxl_hmu_aux_write(struct cxl_hmu_buf *buf, size_t pos, const void *src,
			      size_t len)
{
	size_t first = min(len, buf->length - pos);

	memcpy(buf->base + pos, src, first);
	if (len > first)
		memcpy(buf->base, src + first, len - first);
}

/*
 * Copy drained hotlist entries into the AUX buffer, within what the
 * handle says is free of the ring, wrapping at its end, and end the output
 * handle, beginning a new one unless @stop. Entries that do not fit are
 * dropped and the record is marked truncated, which perf core answers by
 * disabling the event, as for any AUX PMU whose consumer fell behind. A
 * hotlist overflow reported by the device marks the record partial: the
 * data is sound but has gaps.
 */
static int cxl_hmu_perf_flush(struct cxl_hmu_info *hmu, const u64 *entries,
			      unsigned int nr, u8 count_width, bool overflowed,
			      bool stop)
{
	struct perf_output_handle *handle = &hmu->handle;
	struct cxl_hmu_buf *buf = perf_get_aux(handle);
	struct perf_event *event = handle->event;
	size_t size = 0, tocopy, pos;
	u64 header[2];

	/*
	 * A stale or stop-raced interrupt can reach here with no AUX
	 * session: perf_get_aux() returns NULL once the handle has been
	 * closed, and handle->size is only meaningful while one is open.
	 */
	if (!buf || !handle->size)
		return -EINVAL;

	/*
	 * One record per 65535 entries, the header's count field: a delivery
	 * queued while the vector was on another CPU can hold more than one
	 * hotlist.
	 */
	while (nr && handle->size - size >= sizeof(header) + 8) {
		size_t chunk = min_t(size_t, nr, FIELD_MAX(CHMU_HEADER0_SIZE_MASK));

		tocopy = min_t(size_t, chunk * 8,
			       (handle->size - size - sizeof(header)) & ~7UL);
		header[0] = FIELD_PREP(CHMU_HEADER0_SIZE_MASK, tocopy / 8) |
			    FIELD_PREP(CHMU_HEADER0_COUNT_WIDTH, count_width);
		header[1] = CHMU_HEADER1_MAGIC;
		pos = (handle->head + size) % buf->length;
		cxl_hmu_aux_write(buf, pos, header, sizeof(header));
		pos = (pos + sizeof(header)) % buf->length;
		cxl_hmu_aux_write(buf, pos, entries, tocopy);
		size += sizeof(header) + tocopy;
		entries += tocopy / 8;
		nr -= tocopy / 8;
	}
	if (nr)
		perf_aux_output_flag(handle, PERF_AUX_FLAG_TRUNCATED);
	if (overflowed)
		perf_aux_output_flag(handle, PERF_AUX_FLAG_PARTIAL);

	perf_aux_output_end(handle, size);
	if (!stop && !perf_aux_output_begin(handle, event))
		return -EINVAL;
	return 0;
}

/* With hmu->lock held, on the event's CPU */
static void cxl_hmu_perf_flush_pending(struct cxl_hmu_info *hmu, bool stop)
{
	if (!hmu->nr_pending && !hmu->pending_overflowed)
		return;
	cxl_hmu_perf_flush(hmu, hmu->pending, hmu->nr_pending, hmu->pending_width,
			   hmu->pending_overflowed, stop);
	hmu->nr_pending = 0;
	hmu->pending_overflowed = false;
}

static void cxl_hmu_perf_csd(void *data)
{
	struct cxl_hmu_info *hmu = data;

	guard(raw_spinlock_irqsave)(&hmu->lock);
	cxl_hmu_perf_flush_pending(hmu, false);
}

/*
 * From the hard interrupt. The AUX buffer belongs to the event's CPU, so a
 * delivery elsewhere is queued for it; a stop on that CPU, with interrupts
 * off, cannot interleave with a delivery there.
 */
static void cxl_hmu_perf_hotlist(struct cxl_hmu_inst *inst, const u64 *entries,
				 unsigned int nr, u8 counter_width, bool overflowed)
{
	struct cxl_hmu_info *hmu = inst->drvdata;
	unsigned int room;

	guard(raw_spinlock_irqsave)(&hmu->lock);
	if (smp_processor_id() == hmu->on_cpu) {
		cxl_hmu_perf_flush_pending(hmu, false);
		if (cxl_hmu_perf_flush(hmu, entries, nr, counter_width, overflowed, false))
			dev_dbg(hmu->pmu.dev, "interrupt update failed\n");
		return;
	}
	room = 2 * inst->caps.hotlist_size - hmu->nr_pending;
	if (nr > room) {
		nr = room;
		hmu->pending_overflowed = true;
	}
	memcpy(hmu->pending + hmu->nr_pending, entries, nr * 8);
	hmu->nr_pending += nr;
	hmu->pending_width = counter_width;
	hmu->pending_overflowed |= overflowed;
	/* -EBUSY means a call is already on its way; it takes everything pending */
	smp_call_function_single_async(hmu->on_cpu, &hmu->csd);
}

static int __cxl_hmu_perf_start(struct perf_event *event, int flags)
{
	struct cxl_hmu_info *hmu = pmu_to_cxl_hmu(event->pmu);
	struct cxl_hmu_inst *inst = hmu->inst;
	struct hw_perf_event *hwc = &event->hw;
	struct device *dev = event->pmu->dev;
	int cpu = event->cpu;
	int ret;

	hwc->state = 0;
	if (inst->active) {
		dev_dbg(dev, "trace already started\n");
		return -EBUSY;
	}
	hmu->on_cpu = cpu;
	hmu->nr_pending = 0;
	hmu->pending_overflowed = false;

	if (!perf_aux_output_begin(&hmu->handle, event)) {
		dev_dbg(event->pmu->dev, "aux output begin failed\n");
		return -EINVAL;
	}

	ret = cxl_hmu_hw_start(inst, event->pmu_private, cxl_hmu_perf_hotlist, true);
	if (ret) {
		/*
		 * Unwind the session: perf_aux_output_begin() nested the handle
		 * and took references that only the matching end releases.
		 */
		perf_aux_output_end(&hmu->handle, 0);
		return ret;
	}
	/*
	 * TODO: Figure out what to do as very likely this is shared
	 *  - Hopefully only with other HMU instances
	 */
	if (irq_set_affinity(inst->irq, cpumask_of(cpu)))
		dev_warn(dev, "failed to affinity of HMU interrupt\n");
	return 0;
}

static void cxl_hmu_perf_start(struct perf_event *event, int flags)
{
	struct cxl_hmu_info *hmu = pmu_to_cxl_hmu(event->pmu);
	int ret;

	guard(raw_spinlock_irqsave)(&hmu->lock);

	ret = __cxl_hmu_perf_start(event, flags);
	if (ret)
		event->hw.state |= PERF_HES_STOPPED;
}

/*
 * Stop the hardware if this PMU runs it, and close the AUX session either
 * way: the instance may already have been stopped on perf's behalf at
 * unbind, and the session's references are perf's to release.
 */
static void cxl_hmu_perf_stop(struct perf_event *event, int flags)
{
	struct cxl_hmu_info *hmu = pmu_to_cxl_hmu(event->pmu);
	struct cxl_hmu_inst *inst = hmu->inst;
	struct hw_perf_event *hwc = &event->hw;
	bool overflowed = false;
	u8 width = 0;
	int n = 0;

	if (hwc->state & PERF_HES_STOPPED)
		return;

	guard(raw_spinlock_irqsave)(&hmu->lock);
	if (cxl_hmu_hw_stop(inst))
		n = cxl_hmu_drain(inst, inst->entries, inst->caps.hotlist_size,
				  &width, &overflowed);
	/* Deliver what was pending and what the device still held, then close */
	cxl_hmu_perf_flush_pending(hmu, false);
	cxl_hmu_perf_flush(hmu, inst->entries, n, width, overflowed, true);
	hwc->state |= PERF_HES_STOPPED | PERF_HES_UPTODATE;
}

static void cxl_hmu_read(struct perf_event *event)
{
	/* Nothing to do */
}

static int cxl_hmu_add(struct perf_event *event, int flags)
{
	struct hw_perf_event *hwc = &event->hw;

	hwc->state = PERF_HES_STOPPED | PERF_HES_UPTODATE;
	if (flags & PERF_EF_START) {
		cxl_hmu_perf_start(event, PERF_EF_RELOAD);
		if (hwc->state & PERF_HES_STOPPED)
			return -EINVAL;
	}
	return 0;
}

static void cxl_hmu_del(struct perf_event *event, int flags)
{
	cxl_hmu_perf_stop(event, PERF_EF_UPDATE);
}

static void *cxl_hmu_setup_aux(struct perf_event *event, void **pages,
			int nr_pages, bool overwrite)
{
	int i;

	if (overwrite) {
		dev_warn(event->pmu->dev, "Overwrite mode is not supported\n");
		return NULL;
	}

	if (nr_pages < 1)
		return NULL;

	struct cxl_hmu_buf *buf __free(kfree) =
		kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return NULL;

	struct page **pagelist __free(kfree) =
		kcalloc(nr_pages, sizeof(*pagelist), GFP_KERNEL);
	if (!pagelist)
		return NULL;

	for (i = 0; i < nr_pages; i++)
		pagelist[i] = virt_to_page(pages[i]);

	buf->base = vmap(pagelist, nr_pages, VM_MAP, PAGE_KERNEL);
	if (!buf->base)
		return NULL;

	buf->nr_pages = nr_pages;
	buf->length = nr_pages * PAGE_SIZE;

	return_ptr(buf);
}

static void cxl_hmu_free_aux(void *aux)
{
	struct cxl_hmu_buf *buf = aux;

	vunmap(buf->base);
	kfree(buf);
}

static void cxl_hmu_perf_unregister(void *_info)
{
	struct cxl_hmu_info *info = _info;

	perf_pmu_unregister(&info->pmu);
}

static void cxl_hmu_cpuhp_remove(void *_info)
{
	struct cxl_hmu_info *info = _info;

	cpuhp_state_remove_instance_nocalls(cxl_hmu_cpuhp_state_num,
					    &info->node);
}

int cxl_hmu_perf_probe_inst(struct cxl_hmu_inst *inst)
{
	struct device *idev = &inst->dev;
	struct cxl_hmu *hmu = inst->block;
	struct device *dev = &hmu->dev;
	struct cxl_hmu_info *info;
	char *pmu_name;
	int rc;

	/*
	 * TODO add polling support - for now require the level-crossing
	 * interrupt, and a vector to deliver it: without one the core refuses
	 * every consumer, and a PMU would only have its CPU hotplug callbacks
	 * fail to set an affinity.
	 */
	if (!inst->caps.irq_level || inst->irq < 0)
		return 0;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->on_cpu = -1;
	info->inst = inst;
	raw_spin_lock_init(&info->lock);
	INIT_CSD(&info->csd, cxl_hmu_perf_csd, info);
	info->pending = devm_kcalloc(dev, 2 * inst->caps.hotlist_size,
				     sizeof(*info->pending), GFP_KERNEL);
	if (!info->pending)
		return -ENOMEM;
	pmu_name = devm_kasprintf(dev, GFP_KERNEL, "cxl_hmu_mem%d.%d.%u",
				  hmu->assoc_id, hmu->index, inst->index);
	if (!pmu_name)
		return -ENOMEM;
	info->pmu = (struct pmu) {
		.name = pmu_name,
		.parent = idev,
		.module = THIS_MODULE,
		.capabilities = PERF_PMU_CAP_EXCLUSIVE |
				PERF_PMU_CAP_NO_EXCLUDE,
		.task_ctx_nr = perf_sw_context,
		.attr_groups = cxl_hmu_groups,
		.event_init = cxl_hmu_event_init,
		.setup_aux = cxl_hmu_setup_aux,
		.free_aux = cxl_hmu_free_aux,
		.start = cxl_hmu_perf_start,
		.stop = cxl_hmu_perf_stop,
		.add = cxl_hmu_add,
		.del = cxl_hmu_del,
		.read = cxl_hmu_read,
	};
	rc = cpuhp_state_add_instance(cxl_hmu_cpuhp_state_num, &info->node);
	if (rc)
		return rc;
	rc = devm_add_action_or_reset(dev, cxl_hmu_cpuhp_remove, info);
	if (rc)
		return rc;

	rc = perf_pmu_register(&info->pmu, info->pmu.name, -1);
	if (rc)
		return rc;
	rc = devm_add_action_or_reset(dev, cxl_hmu_perf_unregister, info);
	if (rc)
		return rc;
	inst->drvdata = info;
	return 0;
}

static int cxl_hmu_online_cpu(unsigned int cpu, struct hlist_node *node)
{
	struct cxl_hmu_info *info =
		hlist_entry_safe(node, struct cxl_hmu_info, node);

	if (info->on_cpu != -1)
		return 0;

	info->on_cpu = cpu;

	WARN_ON(irq_set_affinity(info->inst->irq, cpumask_of(cpu)));

	return 0;
}

static int cxl_hmu_offline_cpu(unsigned int cpu, struct hlist_node *node)
{
	struct cxl_hmu_info *info =
		hlist_entry_safe(node, struct cxl_hmu_info, node);
	unsigned int target;

	if (info->on_cpu != cpu)
		return 0;

	info->on_cpu = -1;
	target = cpumask_any_but(cpu_online_mask, cpu);
	if (target >= nr_cpu_ids) {
		dev_err(info->pmu.dev, "Unable to find a suitable CPU\n");
		return 0;
	}

	perf_pmu_migrate_context(&info->pmu, cpu, target);
	info->on_cpu = target;
	/*
	 * CPU HP lock is held so we should be guaranteed that this CPU hasn't
	 * yet gone away.
	 */
	WARN_ON(irq_set_affinity(info->inst->irq, cpumask_of(target)));
	return 0;
}

int cxl_hmu_perf_init(void)
{
	int rc;

	rc = cpuhp_setup_state_multi(CPUHP_AP_ONLINE_DYN,
				     "AP_PERF_CXL_HMU_ONLINE",
				     cxl_hmu_online_cpu, cxl_hmu_offline_cpu);
	if (rc < 0)
		return rc;
	cxl_hmu_cpuhp_state_num = rc;
	return 0;
}

void cxl_hmu_perf_exit(void)
{
	cpuhp_remove_multi_state(cxl_hmu_cpuhp_state_num);
}
