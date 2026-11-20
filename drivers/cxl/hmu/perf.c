// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for CXL Hotness Monitoring unit
 *
 * Based on hisi_ptt.c (Author Yicong Yang <yangyicong@hisilicon.com>)
 * Copyright (c) 2022-2024 HiSilicon Technologies Co., Ltd.
 *
 * TODO:
 * - Add capabilities attributes to help userspace know what can be set.
 */
#include <linux/dev_printk.h>
#include <linux/perf_event.h>
#include <linux/bitfield.h>
#include <linux/spinlock.h>
#include <linux/cleanup.h>
#include <linux/smp.h>
#include <linux/vmalloc.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq_work.h>
#include <linux/list.h>
#include <linux/minmax.h>
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
#define CXL_HMU_ATTR_CONFIG_RSVD_MASK GENMASK_ULL(63, 35)

#define CXL_HMU_ATTR_CONFIG1_HOTNESS_THRESH_MASK GENMASK(31, 0)
#define CXL_HMU_ATTR_CONFIG1_HOTNESS_GRANUAL_MASK GENMASK_ULL(63, 32)

/* In multiples of 256MiB */
#define CXL_HMU_ATTR_CONFIG2_DPA_BASE_MASK GENMASK(31, 0)
#define CXL_HMU_ATTR_CONFIG2_DPA_SIZE_MASK GENMASK_ULL(63, 32)

static int cxl_hmu_cpuhp_state_num;

struct cxl_hmu_info {
	struct pmu pmu;
	/* Open for the length of one delivery only: see cxl_hmu_perf_deliver() */
	struct perf_output_handle handle;
	struct perf_event *event;	/* the one running the instance, with cpu */
	struct cxl_hmu_inst *inst;
	struct hlist_node node;
	/*
	 * The CPU events are opened on and the vector is made affine to.
	 * Written by the CPU hotplug callbacks only.
	 */
	int on_cpu;
	/*
	 * The CPU the event runs the instance on, where its AUX buffer must
	 * be written from, or -1 while no event does. Set by start and stop
	 * under the lock; while it is -1 the instance's interrupts are not
	 * perf's.
	 */
	int cpu;
	/*
	 * An interrupt taken on another CPU (the hotplug callback has moved
	 * the event and not yet the vector, a failed affinity change) reads
	 * nothing: it queues this on the event's CPU, which drains there.
	 */
	struct irq_work work;
	u64 *entries;		/* drain scratch, hotlist_size entries */
	/*
	 * Why the event that holds the instance cannot start, once a start
	 * has failed on the device. It will not do better on a second try,
	 * and perf core tries again at every multiplexing tick for as long
	 * as the event is open: see __cxl_hmu_perf_start(). Cleared when an
	 * event takes the instance, which no other event's start can race
	 * with, since there is one event per claim.
	 */
	int start_err;

	/*
	 * Protect the pmu state; the device state has the instance lock,
	 * which nests inside this one.
	 */
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

	int cpu = READ_ONCE(hmu->on_cpu);

	/* None while the hotplug callback is between two CPUs */
	return sysfs_emit(buf, "%*pbl\n",
			  cpumask_pr_args(cpu < 0 ? cpu_none_mask : cpumask_of(cpu)));
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
	struct cxl_hmu_info *hmu = pmu_to_cxl_hmu(event->pmu);

	cxl_hmu_release(hmu->inst);
}

/*
 * The configuration is the event's: decoded here, handed to the claim and
 * programmed when the event starts. Nothing is written to the device or to
 * the instance before the claim succeeds, and then only the unit size, to
 * learn the counter width from the device. An event holds the instance
 * from here to its destroy, running or not, so a second event, or another
 * user of the instance, gets -EBUSY when it tries rather than a start that
 * fails later.
 */
static int cxl_hmu_event_init(struct perf_event *event)
{
	struct cxl_hmu_info *hmu = pmu_to_cxl_hmu(event->pmu);
	struct cxl_hmu_inst *inst = hmu->inst;
	const struct cxl_hmu_caps *caps = &inst->caps;
	struct device *dev = event->pmu->dev;
	struct cxl_hmu_config config = {}, *cfg = &config;
	u64 gran;
	int cpu, rc;

	if (event->attr.type != hmu->pmu.type)
		return -ENOENT;

	/* Hotlist entries carry raw device physical addresses */
	if (!perfmon_capable())
		return -EACCES;

	/*
	 * The device is counted, not a task. perf core refuses a cgroup's
	 * event with -EINVAL after this returns, for the same reason.
	 */
	if (event->attach_state & PERF_ATTACH_TASK)
		return -EOPNOTSUPP;

	/*
	 * One CPU serves the PMU, the one its cpumask shows, whatever CPU the
	 * event was asked for: the hotplug callbacks move the events and the
	 * interrupt together, and can only do that if every event is where
	 * they expect it. None is online for a moment while they move.
	 */
	cpu = READ_ONCE(hmu->on_cpu);
	if (cpu < 0)
		return -ENODEV;
	event->cpu = cpu;

	/* No format attribute sets these; keep them free to be given a meaning */
	if (event->attr.config & CXL_HMU_ATTR_CONFIG_RSVD_MASK)
		return -EINVAL;

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

	rc = cxl_hmu_claim(inst, "perf", cfg);
	if (rc) {
		dev_dbg(dev, "claim refused: %d\n", rc);
		return rc;
	}
	/*
	 * The threshold's bound is the counter width, which the device only
	 * shows once the unit size is programmed. The instance is this
	 * event's now, so that can be done here, where there is an errno to
	 * return; a start that fails has none, and is tried again for ever.
	 */
	rc = cxl_hmu_check_threshold(inst);
	if (rc) {
		cxl_hmu_release(inst);
		return rc;
	}
	hmu->start_err = 0;
	event->destroy = cxl_hmu_event_destroy;
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
 * One delivery, on an interrupt or at the stop: open an AUX output, move
 * what fits of the hotlist into it as one record, close it. With hmu->lock
 * held and interrupts off, on the event's CPU: the AUX buffer belongs to
 * it, and a stop there cannot interleave.
 *
 * The output is not held open between deliveries. Its free space is what
 * the reader had left when it was opened, so an output opened at the end
 * of the previous delivery would judge this one against a buffer the
 * reader may have emptied since, and a full buffer makes perf core disable
 * the event. Opened here, the space is as fresh as it can be, and it bounds
 * the drain: what does not fit is not taken from the device.
 *
 * A record's count field holds 65535 entries, which is also the most a
 * hotlist can hold, so a delivery is one record.
 *
 * Flags, one per way of losing data. PARTIAL is the device's: its hotlist
 * overflowed and it lost hot units for it; the data is sound but has gaps.
 * An instance that postpones the units it could not insert lost nothing
 * and its record is not marked. TRUNCATED is the kernel's: entries were
 * left in the hotlist for want of room in the AUX buffer. perf core answers
 * it by disabling the event, as for any AUX PMU whose reader fell behind,
 * and it has to: no later interrupt can be counted on to come back for
 * them, since the fill level would have to be crossed again. The stop that
 * follows is the @last delivery. It takes what the reader has made room for
 * by then, and what is still left is discarded and counted as dropped.
 */
static void cxl_hmu_perf_deliver(struct cxl_hmu_info *hmu, bool last)
{
	struct perf_output_handle *handle = &hmu->handle;
	struct cxl_hmu_inst *inst = hmu->inst;
	unsigned int max = 0, n, flags;
	struct cxl_hmu_buf *buf;
	size_t size = 0, pos;
	u64 header[2];

	/*
	 * No output: the buffer is full, and perf core has just disabled the
	 * event for it, or it is being unmapped, and perf core stops the
	 * event first. Leave the device alone unless this is the stop.
	 */
	buf = perf_aux_output_begin(handle, hmu->event);
	if (!buf) {
		if (last)
			cxl_hmu_drain(inst, NULL, UINT_MAX, &flags);
		return;
	}

	if (handle->size >= sizeof(header) + 8)
		max = min3((handle->size - sizeof(header)) / 8,
			   (size_t)inst->caps.hotlist_size,
			   (size_t)FIELD_MAX(CHMU_HEADER0_SIZE_MASK));
	n = cxl_hmu_drain(inst, hmu->entries, max, &flags);
	if (n) {
		header[0] = FIELD_PREP(CHMU_HEADER0_SIZE_MASK, n) |
			    FIELD_PREP(CHMU_HEADER0_COUNT_WIDTH, inst->live_width);
		header[1] = CHMU_HEADER1_MAGIC;
		pos = handle->head % buf->length;
		cxl_hmu_aux_write(buf, pos, header, sizeof(header));
		pos = (pos + sizeof(header)) % buf->length;
		cxl_hmu_aux_write(buf, pos, hmu->entries, n * 8);
		size = sizeof(header) + n * 8;
	}
	if (flags & CXL_HMU_DRAIN_LOST)
		perf_aux_output_flag(handle, PERF_AUX_FLAG_PARTIAL);
	if ((flags & CXL_HMU_DRAIN_MORE) && max < inst->caps.hotlist_size)
		perf_aux_output_flag(handle, PERF_AUX_FLAG_TRUNCATED);
	if (last && (flags & CXL_HMU_DRAIN_MORE))
		cxl_hmu_drain(inst, NULL, UINT_MAX, &flags);
	perf_aux_output_end(handle, size);
}

/*
 * The interrupt was taken on another CPU. If the event has stopped since,
 * its stop took what the hotlist held; if it has started again elsewhere,
 * that start cleared the status and the device will interrupt again.
 */
static void cxl_hmu_perf_work(struct irq_work *work)
{
	struct cxl_hmu_info *hmu = container_of(work, struct cxl_hmu_info, work);

	guard(raw_spinlock_irqsave)(&hmu->lock);
	if (hmu->cpu == smp_processor_id() && cxl_hmu_pending(hmu->inst))
		cxl_hmu_perf_deliver(hmu, false);
}

/*
 * The instance core requests no interrupt; this is perf's, on the vector
 * the core looked up. It may be shared with other instances (the spec only
 * recommends against it), so the handler answers for its own instance
 * only, and only while its event runs it: an instance that another holder
 * polls, or that nobody runs, is not perf's to acknowledge. A vector the
 * device also uses for the cxl_pci mailbox or events cannot be had: those
 * are IRQF_ONESHOT and requested first, so such an instance has no PMU.
 * IRQF_NO_THREAD: the drain is a hard interrupt by design and takes raw
 * locks, so forced threading must not move it.
 *
 * On a CPU other than the event's nothing is read and nothing is
 * acknowledged. The status bits stay set, which keeps the device from
 * interrupting again until the work has drained on the right CPU, so there
 * is nothing to queue but the work itself. hmu->cpu is online: the event
 * runs there, and perf core stops it before that CPU goes.
 */
static irqreturn_t cxl_hmu_perf_irq(int irq, void *data)
{
	struct cxl_hmu_info *hmu = data;

	guard(raw_spinlock)(&hmu->lock);
	if (hmu->cpu < 0 || !cxl_hmu_pending(hmu->inst))
		return IRQ_NONE;
	if (hmu->cpu == smp_processor_id())
		cxl_hmu_perf_deliver(hmu, false);
	else
		irq_work_queue_on(&hmu->work, hmu->cpu);
	return IRQ_HANDLED;
}

static int __cxl_hmu_perf_start(struct perf_event *event, int flags)
{
	struct cxl_hmu_info *hmu = pmu_to_cxl_hmu(event->pmu);
	struct cxl_hmu_inst *inst = hmu->inst;
	struct hw_perf_event *hwc = &event->hw;
	int ret;

	hwc->state = 0;

	/*
	 * An add that fails leaves the event inactive, and perf core adds a
	 * flexible event again at every multiplexing tick. A start that the
	 * device refused would then reset and reprogram it a thousand times
	 * a second, with interrupts off, to fail the same way: fail it here
	 * instead, before anything is touched, until the event is closed.
	 */
	if (hmu->start_err)
		return hmu->start_err;

	/*
	 * Each delivery opens its own AUX output. Open and close an empty one
	 * here all the same, so that an event without an AUX buffer, or with
	 * a full one, does not run the device for nothing.
	 */
	if (!perf_aux_output_begin(&hmu->handle, event)) {
		dev_dbg(event->pmu->dev, "aux output begin failed\n");
		return -EINVAL;
	}
	perf_aux_output_end(&hmu->handle, 0);

	ret = cxl_hmu_start(inst, CXL_HMU_START_IRQ);
	if (ret) {
		hmu->start_err = ret;
		dev_err(event->pmu->dev, "start failed (%d): the event stays stopped until it is opened again\n",
			ret);
		return ret;
	}
	hmu->event = event;
	/*
	 * Where perf core runs the event, which is on_cpu but for the moment
	 * the hotplug callback has moved the event and not yet the vector. A
	 * delivery on any other CPU is handed over to this one.
	 */
	hmu->cpu = smp_processor_id();
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

/* Stop the hardware and write what the hotlist still holds */
static void cxl_hmu_perf_stop(struct perf_event *event, int flags)
{
	struct cxl_hmu_info *hmu = pmu_to_cxl_hmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;

	if (hwc->state & PERF_HES_STOPPED)
		return;

	guard(raw_spinlock_irqsave)(&hmu->lock);
	cxl_hmu_stop(hmu->inst);
	cxl_hmu_perf_deliver(hmu, true);
	hmu->event = NULL;
	hmu->cpu = -1;
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

static void cxl_hmu_perf_free_irq(void *_info)
{
	struct cxl_hmu_info *info = _info;

	free_irq(info->inst->irq, info);
	/* Only the handler queues it; the PMU went first, so it finds no event */
	irq_work_sync(&info->work);
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
	struct cxl_hmu *hmu = inst->block->hmu;
	struct device *dev = &hmu->dev;
	struct cxl_hmu_info *info;
	char *pmu_name;
	int rc;

	/*
	 * TODO add polling support - for now require the level-crossing
	 * interrupt, and a vector to deliver it. An instance without either
	 * has no PMU; it is still registered and can be polled by an
	 * in-kernel user.
	 */
	if (!inst->caps.irq_level || inst->irq < 0)
		return 0;

	info = devm_kzalloc_obj(dev, *info);
	if (!info)
		return -ENOMEM;
	info->on_cpu = -1;
	info->cpu = -1;
	info->inst = inst;
	raw_spin_lock_init(&info->lock);
	/* Hard on PREEMPT_RT too: it does what the hard handler would have */
	info->work = IRQ_WORK_INIT_HARD(cxl_hmu_perf_work);
	info->entries = devm_kzalloc_objs(dev, *info->entries,
					  inst->caps.hotlist_size);
	if (!info->entries)
		return -ENOMEM;
	pmu_name = devm_kasprintf(dev, GFP_KERNEL, "cxl_hmu_mem%d.%d.%u",
				  hmu->assoc_id, hmu->index, inst->index);
	if (!pmu_name)
		return -ENOMEM;

	/*
	 * Requested once everything the handler uses exists, and freed before
	 * any of it is: the handler takes info->lock from the first moment.
	 * A vector that cannot be had costs this instance its PMU, not the
	 * block its probe.
	 */
	rc = request_irq(inst->irq, cxl_hmu_perf_irq,
			 IRQF_SHARED | IRQF_NO_THREAD | IRQF_NOBALANCING,
			 pmu_name, info);
	if (rc) {
		dev_warn(idev, "interrupt %d not available (%d): no PMU\n",
			 inst->irq, rc);
		return 0;
	}
	rc = devm_add_action_or_reset(dev, cxl_hmu_perf_free_irq, info);
	if (rc)
		return rc;
	info->pmu = (struct pmu) {
		.name = pmu_name,
		.parent = idev,
		.module = THIS_MODULE,
		.capabilities = PERF_PMU_CAP_EXCLUSIVE |
				PERF_PMU_CAP_NO_EXCLUDE,
		/* The device is counted: no task and no cgroup has a share */
		.task_ctx_nr = perf_invalid_context,
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
	return devm_add_action_or_reset(dev, cxl_hmu_perf_unregister, info);
}

static int cxl_hmu_online_cpu(unsigned int cpu, struct hlist_node *node)
{
	struct cxl_hmu_info *info =
		hlist_entry_safe(node, struct cxl_hmu_info, node);

	if (info->on_cpu != -1)
		return 0;

	WRITE_ONCE(info->on_cpu, cpu);

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

	/* Opens fail until the events and the vector have a new home */
	WRITE_ONCE(info->on_cpu, -1);
	target = cpumask_any_but(cpu_online_mask, cpu);
	if (target >= nr_cpu_ids) {
		dev_err(info->pmu.dev, "Unable to find a suitable CPU\n");
		return 0;
	}

	/*
	 * The one event the instance can have, if it is open, is stopped
	 * here and started again there. It keeps its claim throughout.
	 */
	perf_pmu_migrate_context(&info->pmu, cpu, target);
	WRITE_ONCE(info->on_cpu, target);
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
