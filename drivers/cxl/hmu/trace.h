/* SPDX-License-Identifier: GPL-2.0-only */
/* CXL Hot-range Monitoring Unit tracepoints */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM cxl_hmu

#if !defined(_CXL_HMU_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _CXL_HMU_TRACE_H

#include <linux/tracepoint.h>

#include "hmu.h"

/* One drained hotlist: how many entries it held and what the device reported */
TRACE_EVENT(cxl_hmu_hotlist,
	TP_PROTO(const struct cxl_hmu_inst *inst, unsigned int nr,
		 u8 counter_width, bool overflowed),
	TP_ARGS(inst, nr, counter_width, overflowed),
	TP_STRUCT__entry(
		__string(dev, dev_name(&inst->dev))
		__field(unsigned int, nr)
		__field(u8, counter_width)
		__field(bool, overflowed)
	),
	TP_fast_assign(
		__assign_str(dev);
		__entry->nr = nr;
		__entry->counter_width = counter_width;
		__entry->overflowed = overflowed;
	),
	TP_printk("%s nr=%u counter_width=%u overflowed=%d",
		  __get_str(dev), __entry->nr, __entry->counter_width,
		  __entry->overflowed)
);

/* One contiguous host-physical range of a hot unit */
TRACE_EVENT(cxl_hmu_hpa_range,
	TP_PROTO(const struct cxl_hmu_inst *inst, u64 unit_id, u64 hpa,
		 u64 len, u32 nr_accesses),
	TP_ARGS(inst, unit_id, hpa, len, nr_accesses),
	TP_STRUCT__entry(
		__string(dev, dev_name(&inst->dev))
		__field(u64, unit_id)
		__field(u64, hpa)
		__field(u64, len)
		__field(u32, nr_accesses)
	),
	TP_fast_assign(
		__assign_str(dev);
		__entry->unit_id = unit_id;
		__entry->hpa = hpa;
		__entry->len = len;
		__entry->nr_accesses = nr_accesses;
	),
	TP_printk("%s unit_id=%llu hpa=%#llx len=%llu nr_accesses=%u",
		  __get_str(dev), __entry->unit_id, __entry->hpa, __entry->len,
		  __entry->nr_accesses)
);

#if IS_ENABLED(CONFIG_CXL_HMU_DAMON)
/*
 * One fetch by DAMON from the instance as a hot range source: the entries
 * taken off the hotlist, the units of those handed over whole, and the
 * entries that had to be dropped, those of a unit with no range to hand
 * over included. @err is what ended it: 0 if no entry was refused, else
 * why DAMON took no more (-ENOSPC is the scheme's quota) or why a unit
 * could not be walked.
 */
TRACE_EVENT(cxl_hmu_fetch,
	TP_PROTO(const struct cxl_hmu_inst *inst, unsigned int drained,
		 unsigned int pushed, unsigned int dropped, bool lost, int err),
	TP_ARGS(inst, drained, pushed, dropped, lost, err),
	TP_STRUCT__entry(
		__string(dev, dev_name(&inst->dev))
		__field(unsigned int, drained)
		__field(unsigned int, pushed)
		__field(unsigned int, dropped)
		__field(bool, lost)
		__field(int, err)
	),
	TP_fast_assign(
		__assign_str(dev);
		__entry->drained = drained;
		__entry->pushed = pushed;
		__entry->dropped = dropped;
		__entry->lost = lost;
		__entry->err = err;
	),
	TP_printk("%s drained=%u pushed=%u dropped=%u lost=%d err=%d",
		  __get_str(dev), __entry->drained, __entry->pushed,
		  __entry->dropped, __entry->lost, __entry->err)
);
#endif /* CONFIG_CXL_HMU_DAMON */

#endif /* _CXL_HMU_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace
#include <trace/define_trace.h>
