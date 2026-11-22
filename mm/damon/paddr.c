// SPDX-License-Identifier: GPL-2.0
/*
 * DAMON Code for The Physical Address Space
 */

#define pr_fmt(fmt) "damon-pa: " fmt

#include <linux/mmu_notifier.h>
#include <linux/page_idle.h>
#include <linux/pagemap.h>
#include <linux/rmap.h>
#include <linux/swap.h>
#include <linux/memory-tiers.h>
#include <linux/migrate.h>
#include <linux/mm_inline.h>

#include "../internal.h"
#include "ops-common.h"

static phys_addr_t damon_pa_phys_addr(
		unsigned long addr, unsigned long addr_unit)
{
	return (phys_addr_t)addr * addr_unit;
}

static unsigned long damon_pa_core_addr(
		phys_addr_t pa, unsigned long addr_unit)
{
	/*
	 * Use div_u64() for avoiding linking errors related with __udivdi3,
	 * __aeabi_uldivmod, or similar problems.  This should also improve the
	 * performance optimization (read div_u64() comment for the detail).
	 */
	if (sizeof(pa) == 8 && sizeof(addr_unit) == 4)
		return div_u64(pa, addr_unit);
	return pa / addr_unit;
}

static void damon_pa_mkold(phys_addr_t paddr)
{
	struct folio *folio = damon_get_monitor_folio(PHYS_PFN(paddr));

	if (!folio)
		return;

	damon_folio_mkold(folio);
	folio_put(folio);
}

static void __damon_pa_prepare_access_check(struct damon_region *r,
		struct damon_ctx *ctx)
{
	r->sampling_addr = damon_rand(ctx, r->ar.start, r->ar.end);

	damon_pa_mkold(damon_pa_phys_addr(r->sampling_addr, ctx->addr_unit));
}

static void damon_pa_prepare_access_checks(struct damon_ctx *ctx)
{
	struct damon_target *t;
	struct damon_region *r;

	damon_for_each_target(t, ctx) {
		damon_for_each_region(r, t)
			__damon_pa_prepare_access_check(r, ctx);
	}
}

static bool damon_pa_young(phys_addr_t paddr)
{
	struct folio *folio = damon_get_monitor_folio(PHYS_PFN(paddr));
	bool accessed;

	if (!folio)
		return false;

	accessed = damon_folio_young(folio);
	folio_put(folio);
	return accessed;
}

static void __damon_pa_check_access(struct damon_region *r,
		unsigned long addr_unit)
{
	bool accessed;
	phys_addr_t sampling_addr = damon_pa_phys_addr(
			r->sampling_addr, addr_unit);

	accessed = damon_pa_young(sampling_addr);
	damon_update_region_access_rate(r, accessed);
}

static unsigned int damon_pa_check_accesses(struct damon_ctx *ctx)
{
	struct damon_target *t;
	struct damon_region *r;
	unsigned int max_nr_accesses = 0;

	damon_for_each_target(t, ctx) {
		damon_for_each_region(r, t) {
			__damon_pa_check_access(r, ctx->addr_unit);
			max_nr_accesses = max(r->nr_accesses, max_nr_accesses);
		}
	}

	return max_nr_accesses;
}

static void damon_pa_prep_probes_region(struct damon_region *r,
		struct damon_probe *probe, struct damon_ctx *ctx)
{
	struct damon_prep *p;

	damon_for_each_prep(p, probe) {
		switch (p->action) {
		case DAMON_PREP_SET_PGIDLE:
			damon_pa_mkold(damon_pa_phys_addr(r->sampling_addr,
						ctx->addr_unit));
			break;
		default:
			break;
		}
	}
}

static void damon_pa_prep_probes(struct damon_ctx *ctx, bool set_samples)
{
	struct damon_target *t;
	struct damon_region *r;
	struct damon_probe *p;

	damon_for_each_target(t, ctx) {
		damon_for_each_region(r, t) {
			if (set_samples)
				r->sampling_addr = damon_rand(ctx, r->ar.start,
						r->ar.end);
			damon_for_each_probe(p, ctx)
				damon_pa_prep_probes_region(r, p, ctx);
		}
	}
}

static bool damon_pa_filter_match(struct damon_filter *filter,
		struct folio *folio)
{
	bool matched = false;

	switch (filter->type) {
	case DAMON_FILTER_TYPE_PGIDLE_UNSET:
		if (!folio)
			matched = false;
		else
			matched = damon_folio_young(folio);
		break;
	case DAMON_FILTER_TYPE_PGIDLE_SET:
		if (!folio)
			matched = false;
		else
			matched = damon_folio_young(folio) == false;
		break;
	default:
		return damon_ops_filter_match(filter, folio);
	}
	return matched == filter->matching;
}

static bool damon_pa_filter_pass(struct folio *folio, struct damon_probe *p)
{
	struct damon_filter *f;
	bool pass = true;

	damon_for_each_filter(f, p) {
		if (damon_pa_filter_match(f, folio)) {
			pass = f->allow;
			break;
		}
		pass = !f->allow;
	}
	return pass;
}

static unsigned int damon_pa_apply_probes(struct damon_ctx *ctx,
		bool set_samples, bool return_max_wsum)
{
	struct damon_target *t;
	struct damon_region *r;
	struct damon_probe *p;
	unsigned int max_wsum = 0;

	damon_for_each_target(t, ctx) {
		damon_for_each_region(r, t) {
			int i = 0;
			phys_addr_t pa;
			struct folio *folio;

			if (set_samples)
				r->sampling_addr = damon_rand(ctx, r->ar.start,
						r->ar.end);
			pa = damon_pa_phys_addr(r->sampling_addr,
					ctx->addr_unit);
			folio = damon_get_folio(PHYS_PFN(pa));
			damon_for_each_probe(p, ctx) {
				if (damon_pa_filter_pass(folio, p))
					r->probe_hits[i]++;
				i++;
			}
			if (folio)
				folio_put(folio);
			if (return_max_wsum)
				max_wsum = max(damon_probe_hits_wsum(r, false,
							false, ctx), max_wsum);
		}
	}
	return max_wsum;
}

/*
 * damos_pa_filter_out - Return true if the page should be filtered out.
 */
static bool damos_pa_filter_out(struct damos *scheme, struct folio *folio)
{
	struct damos_filter *filter;

	if (scheme->core_filters_allowed)
		return false;

	damos_for_each_ops_filter(filter, scheme) {
		if (damos_folio_filter_match(filter, folio))
			return !filter->allow;
	}
	return scheme->ops_filters_default_reject;
}

static bool damon_pa_invalid_damos_folio(struct folio *folio, struct damos *s)
{
	if (!folio)
		return true;
	if (folio == s->last_applied) {
		folio_put(folio);
		return true;
	}
	return false;
}

static unsigned long damon_pa_pageout(struct damon_region *r,
		unsigned long addr_unit, struct damos *s,
		unsigned long *sz_filter_passed)
{
	phys_addr_t addr, applied;
	LIST_HEAD(folio_list);
	bool install_young_filter = true;
	struct damos_filter *filter;
	struct folio *folio = NULL;

	/* check access in page level again by default */
	damos_for_each_ops_filter(filter, s) {
		if (filter->type == DAMOS_FILTER_TYPE_YOUNG) {
			install_young_filter = false;
			break;
		}
	}
	if (install_young_filter) {
		filter = damos_new_filter(
				DAMOS_FILTER_TYPE_YOUNG, true, false);
		if (!filter)
			return 0;
		damos_add_filter(s, filter);
	}

	addr = damon_pa_phys_addr(r->ar.start, addr_unit);
	while (addr < damon_pa_phys_addr(r->ar.end, addr_unit)) {
		folio = damon_get_folio(PHYS_PFN(addr));
		if (damon_pa_invalid_damos_folio(folio, s)) {
			addr += PAGE_SIZE;
			continue;
		}

		if (damos_pa_filter_out(s, folio))
			goto put_folio;
		else
			*sz_filter_passed += folio_size(folio) / addr_unit;

		folio_clear_referenced(folio);
		folio_test_clear_young(folio);
		if (!folio_isolate_lru(folio))
			goto put_folio;
		if (folio_test_unevictable(folio))
			folio_putback_lru(folio);
		else
			list_add(&folio->lru, &folio_list);
put_folio:
		addr = PFN_PHYS(folio_pfn(folio)) + folio_size(folio);
		folio_put(folio);
	}
	if (install_young_filter)
		damos_destroy_filter(filter);
	applied = reclaim_pages(&folio_list);
	cond_resched();
	s->last_applied = folio;
	return damon_pa_core_addr(applied * PAGE_SIZE, addr_unit);
}

static inline unsigned long damon_pa_de_activate(
		struct damon_region *r, unsigned long addr_unit,
		struct damos *s, bool activate,
		unsigned long *sz_filter_passed)
{
	phys_addr_t addr, applied = 0;
	struct folio *folio = NULL;

	addr = damon_pa_phys_addr(r->ar.start, addr_unit);
	while (addr < damon_pa_phys_addr(r->ar.end, addr_unit)) {
		folio = damon_get_folio(PHYS_PFN(addr));
		if (damon_pa_invalid_damos_folio(folio, s)) {
			addr += PAGE_SIZE;
			continue;
		}

		if (damos_pa_filter_out(s, folio))
			goto put_folio;
		else
			*sz_filter_passed += folio_size(folio) / addr_unit;

		if (activate)
			folio_activate(folio);
		else
			folio_deactivate(folio);
		applied += folio_nr_pages(folio);
put_folio:
		addr = PFN_PHYS(folio_pfn(folio)) + folio_size(folio);
		folio_put(folio);
	}
	s->last_applied = folio;
	return damon_pa_core_addr(applied * PAGE_SIZE, addr_unit);
}

static unsigned long damon_pa_activate_pages(struct damon_region *r,
		unsigned long addr_unit, struct damos *s,
		unsigned long *sz_filter_passed)
{
	return damon_pa_de_activate(r, addr_unit, s, true, sz_filter_passed);
}

static unsigned long damon_pa_deactivate_pages(struct damon_region *r,
		unsigned long addr_unit, struct damos *s,
		unsigned long *sz_filter_passed)
{
	return damon_pa_de_activate(r, addr_unit, s, false, sz_filter_passed);
}

static unsigned long damon_pa_migrate(struct damon_region *r,
		unsigned long addr_unit, struct damos *s,
		unsigned long *sz_filter_passed)
{
	phys_addr_t addr, applied;
	LIST_HEAD(folio_list);
	struct folio *folio = NULL;

	addr = damon_pa_phys_addr(r->ar.start, addr_unit);
	while (addr < damon_pa_phys_addr(r->ar.end, addr_unit)) {
		folio = damon_get_folio(PHYS_PFN(addr));
		if (damon_pa_invalid_damos_folio(folio, s)) {
			addr += PAGE_SIZE;
			continue;
		}

		if (damos_pa_filter_out(s, folio))
			goto put_folio;
		else
			*sz_filter_passed += folio_size(folio) / addr_unit;

		if (!folio_isolate_lru(folio))
			goto put_folio;
		node_stat_add_folio(folio, NR_ISOLATED_ANON +
				folio_is_file_lru(folio));
		list_add(&folio->lru, &folio_list);
put_folio:
		addr = PFN_PHYS(folio_pfn(folio)) + folio_size(folio);
		folio_put(folio);
	}
	applied = damon_migrate_pages(&folio_list, s->target_nid);
	cond_resched();
	s->last_applied = folio;
	return damon_pa_core_addr(applied * PAGE_SIZE, addr_unit);
}

static bool damon_pa_folio_action(enum damos_action action)
{
	return action == DAMOS_MIGRATE_HOT || action == DAMOS_MIGRATE_COLD;
}

/* Folios damos_add_paddr_range() isolates before it queues them */
#define DAMOS_PA_PUSH_BATCH	512

/*
 * The add_range() operation: queue the folios of a physical range for @s.
 *
 * Isolate every LRU folio in the range that the operations set filters of
 * @s let through and queue it with damos_add_folios(), in batches of
 * DAMOS_PA_PUSH_BATCH, so a range of any size can be queued and a refusal
 * puts back one batch.  A batch is pushed as soon as it holds what the quota
 * of @s leaves, so no more is isolated than the apply takes.  The filters
 * run before the isolation, as they do for a region, so a folio they reject
 * never leaves the LRU.  Folios that cannot be isolated are skipped.  If the
 * scheme does not take a batch, its folios are returned to the LRU here and
 * the rest of the range is not isolated, so the caller never holds folios.
 *
 * Returns 0, -EOPNOTSUPP if the action of @s cannot be applied to a list of
 * folios (only migrate_hot and migrate_cold can), or what damos_add_folios()
 * returned.
 */
static int damos_add_paddr_range(struct damon_ctx *ctx, struct damos *s,
				 phys_addr_t start, phys_addr_t size)
{
	phys_addr_t addr = ALIGN_DOWN(start, PAGE_SIZE), end = start + size;
	unsigned long nr = 0, sz = 0, room;
	LIST_HEAD(folios);
	int err;

	if (!damon_pa_folio_action(s->action))
		return -EOPNOTSUPP;
	/* Isolating is the expensive part; do not do it to be refused */
	err = damos_folios_wanted(ctx, s);
	if (err)
		return err;
	room = damos_folios_room(ctx, s);

	/* No region, so no core filter has spoken for these folios */
	s->core_filters_allowed = false;
	while (addr < end) {
		struct folio *folio = damon_get_folio(PHYS_PFN(addr));

		cond_resched();
		if (!folio) {
			addr += PAGE_SIZE;
			continue;
		}
		if (!damos_pa_filter_out(s, folio) && folio_isolate_lru(folio)) {
			node_stat_add_folio(folio, NR_ISOLATED_ANON +
					folio_is_file_lru(folio));
			list_add_tail(&folio->lru, &folios);
			nr++;
			sz += folio_size(folio);
		}
		/* The folio may start before @addr; step past the whole of it */
		addr = PFN_PHYS(folio_pfn(folio) + folio_nr_pages(folio));
		folio_put(folio);
		if (nr == DAMOS_PA_PUSH_BATCH || sz >= room) {
			err = damos_add_folios(ctx, s, &folios);
			if (err || addr >= end)
				break;
			nr = 0;
			sz = 0;
			err = damos_folios_wanted(ctx, s);
			if (err)
				break;
			room = damos_folios_room(ctx, s);
		}
	}
	if (!err && !list_empty(&folios))
		err = damos_add_folios(ctx, s, &folios);
	/* Empty unless the last push was refused */
	putback_movable_pages(&folios);
	return err;
}

/*
 * Migrate the pushed folios that fit the budget, counting in bytes so that
 * an address unit larger than a folio cannot zero the accounting, and
 * taking at least one folio when any budget is left so that a quota smaller
 * than a folio still makes progress, as a region does.  What is not taken
 * stays on @folios for the core to put back.  The operations set filters
 * are not run here: damos_add_paddr_range() ran them before it isolated,
 * and running a filter that clears what it tests, as the young one does,
 * a second time would reject what it let through.
 */
static unsigned long damon_pa_apply_folios(struct damon_ctx *ctx,
		struct damos *s, struct list_head *folios,
		unsigned long sz_budget, unsigned long *sz_tried,
		unsigned long *sz_filter_passed)
{
	unsigned long aunit = ctx->addr_unit;
	unsigned long budget = sz_budget > ULONG_MAX / aunit ? ULONG_MAX :
		sz_budget * aunit;
	unsigned long bytes = 0, applied = 0;
	struct folio *folio, *next;
	LIST_HEAD(migrate_list);

	if (!damon_pa_folio_action(s->action))
		return 0;

	list_for_each_entry_safe(folio, next, folios, lru) {
		if (bytes >= budget)
			break;
		bytes += folio_size(folio);
		list_move_tail(&folio->lru, &migrate_list);
	}
	if (!list_empty(&migrate_list))
		applied = damon_migrate_pages(&migrate_list, s->target_nid);
	*sz_tried = DIV_ROUND_UP(bytes, aunit);
	*sz_filter_passed += *sz_tried;
	return damon_pa_core_addr(applied * PAGE_SIZE, aunit);
}

static unsigned long damon_pa_stat(struct damon_region *r,
		unsigned long addr_unit, struct damos *s,
		unsigned long *sz_filter_passed)
{
	phys_addr_t addr;
	struct folio *folio = NULL;

	if (!damos_ops_has_filter(s))
		return 0;

	addr = damon_pa_phys_addr(r->ar.start, addr_unit);
	while (addr < damon_pa_phys_addr(r->ar.end, addr_unit)) {
		folio = damon_get_folio(PHYS_PFN(addr));
		if (damon_pa_invalid_damos_folio(folio, s)) {
			addr += PAGE_SIZE;
			continue;
		}

		if (!damos_pa_filter_out(s, folio))
			*sz_filter_passed += folio_size(folio) / addr_unit;
		addr = PFN_PHYS(folio_pfn(folio)) + folio_size(folio);
		folio_put(folio);
	}
	s->last_applied = folio;
	return 0;
}

static unsigned long damon_pa_apply_scheme(struct damon_ctx *ctx,
		struct damon_target *t, struct damon_region *r,
		struct damos *scheme, unsigned long *sz_filter_passed)
{
	unsigned long aunit = ctx->addr_unit;

	switch (scheme->action) {
	case DAMOS_PAGEOUT:
		return damon_pa_pageout(r, aunit, scheme, sz_filter_passed);
	case DAMOS_LRU_PRIO:
		return damon_pa_activate_pages(r, aunit, scheme,
				sz_filter_passed);
	case DAMOS_LRU_DEPRIO:
		return damon_pa_deactivate_pages(r, aunit, scheme,
				sz_filter_passed);
	case DAMOS_MIGRATE_HOT:
	case DAMOS_MIGRATE_COLD:
		return damon_pa_migrate(r, aunit, scheme, sz_filter_passed);
	case DAMOS_STAT:
		return damon_pa_stat(r, aunit, scheme, sz_filter_passed);
	default:
		/* DAMOS actions that not yet supported by 'paddr'. */
		break;
	}
	return 0;
}

static int damon_pa_scheme_score(struct damon_ctx *context,
		struct damon_region *r, struct damos *scheme)
{
	switch (scheme->action) {
	case DAMOS_PAGEOUT:
		return damon_cold_score(context, r, scheme);
	case DAMOS_LRU_PRIO:
		return damon_hot_score(context, r, scheme);
	case DAMOS_LRU_DEPRIO:
		return damon_cold_score(context, r, scheme);
	case DAMOS_MIGRATE_HOT:
		return damon_hot_score(context, r, scheme);
	case DAMOS_MIGRATE_COLD:
		return damon_cold_score(context, r, scheme);
	default:
		break;
	}

	return DAMOS_MAX_SCORE;
}

static int __init damon_pa_initcall(void)
{
	struct damon_operations ops = {
		.id = DAMON_OPS_PADDR,
		.init = NULL,
		.update = NULL,
		.prepare_access_checks = damon_pa_prepare_access_checks,
		.check_accesses = damon_pa_check_accesses,
		.prep_probes = damon_pa_prep_probes,
		.apply_probes = damon_pa_apply_probes,
		.target_valid = NULL,
		.apply_scheme = damon_pa_apply_scheme,
		.add_range = damos_add_paddr_range,
		.apply_folios = damon_pa_apply_folios,
		.get_scheme_score = damon_pa_scheme_score,
	};

	return damon_register_ops(&ops);
}

subsys_initcall(damon_pa_initcall);
