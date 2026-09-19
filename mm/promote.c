// SPDX-License-Identifier: GPL-2.0
/*
 * Promotion for counting hotness sources.
 *
 * A counting source hands the kernel a bounded list of hot extents once per
 * epoch. This ranks that list by access density, admits as many pages as the
 * target's headroom and the promotion rate limit allow, and migrates them.
 * Nothing persists between epochs.
 */

#include <linux/promote.h>
#include <linux/memory-tiers.h>
#include <linux/mempolicy.h>
#include <linux/migrate.h>
#include <linux/mm.h>
#include <linux/memcontrol.h>
#include <linux/kthread.h>
#include <linux/math64.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/vmstat.h>

#include "internal.h"

/*
 * Cap any single extent at a fraction of the budget so one enormous region
 * cannot consume an entire epoch's promotion allowance.
 */
#define PROMOTE_EXTENT_BUDGET_SHIFT	2

/*
 * Fractional bits in a density. With whole accesses per page per second a
 * 2MB unit needs 512 accesses in a one second epoch to score at all, and
 * everything below that ties; the shipped threshold is 16.
 */
#define PROMOTE_DENSITY_SHIFT		16

/*
 * Orders on the whole coalescing key, not just the pfn: sort() is not stable
 * and the merge below only compares neighbours, so identical extents have to
 * land next to each other whatever else shares their pfn.
 */
static int promote_cmp_pfn(const void *a, const void *b)
{
	const struct promote_extent *x = a, *y = b;

	if (x->pfn != y->pfn)
		return x->pfn < y->pfn ? -1 : 1;
	if (x->nr_pages != y->nr_pages)
		return x->nr_pages < y->nr_pages ? -1 : 1;
	if (x->nr_counted != y->nr_counted)
		return x->nr_counted < y->nr_counted ? -1 : 1;
	return 0;
}

static int promote_cmp_density(const void *a, const void *b)
{
	const struct promote_extent *x = a, *y = b;

	if (x->density != y->density)
		return x->density > y->density ? -1 : 1;
	/* Stable within equal density: earlier arrival first. */
	return x->seq < y->seq ? -1 : x->seq > y->seq;
}

/*
 * Pages we are willing to promote into @nid this epoch. Bounded by the
 * distance to the promotion watermark so that a burst of hot extents cannot
 * push the target into reclaim.
 *
 * Three things this has to get right, because a node-wide sum of free pages
 * gets all of them wrong:
 *
 * - WMARK_PROMO is defined as WMARK_HIGH plus a margin, so it is the whole
 *   reserve on its own and must not be added to high_wmark_pages().
 * - Only zones the promotion allocation can reach count. It is a
 *   GFP_HIGHUSER_MOVABLE allocation, so lowmem_reserve[] guards the low
 *   zones against it; free pages below that guard can never be promoted
 *   into and must not be counted as headroom.
 * - The clamp is per zone. Summing first lets a comfortable ZONE_DMA32
 *   mask a full ZONE_NORMAL, which on a typical node 0 means this never
 *   reports zero however full the node actually is.
 */
static unsigned long promote_headroom(int nid)
{
	enum zone_type highest = gfp_zone(GFP_HIGHUSER_MOVABLE);
	pg_data_t *pgdat = NODE_DATA(nid);
	unsigned long headroom = 0;
	int z;

	for (z = 0; z <= highest; z++) {
		struct zone *zone = pgdat->node_zones + z;
		long free, reserve;

		if (!managed_zone(zone))
			continue;

		free = zone_page_state(zone, NR_FREE_PAGES);
		reserve = promo_wmark_pages(zone) + zone->lowmem_reserve[highest];
		if (free > reserve)
			headroom += free - reserve;
	}

	return headroom;
}


/* Nearest top-tier node to @src_nid, honouring @allowed. */
static int promote_target_node(int src_nid, const nodemask_t *allowed)
{
	int nid, best = NUMA_NO_NODE, best_dist = INT_MAX;

	for_each_node_state(nid, N_MEMORY) {
		int dist;

		if (!node_isset(nid, *allowed))
			continue;
		if (!node_is_toptier(nid))
			continue;
		dist = node_distance(src_nid, nid);
		if (dist < best_dist) {
			best_dist = dist;
			best = nid;
		}
	}
	return best;
}

static void promote_flush(struct list_head *list, int nid,
			  struct promote_stats *st)
{
	if (list_empty(list))
		return;

	/*
	 * Consumes the list: migrated folios are freed, the rest are put
	 * back internally. It returns 0 or -EAGAIN, never a count, so the
	 * promoted total is read from PGPROMOTE_SUCCESS rather than here.
	 */
	promote_misplaced_memcg_folios(list, nid);
	INIT_LIST_HEAD(list);
}

/*
 * Admit one folio at @pfn. On success returns the folio with the isolation
 * reference held and it is the caller's to place on a batch list; @nr_out is
 * the folio's page count, which is what the budget is charged.
 *
 * @next_out is always set to the pfn the walk continues from. A pfn is a tail
 * page of a large folio far more often than not - a 4K unit lands on one 511
 * times in 512 - so the walk must leave off at the folio's end rather than at
 * @pfn plus its size, which would overshoot into whatever follows; and a folio
 * that is rejected must be stepped over whole rather than a tail page at a
 * time. Both need the folio's size, which is only trustworthy while a
 * reference is held, since a split has to freeze the refcount. So the
 * reference is taken before any test that could reject the folio. The
 * commonest large-folio rejection is a folio that an earlier extent of the
 * same pass already isolated - a unit smaller than the folio has siblings -
 * and that is now one call rather than one per tail page. A pfn whose page
 * is free, which is what a unit looks like once its folio has been promoted
 * away, fails the reference and is still stepped a page at a time.
 */
static struct folio *promote_one_folio(unsigned long pfn, int *nid_out,
				       struct mem_cgroup **memcg_out,
				       unsigned long *nr_out,
				       unsigned long *next_out)
{
	struct mem_cgroup *memcg;
	nodemask_t allowed;
	struct folio *folio;
	struct page *page;
	int src_nid, nid;

	*next_out = pfn + 1;

	page = pfn_to_online_page(pfn);
	if (!page || is_zone_device_page(page))
		return NULL;

	folio = page_folio(page);
	if (!folio_try_get(folio))
		return NULL;
	if (unlikely(page_folio(page) != folio))
		goto put;

	*next_out = folio_pfn(folio) + folio_nr_pages(folio);

	src_nid = folio_nid(folio);
	if (node_is_toptier(src_nid))
		goto put;
	if (!folio_test_lru(folio))
		goto put;
	/*
	 * Deliberately not folio_is_promo_candidate(): that gates on
	 * kernel.numa_balancing having the tiering bit set, which ties a
	 * device-driven promoter to the NUMA balancing scanner being
	 * enabled. The lower-tier test above is the part that matters here;
	 * enablement belongs to this facility's own caller.
	 */

	/*
	 * Resolve cpuset against the FOLIO's cgroup, not current: this runs
	 * from a worker with no relation to the accessing task. Demotion
	 * already does this; fault-path promotion gets it from the vma.
	 */
	allowed = node_states[N_MEMORY];
	rcu_read_lock();
	memcg = folio_memcg(folio);
	if (memcg)
		mem_cgroup_node_filter_allowed(memcg, &allowed);
	rcu_read_unlock();

	nid = promote_target_node(src_nid, &allowed);
	if (nid == NUMA_NO_NODE || nid == src_nid)
		goto put;

	if (migrate_misplaced_folio_prepare(folio, NULL, nid))
		goto put;

	/*
	 * prepare() took the isolation reference. Drop the one we took to
	 * look at the folio; the isolation reference keeps it alive and is
	 * consumed by the migration.
	 */
	*nr_out = folio_nr_pages(folio);
	folio_put(folio);
	*nid_out = nid;
	*memcg_out = memcg;
	return folio;
put:
	folio_put(folio);
	return NULL;
}

static int promote_pass(struct promote_batch *b, struct promote_stats *st,
			int src_nid);

/*
 * A counting source reports from a device interrupt. Promotion migrates
 * pages, which is far too expensive to do there: the reporting thread would
 * stop draining the device's ring while it copies memory. So reports are
 * queued and a per-node worker does the promotion. That separation of
 * observing from migrating is what any non-faulting source needs; what is
 * absent here is the per-page record, frequency accumulation and decay
 * window that only a sampling source requires.
 */
struct promote_queue {
	/* protects the ring and its indices against concurrent reporters */
	spinlock_t		lock;
	struct promote_extent	*ring;		/* bounded; oldest dropped */
	unsigned int		size, head, nr;
	unsigned int		epoch_ms;	/* of the most recent batch */
	unsigned long		nr_dropped;
	int			nid;		/* lower tier this worker drains */
	struct task_struct	*worker;
	wait_queue_head_t	wait;
	struct promote_extent	*scratch;	/* worker's drain buffer */
};

#define PROMOTE_QUEUE_ENTRIES	4096

static struct promote_queue *promote_queues[MAX_NUMNODES];
static DEFINE_MUTEX(promote_queue_lock);

static int promote_worker(void *data)
{
	struct promote_queue *q = data;

	while (!kthread_should_stop()) {
		struct promote_batch b = {};
		struct promote_stats st = {};
		unsigned int n = 0;

		wait_event_interruptible(q->wait,
					 READ_ONCE(q->nr) || kthread_should_stop());
		if (kthread_should_stop())
			break;

		spin_lock(&q->lock);
		while (q->nr && n < q->size) {
			q->scratch[n++] = q->ring[q->head];
			q->head = (q->head + 1) % q->size;
			q->nr--;
		}
		b.epoch_ms = q->epoch_ms ?: 1;
		spin_unlock(&q->lock);

		if (!n)
			continue;

		b.extents = q->scratch;
		b.nr_extents = n;
		b.ranked = true;
		promote_pass(&b, &st, q->nid);
		cond_resched();
	}
	return 0;
}

/* Lazily create the worker for @nid. */
static struct promote_queue *promote_queue_get(int nid)
{
	struct promote_queue *q = READ_ONCE(promote_queues[nid]);

	if (q)
		return q;

	guard(mutex)(&promote_queue_lock);
	q = promote_queues[nid];
	if (q)
		return q;

	q = kzalloc_node(sizeof(*q), GFP_KERNEL, nid);
	if (!q)
		return NULL;
	q->nid = nid;
	q->size = PROMOTE_QUEUE_ENTRIES;
	q->ring = kvcalloc_node(q->size, sizeof(*q->ring), GFP_KERNEL, nid);
	q->scratch = kvcalloc_node(q->size, sizeof(*q->scratch), GFP_KERNEL, nid);
	if (!q->ring || !q->scratch)
		goto err;
	spin_lock_init(&q->lock);
	init_waitqueue_head(&q->wait);

	q->worker = kthread_create_on_node(promote_worker, q, nid,
					   "kpromoted%d", nid);
	if (IS_ERR(q->worker))
		goto err;
	wake_up_process(q->worker);

	WRITE_ONCE(promote_queues[nid], q);
	return q;
err:
	kvfree(q->scratch);
	kvfree(q->ring);
	kfree(q);
	return NULL;
}

/**
 * promote_queue_extents - queue a batch for asynchronous promotion
 * @b: the batch; extents are copied, the caller keeps ownership
 *
 * Context: any sleepable context, including a threaded interrupt handler.
 * Return: number of extents queued, or negative on error.
 */
int promote_queue_extents(struct promote_batch *b)
{
	struct promote_queue *q;
	unsigned int i, queued = 0;
	int nid;

	if (!b || !b->extents || !b->epoch_ms || b->src >= PROMOTE_SRC_MAX)
		return -EINVAL;
	if (!b->nr_extents)
		return 0;

	/*
	 * The queue only decides which worker does the work; the worker
	 * resolves each folio's own target. So any lower-tier node among the
	 * extents will do - scan rather than trusting the first one, which
	 * may be untranslatable or already on the top tier.
	 */
	nid = NUMA_NO_NODE;
	for (i = 0; i < b->nr_extents; i++) {
		int n;

		if (!b->extents[i].nr_pages)
			continue;
		n = pfn_to_nid(b->extents[i].pfn);
		if (n != NUMA_NO_NODE && !node_is_toptier(n)) {
			nid = n;
			break;
		}
	}
	if (nid == NUMA_NO_NODE)
		return 0;

	q = promote_queue_get(nid);
	if (!q)
		return -ENOMEM;

	spin_lock(&q->lock);
	q->epoch_ms = b->epoch_ms;
	for (i = 0; i < b->nr_extents; i++) {
		unsigned int tail;

		if (q->nr == q->size) {
			/*
			 * Full: drop the oldest. A newer report is better
			 * evidence than a stale one, and the worker ranks
			 * whatever it finds.
			 */
			q->head = (q->head + 1) % q->size;
			q->nr--;
			q->nr_dropped++;
		}
		tail = (q->head + q->nr) % q->size;
		q->ring[tail] = b->extents[i];
		q->nr++;
		queued++;
	}
	spin_unlock(&q->lock);

	wake_up_interruptible(&q->wait);
	return queued;
}

/*
 * One promotion pass over a ranked batch. Runs on the per-node worker, never
 * on the reporting device's thread.
 */
static int promote_pass(struct promote_batch *b, struct promote_stats *st,
			int src_nid)
{
	struct mem_cgroup *cur_memcg = NULL;
	struct promote_stats local = {};
	int cur_nid = NUMA_NO_NODE;
	unsigned long budget = 0;
	bool exhausted = false;
	LIST_HEAD(folios);
	unsigned int i, j;
	int seed_nid;

	might_sleep();

	if (!b || !b->extents || !b->epoch_ms || b->src >= PROMOTE_SRC_MAX)
		return -EINVAL;
	if (!st)
		st = &local;

	/*
	 * The per-extent cap is a fraction of the budget, so a budget has to
	 * exist before the first extent is walked or that extent - the densest
	 * one, after the sort below - would be the one extent never capped.
	 * Seed it from the node this batch will mostly promote into: the
	 * nearest top-tier node to the one that reported. A folio whose cgroup
	 * narrows the allowed set can still land elsewhere, and the budget is
	 * recomputed for real on every turnover; this only has to be good
	 * enough to divide by.
	 */
	seed_nid = promote_target_node(src_nid, &node_states[N_MEMORY]);
	if (seed_nid != NUMA_NO_NODE)
		budget = promote_headroom(seed_nid);

	st->nr_extents = b->nr_extents;

	/*
	 * Coalesce repeats of the same extent, summing their weights.
	 *
	 * This is what makes always-on reporting usable. In epoch mode a unit
	 * appears at most once per epoch and its counter carries the
	 * magnitude, so there is little to merge. Under always-on the counter
	 * is defined as invalid and the driver substitutes the programmed
	 * threshold, which would make every extent score alike - but a unit
	 * that keeps crossing the threshold is reported again each time, so
	 * the repeat count is the magnitude. Summing recovers it.
	 */
	for (i = 0; i < b->nr_extents; i++)
		b->extents[i].seq = i;
	sort(b->extents, b->nr_extents, sizeof(*b->extents),
	     promote_cmp_pfn, NULL);
	for (i = 0, j = 0; i < b->nr_extents; i++) {
		if (j && b->extents[j - 1].pfn == b->extents[i].pfn &&
		    b->extents[j - 1].nr_pages == b->extents[i].nr_pages &&
		    b->extents[j - 1].nr_counted == b->extents[i].nr_counted) {
			b->extents[j - 1].weight += b->extents[i].weight;
			/* The merged extent arrived when its first report did. */
			b->extents[j - 1].seq = min(b->extents[j - 1].seq,
						    b->extents[i].seq);
			continue;
		}
		b->extents[j++] = b->extents[i];
	}
	b->nr_extents = j;

	/*
	 * Rank by density: accesses per page per second, over the pages the
	 * count was taken on. A run that is a fragment of a larger counted
	 * region inherits that region's density rather than claiming the
	 * whole count for itself.
	 */
	for (i = 0; i < b->nr_extents; i++) {
		struct promote_extent *e = &b->extents[i];
		unsigned long counted;

		if (!e->nr_pages) {
			e->density = 0;
			continue;
		}
		counted = max(e->nr_counted, e->nr_pages);
		e->density = mul_u64_u64_div_u64(e->weight,
				(u64)MSEC_PER_SEC << PROMOTE_DENSITY_SHIFT,
				(u64)counted * b->epoch_ms);
		st->nr_pages_considered += e->nr_pages;
	}
	sort(b->extents, b->nr_extents, sizeof(*b->extents),
	     promote_cmp_density, NULL);

	for (i = 0; i < b->nr_extents && !exhausted; i++) {
		struct promote_extent *e = &b->extents[i];
		unsigned long pfn, end, cap;

		if (!e->nr_pages)
			continue;

		/*
		 * Cap one extent at a fraction of the remaining budget so a
		 * single enormous region cannot spend the whole epoch's
		 * allowance. Extents are ranked, so anything dropped here is
		 * colder than what has already been admitted. A budget of zero
		 * caps at one page rather than waiving the cap: no headroom is
		 * a reason to admit less, not to admit everything.
		 */
		cap = max_t(unsigned long, 1,
			    budget >> PROMOTE_EXTENT_BUDGET_SHIFT);
		end = e->pfn + min(e->nr_pages, cap);
		if (end - e->pfn < e->nr_pages)
			st->nr_pages_dropped_budget += e->nr_pages - (end - e->pfn);
		st->nr_pages_budgeted += end - e->pfn;
		st->cut_density = e->density;

		for (pfn = e->pfn; pfn < end; ) {
			struct mem_cgroup *memcg = NULL;
			int nid = NUMA_NO_NODE;
			unsigned long nr = 0, next;
			struct folio *folio;

			/*
			 * An extent can span 2GB and every page in it may be
			 * ineligible, so yield here rather than only on the
			 * admitting path.
			 */
			cond_resched();

			folio = promote_one_folio(pfn, &nid, &memcg, &nr, &next);
			if (!folio) {
				pfn = next;
				continue;
			}

			/* Batch by (target, memcg); flush when either turns over. */
			if (cur_nid == NUMA_NO_NODE ||
			    cur_nid != nid || cur_memcg != memcg) {
				promote_flush(&folios, cur_nid, st);
				cur_nid = nid;
				cur_memcg = memcg;
				budget = promote_headroom(nid);
			}

			if (!budget) {
				/* No headroom: put this one back and stop. */
				folio_putback_lru(folio);
				exhausted = true;
				st->nr_pages_dropped_budget += end - pfn;
				break;
			}

			list_add(&folio->lru, &folios);
			st->nr_pages_eligible += nr;
			budget -= min(budget, nr);
			pfn = next;
		}
	}

	promote_flush(&folios, cur_nid, st);
	return 0;
}
