// SPDX-License-Identifier: GPL-2.0
/*
 * Asynchronous execution of NUMA balancing migrations.
 *
 * NUMA balancing decides in the hint fault which folio should move where.
 * The fault stops there: it records the folio and its target and returns,
 * and a per-node thread, numabd, isolates and migrates later, in batches.
 * The decision - the rate limit, the adaptive threshold, the locality
 * checks - is untouched; only the execution leaves the faulting task.
 */
#include <linux/cgroup.h>
#include <linux/cpuset.h>
#include <linux/kthread.h>
#include <linux/memcontrol.h>
#include <linux/memory_hotplug.h>
#include <linux/migrate.h>
#include <linux/mm.h>
#include <linux/node.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

#include "internal.h"

#define NUMAB_RING_ENTRIES	4096
#define NUMAB_BATCH		512

/* What a hint fault hands over. Everything else is re-derived. */
struct numab_entry {
	unsigned long		pfn;
	void			*mapping;	/* folio->mapping at fault time */
	struct cgroup		*cgrp;	/* faulting task's, referenced; NULL = root */
	unsigned int		nr_pages;
	int			src_nid;
};

struct numab_queue {
	spinlock_t		lock;	/* ring, head, nr, nr_full */
	struct numab_entry	*ring;
	unsigned int		head, nr;
	unsigned long		nr_full;	/* enqueues refused */
	int			nid;
	struct task_struct	*thread;
	wait_queue_head_t	wait;
	struct numab_entry	*scratch;	/* thread's copy of a drain */
};

static struct numab_queue *numab_queues[MAX_NUMNODES];
static DEFINE_MUTEX(numab_lock);

/*
 * Attribution. The migration's CPU time belongs to the faulting task's
 * cgroup, as it does on the synchronous path. A primitive for a kernel
 * thread to say which cgroup it is working for is being proposed
 * (set_active_cgroup()); these are where it plugs in. Until then the
 * thread's time is root overhead, as kswapd's and kcompactd's is.
 */
static inline struct cgroup *numab_charge_begin(struct cgroup *cgrp)
{
	return NULL;
}

static inline void numab_charge_end(struct cgroup *old)
{
}

#ifdef CONFIG_CGROUPS
/* A reference on the faulting task's cgroup, or NULL for the root. */
static struct cgroup *numab_cgroup_get(void)
{
	struct cgroup *cgrp;

	rcu_read_lock();
	cgrp = task_dfl_cgroup(current);
	if (!cgroup_parent(cgrp) || !cgroup_tryget(cgrp))
		cgrp = NULL;
	rcu_read_unlock();
	return cgrp;
}

static void numab_cgroup_put(struct cgroup *cgrp)
{
	if (cgrp)
		cgroup_put(cgrp);
}
#else
static inline struct cgroup *numab_cgroup_get(void)
{
	return NULL;
}

static inline void numab_cgroup_put(struct cgroup *cgrp)
{
}
#endif

/*
 * Record a migration decision. Called under the PTL by the hint fault, so
 * it must not sleep. Everything the migration would refuse that can be
 * known now is refused now, in the faulting task's context: the checks
 * that need the VMA, the destination's watermarks, and the task's cpuset,
 * which the thread's own allocation would not see. Returns false if
 * nothing was queued, which the fault reports as a failed migration so
 * that the scan rate backs off.
 */
bool numab_queue(struct folio *folio, struct vm_area_struct *vma, int target_nid)
{
	struct numab_queue *q = smp_load_acquire(&numab_queues[target_nid]);
	struct numab_entry *e;
	struct cgroup *cgrp;
	bool wake;

	if (!q)
		return false;
	if (!node_isset(target_nid, cpuset_current_mems_allowed))
		return false;
	if (migrate_misplaced_folio_check(folio, vma, target_nid))
		return false;

	cgrp = numab_cgroup_get();

	spin_lock(&q->lock);
	if (q->nr == NUMAB_RING_ENTRIES) {
		q->nr_full++;
		spin_unlock(&q->lock);
		numab_cgroup_put(cgrp);
		return false;
	}
	e = &q->ring[(q->head + q->nr) % NUMAB_RING_ENTRIES];
	e->pfn = folio_pfn(folio);
	e->mapping = READ_ONCE(folio->mapping);
	e->nr_pages = folio_nr_pages(folio);
	e->src_nid = folio_nid(folio);
	e->cgrp = cgrp;
	wake = !q->nr++;
	spin_unlock(&q->lock);

	/* The thread drains everything before it sleeps; wake it on 0 -> 1. */
	if (wake)
		wake_up_interruptible(&q->wait);
	return true;
}

/*
 * Turn an entry back into an isolated folio, or NULL if the world moved
 * on: the page is gone, the folio is not the one the fault saw, it left
 * the node, or it is not on the LRU. The mapping check catches a pfn freed
 * and reused by another owner in between; a folio reused within the same
 * anon_vma or file is indistinguishable, and harmless to move. On success
 * the isolation reference is held, as after
 * migrate_misplaced_folio_prepare().
 */
static struct folio *numab_get_folio(struct numab_entry *e, int target_nid)
{
	struct page *page = pfn_to_online_page(e->pfn);
	struct folio *folio;

	if (!page)
		return NULL;
	folio = page_folio(page);
	if (!folio_try_get(folio))
		return NULL;
	if (unlikely(page_folio(page) != folio) ||
	    folio_pfn(folio) != e->pfn ||
	    folio_nr_pages(folio) != e->nr_pages ||
	    READ_ONCE(folio->mapping) != e->mapping ||
	    folio_nid(folio) != e->src_nid ||
	    !folio_test_lru(folio))
		goto put;
	if (migrate_misplaced_folio_prepare(folio, NULL, target_nid))
		goto put;
	folio_put(folio);
	return folio;
put:
	folio_put(folio);
	return NULL;
}

/* Migrate a batch, then drop the batch's cgroup reference. */
static void numab_flush(struct list_head *batch, int nid, struct cgroup *cgrp)
{
	struct cgroup *old;

	old = numab_charge_begin(cgrp);
	promote_misplaced_memcg_folios(batch, nid);
	numab_charge_end(old);
	numab_cgroup_put(cgrp);
}

/*
 * Batches share a source node and a memcg, which the migration helper
 * requires, and a cgroup, which attribution does. The batch owns one
 * cgroup reference, taken over from its first entry.
 */
static void numab_run(struct numab_queue *q, struct numab_entry *ents,
		      unsigned int n)
{
	struct mem_cgroup *cur_memcg = NULL;
	struct cgroup *cur_cgrp = NULL;
	int cur_src = NUMA_NO_NODE;
	unsigned int i, nr_batch = 0;
	LIST_HEAD(batch);

	for (i = 0; i < n; i++) {
		struct numab_entry *e = &ents[i];
		struct mem_cgroup *memcg;
		struct folio *folio;

		cond_resched();
		folio = numab_get_folio(e, q->nid);
		if (!folio) {
			numab_cgroup_put(e->cgrp);
			continue;
		}
		/* Compared, never dereferenced: RCU only keeps lockdep happy. */
		rcu_read_lock();
		memcg = folio_memcg(folio);
		rcu_read_unlock();

		if (nr_batch && (nr_batch == NUMAB_BATCH ||
				 e->src_nid != cur_src ||
				 memcg != cur_memcg ||
				 e->cgrp != cur_cgrp)) {
			numab_flush(&batch, q->nid, cur_cgrp);
			nr_batch = 0;
		}
		if (!nr_batch) {
			cur_src = e->src_nid;
			cur_memcg = memcg;
			cur_cgrp = e->cgrp;
		} else {
			numab_cgroup_put(e->cgrp);
		}
		list_add(&folio->lru, &batch);
		nr_batch++;
	}
	if (nr_batch)
		numab_flush(&batch, q->nid, cur_cgrp);
}

static int numabd(void *data)
{
	struct numab_queue *q = data;

	while (!kthread_should_stop()) {
		unsigned int n = 0;

		wait_event_interruptible(q->wait, READ_ONCE(q->nr) ||
					 kthread_should_stop());
		if (kthread_should_stop())
			break;

		spin_lock(&q->lock);
		while (q->nr) {
			q->scratch[n++] = q->ring[q->head];
			q->head = (q->head + 1) % NUMAB_RING_ENTRIES;
			q->nr--;
		}
		spin_unlock(&q->lock);

		numab_run(q, q->scratch, n);
	}
	return 0;
}

/* Process context; under numab_lock. */
static int numab_queue_create(int nid)
{
	struct numab_queue *q;
	int err = -ENOMEM;

	lockdep_assert_held(&numab_lock);
	if (numab_queues[nid])
		return 0;

	q = kzalloc_node(sizeof(*q), GFP_KERNEL, nid);
	if (!q)
		return -ENOMEM;
	q->nid = nid;
	q->ring = kvcalloc_node(NUMAB_RING_ENTRIES, sizeof(*q->ring),
				GFP_KERNEL, nid);
	q->scratch = kvcalloc_node(NUMAB_RING_ENTRIES, sizeof(*q->scratch),
				   GFP_KERNEL, nid);
	if (!q->ring || !q->scratch)
		goto err;
	spin_lock_init(&q->lock);
	init_waitqueue_head(&q->wait);
	q->thread = kthread_create_on_node(numabd, q, nid, "numabd%d", nid);
	if (IS_ERR(q->thread)) {
		err = PTR_ERR(q->thread);
		goto err;
	}
	wake_up_process(q->thread);
	/* Pairs with smp_load_acquire() in numab_queue(). */
	smp_store_release(&numab_queues[nid], q);
	return 0;
err:
	kvfree(q->scratch);
	kvfree(q->ring);
	kfree(q);
	return err;
}

static int numab_node_callback(struct notifier_block *self,
			       unsigned long action, void *arg)
{
	struct node_notify *nn = arg;

	if (action == NODE_ADDED_FIRST_MEMORY) {
		guard(mutex)(&numab_lock);
		if (numab_queue_create(nn->nid))
			pr_warn("numabd: no thread for node %d\n", nn->nid);
	}
	return NOTIFY_OK;
}

/*
 * One thread per node with memory, as kswapd. A node without one, because
 * this failed or it has no memory, refuses every enqueue and the fault
 * reports the migration as failed, which is what the allocation would
 * have done.
 */
static int __init numab_init(void)
{
	int nid;

	hotplug_node_notifier(numab_node_callback, 0);
	guard(mutex)(&numab_lock);
	for_each_node_state(nid, N_MEMORY)
		if (numab_queue_create(nid))
			pr_warn("numabd: no thread for node %d\n", nid);
	return 0;
}
subsys_initcall(numab_init);
