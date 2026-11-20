/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * CXL Hot-range Monitoring Unit: the interface for in-kernel clients.
 *
 * An instance has one holder at a time, first come, first served: a client
 * from its claim to its release, or a perf event. A client claims with the
 * configuration staged in the instance's sysfs files, and reads the hot
 * units the instance reports as host physical address ranges.
 */
#ifndef __CXL_CXL_HMU_H__
#define __CXL_CXL_HMU_H__
#include <linux/bits.h>
#include <linux/list.h>
#include <linux/types.h>

struct device;
struct cxl_hmu_inst;

/**
 * struct cxl_hmu_client - an in-kernel user of CHMU instances
 * @name: unique, and neither "perf" nor "none"; what an instance's
 *	"owner" attribute shows while the client holds it
 * @add: an instance is there, and the client may claim it from now on.
 *	Returning 0 binds the client to the instance and keeps *@data for
 *	@remove; an error leaves the instance to the other clients
 * @remove: the instance is going, or the client is: stop using the
 *	instance and release it if held. Not called for an instance @add
 *	refused
 * @max_unit_size: the largest unit, in bytes, the client claims an
 *	instance with; a claim with a larger staged unit size fails with
 *	-EINVAL before the device is touched. 0 for no limit
 *
 * @add and @remove are called with a lock held that cxl_hmu_register_client()
 * and cxl_hmu_unregister_client() take, and that nothing else of this
 * interface takes. They are also called from the probe and the unbind of
 * a CHMU device, with that device's lock held, so they must not wait for
 * the probe or unbind of a CHMU device. They may sleep.
 */
struct cxl_hmu_client {
	const char *name;
	int (*add)(struct cxl_hmu_inst *inst, void **data);
	void (*remove)(struct cxl_hmu_inst *inst, void *data);
	u64 max_unit_size;
/* private: */
	struct list_head node;
};

int cxl_hmu_register_client(struct cxl_hmu_client *client);
void cxl_hmu_unregister_client(struct cxl_hmu_client *client);

struct device *cxl_hmu_dev(struct cxl_hmu_inst *inst);

int cxl_hmu_claim(struct cxl_hmu_inst *inst,
		  const struct cxl_hmu_client *client);

/*
 * Called in hard interrupt context when the hotlist of a claimed instance
 * fills or overflows; it may only queue work.
 */
typedef void (*cxl_hmu_notify_fn)(void *arg);
int cxl_hmu_claim_notify(struct cxl_hmu_inst *inst,
			 const struct cxl_hmu_client *client,
			 cxl_hmu_notify_fn fn, void *arg);
void cxl_hmu_release(struct cxl_hmu_inst *inst);
u64 cxl_hmu_unit_size(struct cxl_hmu_inst *inst);

#define CXL_HMU_READ_MAX	32	/* entries one read takes at most */
#define CXL_HMU_READ_MORE	BIT(0)	/* entries are left in the hotlist */
#define CXL_HMU_READ_LOST	BIT(1)	/* the device lost hot units since the last read */

/* Returns 0 to go on, or a negative errno to end the read with it */
typedef int (*cxl_hmu_range_fn)(void *arg, u64 hpa, u64 len, u32 count);
int cxl_hmu_read_ranges(struct cxl_hmu_inst *inst, unsigned int max,
			cxl_hmu_range_fn fn, void *arg, unsigned int *flags);

#endif /* __CXL_CXL_HMU_H__ */
