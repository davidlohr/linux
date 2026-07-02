// SPDX-License-Identifier: GPL-2.0
/*
 * Test consumer for PCIe UIO routes over CXL regions.
 *
 * Exercises the whole UIO consumer contract from debugfs so a guest
 * test harness can drive it: provider lookup from a committed region,
 * pci_p2pdma_map_info() with all three policies, attribute derivation,
 * dma_map_phys(), route validity tracking across fabric events, and
 * the quiesce/revoke ops that model a userspace-facing importer.
 *
 * /sys/kernel/debug/cxl_uio_test/
 *   requester  (rw) requester BDF, e.g. 0000:10:00.0
 *   region     (rw) cxl region device name, e.g. region0
 *   policy     (rw) forbidden | preferred | required
 *   offset     (rw) subrange start offset into the region
 *   len        (rw) subrange length (0 = whole region)
 *   acquire    (w1) run map_info + map; (r) result dump
 *   release    (w1) unmap + route put
 *   misuse     (w1) deliberate DMA API misuse for the dma-debug UIO
 *                   checks (RAM + no-route mapping, mismatched unmap)
 *   valid      (r)  current route validity
 *   events     (r)  quiesce/revoke callback counters
 *
 * Copyright (C) 2026 Davidlohr Bueso <dave@stgolabs.net>
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/pci-p2pdma.h>
#include <linux/pci-uio.h>

#include "cxl.h"

static struct cxl_uio_test {
	struct mutex lock;
	char requester[32];
	char region[32];
	u32 policy;
	u64 offset;
	u64 len;

	/* live acquisition state */
	int last_rc;
	bool active;
	struct pci_dev *pdev;
	struct device *region_dev;
	struct p2pdma_provider *provider;
	struct pci_p2pdma_map_info info;
	unsigned long attrs;
	dma_addr_t dma_addr;
	resource_size_t mapped_len;

	atomic_t quiesce_calls;
	atomic_t revoke_calls;
} cut = {
	.lock = __MUTEX_INITIALIZER(cut.lock),
	.policy = PCI_UIO_REQUIRED,
};

static int cut_quiesce(struct pci_uio_route *route, void *priv)
{
	atomic_inc(&cut.quiesce_calls);
	return 0;
}

static void cut_revoke(struct pci_uio_route *route, void *priv)
{
	atomic_inc(&cut.revoke_calls);
}

static const struct pci_uio_route_ops cut_route_ops = {
	.quiesce = cut_quiesce,
	.revoke = cut_revoke,
};

static void cut_release_locked(void)
{
	if (!cut.active)
		return;

	if (cut.dma_addr) {
		dma_unmap_phys(&cut.pdev->dev, cut.dma_addr, cut.mapped_len,
			       DMA_BIDIRECTIONAL, cut.attrs);
		cut.dma_addr = 0;
	}
	pci_uio_route_put(cut.info.uio_route);
	cut.info.uio_route = NULL;
	pci_dev_put(cut.pdev);
	cut.pdev = NULL;
	put_device(cut.region_dev);
	cut.region_dev = NULL;
	cut.provider = NULL;
	cut.active = false;
}

static int cut_acquire_locked(void)
{
	unsigned int domain, bus, dev, fn;
	struct pci_uio_route_req req = {
		.policy = cut.policy,
		.ops = &cut_route_ops,
	};
	struct device *region_dev;
	struct pci_dev *pdev;
	struct cxl_region *cxlr;
	struct p2pdma_provider *prov;
	resource_size_t len;
	int rc;

	cut_release_locked();

	if (sscanf(cut.requester, "%x:%x:%x.%x", &domain, &bus, &dev, &fn) != 4)
		return -EINVAL;

	pdev = pci_get_domain_bus_and_slot(domain, bus, PCI_DEVFN(dev, fn));
	if (!pdev)
		return -ENODEV;

	region_dev = bus_find_device_by_name(&cxl_bus_type, NULL, cut.region);
	if (!region_dev || !is_cxl_region(region_dev)) {
		if (region_dev)
			put_device(region_dev);
		pci_dev_put(pdev);
		return -ENODEV;
	}
	cxlr = container_of(region_dev, struct cxl_region, dev);

	prov = cxl_region_p2pdma_provider(cxlr);
	if (IS_ERR(prov)) {
		rc = PTR_ERR(prov);
		goto out_put;
	}

	len = cut.len ?: prov->size - cut.offset;

	rc = pci_p2pdma_map_info(prov, &pdev->dev, cut.offset, len,
				 &req, &cut.info);
	if (rc)
		goto out_put;

	/*
	 * cxl_pci devices do not normally participate in the streaming
	 * DMA API, so no mask is set up; peer HPAs live well above 4G.
	 */
	rc = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (rc)
		goto out_route;

	cut.attrs = p2pdma_map_attrs(&cut.info, prov);
	cut.dma_addr = dma_map_phys(&pdev->dev, prov->base + cut.offset, len,
				    DMA_BIDIRECTIONAL, cut.attrs);
	if (dma_mapping_error(&pdev->dev, cut.dma_addr)) {
		cut.dma_addr = 0;
		rc = -ENOSPC;
		goto out_route;
	}

	cut.pdev = pdev;
	cut.region_dev = region_dev;
	cut.provider = prov;
	cut.mapped_len = len;
	cut.active = true;
	return 0;

out_route:
	pci_uio_route_put(cut.info.uio_route);
	cut.info.uio_route = NULL;
out_put:
	put_device(region_dev);
	pci_dev_put(pdev);
	return rc;
}

static int cut_acquire_show(struct seq_file *m, void *unused)
{
	struct pci_uio_route *route;

	guard(mutex)(&cut.lock);

	seq_printf(m, "rc: %d\n", cut.last_rc);
	if (!cut.active)
		return 0;

	seq_printf(m, "map_type: %d\n", cut.info.type);
	seq_printf(m, "xport_uio: %d\n",
		   !!(cut.info.xport_flags & PCI_P2PDMA_XPORT_UIO));
	seq_printf(m, "attrs: %#lx\n", cut.attrs);
	seq_printf(m, "attr_mmio: %d\n", !!(cut.attrs & DMA_ATTR_MMIO));
	seq_printf(m, "attr_uio: %d\n", !!(cut.attrs & DMA_ATTR_UIO));
	seq_printf(m, "dma_addr: %pad\n", &cut.dma_addr);

	route = cut.info.uio_route;
	if (route) {
		seq_printf(m, "route_tc: %u\n", route->tc);
		seq_printf(m, "route_vc: %u\n", route->vc);
		seq_printf(m, "route_flags: %#lx\n", route->flags);
		seq_printf(m, "route_generation: %lu\n", route->generation);
		seq_printf(m, "max_write_size: %u\n",
			   route->limits.max_write_size);
		seq_printf(m, "max_read_size: %u\n",
			   route->limits.max_read_size);
		seq_printf(m, "boundary: %u\n", route->limits.boundary);
		seq_printf(m, "update_granule: %u\n",
			   route->limits.update_granule);
		seq_printf(m, "nr_hops: %u\n", route->nr_hops);
		seq_printf(m, "nr_targets: %u\n", route->nr_targets);
	}
	return 0;
}

static ssize_t cut_acquire_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	guard(mutex)(&cut.lock);
	cut.last_rc = cut_acquire_locked();
	return count;
}

static int cut_acquire_open(struct inode *inode, struct file *file)
{
	return single_open(file, cut_acquire_show, NULL);
}

static const struct file_operations cut_acquire_fops = {
	.owner = THIS_MODULE,
	.open = cut_acquire_open,
	.read = seq_read,
	.write = cut_acquire_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t cut_release_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	guard(mutex)(&cut.lock);
	cut_release_locked();
	return count;
}

static const struct file_operations cut_release_fops = {
	.owner = THIS_MODULE,
	.write = cut_release_write,
	.llseek = noop_llseek,
};

/*
 * Deliberate misuse, in order: DMA_ATTR_UIO against cacheable system
 * RAM with no covering route (two dma-debug warnings), then an unmap
 * without the attribute (mismatch warning). Exists purely so a test
 * harness can regression-check the CONFIG_DMA_API_DEBUG UIO checks.
 */
static ssize_t cut_misuse_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	unsigned int domain, busnr, dev, fn;
	struct pci_dev *pdev;
	struct page *page;
	dma_addr_t addr;
	int rc;

	guard(mutex)(&cut.lock);

	if (sscanf(cut.requester, "%x:%x:%x.%x", &domain, &busnr, &dev,
		   &fn) != 4)
		return -EINVAL;

	pdev = pci_get_domain_bus_and_slot(domain, busnr, PCI_DEVFN(dev, fn));
	if (!pdev)
		return -ENODEV;

	rc = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (rc) {
		pci_dev_put(pdev);
		return rc;
	}

	page = alloc_page(GFP_KERNEL);
	if (!page) {
		pci_dev_put(pdev);
		return -ENOMEM;
	}

	addr = dma_map_phys(&pdev->dev, page_to_phys(page), PAGE_SIZE,
			    DMA_BIDIRECTIONAL, DMA_ATTR_UIO);
	if (!dma_mapping_error(&pdev->dev, addr))
		dma_unmap_phys(&pdev->dev, addr, PAGE_SIZE,
			       DMA_BIDIRECTIONAL, 0);

	__free_page(page);
	pci_dev_put(pdev);
	return count;
}

static const struct file_operations cut_misuse_fops = {
	.owner = THIS_MODULE,
	.write = cut_misuse_write,
	.llseek = noop_llseek,
};

static int cut_valid_show(struct seq_file *m, void *unused)
{
	guard(mutex)(&cut.lock);
	seq_printf(m, "%d\n",
		   cut.active && pci_uio_route_valid(cut.info.uio_route));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(cut_valid);

static int cut_events_show(struct seq_file *m, void *unused)
{
	seq_printf(m, "quiesce: %d\nrevoke: %d\n",
		   atomic_read(&cut.quiesce_calls),
		   atomic_read(&cut.revoke_calls));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(cut_events);

static ssize_t cut_str_read(struct file *file, char __user *ubuf,
			    size_t count, loff_t *ppos)
{
	char *s = file->private_data;

	guard(mutex)(&cut.lock);
	return simple_read_from_buffer(ubuf, count, ppos, s, strlen(s));
}

static ssize_t cut_str_write(struct file *file, const char __user *ubuf,
			     size_t count, loff_t *ppos)
{
	char *s = file->private_data;
	ssize_t len;

	guard(mutex)(&cut.lock);
	len = simple_write_to_buffer(s, sizeof(cut.requester) - 1, ppos, ubuf,
				     count);
	if (len < 0)
		return len;
	s[len] = '\0';
	strim(s);
	return count;
}

static const struct file_operations cut_str_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = cut_str_read,
	.write = cut_str_write,
	.llseek = default_llseek,
};

static const char * const cut_policies[] = {
	[PCI_UIO_FORBIDDEN] = "forbidden",
	[PCI_UIO_PREFERRED] = "preferred",
	[PCI_UIO_REQUIRED] = "required",
};

static int cut_policy_show(struct seq_file *m, void *unused)
{
	guard(mutex)(&cut.lock);
	seq_printf(m, "%s\n", cut_policies[cut.policy]);
	return 0;
}

static ssize_t cut_policy_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	char buf[16];
	int policy;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	policy = sysfs_match_string(cut_policies, buf);
	if (policy < 0)
		return policy;

	guard(mutex)(&cut.lock);
	cut.policy = policy;
	return count;
}

static int cut_policy_open(struct inode *inode, struct file *file)
{
	return single_open(file, cut_policy_show, NULL);
}

static const struct file_operations cut_policy_fops = {
	.owner = THIS_MODULE,
	.open = cut_policy_open,
	.read = seq_read,
	.write = cut_policy_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct dentry *cut_debugfs;

static int __init cxl_uio_test_init(void)
{
	struct dentry *d = debugfs_create_dir("cxl_uio_test", NULL);

	if (IS_ERR(d))
		return PTR_ERR(d);
	cut_debugfs = d;

	debugfs_create_file("requester", 0600, d, cut.requester,
			    &cut_str_fops);
	debugfs_create_file("region", 0600, d, cut.region, &cut_str_fops);
	debugfs_create_file("policy", 0600, d, NULL, &cut_policy_fops);
	debugfs_create_x64("offset", 0600, d, &cut.offset);
	debugfs_create_x64("len", 0600, d, &cut.len);
	debugfs_create_file("acquire", 0600, d, NULL, &cut_acquire_fops);
	debugfs_create_file("release", 0200, d, NULL, &cut_release_fops);
	debugfs_create_file("misuse", 0200, d, NULL, &cut_misuse_fops);
	debugfs_create_file("valid", 0400, d, NULL, &cut_valid_fops);
	debugfs_create_file("events", 0400, d, NULL, &cut_events_fops);
	return 0;
}
module_init(cxl_uio_test_init);

static void __exit cxl_uio_test_exit(void)
{
	mutex_lock(&cut.lock);
	cut_release_locked();
	mutex_unlock(&cut.lock);
	debugfs_remove_recursive(cut_debugfs);
}
module_exit(cxl_uio_test_exit);

MODULE_DESCRIPTION("PCIe UIO route test consumer for CXL regions");
MODULE_AUTHOR("Davidlohr Bueso");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("CXL");
