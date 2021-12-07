// SPDX-License-Identifier: GPL-2.0
/* Marvell OcteonTx2 RVU Admin Function driver
 *
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/pci.h>
#include "rvu.h"

#define	PCI_DEVID_OCTEONTX2_RVU_PF	0xA063
#define	PCI_DEVID_OCTEONTX2_SSO_RVU_PF	0xA0F9
#define	PCI_DEVID_OCTEONTX2_NPA_RVU_PF	0xA0FB
#define	PCI_DEVID_OCTEONTX2_CPT_RVU_PF	0xA0FD

static u64 quotas_get_sum(struct rvu_quotas *quotas)
{
	u64 lf_sum = 0;
	int i;

	for (i = 0; i < quotas->cnt; i++)
		lf_sum += quotas->a[i].val;

	return lf_sum;
}

static ssize_t quota_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	struct rvu_quota *quota;
	int val;

	quota = container_of(attr, struct rvu_quota, sysfs);

	if (quota->base->lock)
		mutex_lock(quota->base->lock);
	val = quota->val;
	if (quota->base->lock)
		mutex_unlock(quota->base->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t quota_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t count)
{
	int old_val, new_val, res = 0;
	struct rvu_quota *quota;
	struct rvu_quotas *base;
	struct device *dev;
	u64 lf_sum;

	quota = container_of(attr, struct rvu_quota, sysfs);
	dev = quota->dev;
	base = quota->base;

	if (kstrtoint(buf, 0, &new_val)) {
		dev_err(dev, "Invalid %s quota: %s\n", attr->attr.name, buf);
		return -EIO;
	}
	if (new_val < 0) {
		dev_err(dev, "Invalid %s quota: %d < 0\n", attr->attr.name,
			new_val);
		return -EIO;
	}

	if (new_val > base->max) {
		dev_err(dev, "Invalid %s quota: %d > %d\n", attr->attr.name,
			new_val, base->max);
		return -EIO;
	}

	if (base->lock)
		mutex_lock(base->lock);
	old_val = quota->val;

	if (base->ops.pre_store)
		res = base->ops.pre_store(quota->ops_arg, quota, new_val);

	if (res != 0) {
		res = -EIO;
		goto unlock;
	}

	lf_sum = quotas_get_sum(quota->base);

	if (lf_sum + new_val - quota->val > base->max_sum) {
		dev_err(dev,
			"Not enough resources for %s quota. Used: %lld, Max: %lld\n",
			attr->attr.name, lf_sum, base->max_sum);
		res = -EIO;
		goto unlock;
	}
	quota->val = new_val;

	if (base->ops.post_store)
		base->ops.post_store(quota->ops_arg, quota, old_val);

	res = count;

unlock:
	if (base->lock)
		mutex_unlock(base->lock);
	return res;
}

static int quota_sysfs_destroy(struct rvu_quota *quota)
{
	if (quota == NULL)
		return -EINVAL;
	if (quota->sysfs.attr.mode != 0) {
		sysfs_remove_file(quota->parent, &quota->sysfs.attr);
		quota->sysfs.attr.mode = 0;
	}
	return 0;
}

static struct rvu_quotas *quotas_alloc(u32 cnt, u32 max, u64 max_sum,
				   int init_val, struct mutex *lock,
				   struct rvu_quota_ops *ops)
{
	struct rvu_quotas *quotas;
	u64 i;

	if (cnt == 0)
		return NULL;

	quotas = kzalloc(sizeof(struct rvu_quotas) +
			 cnt * sizeof(struct rvu_quota), GFP_KERNEL);
	if (quotas == NULL)
		return NULL;

	for (i = 0; i < cnt; i++) {
		quotas->a[i].base = quotas;
		quotas->a[i].val = init_val;
	}

	quotas->cnt = cnt;
	quotas->max = max;
	quotas->max_sum = max_sum;
	if (ops) {
		quotas->ops.pre_store = ops->pre_store;
		quotas->ops.post_store = ops->post_store;
	}
	quotas->lock = lock;

	return quotas;
}

static void quotas_free(struct rvu_quotas *quotas)
{
	u64 i;

	if (quotas == NULL)
		return;
	WARN_ON(quotas->cnt == 0);

	for (i = 0; i < quotas->cnt; i++)
		quota_sysfs_destroy(&quotas->a[i]);

	kfree(quotas);
}

static int quota_sysfs_create(const char *name, struct kobject *parent,
			      struct device *log_dev, struct rvu_quota *quota,
			      void *ops_arg)
{
	int err;

	if (name == NULL || quota == NULL || log_dev == NULL)
		return -EINVAL;

	quota->sysfs.show = quota_show;
	quota->sysfs.store = quota_store;
	quota->sysfs.attr.name = name;
	quota->sysfs.attr.mode = 0644;
	quota->parent = parent;
	quota->dev = log_dev;
	quota->ops_arg = ops_arg;

	sysfs_attr_init(&quota->sysfs.attr);
	err = sysfs_create_file(quota->parent, &quota->sysfs.attr);
	if (err) {
		dev_err(quota->dev,
			"Failed to create '%s' quota sysfs for '%s'\n",
			name, kobject_name(quota->parent));
		return -EFAULT;
	}

	return 0;
}

static int rvu_blk_count_rsrc(struct rvu_block *block, u16 pcifunc, u8 rshift)
{
	int count = 0, lf;

	for (lf = 0; lf < block->lf.max; lf++)
		if ((block->fn_map[lf] >> rshift) == (pcifunc >> rshift))
			count++;

	return count;
}

int rvu_check_rsrc_policy(struct rvu *rvu, struct rsrc_attach *req,
			  u16 pcifunc)
{
	struct rvu_pfvf *pfvf = rvu_get_pfvf(rvu, pcifunc);
	int free_lfs, mappedlfs, familylfs, limit, delta;
	struct rvu_hwinfo *hw = rvu->hw;
	int pf = rvu_get_pf(pcifunc);
	struct rvu_block *block;

	/* Only one NPA LF can be attached */
	if (req->npalf) {
		block = &hw->block[BLKADDR_NPA];
		free_lfs = rvu_rsrc_free_count(&block->lf);
		limit = rvu->pf_limits.npa->a[pf].val;
		familylfs = rvu_blk_count_rsrc(block, pcifunc,
					       RVU_PFVF_PF_SHIFT);
		if (!free_lfs || (limit == familylfs))
			goto fail;
	}

	/* Only one NIX LF can be attached */
	if (req->nixlf) {
		block = &hw->block[BLKADDR_NIX0];
		free_lfs = rvu_rsrc_free_count(&block->lf);
		limit = rvu->pf_limits.nix->a[pf].val;
		familylfs = rvu_blk_count_rsrc(block, pcifunc,
					       RVU_PFVF_PF_SHIFT);
		if (!free_lfs || (limit == familylfs))
			goto fail;
	}

	if (req->sso) {
		block = &hw->block[BLKADDR_SSO];
		mappedlfs = rvu_get_rsrc_mapcount(pfvf, block->type);
		free_lfs = rvu_rsrc_free_count(&block->lf);
		limit = rvu->pf_limits.sso->a[pf].val;
		familylfs = rvu_blk_count_rsrc(block, pcifunc,
					       RVU_PFVF_PF_SHIFT);
		/* Check if additional resources are available */
		delta = req->sso - mappedlfs;
		if ((delta > 0) && /* always allow usage decrease */
		    ((limit < familylfs + delta) ||
		     (delta > free_lfs)))
			goto fail;
	}

	if (req->ssow) {
		block = &hw->block[BLKADDR_SSOW];
		mappedlfs = rvu_get_rsrc_mapcount(pfvf, block->type);
		free_lfs = rvu_rsrc_free_count(&block->lf);
		limit = rvu->pf_limits.ssow->a[pf].val;
		familylfs = rvu_blk_count_rsrc(block, pcifunc,
					       RVU_PFVF_PF_SHIFT);
		/* Check if additional resources are available */
		delta = req->ssow - mappedlfs;
		if ((delta > 0) && /* always allow usage decrease */
		    ((limit < familylfs + delta) ||
		     (delta > free_lfs)))
			goto fail;
	}

	if (req->timlfs) {
		block = &hw->block[BLKADDR_TIM];
		mappedlfs = rvu_get_rsrc_mapcount(pfvf, block->type);
		free_lfs = rvu_rsrc_free_count(&block->lf);
		limit = rvu->pf_limits.tim->a[pf].val;
		familylfs = rvu_blk_count_rsrc(block, pcifunc,
					       RVU_PFVF_PF_SHIFT);
		/* Check if additional resources are available */
		delta = req->timlfs - mappedlfs;
		if ((delta > 0) && /* always allow usage decrease */
		    ((limit < familylfs + delta) ||
		     (delta > free_lfs)))
			goto fail;
	}

	if (req->cptlfs) {
		block = &hw->block[BLKADDR_CPT0];
		mappedlfs = rvu_get_rsrc_mapcount(pfvf, block->type);
		free_lfs = rvu_rsrc_free_count(&block->lf);
		limit = rvu->pf_limits.cpt->a[pf].val;
		familylfs = rvu_blk_count_rsrc(block, pcifunc,
					       RVU_PFVF_PF_SHIFT);
		/* Check if additional resources are available */
		delta = req->cptlfs - mappedlfs;
		if ((delta > 0) && /* always allow usage decrease */
		    ((limit < familylfs + delta) ||
		     (delta > free_lfs)))
			goto fail;
	}

	return 0;

fail:
	dev_info(rvu->dev, "Request for %s failed\n", block->name);
	return -ENOSPC;
}

static int check_mapped_rsrcs(void *arg, struct rvu_quota *quota, int new_val)
{
	struct rvu_pfvf *pf = arg;
	int type;

	for (type = 0; type < BLKTYPE_MAX; type++) {
		if (rvu_get_rsrc_mapcount(pf, type) > 0)
			return 1;
	}
	return 0;
}

static struct rvu_quota_ops pf_limit_ops = {
	.pre_store = check_mapped_rsrcs,
};

static void rvu_set_default_limits(struct rvu *rvu)
{
	int i, sso_rvus = 0, totalvfs;

	/* First pass, count number of SSO/TIM PFs. */
	for (i = 0; i < rvu->hw->total_pfs; i++) {
		if (rvu->pf[i].pdev == NULL)
			continue;
		if (rvu->pf[i].pdev->device == PCI_DEVID_OCTEONTX2_SSO_RVU_PF)
			sso_rvus++;
	}
	/* Second pass, set the default limit values. */
	for (i = 0; i < rvu->hw->total_pfs; i++) {
		if (rvu->pf[i].pdev == NULL)
			continue;
		totalvfs = pci_sriov_get_totalvfs(rvu->pf[i].pdev);
		switch (rvu->pf[i].pdev->device) {
		case PCI_DEVID_OCTEONTX2_RVU_AF:
			rvu->pf_limits.nix->a[i].val = totalvfs;
			rvu->pf_limits.npa->a[i].val = totalvfs;
			break;
		case PCI_DEVID_OCTEONTX2_RVU_PF:
			rvu->pf_limits.nix->a[i].val = 1 + totalvfs;
			rvu->pf_limits.npa->a[i].val = 1 + totalvfs;
			break;
		case PCI_DEVID_OCTEONTX2_SSO_RVU_PF:
			rvu->pf_limits.npa->a[i].val = totalvfs;
			rvu->pf_limits.sso->a[i].val =
				rvu->hw->block[BLKADDR_SSO].lf.max / sso_rvus;
			rvu->pf_limits.ssow->a[i].val =
				rvu->hw->block[BLKADDR_SSOW].lf.max / sso_rvus;
			rvu->pf_limits.tim->a[i].val =
				rvu->hw->block[BLKADDR_TIM].lf.max / sso_rvus;
			/* All users of CPT should not share CPUs so if there
			 * are multiple SSO/TIM PFs, then divide CPTs equally.
			 */
			rvu->pf_limits.cpt->a[i].val =
				num_online_cpus() / sso_rvus;
			break;
		case PCI_DEVID_OCTEONTX2_NPA_RVU_PF:
			rvu->pf_limits.npa->a[i].val = totalvfs;
			break;
		case PCI_DEVID_OCTEONTX2_CPT_RVU_PF:
			rvu->pf_limits.cpt->a[i].val = num_online_cpus();
			rvu->pf_limits.npa->a[i].val = 1;
			break;
		}
	}
}

static int rvu_create_limits_sysfs(struct rvu *rvu)
{
	struct pci_dev *pdev;
	struct rvu_pfvf *pf;
	int i, err = 0;

	for (i = 0; i < rvu->hw->total_pfs; i++) {
		pf = &rvu->pf[i];
		pdev = pf->pdev;

		pf->limits_kobj = kobject_create_and_add("limits",
							 &pdev->dev.kobj);

		if (quota_sysfs_create("sso", pf->limits_kobj, rvu->dev,
				       &rvu->pf_limits.sso->a[i], pf)) {
			dev_err(rvu->dev,
				"Failed to allocate quota for sso on %s\n",
				pci_name(pdev));
			err = -EFAULT;
			break;
		}

		if (quota_sysfs_create("ssow", pf->limits_kobj, rvu->dev,
				       &rvu->pf_limits.ssow->a[i], pf)) {
			dev_err(rvu->dev,
				"Failed to allocate quota for ssow, on %s\n",
				pci_name(pdev));
			err = -EFAULT;
			break;
		}

		if (quota_sysfs_create("tim", pf->limits_kobj, rvu->dev,
				       &rvu->pf_limits.tim->a[i], pf)) {
			dev_err(rvu->dev,
				"Failed to allocate quota for tim, on %s\n",
				pci_name(pdev));
			err = -EFAULT;
			break;
		}

		if (quota_sysfs_create("cpt", pf->limits_kobj, rvu->dev,
				       &rvu->pf_limits.cpt->a[i], pf)) {
			dev_err(rvu->dev,
				"Failed to allocate quota for cpt, on %s\n",
				pci_name(pdev));
			err = -EFAULT;
			break;
		}

		if (quota_sysfs_create("npa", pf->limits_kobj, rvu->dev,
				       &rvu->pf_limits.npa->a[i], pf)) {
			dev_err(rvu->dev,
				"Failed to allocate quota for npa, on %s\n",
				pci_name(pdev));
			err = -EFAULT;
			break;
		}

		if (quota_sysfs_create("nix", pf->limits_kobj, rvu->dev,
				       &rvu->pf_limits.nix->a[i], pf)) {
			dev_err(rvu->dev,
				"Failed to allocate quota for nix, on %s\n",
				pci_name(pdev));
			err = -EFAULT;
			break;
		}
	}

	return err;
}

void rvu_policy_destroy(struct rvu *rvu)
{
	struct rvu_pfvf *pf = NULL;
	int i;

	quotas_free(rvu->pf_limits.sso);
	quotas_free(rvu->pf_limits.ssow);
	quotas_free(rvu->pf_limits.npa);
	quotas_free(rvu->pf_limits.cpt);
	quotas_free(rvu->pf_limits.tim);
	quotas_free(rvu->pf_limits.nix);
	rvu->pf_limits.sso = NULL;
	rvu->pf_limits.ssow = NULL;
	rvu->pf_limits.npa = NULL;
	rvu->pf_limits.cpt = NULL;
	rvu->pf_limits.tim = NULL;
	rvu->pf_limits.nix = NULL;

	for (i = 0; i < rvu->hw->total_pfs; i++) {
		pf = &rvu->pf[i];
		kobject_del(pf->limits_kobj);
	}
}

int rvu_policy_init(struct rvu *rvu)
{
	struct pci_dev *pdev = rvu->pdev;
	struct rvu_hwinfo *hw = rvu->hw;
	int err, i = 0;
	u32 max = 0;

	max = hw->block[BLKADDR_SSO].lf.max;
	rvu->pf_limits.sso = quotas_alloc(rvu->hw->total_pfs, max, max,
					  0, &rvu->rsrc_lock, &pf_limit_ops);
	if (!rvu->pf_limits.sso) {
		dev_err(rvu->dev, "Failed to allocate sso limits\n");
		err = -EFAULT;
		goto error;
	}

	max = hw->block[BLKADDR_SSOW].lf.max;
	rvu->pf_limits.ssow = quotas_alloc(rvu->hw->total_pfs, max, max,
					   0, &rvu->rsrc_lock, &pf_limit_ops);
	if (!rvu->pf_limits.ssow) {
		dev_err(rvu->dev, "Failed to allocate ssow limits\n");
		err = -EFAULT;
		goto error;
	}

	max = hw->block[BLKADDR_TIM].lf.max;
	rvu->pf_limits.tim = quotas_alloc(rvu->hw->total_pfs, max, max,
					  0, &rvu->rsrc_lock, &pf_limit_ops);
	if (!rvu->pf_limits.tim) {
		dev_err(rvu->dev, "Failed to allocate tim limits\n");
		err = -EFAULT;
		goto error;
	}

	max = hw->block[BLKADDR_CPT0].lf.max;
	rvu->pf_limits.cpt = quotas_alloc(rvu->hw->total_pfs, max, max,
					  0, &rvu->rsrc_lock, &pf_limit_ops);
	if (!rvu->pf_limits.cpt) {
		dev_err(rvu->dev, "Failed to allocate cpt limits\n");
		err = -EFAULT;
		goto error;
	}

	/* Because limits track also VFs under PF, the maximum NPA LF limit for
	 * a single PF has to be max, not 1. Same for NIX below.
	 */
	max = hw->block[BLKADDR_NPA].lf.max;
	rvu->pf_limits.npa = quotas_alloc(rvu->hw->total_pfs, max, max,
					  0, &rvu->rsrc_lock, &pf_limit_ops);
	if (!rvu->pf_limits.npa) {
		dev_err(rvu->dev, "Failed to allocate npa limits\n");
		err = -EFAULT;
		goto error;
	}

	max = hw->block[BLKADDR_NIX0].lf.max;
	rvu->pf_limits.nix = quotas_alloc(rvu->hw->total_pfs, max, max,
					  0, &rvu->rsrc_lock, &pf_limit_ops);
	if (!rvu->pf_limits.nix) {
		dev_err(rvu->dev, "Failed to allocate nix limits\n");
		err = -EFAULT;
		goto error;
	}

	for (i = 0; i < hw->total_pfs; i++)
		rvu->pf[i].pdev =
			pci_get_domain_bus_and_slot(pci_domain_nr(pdev->bus),
						    i + 1, 0);

	rvu_set_default_limits(rvu);

	err = rvu_create_limits_sysfs(rvu);
	if (err) {
		dev_err(rvu->dev, "Failed to create limits sysfs\n");
		goto error;
	}

	return 0;

error:
	rvu_policy_destroy(rvu);
	return err;
}
