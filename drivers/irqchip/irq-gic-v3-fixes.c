// SPDX-License-Identifier: GPL-2.0
/* Marvell Silicon gicv3 definitions
 *
 * Copyright (C) 2020 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/percpu.h>
#include <linux/arm-smccc.h>
#include <linux/irqchip.h>
#include <linux/irqchip/arm-gic-v3.h>
#include <linux/irqchip/irq-gic-v3-fixes.h>

#define OCTEONTX_LPI_PROP_TBL_BASE      0xc2000c0c
#define gic_data_rdist()		(raw_cpu_ptr(gic_rdists->rdist))

struct page *its_prop_alloc_pages(struct rdists *gic_rdists,
				size_t prop_tbl_sz,  gfp_t gfp_flags)
{
	struct page *prop_page;
	struct arm_smccc_res res;
	void *vaddr;

	gic_rdists->prop_table_pa = 0;
	gic_rdists->prop_table_va = NULL;
	arm_smccc_smc(OCTEONTX_LPI_PROP_TBL_BASE, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0 == SMCCC_RET_SUCCESS) {
		vaddr = memremap(res.a1, prop_tbl_sz, MEMREMAP_WB);
		if (!vaddr)
			return NULL;

		gic_rdists->prop_table_pa = res.a1;
		gic_rdists->prop_table_va = vaddr;
		prop_page = NULL;
	} else {
		pr_warn("Failed to allocate LPI tables in atf\n");
		prop_page = alloc_pages(gfp_flags, get_order(prop_tbl_sz));
		if (!prop_page)
			return NULL;

		gic_rdists->prop_table_pa = page_to_phys(prop_page);
		gic_rdists->prop_table_va = page_address(prop_page);
	}

	return prop_page;
}

struct page *its_pend_alloc_pages(struct rdists *gic_rdists,
		size_t prop_tbl_sz, size_t pend_tbl_sz, gfp_t gfp_flags)
{
	struct page *pend_page;
	void *vaddr;

	gic_data_rdist()->pend_paddr = 0;
	gic_data_rdist()->pend_vaddr = NULL;

	if (!gic_rdists->prop_page) {
		phys_addr_t paddr;

		paddr = gic_rdists->prop_table_pa +
			prop_tbl_sz +
			pend_tbl_sz * smp_processor_id();
		vaddr = memremap(paddr, pend_tbl_sz, MEMREMAP_WB);
		if (!vaddr)
			return NULL;

		memset(vaddr, 0, pend_tbl_sz);
		gic_data_rdist()->pend_paddr = paddr;
		gic_data_rdist()->pend_vaddr = vaddr;
		pend_page = NULL;
	} else {
		pend_page = alloc_pages(gfp_flags | __GFP_ZERO,
			      get_order(max_t(u32, pend_tbl_sz, SZ_64K)));
		if (!pend_page)
			return NULL;

		gic_data_rdist()->pend_paddr = page_to_phys(pend_page);
		gic_data_rdist()->pend_vaddr = page_address(pend_page);
	}

	return pend_page;
}

int redist_lpis_enabled(struct rdists *gic_rdists)
{
	void __iomem *rbase = gic_data_rdist()->rd_base;

	return (readl_relaxed(rbase + GICR_CTLR) & GICR_CTLR_ENABLE_LPIS);
}
