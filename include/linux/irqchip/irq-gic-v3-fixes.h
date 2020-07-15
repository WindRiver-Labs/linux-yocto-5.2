/* SPDX-License-Identifier: GPL-2.0
 * Marvell Silicon GICv3/ITS hardware quirks
 *
 * Copyright (C) 2020 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_IRQCHIP_MARVELL_GIC_V3_H
#define __LINUX_IRQCHIP_MARVELL_GIC_V3_H

#define GICV3_QUIRK_IPI_MISS	(1 << 0)

u32 gic_rdist_pend_reg(int cpu, int offset);
u32 gic_rdist_active_reg(int cpu, int offset);

void gic_ipi_rxcount_inc(int cpu, int irq);
void gic_write_sgi1r_retry(int dest_cpu, int irq, u64 val);
void gic_v3_enable_quirks(void __iomem *base);
void gic_v3_enable_ipimiss_quirk(void);

struct page *its_prop_alloc_pages(struct rdists *gic_rdists,
				size_t prop_tbl_sz,  gfp_t gfp_flags);
struct page *its_pend_alloc_pages(struct rdists *gic_rdists,
		size_t prop_tbl_sz, size_t pend_tbl_sz, gfp_t gfp_flags);
int redist_lpis_enabled(struct rdists *gic_rdists);

#endif

