/* SPDX-License-Identifier: GPL-2.0
 * Marvell Silicon gicv3 definitions
 *
 * Copyright (C) 2020 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_IRQCHIP_MARVELL_GIC_V3_H
#define __LINUX_IRQCHIP_MARVELL_GIC_V3_H

struct page *its_prop_alloc_pages(struct rdists *gic_rdists,
				size_t prop_tbl_sz,  gfp_t gfp_flags);
struct page *its_pend_alloc_pages(struct rdists *gic_rdists,
		size_t prop_tbl_sz, size_t pend_tbl_sz, gfp_t gfp_flags);
int redist_lpis_enabled(struct rdists *gic_rdists);

#endif

