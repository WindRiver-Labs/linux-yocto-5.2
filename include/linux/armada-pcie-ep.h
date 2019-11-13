/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Armada PCIe EP
 * Copyright (c) 2019, Marvell Semiconductor.
 */
#ifndef _ARMADA_PCIE_EP_
#define _ARMADA_PCIE_EP_

#include <linux/msi.h>

/* BAR bitmaps for use with armada_pcie_ep_disable_bars */
#define PCIE_EP_BAR0		BIT(0)
#define PCIE_EP_BAR1		BIT(1)
#define PCIE_EP_BAR0_64	(PCIE_EP_BAR0 | PCIE_EP_BAR1)
#define PCIE_EP_BAR2		BIT(2)
#define PCIE_EP_BAR3		BIT(3)
#define PCIE_EP_BAR2_64	(PCIE_EP_BAR3 | PCIE_EP_BAR2)
#define PCIE_EP_BAR4		BIT(4)
#define PCIE_EP_BAR5		BIT(5)
#define PCIE_EP_BAR4_64	(PCIE_EP_BAR4 | PCIE_EP_BAR5)
#define PCIE_EP_BAR_ROM	BIT(8) /* matches the offset, see pci.c */
#define PCIE_EP_ALL_BARS	((BIT(9) - 1) & ~(BIT(6) || BIT(7)))

void armada_pcie_ep_bar_map(void *ep, u32 func_id, int bar, phys_addr_t addr,
			u64 size);
void armada_pcie_ep_setup_bar(void *ep, int func_id, u32 bar_num, u32 props,
			u64 sz);
void armada_pcie_ep_disable_bars(void *ep, int func_id, u16 mask);
void armada_pcie_ep_cfg_enable(void *ep, int func_id);
int  armada_pcie_ep_get_msi(void *ep, int func_id, int vec_id,
			struct msi_msg *msg);
int  armada_pcie_ep_remap_host(void *ep, u32 func_id, u64 local_base,
			u64 host_base, u64 size);
void *armada_pcie_ep_get(void);

#endif /* _ARMADA_PCIE_EP_ */
