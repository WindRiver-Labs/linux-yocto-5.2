/*
 * PCIe host controller driver for Freescale S32V SoCs
 *
 * Copyright (C) 2014-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2013 Kosagi
 *		http://www.kosagi.com
 *
 * Author: Sean Cross <xobs@kosagi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/s32v234_src.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/sizes.h>
#include <linux/of_platform.h>
#include <linux/rcupdate.h>

#include <linux/regulator/consumer.h>

#include "pcie-designware.h"

#define to_s32v234_pcie(x)	container_of(x, struct s32v234_pcie, pp)

struct s32v234_pcie {
	int			reset_gpio;
	int			power_on_gpio;
	struct clk		*pcie_bus;
	struct clk		*pcie_phy;
	struct clk		*pcie;
	struct pcie_port	pp;
	struct regmap		*src;
	void __iomem		*mem_base;
};
#ifdef CONFIG_PCI_S32V234_EP
#define SETUP_OUTBOUND		_IOWR('S', 1, struct s32v_outbound_region)
#define SETUP_INBOUND		_IOWR('S', 2, struct s32v_inbound_region)
#define SEND_MSI			_IOWR('S', 3, u64)
#define GET_BAR_INFO		_IOWR('S', 4, struct s32v_bar)
#define STORE_PID			_IOR('S', 7,  s32)
#define SEND_SIGNAL			_IOR('S', 8,  int)
#ifdef CONFIG_PCI_DW_DMA
#define SEND_SINGLE_DMA		_IOWR('S', 6, struct dma_data_elem)
#define GET_DMA_CH_ERRORS		_IOR('S', 9,  u32)
#define RESET_DMA_WRITE			_IOW('S', 10,  u32)
#define RESET_DMA_READ			_IOW('S', 11,  u32)
#define STORE_LL_INFO		_IOR('S', 12,  struct dma_ll_info)
#define SEND_LL				_IOWR('S', 13, struct dma_list(*)[])
#define START_LL				_IOWR('S', 14, u32)
#endif
#endif /* CONFIG_PCI_S32V234_EP */
#define PCIE_MSI_CAP			0x50
#define PCIE_MSI_ADDR_LOWER		0x54
#define PCIE_MSI_ADDR_UPPER		0x58
#define PCIE_MSI_DATA			0x5C
#define PCIE_ATU_VIEWPORT		0x900
#define PCIE_ATU_REGION_INBOUND		(0x1 << 31)
#define PCIE_ATU_REGION_OUTBOUND	(0x0 << 31)
#define PCIE_ATU_REGION_INDEX1		(0x1 << 0)
#define PCIE_ATU_REGION_INDEX0		(0x0 << 0)
#define PCIE_ATU_CR1			0x904
#define PCIE_ATU_TYPE_MEM		(0x0 << 0)
#define PCIE_ATU_TYPE_IO		(0x2 << 0)
#define PCIE_ATU_TYPE_CFG0		(0x4 << 0)
#define PCIE_ATU_TYPE_CFG1		(0x5 << 0)
#define PCIE_ATU_CR2			0x908
#define PCIE_ATU_ENABLE			(0x1 << 31)
#define PCIE_ATU_BAR_MODE_ENABLE	(0x1 << 30)
#define PCIE_ATU_BAR_NUM(bar)	((bar) << 8)
#define PCIE_ATU_LOWER_BASE		0x90C
#define PCIE_ATU_UPPER_BASE		0x910
#define PCIE_ATU_LIMIT			0x914
#define PCIE_ATU_LOWER_TARGET		0x918
#define PCIE_ATU_BUS(x)			(((x) & 0xff) << 24)
#define PCIE_ATU_DEV(x)			(((x) & 0x1f) << 19)
#define PCIE_ATU_FUNC(x)		(((x) & 0x7) << 16)
#define PCIE_ATU_UPPER_TARGET		0x91C

/* PCIe Root Complex registers (memory-mapped) */
#define PCIE_RC_LCR				0x7c
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN1	0x1
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN2	0x2
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK	0xf

/* PCIe Port Logic registers (memory-mapped) */
#define PL_OFFSET 0x700
#define PCIE_PL_PFLR (PL_OFFSET + 0x08)
#define PCIE_PL_PFLR_LINK_STATE_MASK		(0x3f << 16)
#define PCIE_PL_PFLR_FORCE_LINK			(1 << 15)
#define PCIE_PHY_DEBUG_R0 (PL_OFFSET + 0x28)
#define PCIE_PHY_DEBUG_R1 (PL_OFFSET + 0x2c)
#define PCIE_PHY_DEBUG_R1_XMLH_LINK_IN_TRAINING	(1 << 29)
#define PCIE_PHY_DEBUG_R1_XMLH_LINK_UP		(1 << 4)

#define PCIE_PHY_CTRL (PL_OFFSET + 0x114)
#define PCIE_PHY_CTRL_DATA_LOC 0
#define PCIE_PHY_CTRL_CAP_ADR_LOC 16
#define PCIE_PHY_CTRL_CAP_DAT_LOC 17
#define PCIE_PHY_CTRL_WR_LOC 18
#define PCIE_PHY_CTRL_RD_LOC 19

#define PCIE_PHY_STAT (PL_OFFSET + 0x110)
#define PCIE_PHY_STAT_ACK_LOC 16

#define PCIE_LINK_WIDTH_SPEED_CONTROL	0x80C
#define PORT_LOGIC_SPEED_CHANGE		(0x1 << 17)

/* PHY registers (not memory-mapped) */
#define PCIE_PHY_RX_ASIC_OUT 0x100D

#define PHY_RX_OVRD_IN_LO 0x1005
#define PHY_RX_OVRD_IN_LO_RX_DATA_EN (1 << 5)
#define PHY_RX_OVRD_IN_LO_RX_PLL_EN (1 << 3)

#ifdef CONFIG_PCI_S32V234_EP
/* MSI base region  */
#define MSI_REGION			0x72FB0000
#define PCI_BASE_ADDR		0x72000000
#define PCI_BASE_DBI		0x72FFC000
#define MSI_REGION_NR	3
#define NR_REGIONS		4
/* BAR generic definitions */
#define PCI_REGION_MEM			0x00000000	/* PCI mem space */
#define PCI_REGION_IO			0x00000001	/* PCI IO space */
#define PCI_WIDTH_32b			0x00000000	/* 32-bit BAR */
#define PCI_WIDTH_64b			0x00000004	/* 64-bit BAR */
#define PCI_REGION_PREFETCH		0x00000008	/* prefetch PCI mem */
#define PCI_REGION_NON_PREFETCH	0x00000000	/* non-prefetch PCI mem */
/* BARs sizing */
#define PCIE_BAR0_SIZE		SZ_1M	/* 1MB */
#define PCIE_BAR1_SIZE		0
#define PCIE_BAR2_SIZE		SZ_1M	/* 1MB */
#define PCIE_BAR3_SIZE		0		/* 256B Fixed sizing  */
#define PCIE_BAR4_SIZE		0		/* 4K Fixed sizing  */
#define PCIE_BAR5_SIZE		0		/* 64K Fixed sizing  */
#define PCIE_ROM_SIZE		0
/* BARs individual en/dis  */
#define PCIE_BAR0_EN_DIS		1
#define PCIE_BAR1_EN_DIS		0
#define PCIE_BAR2_EN_DIS		1
#define PCIE_BAR3_EN_DIS		1
#define PCIE_BAR4_EN_DIS		1
#define PCIE_BAR5_EN_DIS		1
#define PCIE_ROM_EN_DIS			0
/* BARs configuration */
#define PCIE_BAR0_INIT	(PCI_REGION_MEM | PCI_WIDTH_32b | \
		PCI_REGION_NON_PREFETCH)
#define PCIE_BAR1_INIT	(PCI_REGION_MEM | PCI_WIDTH_32b | \
		PCI_REGION_NON_PREFETCH)
#define PCIE_BAR2_INIT	(PCI_REGION_MEM | PCI_WIDTH_32b | \
		PCI_REGION_NON_PREFETCH)
#define PCIE_BAR3_INIT	(PCI_REGION_MEM | PCI_WIDTH_32b | \
		PCI_REGION_NON_PREFETCH)
#define PCIE_BAR4_INIT	0
#define PCIE_BAR5_INIT	0
#define PCIE_ROM_INIT	0
struct task_struct *task;
struct s32v_bar {
	u32 bar_nr;
	u32 size;
	u32 addr;
};

struct s32v_inbound_region {
	u32 bar_nr;
	u32 target_addr;
	u32 region;
};
struct s32v_outbound_region {
	u64 target_addr;
	u64 base_addr;
	u32 size;
	u32 region;
	u32 region_type;
};
struct s32v_outbound_region restore_outb_arr[4];
struct s32v_inbound_region restore_inb_arr[4];
static struct pcie_port *pcie_port_ep;

#ifdef CONFIG_PCI_DW_DMA
static int s32v_store_ll_array(struct pcie_port *pp, void __user *argp)
{
	int ret = 0;
	u32 ll_nr_elem = pp->ll_info.nr_elem;

	if (argp) {
		if (copy_from_user(pp->dma_linked_list, argp,
			sizeof(struct dma_list) * ll_nr_elem))
			return -EFAULT;
	} else {/* Null argument */
		return -EFAULT;
	}
	ret = dw_pcie_dma_load_linked_list(pp, pp->dma_linked_list,
		ll_nr_elem, pp->ll_info.phy_list_addr,
		pp->ll_info.next_phy_list_addr,
		pp->ll_info.direction);

	return ret;
}

int s32v_start_dma_ll(struct pcie_port *pp, void __user *argp)
{
	int ret = 0;
	u32 phy_addr;

	if (argp) {
		if (copy_from_user(&phy_addr, argp, sizeof(phy_addr)))
			return -EFAULT;
	} else {/* Null argument */
		return -EFAULT;
	}
	ret = dw_pcie_dma_start_linked_list(pp,
		 phy_addr,
		pp->ll_info.direction);
	return ret;
}

int s32v_store_ll_array_info(struct pcie_port *pp, void __user *argp)
{
	int ret = 0;

	if (argp) {
		if (copy_from_user(&pp->ll_info, argp,
			sizeof(struct dma_ll_info)))
			return -EFAULT;
	} else {/* Null argument */
		return -EFAULT;
	}
	/* Alloc here space for pointer to array of structs */
	/* Make sure it is null before allocating space */
	if (!pp->dma_linked_list) {
		pp->dma_linked_list =
			(struct dma_list(*)[])kzalloc(sizeof(struct dma_list) *
			pp->ll_info.nr_elem, GFP_KERNEL);
	}

	return ret;
}

static int s32v_send_dma_errors(struct pcie_port *pp, void __user *argp)
{
	int ret = 0;
	u32 dma_errors;

	dma_errors = ((pp->wr_ch.errors) << 16) | pp->rd_ch.errors;

	if (copy_to_user((unsigned int *)argp, &dma_errors, sizeof(u32)))
		return -EFAULT;
	return ret;
}

static int s32v_send_dma_single(struct pcie_port *pp, void __user *argp)
{
	int ret = 0;
	struct dma_data_elem dma_elem_local;

	if (argp) {
		if (copy_from_user(&dma_elem_local, argp,
			sizeof(struct dma_data_elem)))
			return -EFAULT;
	} else
		return -EFAULT;
	ret = dw_pcie_dma_single_rw(pp, &dma_elem_local);
	return ret;
}

void s32v_reset_dma_write(struct pcie_port *pp)
{
	dw_pcie_dma_write_soft_reset(pp);
}

void s32v_reset_dma_read(struct pcie_port *pp)
{
	dw_pcie_dma_read_soft_reset(pp);
}

static irqreturn_t s32v234_pcie_dma_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;

	return dw_handle_dma_irq(pp);
}

#endif /* CONFIG_PCI_DW_DMA */

int send_signal_to_user(struct pcie_port *pp)
{
	int ret = 0;

	rcu_read_lock();
	task = pid_task(find_pid_ns(pp->user_pid, &init_pid_ns), PIDTYPE_PID);
	rcu_read_unlock();

	ret = send_sig_info(SIGUSR1, &pp->info, task);
	if (ret < 0)
		ret = -EFAULT;

	return ret;
}

int s32v_store_pid(struct pcie_port *pp, void __user *argp)
{
	int ret = 0;

	if (argp) {
		if (copy_from_user(&pp->user_pid, argp, sizeof(pp->user_pid)))
			return -EFAULT;
	}
	return ret;
}

static int s32v_setup_MSI(struct pcie_port *pp, void __user *argp)
{
	int ret = 0;
	u32 *ptr;
	u64 msi_addr;

	if (argp) {
		if (copy_from_user(&msi_addr, argp, sizeof(msi_addr)))
			return -EFAULT;
	} else {
		msi_addr = (((u64)(readl(pp->dbi_base + PCIE_MSI_ADDR_UPPER))
		<< 32) | readl(pp->dbi_base + PCIE_MSI_ADDR_LOWER));
	}

	if (!((msi_addr >= PCI_BASE_ADDR) &&
		(msi_addr < PCI_BASE_DBI))) {
		/* Region 3 used */
		writel(PCIE_ATU_REGION_OUTBOUND | MSI_REGION_NR,
			pp->dbi_base + PCIE_ATU_VIEWPORT);
		/* Setup last aligned 64K before DBI */
		writel(MSI_REGION, pp->dbi_base + PCIE_ATU_LOWER_BASE);
		writel(0, pp->dbi_base + PCIE_ATU_UPPER_BASE);
		writel(MSI_REGION + SZ_64K, pp->dbi_base + PCIE_ATU_LIMIT);
		writel(lower_32_bits(msi_addr), pp->dbi_base +
			PCIE_ATU_LOWER_TARGET);
		writel(upper_32_bits(msi_addr), pp->dbi_base +
			PCIE_ATU_UPPER_TARGET);
		writel(PCIE_ATU_TYPE_MEM, pp->dbi_base + PCIE_ATU_CR1);
		/* Enable region */
		writel(PCIE_ATU_ENABLE, pp->dbi_base + PCIE_ATU_CR2);
		ptr = (u32 *)ioremap(MSI_REGION, SZ_4K);
		*ptr = 0;
	} else { /* MSI addr is inside outbound region for MSI */
		ptr = (u32 *)ioremap(msi_addr, SZ_4K);
		*ptr = 0;
	}
	return ret;
}

static void store_inb_atu(struct s32v_inbound_region *ptrInb)
{
	int region_nr;

	region_nr = ptrInb->region;
	restore_inb_arr[region_nr].bar_nr = ptrInb->bar_nr;
	restore_inb_arr[region_nr].target_addr = ptrInb->target_addr;
	restore_inb_arr[region_nr].region = ptrInb->region;
}

static void store_outb_atu(struct s32v_outbound_region *ptrOutb)
{
	int region_nr;

	region_nr = ptrOutb->region;
	restore_outb_arr[region_nr].target_addr = ptrOutb->target_addr;
	restore_outb_arr[region_nr].base_addr = ptrOutb->base_addr;
	restore_outb_arr[region_nr].size = ptrOutb->size;
	restore_outb_arr[region_nr].region_type = ptrOutb->region_type;
	restore_outb_arr[region_nr].region = ptrOutb->region;
}

static int s32v_pcie_iatu_outbound_set(struct pcie_port *pp,
		struct s32v_outbound_region *ptrOutb)
{
	int ret = 0;

	if ((ptrOutb->size < (64 * SZ_1K)) ||
		(ptrOutb->region > NR_REGIONS - 1))
		return -EINVAL;

	writel(PCIE_ATU_REGION_OUTBOUND | ptrOutb->region,
		pp->dbi_base + PCIE_ATU_VIEWPORT);
	writel(lower_32_bits(ptrOutb->base_addr),
		pp->dbi_base +  PCIE_ATU_LOWER_BASE);
	writel(upper_32_bits(ptrOutb->base_addr),
		pp->dbi_base + PCIE_ATU_UPPER_BASE);
	writel(lower_32_bits(ptrOutb->base_addr + ptrOutb->size - 1),
		pp->dbi_base + PCIE_ATU_LIMIT);
	writel(lower_32_bits(ptrOutb->target_addr),
		pp->dbi_base + PCIE_ATU_LOWER_TARGET);
	writel(upper_32_bits(ptrOutb->target_addr),
		pp->dbi_base + PCIE_ATU_UPPER_TARGET);
	writel(ptrOutb->region_type, pp->dbi_base + PCIE_ATU_CR1);
	writel(PCIE_ATU_ENABLE, pp->dbi_base + PCIE_ATU_CR2);

	return ret;
}

static int s32v_pcie_iatu_inbound_set(struct pcie_port *pp,
		struct s32v_inbound_region *ptrInb)
{
	int ret = 0;

	if (ptrInb->region > NR_REGIONS - 1)
		return -EINVAL;

	writel(PCIE_ATU_REGION_INBOUND | ptrInb->region,
		pp->dbi_base + PCIE_ATU_VIEWPORT);
	writel(lower_32_bits(ptrInb->target_addr),
		pp->dbi_base + PCIE_ATU_LOWER_TARGET);
	writel(upper_32_bits(ptrInb->target_addr),
		pp->dbi_base + PCIE_ATU_UPPER_TARGET);
	writel(PCIE_ATU_TYPE_MEM, pp->dbi_base + PCIE_ATU_CR1);
	writel(PCIE_ATU_ENABLE | PCIE_ATU_BAR_MODE_ENABLE |
		PCIE_ATU_BAR_NUM(ptrInb->bar_nr), pp->dbi_base + PCIE_ATU_CR2);

	return ret;
}

#ifdef CONFIG_PCI_S32V234_IGNORE_ERR009852
/* User choice: ignore erratum regardless of chip version. */
static bool s32v234_pcie_ignore_err009852(void)
{
	return true;
}
#else
/* It is safe to override Kconfig selection if the chip revision is in fact
 * not affected by the erratum.
 * We rely on u-boot passing the chip revision along, via the fdt.
 */
static bool s32v234_pcie_ignore_err009852(void)
{
	const char *path = "/chosen";
	struct device_node *node;
	const u32 *rev;
	int len;
	bool ret = false;

	node = of_find_node_by_path(path);
	if (!node)
		goto err_find_node;

	rev = of_get_property(node, "soc_revision", &len);
	if (rev && (len == sizeof(u32)))
		ret = (*rev > 0);

err_find_node:
	return ret;
}
#endif

static void restore_inb_atu(struct pcie_port *pp)
{
	int i;

	for (i = 0 ; i < NR_REGIONS ; i++) {
		struct s32v_inbound_region *ptrInb = &restore_inb_arr[i];

		if (ptrInb->target_addr == 0)
			continue;
		s32v_pcie_iatu_inbound_set(pp, ptrInb);
	}
}

static void restore_outb_atu(struct pcie_port *pp)
{
	int i;

	for (i = 0 ; i < NR_REGIONS ; i++) {
		struct s32v_outbound_region *ptrOutb = &restore_outb_arr[i];

		if (ptrOutb->base_addr == 0)
			continue;
		s32v_pcie_iatu_outbound_set(pp, ptrOutb);
	}
}

static int s32v_get_bar_info(struct pcie_port *pp, void __user *argp)
{
	struct s32v_bar bar_info;
	u8	bar_nr = 0;
	u32 addr = 0;
	int ret = 0;

	if (copy_from_user(&bar_info, argp, sizeof(bar_info))) {
		dev_err(pp->dev, "Error while copying from user");
		return -EFAULT;
	}
	if (bar_info.bar_nr)
		bar_nr = bar_info.bar_nr;

	addr = readl(pp->dbi_base + (PCI_BASE_ADDRESS_0 + bar_info.bar_nr * 4));
	bar_info.addr = addr & 0xFFFFFFF0;
	writel(0xFFFFFFFF, pp->dbi_base +
		(PCI_BASE_ADDRESS_0 + bar_nr * 4));
	bar_info.size = readl(pp->dbi_base +
		(PCI_BASE_ADDRESS_0 + bar_nr * 4));
	bar_info.size = ~(bar_info.size & 0xFFFFFFF0) + 1;
	writel(addr, pp->dbi_base +
		(PCI_BASE_ADDRESS_0 + bar_nr * 4));

	if (copy_to_user(argp, &bar_info, sizeof(bar_info)))
		return -EFAULT;

	return ret;
}

static ssize_t s32v_ioctl(struct file *filp, u32 cmd,
		unsigned long data)
{
	int ret = 0;
	void __user *argp = (void __user *)data;
	struct pcie_port *pp = (struct pcie_port *)(filp->private_data);
	struct s32v_inbound_region	inbStr;
	struct s32v_outbound_region	outbStr;

	switch (cmd) {
		/* Call to retrieve BAR setup*/
	case GET_BAR_INFO:
		ret = s32v_get_bar_info(pp, argp);
		break;
	case SETUP_OUTBOUND:
		/* Call to setup outbound region */
		if (copy_from_user(&outbStr, argp, sizeof(outbStr)))
			return -EFAULT;
		store_outb_atu(&outbStr);
		ret = s32v_pcie_iatu_outbound_set(pp, &outbStr);
		return ret;
	case SETUP_INBOUND:
		/* Call to setup inbound region */
		if (copy_from_user(&inbStr, argp, sizeof(inbStr)))
			return -EFAULT;
		store_inb_atu(&inbStr);
		ret = s32v_pcie_iatu_inbound_set(pp, &inbStr);
		return ret;
	case SEND_MSI:
		/* Send MSI */
		ret = s32v_setup_MSI(pp, argp);
		return ret;
	case STORE_PID:
		ret = s32v_store_pid(pp, argp);
		return ret;
	case SEND_SIGNAL:
		ret = send_signal_to_user(pp);
		return ret;
	#ifdef CONFIG_PCI_DW_DMA
	case SEND_SINGLE_DMA:
		ret = s32v_send_dma_single(pp, argp);
		return ret;
	case GET_DMA_CH_ERRORS:
		ret = s32v_send_dma_errors(pp, argp);
		return ret;
	case RESET_DMA_WRITE:
		s32v_reset_dma_write(pp);
		return ret;
	case RESET_DMA_READ:
		s32v_reset_dma_read(pp);
		return ret;
	case STORE_LL_INFO:
		ret = s32v_store_ll_array_info(pp, argp);
		return ret;
	case SEND_LL:
		ret = s32v_store_ll_array(pp, argp);
		return ret;
	case START_LL:
		ret = s32v_start_dma_ll(pp, argp);
		return ret;
	#endif
	default:
		return -EINVAL;
	}
	return ret;
}
static const struct file_operations s32v_pcie_ep_dbgfs_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.unlocked_ioctl = s32v_ioctl,
};
static void s32v234_pcie_set_bar(struct pcie_port *pp,
				 int baroffset, int enable,
				 unsigned int size,
				 unsigned int init)
{
	uint32_t mask = (enable) ? ((size - 1) & ~1) : 0;
	/* According to the RM, you have to enable the BAR before you
	 * can modify the mask value. While it appears that this may
	 * be ok in a single write anyway, we play it safe.
	 */
	writel(1, pp->dbi_base + 0x1000 + baroffset);

	writel(enable | mask, pp->dbi_base + 0x1000 + baroffset);
	writel(init, pp->dbi_base + baroffset);
}

static void s32v234_pcie_setup_ep(struct pcie_port *pp)
{
	/* CMD reg:I/O space, MEM space, and Bus Master Enable */

	/*
	 * configure the class_rev(emulate one memory EP device),
	 * BAR0 and BAR2 of EP
	 */
	writel(readl(pp->dbi_base + PCI_CLASS_REVISION)
		| ((PCI_BASE_CLASS_PROCESSOR << 24) |
			     (0x80 /* other */ << 16)),
		pp->dbi_base + PCI_CLASS_REVISION);

	/* Erratum ERR009852 requires us to avoid
	 * any memory access from the RC! We solve this
	 * by disabling all BARs and ROM access
	 */
	if (s32v234_pcie_ignore_err009852()) {
		s32v234_pcie_set_bar(pp, PCI_BASE_ADDRESS_0,
				     PCIE_BAR0_EN_DIS,
				     PCIE_BAR0_SIZE,
				     PCIE_BAR0_INIT);
		s32v234_pcie_set_bar(pp, PCI_BASE_ADDRESS_1,
				     PCIE_BAR1_EN_DIS,
				     PCIE_BAR1_SIZE,
				     PCIE_BAR1_INIT);
		s32v234_pcie_set_bar(pp, PCI_BASE_ADDRESS_2,
				     PCIE_BAR2_EN_DIS,
				     PCIE_BAR2_SIZE,
				     PCIE_BAR2_INIT);
		s32v234_pcie_set_bar(pp, PCI_BASE_ADDRESS_3,
				     PCIE_BAR3_EN_DIS,
				     PCIE_BAR3_SIZE,
				     PCIE_BAR3_INIT);
		s32v234_pcie_set_bar(pp, PCI_BASE_ADDRESS_4,
				     PCIE_BAR4_EN_DIS,
				     PCIE_BAR4_SIZE,
				     PCIE_BAR4_INIT);
		s32v234_pcie_set_bar(pp, PCI_BASE_ADDRESS_5,
				     PCIE_BAR5_EN_DIS,
				     PCIE_BAR5_SIZE,
				     PCIE_BAR5_INIT);
		s32v234_pcie_set_bar(pp, PCI_ROM_ADDRESS,
				     PCIE_ROM_EN_DIS,
				     PCIE_ROM_SIZE,
				     PCIE_ROM_INIT);

		writel(readl(pp->dbi_base + PCI_COMMAND)
				| PCI_COMMAND_IO
				| PCI_COMMAND_MEMORY
				| PCI_COMMAND_MASTER,
				pp->dbi_base + PCI_COMMAND);
	} else {
		s32v234_pcie_set_bar(pp, PCI_BASE_ADDRESS_0, 0, 0, 0);
		s32v234_pcie_set_bar(pp, PCI_BASE_ADDRESS_1,
				     0, 0, 0);
		s32v234_pcie_set_bar(pp, PCI_BASE_ADDRESS_2,
				     0, 0, 0);
		s32v234_pcie_set_bar(pp, PCI_BASE_ADDRESS_3,
				     0, 0, 0);
		s32v234_pcie_set_bar(pp, PCI_BASE_ADDRESS_4,
				     0, 0, 0);
		s32v234_pcie_set_bar(pp, PCI_BASE_ADDRESS_5,
				     0, 0, 0);
		s32v234_pcie_set_bar(pp, PCI_ROM_ADDRESS,
				     0, 0, 0);
	}
}

static irqreturn_t s32v234_pcie_link_req_rst_not_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;
	struct s32v234_pcie *s32v234_pcie = to_s32v234_pcie(pp);

	regmap_update_bits(s32v234_pcie->src, SRC_PCIE_CONFIG0,
		SRC_CONFIG0_PCIE_LNK_REQ_RST_CLR, 1);
	s32v234_pcie_setup_ep(pp);
	regmap_update_bits(s32v234_pcie->src, SRC_GPR11,
				SRC_GPR11_PCIE_PCIE_CFG_READY,
				SRC_GPR11_PCIE_PCIE_CFG_READY);
	if (s32v234_pcie_ignore_err009852()) {
		restore_inb_atu(pp);
		restore_outb_atu(pp);
	}
	return IRQ_HANDLED;
}

int s32v_pcie_setup_outbound(void *data)
{
	int ret = 0;
	struct s32v_outbound_region *outbStr =
			(struct s32v_outbound_region *)data;

	if (!pcie_port_ep)
		return -ENODEV;

	if (!data)
		return -EINVAL;

	/* Call to setup outbound region */
	store_outb_atu(outbStr);
	ret = s32v_pcie_iatu_outbound_set(pcie_port_ep, outbStr);

	return ret;
}
EXPORT_SYMBOL(s32v_pcie_setup_outbound);

int s32v_pcie_setup_inbound(void *data)
{
	int ret = 0;
	struct s32v_inbound_region *inbStr =
			(struct s32v_inbound_region *)data;

	if (!pcie_port_ep)
		return -ENODEV;

	if (!data)
		return -EINVAL;

	/* Call to setup inbound region */
	store_inb_atu(inbStr);
	ret = s32v_pcie_iatu_inbound_set(pcie_port_ep, inbStr);
	return ret;
}
EXPORT_SYMBOL(s32v_pcie_setup_inbound);
#endif /* CONFIG_PCI_S32V234_EP */

#ifndef CONFIG_PCI_S32V234_EP
static int pcie_phy_poll_ack(void __iomem *dbi_base, int exp_val)
{
	u32 val;
	u32 max_iterations = 10;
	u32 wait_counter = 0;

	do {
		val = readl(dbi_base + PCIE_PHY_STAT);
		val = (val >> PCIE_PHY_STAT_ACK_LOC) & 0x1;
		wait_counter++;

		if (val == exp_val)
			return 0;

		udelay(1);
	} while (wait_counter < max_iterations);

	return -ETIMEDOUT;
}

static int pcie_phy_wait_ack(void __iomem *dbi_base, int addr)
{
	u32 val;
	int ret;

	val = addr << PCIE_PHY_CTRL_DATA_LOC;
	writel(val, dbi_base + PCIE_PHY_CTRL);

	val |= (0x1 << PCIE_PHY_CTRL_CAP_ADR_LOC);
	writel(val, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 1);
	if (ret)
		return ret;

	val = addr << PCIE_PHY_CTRL_DATA_LOC;
	writel(val, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 0);
	if (ret)
		return ret;

	return 0;
}

/* Read from the 16-bit PCIe PHY control registers (not memory-mapped) */
static int pcie_phy_read(void __iomem *dbi_base, int addr, int *data)
{
	u32 val, phy_ctl;
	int ret;

	ret = pcie_phy_wait_ack(dbi_base, addr);
	if (ret)
		return ret;

	/* assert Read signal */
	phy_ctl = 0x1 << PCIE_PHY_CTRL_RD_LOC;
	writel(phy_ctl, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 1);
	if (ret)
		return ret;

	val = readl(dbi_base + PCIE_PHY_STAT);
	*data = val & 0xffff;

	/* deassert Read signal */
	writel(0x00, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 0);
	if (ret)
		return ret;

	return 0;
}

static int pcie_phy_write(void __iomem *dbi_base, int addr, int data)
{
	u32 var;
	int ret;

	/* write addr */
	/* cap addr */
	ret = pcie_phy_wait_ack(dbi_base, addr);
	if (ret)
		return ret;

	var = data << PCIE_PHY_CTRL_DATA_LOC;
	writel(var, dbi_base + PCIE_PHY_CTRL);

	/* capture data */
	var |= (0x1 << PCIE_PHY_CTRL_CAP_DAT_LOC);
	writel(var, dbi_base + PCIE_PHY_CTRL);

	ret = pcie_phy_poll_ack(dbi_base, 1);
	if (ret)
		return ret;

	/* deassert cap data */
	var = data << PCIE_PHY_CTRL_DATA_LOC;
	writel(var, dbi_base + PCIE_PHY_CTRL);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(dbi_base, 0);
	if (ret)
		return ret;

	/* assert wr signal */
	var = 0x1 << PCIE_PHY_CTRL_WR_LOC;
	writel(var, dbi_base + PCIE_PHY_CTRL);

	/* wait for ack */
	ret = pcie_phy_poll_ack(dbi_base, 1);
	if (ret)
		return ret;

	/* deassert wr signal */
	var = data << PCIE_PHY_CTRL_DATA_LOC;
	writel(var, dbi_base + PCIE_PHY_CTRL);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(dbi_base, 0);
	if (ret)
		return ret;

	writel(0x0, dbi_base + PCIE_PHY_CTRL);

	return 0;
}

static inline bool is_S32V234_pcie(struct s32v234_pcie *s32v234_pcie)
{
	struct pcie_port *pp	= &s32v234_pcie->pp;
	struct device_node *np	= pp->dev->of_node;

	return of_device_is_compatible(np, "fsl,s32v234-pcie");
}

static int s32v234_pcie_assert_core_reset(struct pcie_port *pp)
{
	struct s32v234_pcie *s32v234_pcie = to_s32v234_pcie(pp);

	if (is_S32V234_pcie(s32v234_pcie)) {
		regmap_update_bits(s32v234_pcie->src, SRC_GPR5,
				SRC_GPR5_GPR_PCIE_BUTTON_RST_N,
				SRC_GPR5_GPR_PCIE_BUTTON_RST_N);
	}

	return 0;
}

static int s32v234_pcie_deassert_core_reset(struct pcie_port *pp)
{
	struct s32v234_pcie *s32v234_pcie = to_s32v234_pcie(pp);

	/* allow the clocks to stabilize */
	mdelay(10);
	/*
	 * Release the PCIe PHY reset here, that we have set in
	 * s32v234_pcie_init_phy() now
	 */
	if (is_S32V234_pcie(s32v234_pcie))
		regmap_update_bits(s32v234_pcie->src, SRC_GPR5,
		SRC_GPR5_GPR_PCIE_BUTTON_RST_N, 0);
	mdelay(10);
	return 0;

}

static int s32v234_pcie_init_phy(struct pcie_port *pp)
{
	struct s32v234_pcie *s32v234_pcie = to_s32v234_pcie(pp);

	regmap_update_bits(s32v234_pcie->src, SRC_GPR5,
			SRC_GPR5_PCIE_APP_LTSSM_ENABLE, 0 << 10);
	mdelay(10);
	regmap_update_bits(s32v234_pcie->src, SRC_GPR5,
			SRC_GPR5_PCIE_DEVICE_TYPE_MASK,
			PCI_EXP_TYPE_ROOT_PORT << 1);
	mdelay(10);
	regmap_update_bits(s32v234_pcie->src, SRC_GPR5,
			SRC_GPR5_PCIE_PHY_LOS_LEVEL_MASK, (0x9 << 22));
	mdelay(10);
	return 0;
}

static int s32v234_pcie_wait_for_link(struct pcie_port *pp)
{
	int count	= 1000;

	while (!dw_pcie_link_up(pp)) {
		udelay(100);
		if (--count)
			continue;
		dev_err(pp->dev, "phy link never came up\n");
		dev_dbg(pp->dev, "DEBUG_R0: 0x%08x, DEBUG_R1: 0x%08x\n",
			readl(pp->dbi_base + PCIE_PHY_DEBUG_R0),
			readl(pp->dbi_base + PCIE_PHY_DEBUG_R1));
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_PCI_MSI
static irqreturn_t s32v234_pcie_msi_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;

	return dw_handle_msi_irq(pp);
}
#endif

static int s32v234_pcie_start_link(struct pcie_port *pp)
{
	struct s32v234_pcie *s32v234_pcie = to_s32v234_pcie(pp);
	uint32_t tmp;
	int ret, count;

	/*
	 * Force Gen1 operation when starting the link.  In case the link is
	 * started in Gen2 mode, there is a possibility the devices on the
	 * bus will not be detected at all.  This happens with PCIe switches.
	 */
	tmp = readl(pp->dbi_base + PCIE_RC_LCR);
	tmp &= ~PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK;
	tmp |= PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN1;
	writel(tmp, pp->dbi_base + PCIE_RC_LCR);

	/* Start LTSSM. */

	regmap_update_bits(s32v234_pcie->src, SRC_GPR5,
			SRC_GPR5_PCIE_APP_LTSSM_ENABLE,
			SRC_GPR5_PCIE_APP_LTSSM_ENABLE);

	ret = s32v234_pcie_wait_for_link(pp);

	udelay(200);
	if (ret)
		goto out;

	/* Allow Gen2 mode after the link is up. */
	tmp = readl(pp->dbi_base + PCIE_RC_LCR);
	tmp &= ~PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK;
	tmp |= PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN2;
	writel(tmp, pp->dbi_base + PCIE_RC_LCR);

	/*
	 * Start Directed Speed Change so the best possible speed both link
	 * partners support can be negotiated.
	 */
	tmp = readl(pp->dbi_base + PCIE_LINK_WIDTH_SPEED_CONTROL);
	tmp |= PORT_LOGIC_SPEED_CHANGE;
	writel(tmp, pp->dbi_base + PCIE_LINK_WIDTH_SPEED_CONTROL);

	count = 1000;
	while (count--) {
		tmp = readl(pp->dbi_base + PCIE_LINK_WIDTH_SPEED_CONTROL);
		/* Test if the speed change finished. */
		if (!(tmp & PORT_LOGIC_SPEED_CHANGE))
			break;
		udelay(150);
	}

	/* Make sure link training is finished as well! */
	if (count)
		ret = s32v234_pcie_wait_for_link(pp);
	else
		ret = -EINVAL;

out:
	if (ret) {
		dev_err(pp->dev, "Failed to bring link up!\n");
	} else {
		tmp = readl(pp->dbi_base + 0x80);
		dev_dbg(pp->dev, "Link up, Gen=%i\n", (tmp >> 16) & 0xf);
	}
	return ret;
}

static void s32v234_pcie_host_init(struct pcie_port *pp)
{
	/* enable disp_mix power domain */
	pm_runtime_get_sync(pp->dev);

	s32v234_pcie_assert_core_reset(pp);

	if (s32v234_pcie_init_phy(pp) < 0) {
		pr_warn("Error initializing s32v234 pcie phy");
		return;
	}

	if (s32v234_pcie_deassert_core_reset(pp) < 0) {
		pr_warn("Error deasserting core reset");
		return;
	}

	dw_pcie_setup_rc(pp);

	if (s32v234_pcie_start_link(pp) < 0) {
		pr_warn("Error starting pcie link");
		return;
	}

#ifdef CONFIG_PCI_MSI
	dw_pcie_msi_init(pp);
#endif
}

static void s32v234_pcie_reset_phy(struct pcie_port *pp)
{
	uint32_t temp;

	pcie_phy_read(pp->dbi_base, PHY_RX_OVRD_IN_LO, &temp);
	temp |= (PHY_RX_OVRD_IN_LO_RX_DATA_EN |
		 PHY_RX_OVRD_IN_LO_RX_PLL_EN);
	pcie_phy_write(pp->dbi_base, PHY_RX_OVRD_IN_LO, temp);

	udelay(2000);

	pcie_phy_read(pp->dbi_base, PHY_RX_OVRD_IN_LO, &temp);
	temp &= ~(PHY_RX_OVRD_IN_LO_RX_DATA_EN |
		  PHY_RX_OVRD_IN_LO_RX_PLL_EN);
	pcie_phy_write(pp->dbi_base, PHY_RX_OVRD_IN_LO, temp);
}

static int s32v234_pcie_link_up(struct pcie_port *pp)
{
	u32 rc, debug_r0, rx_valid;
	int count = 15000;
	/*
	 * Test if the PHY reports that the link is up and also that the LTSSM
	 * training finished. There are three possible states of the link when
	 * this code is called:
	 * 1) The link is DOWN (unlikely)
	 *     The link didn't come up yet for some reason. This usually means
	 *     we have a real problem somewhere. Reset the PHY and exit. This
	 *     state calls for inspection of the DEBUG registers.
	 * 2) The link is UP, but still in LTSSM training
	 *     Wait for the training to finish, which should take a very short
	 *     time. If the training does not finish, we have a problem and we
	 *     need to inspect the DEBUG registers. If the training does finish,
	 *     the link is up and operating correctly.
	 * 3) The link is UP and no longer in LTSSM training
	 *     The link is up and operating correctly.
	 */
	while (1) {
		rc = readl(pp->dbi_base + PCIE_PHY_DEBUG_R1);
		if (!(rc & PCIE_PHY_DEBUG_R1_XMLH_LINK_UP))
			break;
		if (!(rc & PCIE_PHY_DEBUG_R1_XMLH_LINK_IN_TRAINING))
			return 1;
		if (!count--)
			break;
		dev_dbg(pp->dev, "Link is up, but still in training\n");
		/*
		 * Wait a little bit, then re-check if the link finished
		 * the training.
		 */
		udelay(10);
	}
	/*
	 * From L0, initiate MAC entry to gen2 if EP/RC supports gen2.
	 * Wait 2ms (LTSSM timeout is 24ms, PHY lock is ~5us in gen2).
	 * If (MAC/LTSSM.state == Recovery.RcvrLock)
	 * && (PHY/rx_valid==0) then pulse PHY/rx_reset. Transition
	 * to gen2 is stuck
	 */
	pcie_phy_read(pp->dbi_base, PCIE_PHY_RX_ASIC_OUT, &rx_valid);
	debug_r0 = readl(pp->dbi_base + PCIE_PHY_DEBUG_R0);

	if (rx_valid & 0x01)
		return 0;

	if ((debug_r0 & 0x3f) != 0x0d)
		return 0;

	dev_err(pp->dev, "transition to gen2 is stuck, reset PHY!\n");
	dev_dbg(pp->dev, "debug_r0=%08x debug_r1=%08x\n", debug_r0, rc);

	s32v234_pcie_reset_phy(pp);

	return 0;
}

static struct pcie_host_ops s32v234_pcie_host_ops = {
	.link_up = s32v234_pcie_link_up,
	.host_init = s32v234_pcie_host_init,
};

static int __init s32v234_add_pcie_port(struct pcie_port *pp,
			struct platform_device *pdev)
{
	int ret;
#ifdef CONFIG_PCI_MSI
	pp->msi_irq = platform_get_irq_byname(pdev, "msi");
	if (pp->msi_irq <= 0) {
		dev_err(&pdev->dev, "failed to get MSI irq\n");
		return -ENODEV;
	}

	ret = devm_request_irq(&pdev->dev, pp->msi_irq,
		s32v234_pcie_msi_handler,
		IRQF_SHARED, "s32v-pcie-msi", pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to request MSI irq\n");
		return -ENODEV;
	}
#endif
	pp->root_bus_nr = 0;
	pp->ops = &s32v234_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static void s32v234_pcie_shutdown(struct platform_device *pdev)
{
	struct s32v234_pcie *s32v234_pcie = platform_get_drvdata(pdev);

	/* bring down link, so bootloader gets clean state in case of reboot */
	s32v234_pcie_assert_core_reset(&s32v234_pcie->pp);
	mdelay(10);
}
#endif /* !CONFIG_PCI_S32V234_EP */

static int s32v234_pcie_probe(struct platform_device *pdev)
{
	struct s32v234_pcie *s32v234_pcie;
	struct pcie_port *pp;
	struct resource *dbi_base;
	int ret;
	#ifdef CONFIG_PCI_S32V234_EP
	struct dentry *pfile;
	#endif

	s32v234_pcie = devm_kzalloc(&pdev->dev, sizeof(*s32v234_pcie)
					, GFP_KERNEL);
	if (!s32v234_pcie)
		return -ENOMEM;

	pp = &s32v234_pcie->pp;
	pp->dev = &pdev->dev;

	/* Added for PCI abort handling */
	dbi_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pp->dbi_base = devm_ioremap_resource(&pdev->dev, dbi_base);
	if (IS_ERR(pp->dbi_base))
		return PTR_ERR(pp->dbi_base);

	/* Grab SRC config register range */
	s32v234_pcie->src =
		 syscon_regmap_lookup_by_compatible("fsl,s32v234-src");
	if (IS_ERR(s32v234_pcie->src)) {
		dev_err(&pdev->dev, "unable to find SRC registers\n");
		return PTR_ERR(s32v234_pcie->src);
	}
	#ifndef CONFIG_PCI_S32V234_EP
	ret = s32v234_add_pcie_port(pp, pdev);
	if (ret < 0)
		return ret;
		platform_set_drvdata(pdev, s32v234_pcie);
	#else
	pcie_port_ep = pp;
	pp->link_req_rst_not_irq = platform_get_irq_byname(pdev,
					"link_req_rst_not");
	if (pp->link_req_rst_not_irq <= 0) {
		dev_err(&pdev->dev, "failed to get link_req_rst_not irq\n");
		return -ENODEV;
	}
	ret = devm_request_irq(&pdev->dev, pp->link_req_rst_not_irq,
		s32v234_pcie_link_req_rst_not_handler,
		IRQF_SHARED, "link_req_rst_not", pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to request link_req_rst_not irq\n");
		return -ENODEV;
	}
	#ifdef CONFIG_PCI_DW_DMA
	pp->dma_irq = platform_get_irq_byname(pdev, "dma");
	if (pp->dma_irq <= 0) {
		dev_err(&pdev->dev, "failed to get DMA irq\n");
		return -ENODEV;
	}
	ret = devm_request_irq(&pdev->dev, pp->dma_irq,
		s32v234_pcie_dma_handler,
		IRQF_SHARED, "s32v-pcie-dma", pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to request DMA irq\n");
		return -ENODEV;
	}
	dw_pcie_dma_clear_regs(pp);

	memset(&pp->info, 0, sizeof(struct siginfo));
	pp->info.si_signo = SIGUSR1;
	pp->info.si_code = SI_USER;
	pp->info.si_int = 0;
	#endif /* CONFIG_PCI_DW_DMA */

	pp->dir = debugfs_create_dir("ep_dbgfs", NULL);
	if (!pp->dir)
		dev_info(pp->dev, "Creating debugfs dir failed\n");
	pfile = debugfs_create_file("ep_file", 0444, pp->dir,
		(void *)pp, &s32v_pcie_ep_dbgfs_fops);
	if (!pfile)
		dev_info(pp->dev, "debugfs regs for failed\n");

	writel((readl(pp->dbi_base + PCIE_MSI_CAP) | 0x10000),
	pp->dbi_base +  PCIE_MSI_CAP);

	#endif /* CONFIG_PCI_S32V234_EP */

	return 0;
}

static const struct of_device_id s32v234_pcie_of_match[] = {
	{ .compatible = "fsl,s32v234-pcie", },
	{},
};
MODULE_DEVICE_TABLE(of, s32v234_pcie_of_match);

static struct platform_driver s32v234_pcie_driver = {
	.driver = {
		.name	= "s32v234-pcie",
		.owner	= THIS_MODULE,
		.of_match_table = s32v234_pcie_of_match,
	},
	.probe = s32v234_pcie_probe,
	#ifndef CONFIG_PCI_S32V234_EP
	.shutdown = s32v234_pcie_shutdown,
	#endif
};

/* Freescale PCIe driver does not allow module unload */

static int __init s32v234_pcie_init(void)
{
	return platform_driver_probe(&s32v234_pcie_driver, s32v234_pcie_probe);
}
module_init(s32v234_pcie_init);

MODULE_AUTHOR("Sean Cross <xobs@kosagi.com>");
MODULE_DESCRIPTION("Freescale S32V PCIe host controller driver");
MODULE_LICENSE("GPL v2");
