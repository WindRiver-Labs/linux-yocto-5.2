// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe end point controller driver for Marvell Armada
 *
 * Armada PCIe Glue Layer Source Code
 *
 * Based on Armada-SP2 PCIe end-point driver
 */
#define MODULE_NAME "armada-pcie-ep"

#include <linux/armada-pcie-ep.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/resource.h>
#include <linux/of_pci.h>
#include <linux/of_irq.h>
#include <uapi/linux/pci_regs.h>
#include <linux/memory.h>

#define PCIE_GLOBAL_CTRL		0x0
#define PCIE_GLOBAL_CTRL_CRS_EN	(1 << 9)
#define PCIE_GLOBAL_CTRL_TYPE_OFF	4
#define PCIE_GLOBAL_CTRL_TYPE_MASK	0xF
#define PCIE_GLOBAL_CTRL_TYPE_RC	(0x4)

#define PCIE_ATU_VIEWPORT		0x900
#define PCIE_ATU_REGION_INBOUND	(0x1 << 31)
#define PCIE_ATU_REGION_OUTBOUND	(0x0 << 31)
#define PCIE_ATU_REGION_INDEX1		(0x1 << 0)
#define PCIE_ATU_REGION_INDEX0		(0x0 << 0)
#define PCIE_ATU_CR1			0x904
#define PCIE_ATU_CR1_FUNC_OFF		20
#define PCIE_ATU_CR1_FUNC_MASK		0x1F
#define PCIE_ATU_TYPE_MEM		(0x0 << 0)
#define PCIE_ATU_TYPE_IO		(0x2 << 0)
#define PCIE_ATU_TYPE_CFG0		(0x4 << 0)
#define PCIE_ATU_TYPE_CFG1		(0x5 << 0)
#define PCIE_ATU_CR2			0x908
#define PCIE_ATU_CR2_REGION_EN		(0x1 << 31)
#define PCIE_ATU_CR2_BAR_EN		(0x1 << 30)
#define PCIE_ATU_CR2_FUNC_EN		(0x1 << 19)
#define PCIE_ATU_CR2_BAR_OFF		8
#define PCIE_ATU_LOWER_BASE		0x90C
#define PCIE_ATU_UPPER_BASE		0x910
#define PCIE_ATU_LIMIT			0x914
#define PCIE_ATU_LOWER_TARGET		0x918
#define PCIE_ATU_BUS(x)			(((x) & 0xff) << 24)
#define PCIE_ATU_DEV(x)			(((x) & 0x1f) << 19)
#define PCIE_ATU_FUNC(x)		(((x) & 0x7) << 16)
#define PCIE_ATU_UPPER_TARGET		0x91C

#define PCIE_CAP_MSI_OFFSET		0x50
#define PCIE_MSI_MSG_CTL		0x2
#define PCIE_MSI_MSG_ADDR_L		0x4
#define PCIE_MSI_MSG_ADDR_H		0x8
#define PCI_MSI_FLAGS_QSIZE_OFF		4
#define PCIE_MSI_MSG_DATA(is_64)	(is_64 ? 0xC : 0x8)

#define PCIE_SRIOV_DEVID_OFFSET		0x192

#define PCIE_RESBAR_EXT_CAP_HDR_REG	0x25c
#define PCIE_RESBAR_EXT_CAP_REG(bar)	(PCIE_RESBAR_EXT_CAP_HDR_REG + 4 + \
					(((bar) / 2 + (bar) % 2) & 0x3) * 8)
#define PCIE_RESBAR_EXT_CAP_REG_MASK	0x000fffff
#define PCIE_RESBAR_EXT_CAP_REG_SHIFT	4

#define PCIE_BAR_IS_RESIZABLE(bar)	((bar) == 5 || (bar) == 4 || \
					 (bar) == 2 || (bar) == 0)
#define MAX_ATU_REGIONS	16
#define MAX_ATU_SIZE	(4ul * SZ_1G)

#define  BAR_ENABLE_OFFSET	0
#define  BAR_ENABLE_MASK		(1 << BAR_ENABLE_OFFSET)

struct armada_pcie_ep {
	void __iomem	*regs;
	void __iomem	*shadow_regs;
	void __iomem	*lm_regs;
	void __iomem	*pl_regs;	/*port logical register only PF0*/
	struct device	*dev;
	struct clk	*clk;
};

#define cfg_space_addr(func_id)   (0x1000 * (func_id))

#define cfg_func_base(ep, func_id, off)		\
	((ep)->regs + cfg_space_addr(func_id) + (off))

#define cfg_shadow_func_base(ep, func_id, off)		\
	((ep)->shadow_regs + cfg_space_addr(func_id) + (off))


#define get_out_region_idx(func_id, id)  (func_id + id)
#define get_in_region_idx(func_id, bar)  (func_id + bar)

struct armada_pcie_ep *armada_ep;

void armada_pcie_ep_setup_bar(void *ep_hdl, int func_id, u32 bar_num, u32 props,
			   u64 sz)
{
	struct armada_pcie_ep *ep = (struct armada_pcie_ep *)ep_hdl;
	u32 space_type = props & PCI_BASE_ADDRESS_SPACE;
	u32 sz_type	= (props & PCI_BASE_ADDRESS_MEM_TYPE_MASK);
	u32 v = 0;
	void __iomem *resbar = ep->pl_regs + PCIE_RESBAR_EXT_CAP_REG(bar_num);
	void __iomem *bar = cfg_func_base(ep, func_id,
				PCI_BASE_ADDRESS_0 + (bar_num * 4));
	void __iomem *bar_mask = cfg_shadow_func_base(ep, func_id,
				PCI_BASE_ADDRESS_0 + (bar_num * 4));

	dev_dbg(ep->dev, "func%d: BAR%d size=0x%llx set requested\n",
		func_id, bar_num, sz);
	if (space_type == PCI_BASE_ADDRESS_SPACE_IO) {
		v = props & (~PCI_BASE_ADDRESS_IO_MASK);
		writel_relaxed(v, bar);
	} else {
		/* clear the top 32 bits of the size */
		if (sz_type == PCI_BASE_ADDRESS_MEM_TYPE_64)
			writel_relaxed(0, bar + 4);

		v = props & (~PCI_BASE_ADDRESS_MEM_MASK);
		writel_relaxed(v, bar);
	}

	/*
	 * Set the BAR using resizable BAR capability registers
	 * The minimum (and the default) BAR size is 1MB
	 * Once the Resizable BAR capability register is set
	 * the resizable BAR control register at next offset gets
	 * updated automatically.
	 */
	if (sz > SZ_1M && PCIE_BAR_IS_RESIZABLE(bar_num)) {
		/* BAR size should be power of 2 already */
		v = ((sz >> 20) & PCIE_RESBAR_EXT_CAP_REG_MASK);
		v <<= PCIE_RESBAR_EXT_CAP_REG_SHIFT;
		writel_relaxed(v, resbar);
	}

	/* Enable bar */
	writel_relaxed(BAR_ENABLE_MASK, bar_mask);

}
EXPORT_SYMBOL(armada_pcie_ep_setup_bar);

void armada_pcie_ep_disable_bars(void *ep_hdl, int func_id, u16 mask)
{
	struct armada_pcie_ep *ep = (struct armada_pcie_ep *)ep_hdl;
	void __iomem *bar_mask = cfg_shadow_func_base(ep, func_id,
						      PCI_BASE_ADDRESS_0);
	int bar;

	dev_dbg(ep->dev, "func%d: disable BARs 0x%x\n", func_id, mask);
	mask &= PCIE_EP_ALL_BARS;
	for (bar = 0; mask; mask >>= 1, bar++) {
		if (mask & 1)
			writel_relaxed(0, bar_mask + bar * 4);
	}
}
EXPORT_SYMBOL(armada_pcie_ep_disable_bars);

int armada_pcie_ep_get_msi(void *ep_hdl, int func_id, int vec_id,
			struct msi_msg *msg)
{
	struct armada_pcie_ep *ep = (struct armada_pcie_ep *)ep_hdl;
	void __iomem  *p = cfg_func_base(ep, func_id, PCIE_CAP_MSI_OFFSET);
	u16 flags, vec_cnt;

	/* check if MSI is enabled and there are enough vectors
	 * QSIZE field indicates log2 of the amount of MSI vectors
	 */
	flags = readw(p + PCI_MSI_FLAGS);
	vec_cnt =
		1 << ((flags & PCI_MSI_FLAGS_QSIZE) >> PCI_MSI_FLAGS_QSIZE_OFF);
	if (!(flags & PCI_MSI_FLAGS_ENABLE) || (vec_id > vec_cnt))
		return -EINVAL;

	dev_dbg(ep->dev, "func%d: get msi vector id/counter 0x%x/%d\n",
		func_id, vec_id, vec_cnt);
	msg->address_lo = readl(p + PCI_MSI_ADDRESS_LO);
	if (flags & PCI_MSI_FLAGS_64BIT) {
		msg->address_hi = readl(p + PCI_MSI_ADDRESS_HI);
		msg->data = readl(p + PCI_MSI_DATA_64) + vec_id;
	} else {
		msg->address_hi = 0;
		msg->data = readl(p + PCI_MSI_DATA_32) + vec_id;
	}

	return 0;
}
EXPORT_SYMBOL(armada_pcie_ep_get_msi);

void armada_pcie_ep_cfg_enable(void *ep_hdl, int func_id)
{
	struct armada_pcie_ep *ep = (struct armada_pcie_ep *)ep_hdl;
	u32 v;

	dev_dbg(ep->dev, "func%d: config enable\n", func_id);
	v = readl_relaxed(ep->lm_regs + PCIE_GLOBAL_CTRL);
	v &= ~PCIE_GLOBAL_CTRL_CRS_EN;
	writel_relaxed(v, ep->lm_regs + PCIE_GLOBAL_CTRL);
}
EXPORT_SYMBOL(armada_pcie_ep_cfg_enable);

/*
 * Remap the host memory space to the local memory space.
 * By default the memory spaces conflict so we must offset the
 * host memory space in our local memory space
 */
int armada_pcie_ep_remap_host(void *ep_hdl, u32 func_id, u64 local_base,
			   u64 host_base, u64 size)
{
	struct armada_pcie_ep *ep = (struct armada_pcie_ep *)ep_hdl;
	void __iomem *pl_regs = ep->pl_regs;
	u32   v, region = 0;
	u64 remain_size = size;

	/* ATU window size must be power of 2 */
	if (!is_power_of_2(size))
		return -EINVAL;

	dev_dbg(ep->dev, "func%d: remap local:host(size) %llx:%llx(%llx)\n",
		func_id, local_base, host_base, size);

	while (remain_size > 0) {
		if (region > MAX_ATU_REGIONS) {
			dev_err(ep->dev,
				"Insufficient ATU regions to map hosts\n");
			return -1;
		}

		v = PCIE_ATU_REGION_OUTBOUND;
		v |= get_out_region_idx(func_id, region);
		writel_relaxed(v, pl_regs + PCIE_ATU_VIEWPORT);

		writel_relaxed(local_base & U32_MAX,
				pl_regs + PCIE_ATU_LOWER_BASE);
		writel_relaxed(local_base >> 32, pl_regs + PCIE_ATU_UPPER_BASE);
		writel_relaxed(host_base & U32_MAX,
				pl_regs + PCIE_ATU_LOWER_TARGET);
		writel_relaxed(host_base >> 32,
				pl_regs + PCIE_ATU_UPPER_TARGET);

		if (remain_size > MAX_ATU_SIZE)
			v = MAX_ATU_SIZE - 1;
		else
			v = remain_size - 1;
		writel_relaxed(v, pl_regs + PCIE_ATU_LIMIT);

		v = (func_id & PCIE_ATU_CR1_FUNC_MASK) << PCIE_ATU_CR1_FUNC_OFF;
		writel_relaxed(v, pl_regs + PCIE_ATU_CR1);

		v = PCIE_ATU_CR2_REGION_EN;
		writel_relaxed(v, pl_regs + PCIE_ATU_CR2);

		region++;
		local_base += MAX_ATU_SIZE;
		host_base += MAX_ATU_SIZE;
		remain_size -= MAX_ATU_SIZE;
	}

	return 0;
}
EXPORT_SYMBOL(armada_pcie_ep_remap_host);

/* setup the internal target for the BAR. When the PCIe host accesses the bar
 * it will reach the space defined by "addr" and "size"
 */
void armada_pcie_ep_bar_map(void *ep_hdl, u32 func_id, int bar,
			    phys_addr_t addr, u64 size)
{
	struct armada_pcie_ep *ep = (struct armada_pcie_ep *)ep_hdl;
	void __iomem *pl_regs = ep->pl_regs;
	u32   region_indx = get_in_region_idx(func_id, bar);
	u32   v;

	v = PCIE_ATU_REGION_INBOUND | region_indx;
	writel_relaxed(v, pl_regs + PCIE_ATU_VIEWPORT);

	addr = addr & ~(size - 1);
	v = lower_32_bits(addr);
	writel_relaxed(v, pl_regs + PCIE_ATU_LOWER_TARGET);

	v = upper_32_bits(addr);
	writel_relaxed(v, pl_regs + PCIE_ATU_UPPER_TARGET);

	v = (func_id & PCIE_ATU_CR1_FUNC_MASK) << PCIE_ATU_CR1_FUNC_OFF;
	writel_relaxed(v, pl_regs + PCIE_ATU_CR1);

	v = PCIE_ATU_CR2_REGION_EN |
	    PCIE_ATU_CR2_BAR_EN |
	    (bar << PCIE_ATU_CR2_BAR_OFF);
	writel_relaxed(v, pl_regs + PCIE_ATU_CR2);
	dev_dbg(ep->dev, "func%d: BAR%d map size@addr %llx@%llx\n",
		func_id, bar, addr, size);
}
EXPORT_SYMBOL(armada_pcie_ep_bar_map);

void *armada_pcie_ep_get(void)
{
	return (void *)armada_ep;
}
EXPORT_SYMBOL(armada_pcie_ep_get);

static int armada_pcie_ep_probe(struct platform_device *pdev)
{
	struct armada_pcie_ep *ep;
	struct device *dev = &pdev->dev;
	struct resource *base;
	void __iomem *p;
	int ret = 0;

	ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	ep->clk = devm_clk_get(dev, NULL);
	if (PTR_ERR(ep->clk) == -EPROBE_DEFER) {
		dev_info(dev, "PCIE EP probe deferred\n");
		return -EPROBE_DEFER;
	}
	if (IS_ERR(ep->clk)) {
		dev_err(dev, "can't find clock node\n");
		return -ENODEV;
	}

	ret = clk_prepare_enable(ep->clk);
	if (ret) {
		dev_err(dev, "couldn't enable clock\n");
		return ret;
	}

	ep->dev = dev;
	platform_set_drvdata(pdev, ep);

	/* Get registers bases and remap */
	base = platform_get_resource_byname(pdev, IORESOURCE_MEM, "lm");
	p = devm_ioremap_resource(dev, base);
	if (IS_ERR(p)) {
		dev_err(dev, "couldn't remap lm regs base %pR\n", base);
		return PTR_ERR(p);
	}
	ep->lm_regs = p;
	dev_dbg(dev, "reg-%s va:pa(sz) %llx:%llx(%llx)\n",
		"lm    ", (phys_addr_t)p, base->start,
		base->end - base->start);

	base = platform_get_resource_byname(pdev, IORESOURCE_MEM, "core");
	p = devm_ioremap_resource(dev, base);
	if (IS_ERR(p)) {
		dev_err(dev, "couldn't remap core regs base %pR\n", base);
		return PTR_ERR(p);
	}
	ep->regs = p;
	ep->pl_regs = p;
	dev_dbg(dev, "reg-%s va:pa(sz) %llx:%llx(%llx)\n",
		"core  ", (phys_addr_t)p, base->start,
		base->end - base->start);

	base = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					    "shadow_core");
	p = devm_ioremap_resource(dev, base);
	if (IS_ERR(p)) {
		dev_err(dev, "%s: couldn't remap shadow regs base %pR\n",
			MODULE_NAME, base);
		return PTR_ERR(p);
	}
	ep->shadow_regs = p;
	dev_dbg(dev, "reg-%s va:pa(sz) %llx:%llx(%llx)\n",
		"shadow", (phys_addr_t)p, base->start,
		base->end - base->start);

	armada_ep = ep;

	return 0;
}

static const struct of_device_id armada_pcie_ep_of_match[] = {
	{ .compatible = "marvell,armada-pcie-ep", },
	{},
};
MODULE_DEVICE_TABLE(of, armada_pcie_ep_of_match);

static struct platform_driver armada_pcie_ep_driver = {
	.probe		= armada_pcie_ep_probe,
	.driver = {
		.name	= "armada-pcie-ep",
		.of_match_table = of_match_ptr(armada_pcie_ep_of_match),
	},
};

module_platform_driver(armada_pcie_ep_driver);

MODULE_DESCRIPTION("Armada PCIe EP controller driver");
MODULE_AUTHOR("Gang Chen <gangc@marvell.com>");
MODULE_AUTHOR("Yehuda Yitshcak <yehuday@marvell.com>");

