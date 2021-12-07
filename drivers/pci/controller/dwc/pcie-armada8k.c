// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for Marvell Armada-8K SoCs
 *
 * Armada-8K PCIe Glue Layer Source Code
 *
 * Copyright (C) 2016 Marvell Technology Group Ltd.
 *
 * Author: Yehuda Yitshak <yehuday@marvell.com>
 * Author: Shadi Ammouri <shadi@marvell.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/of_pci.h>
#include <linux/of_irq.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include "pcie-designware.h"

#define ARMADA8K_PCIE_MAX_LANES PCIE_LNK_X4

struct armada8k_pcie {
	struct dw_pcie *pci;
	struct clk *clk;
	struct clk *clk_reg;
	struct phy *phy[ARMADA8K_PCIE_MAX_LANES];
	unsigned int phy_count;
	struct regmap *sysctrl_base;
	u32 mac_rest_bitmask;
	struct work_struct recover_link_work;
};

#define PCIE_VENDOR_REGS_OFFSET		0x8000

#define PCIE_GLOBAL_CONTROL_REG		(PCIE_VENDOR_REGS_OFFSET + 0x0)
#define PCIE_APP_LTSSM_EN		BIT(2)
#define PCIE_DEVICE_TYPE_SHIFT		4
#define PCIE_DEVICE_TYPE_MASK		0xF
#define PCIE_DEVICE_TYPE_RC		0x4 /* Root complex */

#define PCIE_GLOBAL_STATUS_REG		(PCIE_VENDOR_REGS_OFFSET + 0x8)
#define PCIE_GLB_STS_RDLH_LINK_UP	BIT(1)
#define PCIE_GLB_STS_PHY_LINK_UP	BIT(9)

#define PCIE_GLOBAL_INT_CAUSE1_REG	(PCIE_VENDOR_REGS_OFFSET + 0x1C)
#define PCIE_GLOBAL_INT_MASK1_REG	(PCIE_VENDOR_REGS_OFFSET + 0x20)
#define PCIE_INT_A_ASSERT_MASK		BIT(9)
#define PCIE_INT_B_ASSERT_MASK		BIT(10)
#define PCIE_INT_C_ASSERT_MASK		BIT(11)
#define PCIE_INT_D_ASSERT_MASK		BIT(12)

#define PCIE_GLOBAL_INT_CAUSE2_REG	(PCIE_VENDOR_REGS_OFFSET + 0x24)
#define PCIE_GLOBAL_INT_MASK2_REG	(PCIE_VENDOR_REGS_OFFSET + 0x28)
#define PCIE_INT2_PHY_RST_LINK_DOWN	BIT(1)

#define PCIE_ARCACHE_TRC_REG		(PCIE_VENDOR_REGS_OFFSET + 0x50)
#define PCIE_AWCACHE_TRC_REG		(PCIE_VENDOR_REGS_OFFSET + 0x54)
#define PCIE_ARUSER_REG			(PCIE_VENDOR_REGS_OFFSET + 0x5C)
#define PCIE_AWUSER_REG			(PCIE_VENDOR_REGS_OFFSET + 0x60)
/*
 * AR/AW Cache defaults: Normal memory, Write-Back, Read / Write
 * allocate
 */
#define ARCACHE_DEFAULT_VALUE		0x3511
#define AWCACHE_DEFAULT_VALUE		0x5311

#define DOMAIN_OUTER_SHAREABLE		0x2
#define AX_USER_DOMAIN_MASK		0x3
#define AX_USER_DOMAIN_SHIFT		4

#define UNIT_SOFT_RESET_CONFIG_REG	0x268

#define to_armada8k_pcie(x)	dev_get_drvdata((x)->dev)

static void armada8k_pcie_disable_phys(struct armada8k_pcie *pcie)
{
	int i;

	for (i = 0; i < ARMADA8K_PCIE_MAX_LANES; i++) {
		phy_power_off(pcie->phy[i]);
		phy_exit(pcie->phy[i]);
	}
}

static int armada8k_pcie_enable_phys(struct armada8k_pcie *pcie)
{
	int ret;
	int i;

	for (i = 0; i < ARMADA8K_PCIE_MAX_LANES; i++) {
		ret = phy_init(pcie->phy[i]);
		if (ret)
			return ret;

		ret = phy_set_mode_ext(pcie->phy[i], PHY_MODE_PCIE,
				       pcie->phy_count);
		if (ret) {
			phy_exit(pcie->phy[i]);
			return ret;
		}

		ret = phy_power_on(pcie->phy[i]);
		if (ret) {
			phy_exit(pcie->phy[i]);
			return ret;
		}
	}

	return 0;
}

static int armada8k_pcie_setup_phys(struct armada8k_pcie *pcie)
{
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	struct device_node *node = dev->of_node;
	int ret = 0;
	int i;

	for (i = 0; i < ARMADA8K_PCIE_MAX_LANES; i++) {
		pcie->phy[i] = devm_of_phy_get_by_index(dev, node, i);
		if (IS_ERR(pcie->phy[i])) {
			if (PTR_ERR(pcie->phy[i]) != -ENODEV)
				return PTR_ERR(pcie->phy[i]);

			pcie->phy[i] = NULL;
			continue;
		}

		pcie->phy_count++;
	}

	/* Old bindings miss the PHY handle, so just warn if there is no PHY */
	if (!pcie->phy_count)
		dev_warn(dev, "No available PHY\n");

	ret = armada8k_pcie_enable_phys(pcie);
	if (ret)
		dev_err(dev, "Failed to initialize PHY(s) (%d)\n", ret);

	return ret;
}

static int armada8k_pcie_link_up(struct dw_pcie *pci)
{
	u32 reg;
	u32 mask = PCIE_GLB_STS_RDLH_LINK_UP | PCIE_GLB_STS_PHY_LINK_UP;

	reg = dw_pcie_readl_dbi(pci, PCIE_GLOBAL_STATUS_REG);

	if ((reg & mask) == mask)
		return 1;

	dev_dbg(pci->dev, "No link detected (Global-Status: 0x%08x).\n", reg);
	return 0;
}

static void armada8k_pcie_establish_link(struct armada8k_pcie *pcie)
{
	struct dw_pcie *pci = pcie->pci;
	u32 reg;

	if (!dw_pcie_link_up(pci)) {
		/* Disable LTSSM state machine to enable configuration */
		reg = dw_pcie_readl_dbi(pci, PCIE_GLOBAL_CONTROL_REG);
		reg &= ~(PCIE_APP_LTSSM_EN);
		dw_pcie_writel_dbi(pci, PCIE_GLOBAL_CONTROL_REG, reg);
	}

	/* Set the device to root complex mode */
	reg = dw_pcie_readl_dbi(pci, PCIE_GLOBAL_CONTROL_REG);
	reg &= ~(PCIE_DEVICE_TYPE_MASK << PCIE_DEVICE_TYPE_SHIFT);
	reg |= PCIE_DEVICE_TYPE_RC << PCIE_DEVICE_TYPE_SHIFT;
	dw_pcie_writel_dbi(pci, PCIE_GLOBAL_CONTROL_REG, reg);

	/* Set the PCIe master AxCache attributes */
	dw_pcie_writel_dbi(pci, PCIE_ARCACHE_TRC_REG, ARCACHE_DEFAULT_VALUE);
	dw_pcie_writel_dbi(pci, PCIE_AWCACHE_TRC_REG, AWCACHE_DEFAULT_VALUE);

	/* Set the PCIe master AxDomain attributes */
	reg = dw_pcie_readl_dbi(pci, PCIE_ARUSER_REG);
	reg &= ~(AX_USER_DOMAIN_MASK << AX_USER_DOMAIN_SHIFT);
	reg |= DOMAIN_OUTER_SHAREABLE << AX_USER_DOMAIN_SHIFT;
	dw_pcie_writel_dbi(pci, PCIE_ARUSER_REG, reg);

	reg = dw_pcie_readl_dbi(pci, PCIE_AWUSER_REG);
	reg &= ~(AX_USER_DOMAIN_MASK << AX_USER_DOMAIN_SHIFT);
	reg |= DOMAIN_OUTER_SHAREABLE << AX_USER_DOMAIN_SHIFT;
	dw_pcie_writel_dbi(pci, PCIE_AWUSER_REG, reg);

	/* Enable INT A-D interrupts */
	reg = dw_pcie_readl_dbi(pci, PCIE_GLOBAL_INT_MASK1_REG);
	reg |= PCIE_INT_A_ASSERT_MASK | PCIE_INT_B_ASSERT_MASK |
	       PCIE_INT_C_ASSERT_MASK | PCIE_INT_D_ASSERT_MASK;
	dw_pcie_writel_dbi(pci, PCIE_GLOBAL_INT_MASK1_REG, reg);

	/* Also enable link down interrupts */
	reg = dw_pcie_readl_dbi(pci, PCIE_GLOBAL_INT_MASK2_REG);
	reg |= PCIE_INT2_PHY_RST_LINK_DOWN;
	dw_pcie_writel_dbi(pci, PCIE_GLOBAL_INT_MASK2_REG, reg);

	if (!dw_pcie_link_up(pci)) {
		/* Configuration done. Start LTSSM */
		reg = dw_pcie_readl_dbi(pci, PCIE_GLOBAL_CONTROL_REG);
		reg |= PCIE_APP_LTSSM_EN;
		dw_pcie_writel_dbi(pci, PCIE_GLOBAL_CONTROL_REG, reg);
	}

	/* Wait until the link becomes active again */
	if (dw_pcie_wait_for_link(pci))
		dev_err(pci->dev, "Link not up after reconfiguration\n");
}

static int armada8k_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct armada8k_pcie *pcie = to_armada8k_pcie(pci);

	dw_pcie_setup_rc(pp);
	armada8k_pcie_establish_link(pcie);

	return 0;
}

static void armada8k_pcie_recover_link(struct work_struct *ws)
{
	struct armada8k_pcie *pcie = container_of(ws, struct armada8k_pcie, recover_link_work);
	struct pcie_port *pp = &pcie->pci->pp;
	struct pci_bus *bus = pp->root_bus;
	struct pci_dev *root_port;
	int ret;

	root_port = pci_get_slot(bus, 0);
	if (!root_port) {
		dev_err(pcie->pci->dev, "failed to get root port\n");
		return;
	}
	pci_lock_rescan_remove();
	pci_stop_and_remove_bus_device(root_port);
	/*
	 * Sleep needed to make sure all pcie transactions and access
	 * are flushed before resetting the mac
	 */
	msleep(100);

	/* Reset mac */
	regmap_update_bits_base(pcie->sysctrl_base, UNIT_SOFT_RESET_CONFIG_REG,
				pcie->mac_rest_bitmask, 0, NULL, false, true);
	udelay(1);
	regmap_update_bits_base(pcie->sysctrl_base, UNIT_SOFT_RESET_CONFIG_REG,
				pcie->mac_rest_bitmask, pcie->mac_rest_bitmask,
				NULL, false, true);
	udelay(1);
	ret = armada8k_pcie_host_init(pp);
	if (ret) {
		dev_err(pcie->pci->dev, "failed to initialize host: %d\n", ret);
		pci_unlock_rescan_remove();
		pci_dev_put(root_port);
		return;
	}

	bus = NULL;
	while ((bus = pci_find_next_bus(bus)) != NULL)
		pci_rescan_bus(bus);
	pci_unlock_rescan_remove();
	pci_dev_put(root_port);
}

static irqreturn_t armada8k_pcie_irq_handler(int irq, void *arg)
{
	struct armada8k_pcie *pcie = arg;
	struct dw_pcie *pci = pcie->pci;
	u32 val;

	/*
	 * Interrupts are directly handled by the device driver of the
	 * PCI device. However, they are also latched into the PCIe
	 * controller, so we simply discard them.
	 */
	val = dw_pcie_readl_dbi(pci, PCIE_GLOBAL_INT_CAUSE1_REG);
	dw_pcie_writel_dbi(pci, PCIE_GLOBAL_INT_CAUSE1_REG, val);

	val = dw_pcie_readl_dbi(pci, PCIE_GLOBAL_INT_CAUSE2_REG);

	if (PCIE_INT2_PHY_RST_LINK_DOWN & val) {
		u32 ctrl_reg = dw_pcie_readl_dbi(pci, PCIE_GLOBAL_CONTROL_REG);
		/*
		 * The link went down. Disable LTSSM immediately. This
		 * unlocks the root complex config registers. Downstream
		 * device accesses will return all-Fs
		 */
		ctrl_reg &= ~(PCIE_APP_LTSSM_EN);
		dw_pcie_writel_dbi(pci, PCIE_GLOBAL_CONTROL_REG, ctrl_reg);
		/*
		 * Mask link down interrupts. They can be re-enabled once
		 * the link is retrained.
		 */
		ctrl_reg = dw_pcie_readl_dbi(pci, PCIE_GLOBAL_INT_MASK2_REG);
		ctrl_reg &= ~PCIE_INT2_PHY_RST_LINK_DOWN;
		dw_pcie_writel_dbi(pci, PCIE_GLOBAL_INT_MASK2_REG, ctrl_reg);
		/*
		 * At this point a worker thread can be triggered to
		 * initiate a link retrain. If link retrains were
		 * possible, that is.
		 */
		if (pcie->sysctrl_base && pcie->mac_rest_bitmask)
			schedule_work(&pcie->recover_link_work);

		dev_dbg(pci->dev, "%s: link went down\n", __func__);
	}

	/* Now clear the second interrupt cause. */
	dw_pcie_writel_dbi(pci, PCIE_GLOBAL_INT_CAUSE2_REG, val);

	return IRQ_HANDLED;
}

static const struct dw_pcie_host_ops armada8k_pcie_host_ops = {
	.host_init = armada8k_pcie_host_init,
};

static int armada8k_add_pcie_port(struct armada8k_pcie *pcie,
				  struct platform_device *pdev)
{
	struct dw_pcie *pci = pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	int ret;

	pp->ops = &armada8k_pcie_host_ops;

	pp->irq = platform_get_irq(pdev, 0);
	if (pp->irq < 0) {
		dev_err(dev, "failed to get irq for port\n");
		return pp->irq;
	}

	ret = devm_request_irq(dev, pp->irq, armada8k_pcie_irq_handler,
			       IRQF_SHARED, "armada8k-pcie", pcie);
	if (ret) {
		dev_err(dev, "failed to request irq %d\n", pp->irq);
		return ret;
	}

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct dw_pcie_ops dw_pcie_ops = {
	.link_up = armada8k_pcie_link_up,
};

static int armada8k_pcie_probe(struct platform_device *pdev)
{
	struct dw_pcie *pci;
	struct armada8k_pcie *pcie;
	struct device *dev = &pdev->dev;
	struct resource *base;
	int ret;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pci->ops = &dw_pcie_ops;

	pcie->pci = pci;

	INIT_WORK(&pcie->recover_link_work, armada8k_pcie_recover_link);

	pcie->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(pcie->clk))
		return PTR_ERR(pcie->clk);

	ret = clk_prepare_enable(pcie->clk);
	if (ret)
		return ret;

	pcie->clk_reg = devm_clk_get(dev, "reg");
	if (pcie->clk_reg == ERR_PTR(-EPROBE_DEFER)) {
		ret = -EPROBE_DEFER;
		goto fail;
	}
	if (!IS_ERR(pcie->clk_reg)) {
		ret = clk_prepare_enable(pcie->clk_reg);
		if (ret)
			goto fail_clkreg;
	}

	/* Get the dw-pcie unit configuration/control registers base. */
	base = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ctrl");
	pci->dbi_base = devm_pci_remap_cfg_resource(dev, base);
	if (IS_ERR(pci->dbi_base)) {
		dev_err(dev, "couldn't remap regs base %p\n", base);
		ret = PTR_ERR(pci->dbi_base);
		goto fail_clkreg;
	}

	pcie->sysctrl_base = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						       "marvell,system-controller");
	if (IS_ERR(pcie->sysctrl_base)) {
		dev_warn(dev, "failed to find marvell,system-controller\n");
		pcie->sysctrl_base = 0x0;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "marvell,mac-reset-bit-mask",
				   &pcie->mac_rest_bitmask);
	if (ret < 0) {
		dev_warn(dev, "couldn't find mac reset bit mask: %d\n", ret);
		pcie->mac_rest_bitmask = 0x0;
	}
	ret = armada8k_pcie_setup_phys(pcie);
	if (ret)
		goto fail_clkreg;

	platform_set_drvdata(pdev, pcie);

	ret = armada8k_add_pcie_port(pcie, pdev);
	if (ret)
		goto disable_phy;

	return 0;

disable_phy:
	armada8k_pcie_disable_phys(pcie);
fail_clkreg:
	clk_disable_unprepare(pcie->clk_reg);
fail:
	clk_disable_unprepare(pcie->clk);

	return ret;
}

static const struct of_device_id armada8k_pcie_of_match[] = {
	{ .compatible = "marvell,armada8k-pcie", },
	{},
};

static struct platform_driver armada8k_pcie_driver = {
	.probe		= armada8k_pcie_probe,
	.driver = {
		.name	= "armada8k-pcie",
		.of_match_table = of_match_ptr(armada8k_pcie_of_match),
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(armada8k_pcie_driver);
