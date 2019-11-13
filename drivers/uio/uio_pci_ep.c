// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Marvell International Ltd.
 *
 * UIO PCIe end point driver
 *
 * This driver exposes PCI EP resources to user-space
 * It is currently coupled to armada PCI EP driver but it
 * in the future it will use the standard PCI EP stack
 */

#include <linux/armada-pcie-ep.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/pci_ids.h>
#include <linux/pci_regs.h>
#include <linux/uio_driver.h>

struct uio_pci {
	struct device	*dev;
	void		*ep;
	struct resource	*host_map;
};

/* make sure we have at least one mem regions to map the host ram */
#define MAX_BAR_MAP	5

/* export the BAR0/2 address/size, used by Facility */
void __iomem	*bar0_addr;
EXPORT_SYMBOL(bar0_addr);
size_t		bar0_size;
EXPORT_SYMBOL(bar0_size);
void __iomem	*bar2_addr;
EXPORT_SYMBOL(bar2_addr);
size_t		bar2_size;
EXPORT_SYMBOL(bar2_size);


static int uio_pci_ep_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	void *ep;
	struct uio_pci *uio_pci;
	struct resource *res;
	struct uio_info *info;
	struct uio_mem *mem;
	char  *name;
	int    bar_id, mem_id;

	ep = armada_pcie_ep_get();
	if (!ep) {
		dev_info(dev, "PCI EP probe deferred\n");
		return -EPROBE_DEFER;
	}

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	uio_pci = devm_kzalloc(dev, sizeof(*uio_pci), GFP_KERNEL);
	if (!uio_pci)
		return -ENOMEM;

	/* connect the objects */
	platform_set_drvdata(pdev, info);
	info->priv = uio_pci;
	uio_pci->dev = dev;

	/* store private data */
	info->name = "uio_pci_ep_0";
	info->version = "1.0.1";

	/* setup the PCI EP topology. This should eventually move to the PCI
	 *  EP Function driver
	 */
	uio_pci->ep = ep;

	/* Setup the BARs according to device tree */
	for (bar_id = 0, mem_id = 0; bar_id < MAX_BAR_MAP;
	     mem_id++, bar_id++) {
		name = devm_kzalloc(dev, 6 * sizeof(char), GFP_KERNEL);
		if (name == NULL)
			return -ENOMEM;
		snprintf(name, 5, "bar%d", bar_id);

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
		if (!res) {
			kfree(name);
			dev_err(dev, "Did not find BAR-%d resource %pR\n",
				bar_id, res);
			return -ENOMEM;
		}
		/* Setup the UIO memory attributes */
		mem = &info->mem[mem_id];
		mem->memtype = UIO_MEM_PHYS;
		mem->size = resource_size(res);
		mem->name = name;

		if (!is_power_of_2(mem->size)) {
			dev_err(dev, "BAR-%d size in not power of 2\n",
				bar_id);
			return -EINVAL;
		}

		if (bar_id == 0) {
			struct page *pg = alloc_pages(GFP_KERNEL | __GFP_ZERO,
						      get_order(mem->size));
			if (!pg) {
				dev_err(dev, "alloc RAM resource %pR failed\n",
					res);
				return -ENOMEM;
			}
			mem->internal_addr = page_address(pg);
			bar0_addr = mem->internal_addr;
			bar0_size = mem->size;
			mem->addr = virt_to_phys(mem->internal_addr);
			armada_pcie_ep_bar_map(ep, 0, bar_id,
					       (phys_addr_t)mem->addr,
					       mem->size);
		} else {
			mem->addr = res->start;
			mem->internal_addr = devm_ioremap(dev, mem->addr,
							  mem->size);
			if (IS_ERR(mem->internal_addr)) {
				dev_err(dev, "map BAR-%d memory %pR failed\n",
					bar_id, res);
				return -ENOMEM;
			}

			if (bar_id == 2) {
				bar2_addr = mem->internal_addr;
				bar2_size = mem->size;
			}
		}

		/* First 2 BARs in HW are 64 bit registers
		 * and consume 2 BAR slots
		 */
		if (bar_id < 4)
			bar_id++;
	}

	/* remap host RAM to local memory space  using shift mapping.
	 * i.e. address 0x0 in host becomes uio_pci->host_map->start.
	 * Additionally map the host physical space to the virtual
	 * memory using ioremap.
	 */
	uio_pci->host_map = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							 "host-map");
	if (!uio_pci->host_map) {
		dev_err(dev, "Device tree missing host mappings\n");
		return -ENODEV;
	}

	/* Describe the host as a UIO space */
	name = devm_kzalloc(dev, 16 * sizeof(char), GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	snprintf(name, 16, "host-map");
	mem = &info->mem[mem_id];
	mem->memtype = UIO_MEM_PHYS;
	mem->size = resource_size(uio_pci->host_map);
	mem->name = name;
	mem->addr = uio_pci->host_map->start;
	mem->internal_addr = devm_ioremap_resource(dev, uio_pci->host_map);
	if (IS_ERR(mem->internal_addr)) {
		dev_err(dev, "map host memory %pR failed\n", uio_pci->host_map);
		return -ENODEV;
	}

	/* register the UIO device */
	if (uio_register_device(dev, info) != 0) {
		dev_err(dev, "UIO registration failed\n");
		return -ENODEV;
	}

	dev_info(dev, "Registered UIO PCI EP successfully\n");

	return 0;
}

static int uio_pci_ep_remove(struct platform_device *pdev)
{
	struct uio_info *info = platform_get_drvdata(pdev);

	uio_unregister_device(info);
	return 0;
}

static const struct of_device_id uio_pci_ep_match[] = {
	{ .compatible = "marvell,pci-ep-uio", },
	{}
};
MODULE_DEVICE_TABLE(of, uio_pci_ep_match);

static struct platform_driver uio_pci_ep_driver = {
	.driver = {
		.name = "marvell,pci-ep-uio",
		.owner = THIS_MODULE,
		.of_match_table = uio_pci_ep_match,
	},
	.probe = uio_pci_ep_probe,
	.remove = uio_pci_ep_remove,
};
module_platform_driver(uio_pci_ep_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Yehuda Yitschak <yehuday@marvell.com>");
MODULE_DESCRIPTION("PCI EP Function UIO driver");
