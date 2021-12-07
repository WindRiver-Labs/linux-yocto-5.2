// SPDX-License-Identifier: GPL-2.0
/* Marvell CN10K RVU Hardware Random Number Generator.
 *
 * Copyright (C) 2020 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>

/* CSRs */
#define RNM_CTL_STATUS		0x000
#define RNM_CONST		0x030
#define RNM_EBG_ENT		0x048
#define RNM_PF_EBG_HEALTH	0x050
#define RNM_PF_RANDOM		0x400

struct cn10k_rng {
	void __iomem *reg_base;
	struct hwrng ops;
};

static int cn10k_rng_read(struct hwrng *hwrng, void *data,
			  size_t max, bool wait)
{
	struct cn10k_rng *rng = (struct cn10k_rng *)hwrng->priv;
	unsigned int size = max;

	while (size >= 8) {
		*((u64 *)data) = readq(rng->reg_base + RNM_PF_RANDOM);
		size -= 8;
		data += 8;
	}
	while (size > 0) {
		*((u8 *)data) = readb(rng->reg_base + RNM_PF_RANDOM);
		size--;
		data++;
	}
	return max;
}

static int cn10k_rng_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct	cn10k_rng *rng;
	int	err;

	rng = devm_kzalloc(&pdev->dev, sizeof(*rng), GFP_KERNEL);
	if (!rng)
		return -ENOMEM;

	pci_set_drvdata(pdev, rng);

	rng->reg_base = pcim_iomap(pdev, 0, 0);
	if (!rng->reg_base) {
		dev_err(&pdev->dev, "Error while mapping CSRs, exiting\n");
		return -ENOMEM;
	}

	rng->ops.name = devm_kasprintf(&pdev->dev, GFP_KERNEL,
				       "cn10k-rng-%s", dev_name(&pdev->dev));
	if (!rng->ops.name)
		return -ENOMEM;

	rng->ops.read    = cn10k_rng_read;
	rng->ops.quality = 1000;
	rng->ops.priv = (unsigned long) rng;

	err = devm_hwrng_register(&pdev->dev, &rng->ops);
	if (err) {
		dev_err(&pdev->dev, "Could not register hwrng device.\n");
		return err;
	}

	return 0;
}

static void cn10k_rng_remove(struct pci_dev *pdev)
{
	/* Nothing to do */
}

static const struct pci_device_id cn10k_rng_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, 0xA098) }, /* RNG PF */
	{0,},
};

MODULE_DEVICE_TABLE(pci, cn10k_rng_id_table);

static struct pci_driver cn10k_rng_driver = {
	.name		= "cn10k_rng",
	.id_table	= cn10k_rng_id_table,
	.probe		= cn10k_rng_probe,
	.remove		= cn10k_rng_remove,
};

module_pci_driver(cn10k_rng_driver);
MODULE_AUTHOR("Sunil Goutham <sgoutham@marvell.com>");
MODULE_DESCRIPTION("Marvell CN10K HW RNG Driver");
MODULE_LICENSE("GPL v2");
