// SPDX-License-Identifier: GPL-2.0+
#include "spi-cadence-xspi.h"
#include <linux/pci.h>

#define DRV_NAME       "spi-cadence-octeon"
#define STIG_OFFSET    (0x10000000)
#define AUX_OFFSET     (0x2000)
#define MAX_CS_COUNT   (4)
#define PCI_CDNS_XSPI             0xA09B


static int cadence_octeon_spi_probe(struct pci_dev *pdev,
			       const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	void __iomem *register_base;
	int ret;

	struct spi_master *master = NULL;
	struct cdns_xspi_dev *cdns_xspi = NULL;
	struct cdns_xspi_platform_data *plat_data = NULL;

	master = cdns_xspi_prepare_master(dev);
	if (!master) {
		ret = -ENOMEM;
		dev_err(dev, "Failed to allocate memory for spi_master\n");
		goto err_no_mem;
	}

	ret = pcim_enable_device(pdev);
	if (ret)
		return -ENODEV;

	ret = pci_request_regions(pdev, DRV_NAME);
	if (ret)
		goto error_disable;

	register_base = pcim_iomap(pdev, 0, pci_resource_len(pdev, 0));
	if (!register_base) {
		ret = -EINVAL;
		goto error_disable;
	}

	plat_data = devm_kmalloc(dev, sizeof(*plat_data), GFP_KERNEL);
	if (!plat_data) {
		ret = -ENOMEM;
		dev_err(dev, "Failed to allocate memory for platform_data\n");
		goto error_disable;
	}

	cdns_xspi = spi_master_get_devdata(master);
	cdns_xspi->plat_data = plat_data;
	cdns_xspi->dev = dev;

	cdns_xspi->iobase   = register_base;
	cdns_xspi->auxbase  = register_base + AUX_OFFSET;
	cdns_xspi->sdmabase = register_base + STIG_OFFSET;
	cdns_xspi->irq = 0;

	init_completion(&cdns_xspi->cmd_complete);
	init_completion(&cdns_xspi->sdma_complete);

	ret = cdns_xspi_configure(cdns_xspi);
	if (ret) {
		dev_err(dev, "Failed to prepare xSPI");
		goto error_disable;
	}

	master->num_chipselect = 1 << cdns_xspi->hw_num_banks;

	ret = devm_spi_register_master(dev, master);
	if (ret)
		dev_err(dev, "Failed to register SPI master\n");

	return 0;

error_disable:
	spi_master_put(master);

err_no_mem:
	return ret;
}

static void cadence_octeon_spi_remove(struct pci_dev *pdev)
{
	pci_disable_device(pdev);
}

static const struct pci_device_id cadence_octeon_spi_pci_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM,
		     PCI_CDNS_XSPI) },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, cadence_octeon_spi_pci_id_table);

static struct pci_driver cadence_octeon_spi_driver = {
	.name		= DRV_NAME,
	.id_table	= cadence_octeon_spi_pci_id_table,
	.probe		= cadence_octeon_spi_probe,
	.remove		= cadence_octeon_spi_remove,
};

module_pci_driver(cadence_octeon_spi_driver);

MODULE_DESCRIPTION("xSPI PCI bus driver");
MODULE_AUTHOR("Marvell Inc.");
MODULE_LICENSE("GPL");
