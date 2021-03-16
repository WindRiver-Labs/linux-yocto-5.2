// SPDX-License-Identifier: GPL-2.0+
#include "spi-cadence-xspi.h"

static int cdns_xspi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_master *master = NULL;
	struct resource *res = NULL;
	struct cdns_xspi_dev *cdns_xspi = NULL;
	struct cdns_xspi_platform_data *plat_data = NULL;
	int ret;

	master = cdns_xspi_prepare_master(dev);
	if (!master) {
		ret = -ENOMEM;
		dev_err(dev, "Failed to allocate memory for spi_master\n");
		goto err_no_mem;
	}

	platform_set_drvdata(pdev, master);

	plat_data = devm_kmalloc(dev, sizeof(*plat_data), GFP_KERNEL);
	if (!plat_data) {
		ret = -ENOMEM;
		dev_err(dev, "Failed to allocate memory for platform_data\n");
		goto err_spi_master;
	}
	pdev->dev.platform_data = plat_data;

	cdns_xspi = spi_master_get_devdata(master);
	cdns_xspi->pdev = pdev;
	cdns_xspi->dev = &pdev->dev;
	cdns_xspi->current_cs = 0;
	cdns_xspi->plat_data = plat_data;

	init_completion(&cdns_xspi->cmd_complete);
	init_completion(&cdns_xspi->sdma_complete);

	ret = cdns_xspi_of_get_plat_data(dev);
	if (ret) {
		ret = -ENODEV;
		dev_err(dev, "Failed to get platform data\n");
		goto err_spi_master;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cdns_xspi->iobase = devm_ioremap_resource(cdns_xspi->dev, res);
	if (IS_ERR(cdns_xspi->iobase)) {
		ret = PTR_ERR(cdns_xspi->iobase);
		dev_err(dev, "Failed to remap controller base address\n");
		goto err_spi_master;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	cdns_xspi->sdmabase = devm_ioremap_resource(cdns_xspi->dev, res);
	if (IS_ERR(cdns_xspi->sdmabase)) {
		ret = PTR_ERR(cdns_xspi->sdmabase);
		dev_err(dev, "Failed to remap SDMA address\n");
		goto err_spi_master;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	cdns_xspi->auxbase = devm_ioremap_resource(cdns_xspi->dev, res);
	if (IS_ERR(cdns_xspi->auxbase)) {
		ret = PTR_ERR(cdns_xspi->auxbase);
		dev_err(dev, "Failed to remap AUX address\n");
		goto err_spi_master;
	}

	cdns_xspi->irq = platform_get_irq(pdev, 0);
	if (cdns_xspi->irq < 0) {
		ret = -ENXIO;
		dev_err(dev, "Failed to get IRQ\n");
		goto err_spi_master;
	}

	ret = devm_request_irq(dev, cdns_xspi->irq, cdns_xspi_irq_handler,
		IRQF_SHARED, pdev->name, cdns_xspi);
	if (ret) {
		dev_err(dev, "Failed to request IRQ: %d\n", cdns_xspi->irq);
		goto err_spi_master;
	}

	ret = cdns_xspi_configure(cdns_xspi);
	if (ret) {
		dev_err(dev, "Failed to prepare xSPI");
		goto err_spi_master;
	}

	master->num_chipselect = 1 << cdns_xspi->hw_num_banks;

	ret = devm_spi_register_master(dev, master);
	if (ret) {
		dev_err(dev, "Failed to register SPI master\n");
		goto err_spi_master;
	}

	dev_info(dev, "Successfully registered SPI master\n");

	return 0;

err_spi_master:
	spi_master_put(master);

err_no_mem:
	dev_err(dev, "Failed to probe Cadence XSPI controller driver\n");

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id cdns_xspi_of_match[] = {
	{
		.compatible = "cdns,xspi-nor-fpga",
	},
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, cdns_xspi_of_match);
#else
#define cdns_xspi_of_match NULL
#endif /* CONFIG_OF */

static struct platform_driver cdns_xspi_platform_driver = {
	.probe          = cdns_xspi_probe,
	.remove         = NULL,
	.driver = {
		.name = CDNS_XSPI_NAME,
		.of_match_table = cdns_xspi_of_match,
	},
};

module_platform_driver(cdns_xspi_platform_driver);

MODULE_DESCRIPTION("Cadence XSPI Controller Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" CDNS_XSPI_NAME);
MODULE_AUTHOR("Konrad Kociolek <konrad@cadence.com>");
