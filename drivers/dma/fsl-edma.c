// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * drivers/dma/fsl-edma.c
 *
 * Copyright 2013-2014 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * Driver for the Freescale eDMA engine with flexible channel multiplexing
 * capability for DMA request sources. The eDMA block can be found on some
 * Vybrid and Layerscape SoCs.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_dma.h>

#include "fsl-edma-common.h"

static irqreturn_t fsl_edma_tx_handler(int irq, void *dev_id)
{
	struct fsl_edma_engine *fsl_edma = dev_id;
	unsigned int intr, ch;
	struct edma_regs *regs = &fsl_edma->regs;
	struct fsl_edma_chan *fsl_chan;

	intr = edma_readl(fsl_edma, regs->intl);
	if (!intr)
		return IRQ_NONE;

	for (ch = 0; ch < fsl_edma->n_chans; ch++) {
		if (intr & (0x1 << ch)) {
			edma_writeb(fsl_edma, EDMA_CINT_CINT(ch), regs->cint);

			fsl_chan = &fsl_edma->chans[ch];

			spin_lock(&fsl_chan->vchan.lock);
			if (!fsl_chan->edesc->iscyclic) {
				fsl_edma_get_realcnt(fsl_chan);
				list_del(&fsl_chan->edesc->vdesc.node);
				vchan_cookie_complete(&fsl_chan->edesc->vdesc);
				fsl_chan->edesc = NULL;
				fsl_chan->status = DMA_COMPLETE;
				fsl_chan->idle = true;
			} else {
				vchan_cyclic_callback(&fsl_chan->edesc->vdesc);
			}

			if (!fsl_chan->edesc)
				fsl_edma_xfer_desc(fsl_chan);

			spin_unlock(&fsl_chan->vchan.lock);
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t fsl_edma_err_handler(int irq, void *dev_id)
{
	struct fsl_edma_engine *fsl_edma = dev_id;
	unsigned int err, ch;
	struct edma_regs *regs = &fsl_edma->regs;

	err = edma_readl(fsl_edma, regs->errl);
	if (!err)
		return IRQ_NONE;

	for (ch = 0; ch < fsl_edma->n_chans; ch++) {
		if (err & (0x1 << ch)) {
			dev_err(fsl_edma->dma_dev.dev, "DMA CH%d Err!\n", ch);
			fsl_edma_disable_request(&fsl_edma->chans[ch]);
			edma_writeb(fsl_edma, EDMA_CERR_CERR(ch), regs->cerr);
			fsl_edma->chans[ch].status = DMA_ERROR;
			fsl_edma->chans[ch].idle = true;
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t fsl_edma_irq_handler(int irq, void *dev_id)
{
	if (fsl_edma_tx_handler(irq, dev_id) == IRQ_HANDLED)
		return IRQ_HANDLED;

	return fsl_edma_err_handler(irq, dev_id);
}

static struct dma_chan *fsl_edma_xlate(struct of_phandle_args *dma_spec,
		struct of_dma *ofdma)
{
	struct fsl_edma_engine *fsl_edma = ofdma->of_dma_data;
	struct dma_chan *chan, *_chan;
	struct fsl_edma_chan *fsl_chan;
	unsigned long chans_per_mux = fsl_edma->n_chans / fsl_edma->dmamux_nr;

	if (dma_spec->args_count != 2)
		return NULL;

	mutex_lock(&fsl_edma->fsl_edma_mutex);
	list_for_each_entry_safe(chan, _chan, &fsl_edma->dma_dev.channels, device_node) {
		if (chan->client_count)
			continue;
		if ((chan->chan_id / chans_per_mux) == dma_spec->args[0]) {
			chan = dma_get_slave_channel(chan);
			if (chan) {
				chan->device->privatecnt++;
				fsl_chan = to_fsl_edma_chan(chan);
				fsl_chan->slave_id = dma_spec->args[1];
				fsl_edma_chan_mux(fsl_chan, fsl_chan->slave_id,
						true);
				mutex_unlock(&fsl_edma->fsl_edma_mutex);
				return chan;
			}
		}
	}
	mutex_unlock(&fsl_edma->fsl_edma_mutex);
	return NULL;
}

static int
fsl_edma_irq_init(struct platform_device *pdev, struct fsl_edma_engine *fsl_edma)
{
	int ret;

	fsl_edma->txirq = platform_get_irq_byname(pdev, "edma-tx");
	if (fsl_edma->txirq < 0) {
		dev_err(&pdev->dev, "Can't get edma-tx irq.\n");
		return fsl_edma->txirq;
	}

	fsl_edma->errirq = platform_get_irq_byname(pdev, "edma-err");
	if (fsl_edma->errirq < 0) {
		dev_err(&pdev->dev, "Can't get edma-err irq.\n");
		return fsl_edma->errirq;
	}

	if (fsl_edma->txirq == fsl_edma->errirq) {
		ret = devm_request_irq(&pdev->dev, fsl_edma->txirq,
				fsl_edma_irq_handler, 0, "eDMA", fsl_edma);
		if (ret) {
			dev_err(&pdev->dev, "Can't register eDMA IRQ.\n");
			return ret;
		}
	} else {
		ret = devm_request_irq(&pdev->dev, fsl_edma->txirq,
				fsl_edma_tx_handler, 0, "eDMA tx", fsl_edma);
		if (ret) {
			dev_err(&pdev->dev, "Can't register eDMA tx IRQ.\n");
			return ret;
		}

		ret = devm_request_irq(&pdev->dev, fsl_edma->errirq,
				fsl_edma_err_handler, 0, "eDMA err", fsl_edma);
		if (ret) {
			dev_err(&pdev->dev, "Can't register eDMA err IRQ.\n");
			return ret;
		}
	}

	return 0;
}

static void fsl_edma_irq_exit(
		struct platform_device *pdev, struct fsl_edma_engine *fsl_edma)
{
	if (fsl_edma->txirq == fsl_edma->errirq) {
		devm_free_irq(&pdev->dev, fsl_edma->txirq, fsl_edma);
	} else {
		devm_free_irq(&pdev->dev, fsl_edma->txirq, fsl_edma);
		devm_free_irq(&pdev->dev, fsl_edma->errirq, fsl_edma);
	}
}

static void fsl_disable_clocks(struct fsl_edma_engine *fsl_edma, int nr_clocks)
{
	int i;

	for (i = 0; i < fsl_edma->dmamux_nr; i++)
		clk_disable_unprepare(fsl_edma->muxclk[i]);

	if (fsl_edma->dmaclk)
		clk_disable_unprepare(fsl_edma->dmaclk);
}

static int
fsl_edma2_irq_init(struct platform_device *pdev,
		   struct fsl_edma_engine *fsl_edma)
{
	struct device_node *np = pdev->dev.of_node;
	int i, ret, irq;
	int count = 0;

	count = of_irq_count(np);
	dev_info(&pdev->dev, "%s Found %d interrupts\r\n", __func__, count);
	if(count < 2){
		dev_err(&pdev->dev, "Interrupts in DTS not correct.\n");
		return -EINVAL;
	}

	for (i = 0; i < count; i++) {
		irq = platform_get_irq(pdev, i);
		if (irq < 0)
			return -ENXIO;

		sprintf(fsl_edma->chans[i].chan_name, "eDMA2-CH%02d", i);

		/* The last IRQ is for eDMA err */
		if (i == count - 1)
			ret = devm_request_irq(&pdev->dev, irq,
						fsl_edma_err_handler,
						0, "eDMA2-ERR", fsl_edma);
		else

			ret = devm_request_irq(&pdev->dev, irq,
						fsl_edma_tx_handler, 0,
						fsl_edma->chans[i].chan_name,
						fsl_edma);
		if(ret)
			return ret;
	}

	return 0;
}

static struct platform_device_id fsl_edma_devtype[] = {
	{
		.name = "vf610-edma",
		.driver_data = 0,
	}, {
		.name = "imx7ulp-edma",
		.driver_data = FSL_EDMA_QUIRK_VLLS_MODE,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, fsl_edma_devtype);

enum fsl_edma_type {
	VF610_EDMA,
	IMX7ULP_EDMA,
};

static const struct of_device_id fsl_edma_dt_ids[] = {
	{
		.compatible = "fsl,vf610-edma",
		.data = &fsl_edma_devtype[VF610_EDMA],
	}, {
		.compatible = "nxp,imx7ulp-edma",
		.data = &fsl_edma_devtype[IMX7ULP_EDMA],
	}, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_edma_dt_ids);

static void fsl_edma_synchronize(struct dma_chan *chan)
{
	struct fsl_edma_chan *fsl_chan = to_fsl_edma_chan(chan);

	vchan_synchronize(&fsl_chan->vchan);
}

static int fsl_edma_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_edma_engine *fsl_edma;
	struct fsl_edma_chan *fsl_chan;
	const struct of_device_id *of_id;
	struct edma_regs *regs;
	struct resource *res;
	int len, chans;
	int ret, i;

	ret = of_property_read_u32(np, "dma-channels", &chans);
	if (ret) {
		dev_err(&pdev->dev, "Can't get dma-channels.\n");
		return ret;
	}

	len = sizeof(*fsl_edma) + sizeof(*fsl_chan) * chans;
	fsl_edma = devm_kzalloc(&pdev->dev, len, GFP_KERNEL);
	if (!fsl_edma)
		return -ENOMEM;

	of_id = of_match_device(fsl_edma_dt_ids, &pdev->dev);
	if (of_id)
		pdev->id_entry = of_id->data;
	fsl_edma->quirks = pdev->id_entry->driver_data;

	fsl_edma->version = v1;
	fsl_edma->n_chans = chans;
	mutex_init(&fsl_edma->fsl_edma_mutex);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	fsl_edma->membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(fsl_edma->membase))
		return PTR_ERR(fsl_edma->membase);

	fsl_edma_setup_regs(fsl_edma);
	regs = &fsl_edma->regs;

	fsl_edma->dmamux_nr = DMAMUX_NR;
	fsl_edma->mux_configure = mux_configure8;
	fsl_edma->version = 1;

	if (of_device_is_compatible(np, "nxp,imx7ulp-edma")) {
		fsl_edma->dmamux_nr = 1;
		fsl_edma->mux_configure = mux_configure32;
		fsl_edma->version = 2;

		fsl_edma->dmaclk = devm_clk_get(&pdev->dev, "dma");
		if (IS_ERR(fsl_edma->dmaclk)) {
			dev_err(&pdev->dev, "Missing DMA block clock.\n");
			return PTR_ERR(fsl_edma->dmaclk);
		}

		ret = clk_prepare_enable(fsl_edma->dmaclk);
		if (ret) {
			dev_err(&pdev->dev, "DMA clk block failed.\n");
			return ret;
		}
	}

	for (i = 0; i < fsl_edma->dmamux_nr; i++) {
		char clkname[32];

		res = platform_get_resource(pdev, IORESOURCE_MEM, 1 + i);
		fsl_edma->muxbase[i] = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(fsl_edma->muxbase[i])) {
			/* on error: disable all previously enabled clks */
			fsl_disable_clocks(fsl_edma, i);
			return PTR_ERR(fsl_edma->muxbase[i]);
		}

		sprintf(clkname, "dmamux%d", i);
		fsl_edma->muxclk[i] = devm_clk_get(&pdev->dev, clkname);
		if (IS_ERR(fsl_edma->muxclk[i])) {
			dev_err(&pdev->dev, "Missing DMAMUX block clock.\n");
			/* on error: disable all previously enabled clks */
			fsl_disable_clocks(fsl_edma, i);
			return PTR_ERR(fsl_edma->muxclk[i]);
		}

		ret = clk_prepare_enable(fsl_edma->muxclk[i]);
		if (ret)
			/* on error: disable all previously enabled clks */
			fsl_disable_clocks(fsl_edma, i);

	}

	edma_writel(fsl_edma, ~0, regs->intl);
	if (fsl_edma->version == 1)
		ret = fsl_edma_irq_init(pdev, fsl_edma);
	else
		ret = fsl_edma2_irq_init(pdev, fsl_edma);
	if (ret)
		return ret;

	fsl_edma->big_endian = of_property_read_bool(np, "big-endian");

	INIT_LIST_HEAD(&fsl_edma->dma_dev.channels);
	for (i = 0; i < fsl_edma->n_chans; i++) {
		struct fsl_edma_chan *fsl_chan = &fsl_edma->chans[i];

		fsl_chan->edma = fsl_edma;
		fsl_chan->pm_state = RUNNING;
		fsl_chan->slave_id = 0;
		fsl_chan->idle = true;
		fsl_chan->dma_dir = DMA_NONE;
		fsl_chan->vchan.desc_free = fsl_edma_free_desc;
		vchan_init(&fsl_chan->vchan, &fsl_edma->dma_dev);

		edma_writew(fsl_edma, 0x0, &regs->tcd[i].csr);
		fsl_chan->vchan.chan.chan_id = i;
		fsl_edma_chan_mux(fsl_chan, 0, false);
		fsl_chan->vchan.chan.chan_id = 0;
	}

	dma_cap_set(DMA_PRIVATE, fsl_edma->dma_dev.cap_mask);
	dma_cap_set(DMA_SLAVE, fsl_edma->dma_dev.cap_mask);
	dma_cap_set(DMA_CYCLIC, fsl_edma->dma_dev.cap_mask);

	fsl_edma->dma_dev.dev = &pdev->dev;
	fsl_edma->dma_dev.device_alloc_chan_resources
		= fsl_edma_alloc_chan_resources;
	fsl_edma->dma_dev.device_free_chan_resources
		= fsl_edma_free_chan_resources;
	fsl_edma->dma_dev.device_tx_status = fsl_edma_tx_status;
	fsl_edma->dma_dev.device_prep_slave_sg = fsl_edma_prep_slave_sg;
	fsl_edma->dma_dev.device_prep_dma_cyclic = fsl_edma_prep_dma_cyclic;
	fsl_edma->dma_dev.device_config = fsl_edma_slave_config;
	fsl_edma->dma_dev.device_pause = fsl_edma_device_pause;
	fsl_edma->dma_dev.device_resume = fsl_edma_device_resume;
	fsl_edma->dma_dev.device_terminate_all = fsl_edma_terminate_all;
	fsl_edma->dma_dev.device_issue_pending = fsl_edma_issue_pending;
	fsl_edma->dma_dev.device_synchronize = fsl_edma_synchronize;

	fsl_edma->dma_dev.src_addr_widths = FSL_EDMA_BUSWIDTHS;
	fsl_edma->dma_dev.dst_addr_widths = FSL_EDMA_BUSWIDTHS;
	fsl_edma->dma_dev.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);

	platform_set_drvdata(pdev, fsl_edma);

	ret = dma_async_device_register(&fsl_edma->dma_dev);
	if (ret) {
		dev_err(&pdev->dev,
			"Can't register Freescale eDMA engine. (%d)\n", ret);
		fsl_disable_clocks(fsl_edma, DMAMUX_NR);
		return ret;
	}

	ret = of_dma_controller_register(np, fsl_edma_xlate, fsl_edma);
	if (ret) {
		dev_err(&pdev->dev,
			"Can't register Freescale eDMA of_dma. (%d)\n", ret);
		dma_async_device_unregister(&fsl_edma->dma_dev);
		fsl_disable_clocks(fsl_edma, DMAMUX_NR);
		return ret;
	}

	/* enable round robin arbitration */
	edma_writel(fsl_edma, EDMA_CR_ERGA | EDMA_CR_ERCA, regs->cr);

	return 0;
}

static int fsl_edma_remove(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_edma_engine *fsl_edma = platform_get_drvdata(pdev);

	fsl_edma_irq_exit(pdev, fsl_edma);
	fsl_edma_cleanup_vchan(&fsl_edma->dma_dev);
	of_dma_controller_free(np);
	dma_async_device_unregister(&fsl_edma->dma_dev);
	fsl_disable_clocks(fsl_edma, DMAMUX_NR);

	return 0;
}

static int fsl_edma_register_save(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct fsl_edma_engine *fsl_edma = platform_get_drvdata(pdev);
	int i, j;

	if (!(fsl_edma->quirks & FSL_EDMA_QUIRK_VLLS_MODE))
		return 0;

	/* save regs */
	fsl_edma->edma_regs[0] =
		edma_readl(fsl_edma, fsl_edma->membase + EDMA_CR);
	fsl_edma->edma_regs[1] =
		edma_readl(fsl_edma, fsl_edma->membase + EDMA_ERQ);
	fsl_edma->edma_regs[2] =
		edma_readl(fsl_edma, fsl_edma->membase + EDMA_EEI);
	for (i = 0; i < fsl_edma->dmamux_nr; i++)
		for (j = 0; j < fsl_edma->n_chans; j++)
			fsl_edma->dmamux_regs[i * fsl_edma->n_chans + j] =
				edma_readl(fsl_edma,
					fsl_edma->muxbase[i] + j * 4);

	return 0;
}

static int fsl_edma_register_restore(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct fsl_edma_engine *fsl_edma = platform_get_drvdata(pdev);
	int i, j;

	if (!(fsl_edma->quirks & FSL_EDMA_QUIRK_VLLS_MODE))
		return 0;

	/* restore the regs */
	for (i = 0; i < fsl_edma->dmamux_nr; i++)
		for (j = 0; j < fsl_edma->n_chans; j++)
			edma_writel(fsl_edma,
			  fsl_edma->dmamux_regs[i * fsl_edma->n_chans + j],
			  fsl_edma->muxbase[i] + j * 4);
	edma_writel(fsl_edma, fsl_edma->edma_regs[1],
			fsl_edma->membase + EDMA_ERQ);
	edma_writel(fsl_edma, fsl_edma->edma_regs[2],
			fsl_edma->membase + EDMA_EEI);
	edma_writel(fsl_edma, fsl_edma->edma_regs[0],
			fsl_edma->membase + EDMA_CR);

	return 0;
}

static int fsl_edma_suspend_late(struct device *dev)
{
	struct fsl_edma_engine *fsl_edma = dev_get_drvdata(dev);
	struct fsl_edma_chan *fsl_chan;
	unsigned long flags;
	int i;

	for (i = 0; i < fsl_edma->n_chans; i++) {
		fsl_chan = &fsl_edma->chans[i];
		spin_lock_irqsave(&fsl_chan->vchan.lock, flags);
		/* Make sure chan is idle or will force disable. */
		if (unlikely(!fsl_chan->idle)) {
			dev_warn(dev, "WARN: There is non-idle channel.");
			fsl_edma_disable_request(fsl_chan);
			fsl_edma_chan_mux(fsl_chan, 0, false);
		}

		fsl_chan->pm_state = SUSPENDED;
		spin_unlock_irqrestore(&fsl_chan->vchan.lock, flags);
	}

	fsl_edma_register_save(dev);

	return 0;
}

static int fsl_edma_resume_early(struct device *dev)
{
	struct fsl_edma_engine *fsl_edma = dev_get_drvdata(dev);
	struct fsl_edma_chan *fsl_chan;
	struct edma_regs *regs = &fsl_edma->regs;
	int i;

	fsl_edma_register_restore(dev);

	for (i = 0; i < fsl_edma->n_chans; i++) {
		fsl_chan = &fsl_edma->chans[i];
		fsl_chan->pm_state = RUNNING;
		edma_writew(fsl_edma, 0x0, &regs->tcd[i].csr);
		if (fsl_chan->slave_id != 0)
			fsl_edma_chan_mux(fsl_chan, fsl_chan->slave_id, true);
	}

	edma_writel(fsl_edma, EDMA_CR_ERGA | EDMA_CR_ERCA, regs->cr);

	return 0;
}

/*
 * eDMA provides the service to others, so it should be suspend late
 * and resume early. When eDMA suspend, all of the clients should stop
 * the DMA data transmission and let the channel idle.
 */
static const struct dev_pm_ops fsl_edma_pm_ops = {
	.suspend_late   = fsl_edma_suspend_late,
	.resume_early   = fsl_edma_resume_early,
};

static struct platform_driver fsl_edma_driver = {
	.driver		= {
		.name	= "fsl-edma",
		.of_match_table = fsl_edma_dt_ids,
		.pm     = &fsl_edma_pm_ops,
	},
	.id_table	= fsl_edma_devtype,
	.probe          = fsl_edma_probe,
	.remove		= fsl_edma_remove,
};

static int __init fsl_edma_init(void)
{
	return platform_driver_register(&fsl_edma_driver);
}
subsys_initcall(fsl_edma_init);

static void __exit fsl_edma_exit(void)
{
	platform_driver_unregister(&fsl_edma_driver);
}
module_exit(fsl_edma_exit);

MODULE_ALIAS("platform:fsl-edma");
MODULE_DESCRIPTION("Freescale eDMA engine driver");
MODULE_LICENSE("GPL v2");
