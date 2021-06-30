// SPDX-License-Identifier: GPL-2.0+
/*
 * Cadence XSPI flash controller driver
 *
 * Copyright (C) 2020 Cadence
 *
 * Author: Konrad Kociolek <konrad@cadence.com>
 */
#include "spi-cadence-xspi.h"

static void cdns_xspi_controller_reset(struct cdns_xspi_dev *cdns_xspi)
{
	u32 driving_reg = 0;

	driving_reg = readl(cdns_xspi->auxbase + CDNS_XSPI_AUX_DRIVING_REG);
	driving_reg |= CDNS_XSPI_AUX_CTRL_RESET;
	writel(driving_reg, cdns_xspi->auxbase + CDNS_XSPI_AUX_DRIVING_REG);

	udelay(10);

	driving_reg &= ~CDNS_XSPI_AUX_CTRL_RESET;
	writel(driving_reg, cdns_xspi->auxbase + CDNS_XSPI_AUX_DRIVING_REG);
}

static int cdns_xspi_read_dqs_delay_training(struct cdns_xspi_dev *cdns_xspi)
{
	int rd_dqs_del;
	int rd_dqs_del_min = -1;
	int rd_dqs_del_max = -1;

	u32 phy_dll_slave_ctrl = 0;
	u32 ctrl_status = 0;

	phy_dll_slave_ctrl = readl(cdns_xspi->auxbase +
		CDNS_XSPI_CCP_PHY_DLL_SLAVE_CTRL);

	dev_info(cdns_xspi->dev,
		"Running PHY training for read_dqs_delay parameter\n");

	for (rd_dqs_del = 0; rd_dqs_del < U8_MAX; rd_dqs_del++) {
		phy_dll_slave_ctrl &= ~CDNS_XSPI_CCP_READ_DQS_DELAY;
		phy_dll_slave_ctrl |= FIELD_PREP(CDNS_XSPI_CCP_READ_DQS_DELAY,
			rd_dqs_del);

		writel(phy_dll_slave_ctrl,
			cdns_xspi->auxbase + CDNS_XSPI_CCP_PHY_DLL_SLAVE_CTRL);

		cdns_xspi_controller_reset(cdns_xspi);

		readl_relaxed_poll_timeout(cdns_xspi->iobase +
			CDNS_XSPI_CTRL_STATUS_REG, ctrl_status,
			(ctrl_status & CDNS_XSPI_INIT_COMPLETED), 10, 10000);

		if (!(ctrl_status & CDNS_XSPI_INIT_COMPLETED) ||
			(ctrl_status & CDNS_XSPI_INIT_FAIL)) {
			if (rd_dqs_del_min != -1)
				rd_dqs_del_max = rd_dqs_del - 1;
		} else {
			if (rd_dqs_del_min == -1)
				rd_dqs_del_min = rd_dqs_del;
		}
	}

	if (rd_dqs_del_min == -1) {
		if (cdns_xspi->skip_sim_check)
			return 0;
		dev_err(cdns_xspi->dev, "PHY training failed\n");
		return -EBUSY;
	} else if (rd_dqs_del_max == -1) {
		rd_dqs_del_max = U8_MAX;
	}

	rd_dqs_del = rd_dqs_del_min + rd_dqs_del_max / 2;
	dev_info(cdns_xspi->dev,
		"Using optimal read_dqs_delay value: %d\n", rd_dqs_del);

	phy_dll_slave_ctrl &= ~CDNS_XSPI_CCP_READ_DQS_DELAY;
	phy_dll_slave_ctrl |= FIELD_PREP(CDNS_XSPI_CCP_READ_DQS_DELAY,
		rd_dqs_del);

	writel(phy_dll_slave_ctrl,
		cdns_xspi->auxbase + CDNS_XSPI_CCP_PHY_DLL_SLAVE_CTRL);

	return 0;
}

static int cdns_xspi_phy_init(struct cdns_xspi_dev *cdns_xspi)
{
	u32 xspi_dll_phy_ctrl = 0;
	u32 phy_dq_timing = CDNS_XSPI_CCP_PHY_DQ_TIMING_INIT_VAL;
	u32 phy_dqs_timing = 0;
	u32 phy_gate_lpbck_ctrl = 0;
	u32 phy_dll_slave_ctrl = 0;

	if (cdns_xspi->plat_data->use_lpbk_dqs) {
		phy_dqs_timing |= FIELD_PREP(CDNS_XSPI_CCP_USE_LPBCK_DQS, 1);

		/*
		 * For XSPI protocol, phony_dqs and lpbk_dqs must
		 * have same value
		 */
		phy_dqs_timing |= FIELD_PREP(CDNS_XSPI_CCP_USE_PHONY, 1);

		if (cdns_xspi->plat_data->use_ext_lpbk_dqs)
			phy_dqs_timing |=
				FIELD_PREP(CDNS_XSPI_CCP_USE_EXT_LPBCK_DQS, 1);
	}

	xspi_dll_phy_ctrl = readl(cdns_xspi->auxbase + CDNS_XSPI_DLL_PHY_CTRL);

	/* While using memory DQS last_data_drop parameter should be enabled */
	if (cdns_xspi->plat_data->dqs_last_data_drop)
		xspi_dll_phy_ctrl |=
			FIELD_PREP(CDNS_XSPI_CCP_DQS_LAST_DATA_DROP_EN, 1);

	phy_dq_timing |= FIELD_PREP(CDNS_XSPI_CCP_DATA_SELECT_OE_START,
		cdns_xspi->plat_data->phy_data_sel_oe_start);
	phy_dq_timing |= FIELD_PREP(CDNS_XSPI_CCP_DATA_SELECT_OE_END,
		cdns_xspi->plat_data->phy_data_sel_oe_end);

	phy_dqs_timing |= FIELD_PREP(CDNS_XSPI_CCP_DQS_SELECT_OE_START,
		cdns_xspi->plat_data->phy_dqs_sel_oe_start);
	phy_dqs_timing |= FIELD_PREP(CDNS_XSPI_CCP_DQS_SELECT_OE_END,
		cdns_xspi->plat_data->phy_dqs_sel_oe_end);

	phy_gate_lpbck_ctrl |= FIELD_PREP(CDNS_XSPI_CCP_GATE_CFG_CLOSE,
		cdns_xspi->plat_data->phy_gate_cfg_close);
	phy_gate_lpbck_ctrl |= FIELD_PREP(CDNS_XSPI_CCP_GATE_CFG,
		cdns_xspi->plat_data->phy_gate_cfg);
	phy_gate_lpbck_ctrl |= FIELD_PREP(CDNS_XSPI_CCP_READ_DATA_DELAY_SEL,
		cdns_xspi->plat_data->phy_rd_del_sel);

	phy_dll_slave_ctrl |= FIELD_PREP(CDNS_XSPI_CCP_CLK_WR_DELAY,
		cdns_xspi->plat_data->clk_wr_delay);

	writel(xspi_dll_phy_ctrl,
		cdns_xspi->auxbase + CDNS_XSPI_DLL_PHY_CTRL);
	writel(phy_dq_timing,
		cdns_xspi->auxbase + CDNS_XSPI_CCP_PHY_DQ_TIMING);
	writel(phy_dqs_timing,
		cdns_xspi->auxbase + CDNS_XSPI_CCP_PHY_DQS_TIMING);
	writel(phy_gate_lpbck_ctrl,
		cdns_xspi->auxbase + CDNS_XSPI_CCP_PHY_GATE_LPBCK_CTRL);
	writel(phy_dll_slave_ctrl,
		cdns_xspi->auxbase + CDNS_XSPI_CCP_PHY_DLL_SLAVE_CTRL);

	writel(CDNS_XSPI_AUX_PHY_ADDONS_VALUE,
		cdns_xspi->auxbase + CDNS_XSPI_AUX_PHY_ADDONS_REG);
	writel(CDNS_XSPI_AUX_DEV_DISC_CONFIG_VALUE,
		cdns_xspi->auxbase + CDNS_XSPI_AUX_DEV_DISC_CONFIG_REG);

	return cdns_xspi_read_dqs_delay_training(cdns_xspi);
}

static int cdns_xspi_wait_for_controller_idle(struct cdns_xspi_dev *cdns_xspi)
{
	u32 ctrl_stat;

	return readl_relaxed_poll_timeout(cdns_xspi->iobase +
		CDNS_XSPI_CTRL_STATUS_REG,
		ctrl_stat, ((ctrl_stat & CDNS_XSPI_CTRL_BUSY) == 0), 100, 1000);
}

static void cdns_xspi_trigger_command(struct cdns_xspi_dev *cdns_xspi,
	u32 cmd_regs[5])
{
	writel(cmd_regs[5], cdns_xspi->iobase + CDNS_XSPI_CMD_REG_5);
	writel(cmd_regs[4], cdns_xspi->iobase + CDNS_XSPI_CMD_REG_4);
	writel(cmd_regs[3], cdns_xspi->iobase + CDNS_XSPI_CMD_REG_3);
	writel(cmd_regs[2], cdns_xspi->iobase + CDNS_XSPI_CMD_REG_2);
	writel(cmd_regs[1], cdns_xspi->iobase + CDNS_XSPI_CMD_REG_1);
	writel(cmd_regs[0], cdns_xspi->iobase + CDNS_XSPI_CMD_REG_0);
}

static int cdns_xspi_check_command_status(struct cdns_xspi_dev *cdns_xspi)
{
	int ret = 0;
	u32 cmd_status = readl(cdns_xspi->iobase + CDNS_XSPI_CMD_STATUS_REG);

	if (cmd_status & CDNS_XSPI_CMD_STATUS_COMPLETED) {
		if ((cmd_status & CDNS_XSPI_CMD_STATUS_FAILED) != 0) {
			if (cmd_status & CDNS_XSPI_CMD_STATUS_DQS_ERROR) {
				dev_err(cdns_xspi->dev,
					"Incorrect DQS pulses were detected during data read operation\n");
				ret = -EPROTO;
			}
			if (cmd_status & CDNS_XSPI_CMD_STATUS_CRC_ERROR) {
				dev_err(cdns_xspi->dev,
					"CRC error received from minicontroller\n");
				ret = -EPROTO;
			}
			if (cmd_status & CDNS_XSPI_CMD_STATUS_BUS_ERROR) {
				dev_err(cdns_xspi->dev,
					"Controller got an error response on the system DMA interface\n");
				ret = -EPROTO;
			}
			if (cmd_status & CDNS_XSPI_CMD_STATUS_INV_SEQ_ERROR) {
				dev_err(cdns_xspi->dev,
					"Invalid command sequence has been detected\n");
				ret = -EPROTO;
			}
		}
	} else {
		if (cdns_xspi->skip_sim_check)
			return 0;
		dev_err(cdns_xspi->dev, "Fatal error - command not completed\n");
		ret = -EPROTO;
	}

	return ret;
}

static void cdns_xspi_set_interrupts(struct cdns_xspi_dev *cdns_xspi,
	bool enabled)
{
	u32 intr_enable;

	if (!cdns_xspi->irq)
		return;

	intr_enable = readl(cdns_xspi->iobase + CDNS_XSPI_INTR_ENABLE_REG);
	if (enabled)
		intr_enable |= CDNS_XSPI_INTR_MASK;
	else
		intr_enable &= ~CDNS_XSPI_INTR_MASK;
	writel(intr_enable, cdns_xspi->iobase + CDNS_XSPI_INTR_ENABLE_REG);
}

static int cdns_xspi_controller_init(struct cdns_xspi_dev *cdns_xspi)
{
	u32 ctrl_ver;
	u32 ctrl_features;
	u16 hw_magic_num;

	ctrl_ver = readl(cdns_xspi->iobase + CDNS_XSPI_CTRL_VERSION_REG);
	hw_magic_num = FIELD_GET(CDNS_XSPI_MAGIC_NUM, ctrl_ver);
	if (hw_magic_num != CDNS_XSPI_MAGIC_NUM_VALUE) {
		dev_err(cdns_xspi->dev,
			"Incorrect XSPI magic nunber: %x, expected: %x\n",
			hw_magic_num, CDNS_XSPI_MAGIC_NUM_VALUE);
		return -EIO;
	}

	ctrl_features = readl(cdns_xspi->iobase + CDNS_XSPI_CTRL_FEATURES_REG);
	cdns_xspi->hw_num_banks = FIELD_GET(CDNS_XSPI_NUM_BANKS, ctrl_features);

	writel(FIELD_PREP(CDNS_XSPI_CTRL_WORK_MODE,
		CDNS_XSPI_CTRL_WORK_MODE_STIG),
		cdns_xspi->iobase + CDNS_XSPI_CTRL_CONFIG_REG);

	cdns_xspi_set_interrupts(cdns_xspi, false);

	return 0;
}

static void cdns_xspi_sdma_handle(struct cdns_xspi_dev *cdns_xspi)
{
	u32 sdma_size, sdma_trd_info;
	u8 sdma_dir;

	sdma_size = readl(cdns_xspi->iobase + CDNS_XSPI_SDMA_SIZE_REG);
	sdma_trd_info = readl(cdns_xspi->iobase + CDNS_XSPI_SDMA_TRD_INFO_REG);
	sdma_dir = FIELD_GET(CDNS_XSPI_SDMA_DIR, sdma_trd_info);

	switch (sdma_dir) {
	case CDNS_XSPI_SDMA_DIR_READ:
		ioread8_rep(cdns_xspi->sdmabase,
			cdns_xspi->in_buffer, sdma_size);
		break;

	case CDNS_XSPI_SDMA_DIR_WRITE:
		iowrite8_rep(cdns_xspi->sdmabase,
			cdns_xspi->out_buffer, sdma_size);
		break;
	}
}

bool cdns_xspi_stig_ready(struct cdns_xspi_dev *cdns_xspi)
{
	u32 ctrl_stat;

	readl_relaxed_poll_timeout
		(cdns_xspi->iobase + CDNS_XSPI_CTRL_STATUS_REG,
		ctrl_stat,
		((ctrl_stat & BIT(3)) == 0),
		10,
		1000);

	return true;
}

bool cdns_xspi_sdma_ready(struct cdns_xspi_dev *cdns_xspi)
{
	u32 ctrl_stat;

	readl_relaxed_poll_timeout
		(cdns_xspi->iobase + CDNS_XSPI_CTRL_STATUS_REG,
		ctrl_stat,
		((ctrl_stat & BIT(0)) == 0),
		10,
		1000);

	return true;
}

static int cdns_verify_stig_mode_config(struct cdns_xspi_dev *cdns_xspi)
{
	int cntrl = readl(cdns_xspi->iobase + CDNS_XSPI_CTRL_CONFIG_REG);

	if (FIELD_GET(CDNS_XSPI_CTRL_WORK_MODE, cntrl) != CDNS_XSPI_CTRL_WORK_MODE_STIG)
		return cdns_xspi_controller_init(cdns_xspi);

	return 0;
}

#define OS_OWN	2
static int cdns_xspi_lock_device(struct cdns_xspi_dev *cdns_xspi)
{
	u32 value;

	if (!readl_poll_timeout_atomic(cdns_xspi->iobase + CDNS_XSPI_PHY_GPIO_CTRL_1,
					value, !value, 100, 1000000))
		writel(OS_OWN, cdns_xspi->iobase + 0x208c);
	else {
		dev_err(cdns_xspi->dev, "Cannot get Lock\n");
		return -ETIMEDOUT;
	}

	if (!readl_poll_timeout_atomic(cdns_xspi->iobase + CDNS_XSPI_PHY_GPIO_CTRL_1,
					value, value != OS_OWN, 100, 1000)) {
		dev_err(cdns_xspi->dev, "Cannot keep Lock\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int cdns_xspi_unlock_device(struct cdns_xspi_dev *cdns_xspi)
{
	u32 value;

	if (!readl_poll_timeout_atomic(cdns_xspi->iobase + CDNS_XSPI_PHY_GPIO_CTRL_1,
					value, (value == OS_OWN), 100, 1000)) {
		writel(0, cdns_xspi->iobase + CDNS_XSPI_PHY_GPIO_CTRL_1);
	} else {
		dev_err(cdns_xspi->dev, "Cannot UN Lock\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int cdns_xspi_send_stig_command(struct cdns_xspi_dev *cdns_xspi,
	const struct spi_mem_op *op, bool data_phase)
{
	struct device *dev = cdns_xspi->dev;
	u32 cmd_regs[5] = {0};
	u32 cmd_status;
	int status = 0;

	if (cdns_xspi_lock_device(cdns_xspi))
		return -EPROTO;

	cdns_xspi_wait_for_controller_idle(cdns_xspi);
	if (cdns_verify_stig_mode_config(cdns_xspi) < 0) {
		dev_err(dev, "Failed to enter STIG mode\n");
		status = -EPROTO;
		goto error;
	}
	cdns_xspi_set_interrupts(cdns_xspi, true);
	cdns_xspi->sdma_error = false;

	cmd_regs[1] = CDNS_XSPI_CMD_FLD_P1_INSTR_CMD_1(op, data_phase);
	cmd_regs[2] = CDNS_XSPI_CMD_FLD_P1_INSTR_CMD_2(op);
	cmd_regs[3] = CDNS_XSPI_CMD_FLD_P1_INSTR_CMD_3(op);
	cmd_regs[4] = CDNS_XSPI_CMD_FLD_P1_INSTR_CMD_4(op,
		cdns_xspi->current_cs);

	cdns_xspi_trigger_command(cdns_xspi, cmd_regs);

	if (data_phase) {

		cmd_regs[0] = CDNS_XSPI_STIG_DONE_FLAG;

		cmd_regs[1] = CDNS_XSPI_CMD_FLD_DSEQ_CMD_1(op);
		cmd_regs[2] = CDNS_XSPI_CMD_FLD_DSEQ_CMD_2(op);
		cmd_regs[3] = CDNS_XSPI_CMD_FLD_DSEQ_CMD_3(op);
		cmd_regs[4] = CDNS_XSPI_CMD_FLD_DSEQ_CMD_4(op,
			cdns_xspi->current_cs);

		cdns_xspi->in_buffer = op->data.buf.in;
		cdns_xspi->out_buffer = op->data.buf.out;

		cdns_xspi_trigger_command(cdns_xspi, cmd_regs);

		if (cdns_xspi->irq) {
			wait_for_completion(&cdns_xspi->sdma_complete);
			if (cdns_xspi->sdma_error) {
				cdns_xspi_set_interrupts(cdns_xspi, false);
				status = -EIO;
				goto error;
			}
		} else {
			cdns_xspi_sdma_ready(cdns_xspi);
		}
		cdns_xspi_sdma_handle(cdns_xspi);
	}

	if (cdns_xspi->irq)
		wait_for_completion(&cdns_xspi->cmd_complete);
	else
		cdns_xspi_stig_ready(cdns_xspi);

	cmd_status = cdns_xspi_check_command_status(cdns_xspi);
	cdns_xspi_set_interrupts(cdns_xspi, false);

	if (cmd_status & CDNS_XSPI_CMD_STATUS_FAILED)
		status = -EPROTO;

error:
	if (cdns_xspi_unlock_device(cdns_xspi))
		status = -EPROTO;

	return status;
}

static int cdns_xspi_mem_op(struct cdns_xspi_dev *cdns_xspi,
	struct spi_mem *mem, const struct spi_mem_op *op)
{
	if (cdns_xspi->current_cs != mem->spi->chip_select)
		cdns_xspi->current_cs = mem->spi->chip_select;

	return cdns_xspi_send_stig_command(cdns_xspi, op,
		(op->data.dir != SPI_MEM_NO_DATA));
}

static int cdns_xspi_mem_op_execute(struct spi_mem *mem,
	const struct spi_mem_op *op)
{
	struct cdns_xspi_dev *cdns_xspi =
		spi_master_get_devdata(mem->spi->master);
	int ret = 0;

	mutex_lock(&cdns_xspi->lock);
	ret = cdns_xspi_mem_op(cdns_xspi, mem, op);
	mutex_unlock(&cdns_xspi->lock);

	return ret;
}

static const struct spi_controller_mem_ops cadence_xspi_mem_ops = {
	.exec_op = cdns_xspi_mem_op_execute,
};

static int cdns_xspi_setup(struct spi_device *spi_dev)
{
	if (spi_dev->chip_select > spi_dev->master->num_chipselect) {
		dev_err(&spi_dev->dev,
			"%d chip-select is out of range\n",
			spi_dev->chip_select);
		return -EINVAL;
	}

	return 0;
}

irqreturn_t cdns_xspi_irq_handler(int this_irq, void *dev)
{
	struct cdns_xspi_dev *cdns_xspi = dev;
	u32 irq_status;
	irqreturn_t result = IRQ_NONE;

	irq_status = readl(cdns_xspi->iobase + CDNS_XSPI_INTR_STATUS_REG);
	if (irq_status) {
		writel(irq_status,
			cdns_xspi->iobase + CDNS_XSPI_INTR_STATUS_REG);

		if (irq_status & CDNS_XSPI_SDMA_ERROR) {
			dev_err(cdns_xspi->dev, "Slave DMA transaction error\n");
			cdns_xspi->sdma_error = true;
			complete(&cdns_xspi->sdma_complete);
		}

		if (irq_status & CDNS_XSPI_SDMA_TRIGGER)
			complete(&cdns_xspi->sdma_complete);

		if (irq_status & CDNS_XSPI_STIG_DONE)
			complete(&cdns_xspi->cmd_complete);

		result = IRQ_HANDLED;
	}

	return result;
}

int cdns_xspi_of_get_plat_data(struct device *dev)
{
	struct device_node *node_prop = dev->of_node;
	struct device_node *node_child;
	struct cdns_xspi_platform_data *plat_data = dev->platform_data;
	unsigned int property;
	unsigned int cs;

	if (of_property_read_u32(node_prop,
		"cdns,phy-data-select-oe-start", &property)) {
		dev_err(dev, "Couldn't determine data select oe start\n");
		return -ENXIO;
	}
	plat_data->phy_data_sel_oe_start = property;

	if (of_property_read_u32(node_prop,
		"cdns,phy-data-select-oe-end", &property)) {
		dev_err(dev, "Couldn't determine data select oe end\n");
		return -ENXIO;
	}
	plat_data->phy_data_sel_oe_end = property;

	if (of_property_read_u32(node_prop,
		"cdns,phy-dqs-select-oe-start", &property)) {
		dev_err(dev, "Couldn't determine DQS select oe start\n");
		return -ENXIO;
	}
	plat_data->phy_dqs_sel_oe_start = property;

	if (of_property_read_u32(node_prop,
		"cdns,phy-dqs-select-oe-end", &property)) {
		dev_err(dev, "Couldn't determine DQS select oe end\n");
		return -ENXIO;
	}
	plat_data->phy_dqs_sel_oe_end = property;

	if (of_property_read_u32(node_prop,
		"cdns,phy-gate-cfg-close", &property)) {
		dev_err(dev, "Couldn't determine gate config close\n");
		return -ENXIO;
	}
	plat_data->phy_gate_cfg_close = property;

	if (of_property_read_u32(node_prop,
		"cdns,phy-gate-cfg", &property)) {
		dev_err(dev, "Couldn't determine gate config\n");
		return -ENXIO;
	}
	plat_data->phy_gate_cfg = property;

	if (of_property_read_u32(node_prop,
		"cdns,phy-rd-del-select", &property)) {
		dev_err(dev, "Couldn't determine read delay select\n");
		return -ENXIO;
	}
	plat_data->phy_rd_del_sel = property;

	if (of_property_read_u32(node_prop,
		"cdns,phy-clk-wr-delay", &property)) {
		dev_err(dev, "Couldn't determine clock write delay\n");
		return -ENXIO;
	}
	plat_data->clk_wr_delay = property;

	plat_data->dqs_last_data_drop = of_property_read_bool(node_prop,
		"cdns,dqs-last-data-drop");
	plat_data->use_lpbk_dqs = of_property_read_bool(node_prop,
		"cdns,phy-use-lpbk-dqs");
	plat_data->use_ext_lpbk_dqs = of_property_read_bool(node_prop,
		"cdns,phy-use-ext-lpbk-dqs");

	for_each_child_of_node(node_prop, node_child) {
		if (!of_device_is_available(node_child))
			continue;

		if (of_property_read_u32(node_child, "reg", &cs)) {
			dev_err(dev, "Couldn't get memory chip select\n");
			return -ENXIO;
		} else if (cs >= CDNS_XSPI_MAX_BANKS) {
			dev_err(dev, "reg (cs) parameter value too large\n");
			return -ENXIO;
		}
	}

	return 0;
}

static void cdns_xspi_print_phy_config(struct cdns_xspi_dev *cdns_xspi)
{
	struct device *dev = cdns_xspi->dev;

	dev_info(dev, "PHY configuration\n");
	dev_info(dev, "   * xspi_dll_phy_ctrl: %08x\n",
		readl(cdns_xspi->iobase + CDNS_XSPI_DLL_PHY_CTRL));
	dev_info(dev, "   * phy_dq_timing: %08x\n",
		readl(cdns_xspi->auxbase + CDNS_XSPI_CCP_PHY_DQ_TIMING));
	dev_info(dev, "   * phy_dqs_timing: %08x\n",
		readl(cdns_xspi->auxbase + CDNS_XSPI_CCP_PHY_DQS_TIMING));
	dev_info(dev, "   * phy_gate_loopback_ctrl: %08x\n",
		readl(cdns_xspi->auxbase + CDNS_XSPI_CCP_PHY_GATE_LPBCK_CTRL));
	dev_info(dev, "   * phy_dll_slave_ctrl: %08x\n",
		readl(cdns_xspi->auxbase + CDNS_XSPI_CCP_PHY_DLL_SLAVE_CTRL));
}

int cdns_xspi_configure(struct cdns_xspi_dev *cdns_xspi)
{
	int ret;
	struct device *dev = cdns_xspi->dev;

	ret = cdns_xspi_phy_init(cdns_xspi);
	if (ret) {
		dev_err(dev, "Failed to initialize PHY\n");
		return ret;
	}

	cdns_xspi_print_phy_config(cdns_xspi);

	ret = cdns_xspi_controller_init(cdns_xspi);
	if (ret) {
		dev_err(dev, "Failed to initialize controller\n");
		return ret;
	}

	mutex_init(&cdns_xspi->lock);

	return ret;
}

struct spi_master *cdns_xspi_prepare_master(struct device *dev)
{
	struct spi_master *master = NULL;

	master = spi_alloc_master(dev, sizeof(struct cdns_xspi_dev));
	if (!master) {
		dev_err(dev, "Failed to allocate memory for spi_master\n");
		return NULL;
	}

	master->mode_bits = SPI_3WIRE | SPI_TX_DUAL  | SPI_TX_QUAD  |
		SPI_RX_DUAL | SPI_RX_QUAD | SPI_TX_OCTAL | SPI_RX_OCTAL |
		SPI_MODE_0  | SPI_MODE_3;

	master->mem_ops = &cadence_xspi_mem_ops;
	master->setup = cdns_xspi_setup;
	master->dev.of_node = dev->of_node;
	master->bus_num = -1;

	return master;
}
