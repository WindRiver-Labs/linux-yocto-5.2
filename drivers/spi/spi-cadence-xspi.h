/* SPDX-License-Identifier: GPL-2.0+ */
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/mtd/spi-nor.h>
#include <linux/bitfield.h>
#include <linux/limits.h>
#include <linux/log2.h>

#define CDNS_XSPI_MAGIC_NUM_VALUE               0x6522
#define CDNS_XSPI_MAX_BANKS                     8
#define CDNS_XSPI_NAME                          "cadence-xspi"

/*
 * Note: below are additional auxiliary registers to
 * configure XSPI controller pin-strap settings
 */

/* PHY DQ timing register */
#define CDNS_XSPI_CCP_PHY_DQ_TIMING           0x0000
#define CDNS_XSPI_CCP_DATA_SELECT_OE_START    GENMASK(6, 4)
#define CDNS_XSPI_CCP_DATA_SELECT_OE_END      GENMASK(2, 0)
#define CDNS_XSPI_CCP_PHY_DQ_TIMING_INIT_VAL  0x80000000

/* PHY DQS timing register */
#define CDNS_XSPI_CCP_PHY_DQS_TIMING          0x0004
#define CDNS_XSPI_CCP_USE_EXT_LPBCK_DQS       BIT(22)
#define CDNS_XSPI_CCP_USE_LPBCK_DQS           BIT(21)
#define CDNS_XSPI_CCP_USE_PHONY               BIT(20)
#define CDNS_XSPI_CCP_DQS_SELECT_OE_START     GENMASK(7, 4)
#define CDNS_XSPI_CCP_DQS_SELECT_OE_END       GENMASK(3, 0)

/* PHY gate loopback control register */
#define CDNS_XSPI_CCP_PHY_GATE_LPBCK_CTRL     0x0008
#define CDNS_XSPI_CCP_READ_DATA_DELAY_SEL     GENMASK(24, 19)
#define CDNS_XSPI_CCP_GATE_CFG_CLOSE          GENMASK(5, 4)
#define CDNS_XSPI_CCP_GATE_CFG                GENMASK(3, 0)

/* PHY DLL slave control register */
#define CDNS_XSPI_CCP_PHY_DLL_SLAVE_CTRL      0x0010
#define CDNS_XSPI_CCP_CLK_WR_DELAY            GENMASK(15, 8)
#define CDNS_XSPI_CCP_READ_DQS_DELAY          GENMASK(7, 0)

#define CDNS_XSPI_AUX_PHY_ADDONS_REG          0x0040
#define CDNS_XSPI_AUX_PHY_ADDONS_VALUE        0xE0012000
#define CDNS_XSPI_AUX_DEV_DISC_CONFIG_REG     0x0048
#define CDNS_XSPI_AUX_DEV_DISC_CONFIG_VALUE   0x00000000
#define CDNS_XSPI_AUX_DRIVING_REG             0x0050
#define CDNS_XSPI_AUX_CTRL_RESET              BIT(0)

/* DLL PHY control register */
#define CDNS_XSPI_DLL_PHY_CTRL                0x1034
#define CDNS_XSPI_CCP_DQS_LAST_DATA_DROP_EN   BIT(20)

/* Command registers */
#define CDNS_XSPI_CMD_REG_0                   0x0000
#define CDNS_XSPI_CMD_REG_1                   0x0004
#define CDNS_XSPI_CMD_REG_2                   0x0008
#define CDNS_XSPI_CMD_REG_3                   0x000C
#define CDNS_XSPI_CMD_REG_4                   0x0010
#define CDNS_XSPI_CMD_REG_5                   0x0014

/* Command status registers */
#define CDNS_XSPI_CMD_STATUS_REG              0x0044
#define CDNS_XSPI_CMD_COMPLETED               BIT(15)
#define CDNS_XSPI_CMD_FAILED                  BIT(14)

/* Controller status register */
#define CDNS_XSPI_CTRL_STATUS_REG             0x0100
#define CDNS_XSPI_INIT_COMPLETED              BIT(16)
#define CDNS_XSPI_INIT_LEGACY                 BIT(9)
#define CDNS_XSPI_INIT_FAIL                   BIT(8)
#define CDNS_XSPI_CTRL_BUSY                   BIT(7)

/* Controller interrupt status register */
#define CDNS_XSPI_INTR_STATUS_REG             0x0110
#define CDNS_XSPI_STIG_DONE                   BIT(23)
#define CDNS_XSPI_SDMA_ERROR                  BIT(22)
#define CDNS_XSPI_SDMA_TRIGGER                BIT(21)

/* Controller interrupt enable register */
#define CDNS_XSPI_INTR_ENABLE_REG             0x0114
#define CDNS_XSPI_INTR_EN                     BIT(31)
#define CDNS_XSPI_STIG_DONE_EN                BIT(23)
#define CDNS_XSPI_SDMA_ERROR_EN               BIT(22)
#define CDNS_XSPI_SDMA_TRIGGER_EN             BIT(21)

#define CDNS_XSPI_INTR_MASK (CDNS_XSPI_INTR_EN | \
	CDNS_XSPI_STIG_DONE_EN  | \
	CDNS_XSPI_SDMA_ERROR_EN | \
	CDNS_XSPI_SDMA_TRIGGER_EN)

/* Controller config register */
#define CDNS_XSPI_CTRL_CONFIG_REG             0x0230
#define CDNS_XSPI_CTRL_WORK_MODE              GENMASK(6, 5)

/* SDMA trigger transaction registers */
#define CDNS_XSPI_SDMA_SIZE_REG               0x0240
#define CDNS_XSPI_SDMA_TRD_INFO_REG           0x0244
#define CDNS_XSPI_SDMA_DIR                    BIT(8)

/* Controller features register */
#define CDNS_XSPI_CTRL_FEATURES_REG           0x0F04
#define CDNS_XSPI_NUM_BANKS                   GENMASK(25, 24)
#define CDNS_XSPI_DMA_DATA_WIDTH              BIT(21)
#define CDNS_XSPI_NUM_THREADS                 GENMASK(3, 0)

/* Controller version register */
#define CDNS_XSPI_CTRL_VERSION_REG            0x0F00
#define CDNS_XSPI_MAGIC_NUM                   GENMASK(31, 16)
#define CDNS_XSPI_CTRL_REV                    GENMASK(7, 0)

/* Controller PHY GPIO register, used for synchronization
 * between firmware and OS
 */
#define CDNS_XSPI_PHY_GPIO_CTRL_1		0x208C

/* STIG Profile 1.0 instruction fields (split into registers) */
#define CDNS_XSPI_CMD_INSTR_TYPE              GENMASK(6, 0)
#define CDNS_XSPI_CMD_P1_R1_ADDR0             GENMASK(31, 24)
#define CDNS_XSPI_CMD_P1_R2_ADDR1             GENMASK(7, 0)
#define CDNS_XSPI_CMD_P1_R2_ADDR2             GENMASK(15, 8)
#define CDNS_XSPI_CMD_P1_R2_ADDR3             GENMASK(23, 16)
#define CDNS_XSPI_CMD_P1_R2_ADDR4             GENMASK(31, 24)
#define CDNS_XSPI_CMD_P1_R3_ADDR5             GENMASK(7, 0)
#define CDNS_XSPI_CMD_P1_R3_CMD               GENMASK(23, 16)
#define CDNS_XSPI_CMD_P1_R3_NUM_ADDR_BYTES    GENMASK(30, 28)
#define CDNS_XSPI_CMD_P1_R4_ADDR_IOS          GENMASK(1, 0)
#define CDNS_XSPI_CMD_P1_R4_CMD_IOS           GENMASK(9, 8)
#define CDNS_XSPI_CMD_P1_R4_BANK              GENMASK(14, 12)

/* STIG data sequence instruction fields (split into registers) */
#define CDNS_XSPI_CMD_DSEQ_R2_DCNT_L          GENMASK(31, 16)
#define CDNS_XSPI_CMD_DSEQ_R3_DCNT_H          GENMASK(15, 0)
#define CDNS_XSPI_CMD_DSEQ_R3_NUM_OF_DUMMY    GENMASK(25, 20)
#define CDNS_XSPI_CMD_DSEQ_R4_BANK            GENMASK(14, 12)
#define CDNS_XSPI_CMD_DSEQ_R4_DATA_IOS        GENMASK(9, 8)
#define CDNS_XSPI_CMD_DSEQ_R4_DIR             BIT(4)

/* STIG command status fields */
#define CDNS_XSPI_CMD_STATUS_COMPLETED        BIT(15)
#define CDNS_XSPI_CMD_STATUS_FAILED           BIT(14)
#define CDNS_XSPI_CMD_STATUS_DQS_ERROR        BIT(3)
#define CDNS_XSPI_CMD_STATUS_CRC_ERROR        BIT(2)
#define CDNS_XSPI_CMD_STATUS_BUS_ERROR        BIT(1)
#define CDNS_XSPI_CMD_STATUS_INV_SEQ_ERROR    BIT(0)

#define CDNS_XSPI_CTRL_WORK_MODE_STIG         0x01

#define CDNS_XSPI_STIG_DONE_FLAG              BIT(0)

/* Helper macros for filling command registers */
#define CDNS_XSPI_CMD_FLD_P1_INSTR_CMD_1(op, data_phase) ( \
	FIELD_PREP(CDNS_XSPI_CMD_INSTR_TYPE, (data_phase) ? \
		CDNS_XSPI_STIG_INSTR_TYPE_1 : CDNS_XSPI_STIG_INSTR_TYPE_0) | \
	FIELD_PREP(CDNS_XSPI_CMD_P1_R1_ADDR0, op->addr.val & 0xff))

#define CDNS_XSPI_CMD_FLD_P1_INSTR_CMD_2(op) ( \
	FIELD_PREP(CDNS_XSPI_CMD_P1_R2_ADDR1, (op->addr.val >> 8)  & 0xFF) | \
	FIELD_PREP(CDNS_XSPI_CMD_P1_R2_ADDR2, (op->addr.val >> 16) & 0xFF) | \
	FIELD_PREP(CDNS_XSPI_CMD_P1_R2_ADDR3, (op->addr.val >> 24) & 0xFF) | \
	FIELD_PREP(CDNS_XSPI_CMD_P1_R2_ADDR4, (op->addr.val >> 32) & 0xFF))

#define CDNS_XSPI_CMD_FLD_P1_INSTR_CMD_3(op) ( \
	FIELD_PREP(CDNS_XSPI_CMD_P1_R3_ADDR5, (op->addr.val >> 40) & 0xFF) | \
	FIELD_PREP(CDNS_XSPI_CMD_P1_R3_CMD, op->cmd.opcode) | \
	FIELD_PREP(CDNS_XSPI_CMD_P1_R3_NUM_ADDR_BYTES, op->addr.nbytes))

#define CDNS_XSPI_CMD_FLD_P1_INSTR_CMD_4(op, chipsel) ( \
	FIELD_PREP(CDNS_XSPI_CMD_P1_R4_ADDR_IOS, ilog2(op->addr.buswidth)) | \
	FIELD_PREP(CDNS_XSPI_CMD_P1_R4_CMD_IOS, ilog2(op->cmd.buswidth)) | \
	FIELD_PREP(CDNS_XSPI_CMD_P1_R4_BANK, chipsel))

#define CDNS_XSPI_CMD_FLD_DSEQ_CMD_1(op) \
	FIELD_PREP(CDNS_XSPI_CMD_INSTR_TYPE, CDNS_XSPI_STIG_INSTR_TYPE_DATA_SEQ)

#define CDNS_XSPI_CMD_FLD_DSEQ_CMD_2(op) \
	FIELD_PREP(CDNS_XSPI_CMD_DSEQ_R2_DCNT_L, op->data.nbytes & 0xFFFF)

#define CDNS_XSPI_CMD_FLD_DSEQ_CMD_3(op) ( \
	FIELD_PREP(CDNS_XSPI_CMD_DSEQ_R3_DCNT_H, \
		(op->data.nbytes >> 16) & 0xffff) | \
	FIELD_PREP(CDNS_XSPI_CMD_DSEQ_R3_NUM_OF_DUMMY, op->dummy.nbytes * 8))

#define CDNS_XSPI_CMD_FLD_DSEQ_CMD_4(op, chipsel) ( \
	FIELD_PREP(CDNS_XSPI_CMD_DSEQ_R4_BANK, chipsel) | \
	FIELD_PREP(CDNS_XSPI_CMD_DSEQ_R4_DATA_IOS, ilog2(op->data.buswidth)) | \
	FIELD_PREP(CDNS_XSPI_CMD_DSEQ_R4_DIR, \
		(op->data.dir == SPI_MEM_DATA_IN) ? \
		CDNS_XSPI_STIG_CMD_DIR_READ : CDNS_XSPI_STIG_CMD_DIR_WRITE))

enum cdns_xspi_stig_instr_type {
	CDNS_XSPI_STIG_INSTR_TYPE_0,
	CDNS_XSPI_STIG_INSTR_TYPE_1,
	CDNS_XSPI_STIG_INSTR_TYPE_DATA_SEQ = 127,
};

enum cdns_xspi_sdma_dir {
	CDNS_XSPI_SDMA_DIR_READ,
	CDNS_XSPI_SDMA_DIR_WRITE,
};

enum cdns_xspi_stig_cmd_dir {
	CDNS_XSPI_STIG_CMD_DIR_READ,
	CDNS_XSPI_STIG_CMD_DIR_WRITE,
};

struct cdns_xspi_platform_data {
	u32 phy_data_sel_oe_start;
	u32 phy_data_sel_oe_end;
	u32 phy_dqs_sel_oe_start;
	u32 phy_dqs_sel_oe_end;
	u32 phy_gate_cfg_close;
	u32 phy_gate_cfg;
	u32 phy_rd_del_sel;
	u32 clk_wr_delay;
	bool dqs_last_data_drop;
	bool use_lpbk_dqs;
	bool use_ext_lpbk_dqs;
};

struct cdns_xspi_dev {
	struct platform_device *pdev;
	struct device *dev;

	void __iomem *iobase;
	void __iomem *auxbase;
	void __iomem *sdmabase;

	int irq;
	int current_cs;
	bool skip_sim_check;

	struct mutex lock;

	struct completion cmd_complete;
	struct completion sdma_complete;
	bool sdma_error;

	void *in_buffer;
	const void *out_buffer;

	u8 hw_num_banks;

	struct cdns_xspi_platform_data *plat_data;
};

struct spi_master *cdns_xspi_prepare_master(struct device *dev);
int cdns_xspi_configure(struct cdns_xspi_dev *cdns_xspi);
int cdns_xspi_of_get_plat_data(struct device *dev);
irqreturn_t cdns_xspi_irq_handler(int this_irq, void *dev);
