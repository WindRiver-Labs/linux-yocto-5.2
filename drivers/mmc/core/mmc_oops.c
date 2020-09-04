// SPDX-License-Identifier: GPL-2.0
/* Marvell MMC oops pstore backend driver
 *
 * Copyright (C) 2020 Marvell International Ltd.
 *  Portions of this driver are based on the patch
 *  https://patchwork.kernel.org/patch/5897821/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pstore.h>
#include <linux/slab.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/scatterlist.h>
#include "block.h"
#include "bus.h"
#include "card.h"
#include "core.h"

#define PART_TYPE		0
#define DEFAULT_RECORD_SIZE	10240

static ulong mem_size = DEFAULT_RECORD_SIZE;
module_param_named(pstore_size, mem_size, ulong, 0400);
MODULE_PARM_DESC(mem_size, "Amount of reserved memory in mmc");

static ulong rcrd_size = DEFAULT_RECORD_SIZE;
module_param_named(pstore_rcrdsize, rcrd_size, ulong, 0400);
MODULE_PARM_DESC(rcrd_size, "Size of one dump(default: DEFAULT_RECORD_SIZE");

static char mmcdev[80];
module_param_string(mmcdev, mmcdev, 80, 0400);
MODULE_PARM_DESC(mmcdev, "MMC device to host crash dumps");

static int mmc_partnum = -1;
module_param_named(mmcdev_partnum, mmc_partnum, int, 0400);
MODULE_PARM_DESC(mmc_partnum, "Partition number of mmcdev");

struct mmc_oops_context {
	struct mmc_card	*card;
	struct mmc_request *mrq;
	unsigned long blk_offset;
	unsigned int mem_size;
	size_t record_size;
	u32 flags;
	unsigned int dump_read_cnt;
	unsigned int dump_write_cnt;
	struct pstore_info pstore;
};

static int mmc_oops_pstore_open(struct pstore_info *psi)
{
	struct mmc_oops_context *cxt = psi->data;

	cxt->dump_read_cnt = 0;
	return 0;
}

static void mmc_part_switch(struct mmc_oops_context *cxt)
{
	unsigned int timeout_ms = cxt->card->ext_csd.part_time;
	struct mmc_card *card = cxt->card;
	struct mmc_host *host = card->host;
	struct mmc_command cmd = {};
	struct mmc_request mrq = {};
	bool use_r1b_resp = true;

	if (timeout_ms && host->max_busy_timeout &&
		(timeout_ms > host->max_busy_timeout))
		use_r1b_resp = false;

	cmd.opcode = MMC_SWITCH;
	cmd.arg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
			(EXT_CSD_PART_CONFIG << 16) |
			(PART_TYPE << 8) |
			EXT_CSD_CMD_SET_NORMAL;
	cmd.flags = MMC_CMD_AC;
	if (use_r1b_resp) {
		cmd.flags |= MMC_RSP_SPI_R1B | MMC_RSP_R1B;
		cmd.busy_timeout = timeout_ms;
	} else {
		cmd.flags |= MMC_RSP_SPI_R1 | MMC_RSP_R1;
	}

	memset(cmd.resp, 0, sizeof(cmd.resp));
	cmd.retries = MMC_CMD_RETRIES;
	cmd.data = NULL;

	mrq.cmd = &cmd;
	mmc_wait_for_oops_req(host, &mrq);
	mdelay(card->ext_csd.part_time);
}

static void mmc_prep_rq(struct mmc_request *mrq, char *buf,
	       unsigned long blk_offset, unsigned int nblks,
	       struct scatterlist *sg, u32 opcode, unsigned int flags)
{
	mrq->cmd->opcode = opcode;
	mrq->cmd->arg = blk_offset;
	mrq->cmd->flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	if (nblks == 1) {
		mrq->stop = NULL;
	} else {
		mrq->stop->opcode = MMC_STOP_TRANSMISSION;
		mrq->stop->arg = 0;
		mrq->stop->flags = MMC_RSP_R1B | MMC_CMD_AC;
	}

	mrq->data->blksz = 512;
	mrq->data->blocks = nblks;
	mrq->data->flags = flags;
	mrq->data->sg = sg;
	mrq->data->sg_len = 1;
}

static void mmc_oops_read(struct mmc_oops_context *cxt,
	char *buf, unsigned long blk_offset, unsigned int size)
{
	struct mmc_request *mrq = cxt->mrq;
	struct mmc_card *card = cxt->card;
	struct mmc_host *host = card->host;
	unsigned int nblks = (size >> 9);
	struct scatterlist sg;
	u32 opcode = (nblks > 1) ?
			MMC_READ_MULTIPLE_BLOCK : MMC_READ_SINGLE_BLOCK;

	mmc_prep_rq(mrq, buf, blk_offset, nblks, &sg, opcode, MMC_DATA_READ);
	sg_init_one(&sg, buf, size);
	mmc_set_data_timeout(mrq->data, cxt->card);

	mmc_claim_host(host);
	mmc_wait_for_req(host, mrq);
	mdelay(mrq->data->timeout_ns / NSEC_PER_MSEC);
	mmc_release_host(host);

	if (mrq->cmd->error)
		pr_err("Failed to post request: %d\n", mrq->cmd->error);
	if (mrq->data->error)
		pr_err("Failed to complete write: %d\n", mrq->data->error);
}

static ssize_t mmc_oops_pstore_read(struct pstore_record *record)
{
	struct mmc_oops_context *cxt = record->psi->data;
	ssize_t size = cxt->record_size;

	if (!cxt->card) {
		pr_err("Unavailable mmc device chosen for pstore\n");
		size = -ENOSPC;
		goto out;
	}

	if (cxt->dump_read_cnt >= 1) {
		size = 0;
		goto out;
	}

	record->buf = kzalloc(size, GFP_KERNEL);
	if (record->buf == NULL) {
		size = -ENOMEM;
		goto out;
	}

	if (mmc_card_mmc(cxt->card))
		mmc_part_switch(cxt);

	mmc_oops_read(cxt, record->buf, cxt->blk_offset, cxt->record_size);
	cxt->dump_read_cnt += 1;
out:
	if (size < 0)
		kfree(record->buf);

	return size;
}

static void mmc_oops_write(struct mmc_oops_context *cxt,
	char *buf, unsigned long blk_offset, unsigned int size)
{
	struct mmc_request *mrq = cxt->mrq;
	struct mmc_card *card = cxt->card;
	struct mmc_host *host = card->host;
	unsigned int nblks = (size >> 9);
	struct scatterlist sg;
	u32 opcode = (nblks > 1) ? MMC_WRITE_MULTIPLE_BLOCK : MMC_WRITE_BLOCK;

	mmc_prep_rq(mrq, buf, blk_offset, nblks, &sg, opcode, MMC_DATA_WRITE);
	sg_init_one(&sg, buf, size);
	mmc_set_data_timeout(mrq->data, cxt->card);

	mmc_claim_host(host);
	mmc_wait_for_oops_req(host, mrq);
	mmc_release_host(card->host);

}

static int mmc_oops_pstore_write(struct pstore_record *record)
{
	struct mmc_oops_context *cxt = record->psi->data;

	if (!cxt->card)
		return -ENOSPC;

	if (record->type != PSTORE_TYPE_DMESG)
		return -EINVAL;

	if (record->reason != KMSG_DUMP_OOPS &&
		record->reason != KMSG_DUMP_PANIC)
		return -EINVAL;

	if (record->reason == KMSG_DUMP_OOPS)
		return -EINVAL;

	if (record->part != 1)
		return -ENOSPC;

	if (mmc_card_mmc(cxt->card))
		mmc_part_switch(cxt);

	mmc_oops_write(cxt, record->buf, cxt->blk_offset, cxt->record_size);
	cxt->dump_write_cnt += 1;

	return 0;
}

static struct mmc_oops_context oops_cxt = {
	.pstore = {
		.owner	= THIS_MODULE,
		.name	= "mmc_oops",
		.open	= mmc_oops_pstore_open,
		.read	= mmc_oops_pstore_read,
		.write	= mmc_oops_pstore_write,
	},
};

int mmc_oops_card_set(struct mmc_card *card)
{
	struct mmc_oops_context *cxt = &oops_cxt;

	if (strcmp(mmc_hostname(card->host), mmcdev) || mmc_partnum == -1)
		return 0;

	if (!mmc_card_mmc(card) && !mmc_card_sd(card))
		return -ENODEV;

	cxt->blk_offset = mmc_blk_get_start(card, mmc_partnum);
	if (!cxt->blk_offset)
		return -EINVAL;

	cxt->card = card;
	pr_info("host is %s, partition is p%d\n",
		mmc_hostname(card->host), mmc_partnum);

	return 0;
}
EXPORT_SYMBOL(mmc_oops_card_set);

static int mmc_oops_probe(struct mmc_card *card)
{
	int ret = 0;

	ret = mmc_oops_card_set(card);
	if (ret)
		return ret;

	mmc_claim_host(card->host);

	return 0;
}

static void mmc_oops_remove(struct mmc_card *card)
{
	mmc_release_host(card->host);
}

static struct mmc_driver mmc_oops_driver = {
	.drv		= {
		.name	= "mmc_oops",
	},
	.probe		= mmc_oops_probe,
	.remove		= mmc_oops_remove,
};

static int __init mmc_oops_init(void)
{
	struct mmc_oops_context *cxt = &oops_cxt;
	struct mmc_command *cmd, *stop;
	struct mmc_request *mrq;
	struct mmc_data *data;
	int err = -EINVAL;

	/* Exit silently if no device specified */
	if (strlen(mmcdev) == 0 || mmc_partnum == -1)
		return -ENODEV;

	if (!mem_size || !rcrd_size)
		goto out;

	err = mmc_register_driver(&mmc_oops_driver);
	if (err)
		goto out;

	cxt->card = NULL;

	cxt->mem_size = mem_size;
	cxt->record_size = (rcrd_size < DEFAULT_RECORD_SIZE) ?
				(rcrd_size < 512 ? 512 : rcrd_size) :
					DEFAULT_RECORD_SIZE;

	mrq = kzalloc(sizeof(struct mmc_request), GFP_KERNEL);
	if (!mrq)
		goto remove_driver;

	cmd = kzalloc(sizeof(struct mmc_command), GFP_KERNEL);
	if (!cmd)
		goto free_mrq;

	stop = kzalloc(sizeof(struct mmc_command), GFP_KERNEL);
	if (!stop)
		goto free_cmd;

	data = kzalloc(sizeof(struct mmc_data), GFP_KERNEL);
	if (!data)
		goto free_stop;

	mrq->cmd = cmd;
	mrq->data = data;
	mrq->stop = stop;
	cxt->mrq = mrq;

	cxt->pstore.data = cxt;
	cxt->pstore.bufsize = cxt->record_size;
	cxt->pstore.buf = kmalloc(cxt->pstore.bufsize, GFP_KERNEL);
	if (!cxt->pstore.buf) {
		err = -ENOMEM;
		goto free_data;
	}

	cxt->pstore.flags = PSTORE_FLAGS_DMESG;

	err = pstore_register(&cxt->pstore);
	if (err) {
		pr_info("pstore registration failed\n");
		goto free_pbuf;
	}

	return err;

free_pbuf:
	kfree(cxt->pstore.buf);
free_data:
	kfree(data);
free_stop:
	kfree(stop);
free_cmd:
	kfree(cmd);
free_mrq:
	kfree(mrq);
remove_driver:
	mmc_unregister_driver(&mmc_oops_driver);
out:
	pr_info("failed to complete init %d\n", err);
	return err;
}

static void __exit mmc_oops_exit(void)
{
	struct mmc_oops_context *cxt = &oops_cxt;

	pstore_unregister(&cxt->pstore);
	kfree(cxt->pstore.buf);
	cxt->pstore.bufsize = 0;

	kfree(cxt->mrq->data);
	kfree(cxt->mrq->stop);
	kfree(cxt->mrq->cmd);
	kfree(cxt->mrq);

	mmc_unregister_driver(&mmc_oops_driver);
}

module_init(mmc_oops_init);
module_exit(mmc_oops_exit);

MODULE_AUTHOR("Bhaskara Budiredla <bbudiredla@marvell.com>");
MODULE_DESCRIPTION("MMC oops pstore backend driver");
