// SPDX-License-Identifier: GPL-2.0
/*
 * MMC pstore support based on pstore/blk
 *
 * Copyright (c) 2020 Marvell.
 * Author: Bhaskara Budiredla <bbudiredla@marvell.com>
 */

#define pr_fmt(fmt) "mmcpstore: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pstore_blk.h>
#include <linux/blkdev.h>
#include <linux/mount.h>
#include <linux/slab.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/scatterlist.h>
#include "block.h"
#include "card.h"
#include "core.h"

static struct mmcpstore_context {
	char dev_name[BDEVNAME_SIZE];
	int partno;
	sector_t start_sect;
	sector_t size;
	struct pstore_blk_config conf;
	struct pstore_blk_info info;

	struct mmc_card	*card;
	struct mmc_request *mrq;
} oops_cxt;

static void mmc_prep_req(struct mmc_request *mrq,
		unsigned int sect_offset, unsigned int nsects,
		struct scatterlist *sg, u32 opcode, unsigned int flags)
{
	mrq->cmd->opcode = opcode;
	mrq->cmd->arg = sect_offset;
	mrq->cmd->flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	if (nsects == 1) {
		mrq->stop = NULL;
	} else {
		mrq->stop->opcode = MMC_STOP_TRANSMISSION;
		mrq->stop->arg = 0;
		mrq->stop->flags = MMC_RSP_R1B | MMC_CMD_AC;
	}

	mrq->data->blksz = SECTOR_SIZE;
	mrq->data->blocks = nsects;
	mrq->data->flags = flags;
	mrq->data->sg = sg;
	mrq->data->sg_len = 1;
}

static int mmcpstore_panic_write_req(const char *buf,
		unsigned int nsects, unsigned int sect_offset)
{
	struct mmcpstore_context *cxt = &oops_cxt;
	struct mmc_request *mrq = cxt->mrq;
	struct mmc_card *card = cxt->card;
	struct mmc_host *host = card->host;
	struct scatterlist sg;
	u32 opcode;
	int ret;

	opcode = (nsects > 1) ? MMC_WRITE_MULTIPLE_BLOCK : MMC_WRITE_BLOCK;
	mmc_prep_req(mrq, sect_offset, nsects, &sg, opcode, MMC_DATA_WRITE);
	sg_init_one(&sg, buf, (nsects << SECTOR_SHIFT));
	mmc_set_data_timeout(mrq->data, cxt->card);

	ret = mmc_claim_host_async(host);
	if (ret)
		return ret;

	mmc_wait_for_pstore_req(host, mrq);
	return 0;
}

static int mmcpstore_panic_write(const char *buf, sector_t off, sector_t sects)
{
	struct mmcpstore_context *cxt = &oops_cxt;
	struct mmc_card *card = cxt->card;
	int ret;

	/* Drop the panic record if parition switching is required */
	if (mmc_card_mmc(card) && mmc_blk_needs_part_switch(card))
		return -EPERM;

	ret = mmcpstore_panic_write_req(buf, sects, cxt->start_sect + off);
	if (ret)
		return ret;

	return 0;
}

static struct block_device *mmcpstore_open_backend(const char *device)
{
	struct block_device *bdev;
	dev_t devt;

	bdev = blkdev_get_by_path(device, FMODE_READ, NULL);
	if (IS_ERR(bdev)) {
		devt = name_to_dev_t(device);
		if (devt == 0)
			return ERR_PTR(-ENODEV);

		bdev = blkdev_get_by_dev(devt, FMODE_READ, NULL);
		if (IS_ERR(bdev))
			return bdev;
	}

	return bdev;
}

static void mmcpstore_close_backend(struct block_device *bdev)
{
	if (!bdev)
		return;
	blkdev_put(bdev, FMODE_READ);
}

void mmcpstore_card_set(struct mmc_card *card, const char *disk_name)
{
	struct mmcpstore_context *cxt = &oops_cxt;
	struct pstore_blk_config *conf = &cxt->conf;
	struct pstore_blk_info *info = &cxt->info;
	struct block_device *bdev;
	struct mmc_command *stop;
	struct mmc_command *cmd;
	struct mmc_request *mrq;
	struct mmc_data *data;
	int ret;

	ret = pstore_blk_get_config(conf);
	if (!conf->device[0]) {
		pr_debug("psblk backend is empty\n");
		return;
	}

	/* Multiple backend devices not allowed */
	if (cxt->dev_name[0])
		return;

	bdev =  mmcpstore_open_backend(conf->device);
	if (IS_ERR(bdev)) {
		pr_err("%s failed to open with %ld\n",
				conf->device, PTR_ERR(bdev));
		return;
	}

	bdevname(bdev, cxt->dev_name);
	cxt->partno = bdev->bd_part->partno;
	mmcpstore_close_backend(bdev);

	if (strncmp(cxt->dev_name, disk_name, strlen(disk_name)))
		return;

	cxt->start_sect = mmc_blk_get_part(card, cxt->partno, &cxt->size);
	if (!cxt->start_sect) {
		pr_err("Non-existent partition %d selected\n", cxt->partno);
		return;
	}

	/* Check for host mmc panic write polling function definitions */
	if (!card->host->ops->req_cleanup_pending ||
			!card->host->ops->req_completion_poll)
		return;

	cxt->card = card;

	mrq = kzalloc(sizeof(struct mmc_request), GFP_KERNEL);
	if (!mrq)
		goto out;

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

	info->major = MMC_BLOCK_MAJOR;
	info->flags = PSTORE_FLAGS_DMESG;
	info->panic_write = mmcpstore_panic_write;
	ret = register_pstore_blk(info);
	if (ret) {
		pr_err("%s registering with psblk failed (%d)\n",
				cxt->dev_name, ret);
		goto free_data;
	}

	pr_info("%s registered as psblk backend\n", cxt->dev_name);
	return;

free_data:
	kfree(data);
free_stop:
	kfree(stop);
free_cmd:
	kfree(cmd);
free_mrq:
	kfree(mrq);
out:
	return;
}

void unregister_mmcpstore(void)
{
	struct mmcpstore_context *cxt = &oops_cxt;

	unregister_pstore_blk(MMC_BLOCK_MAJOR);
	kfree(cxt->mrq->data);
	kfree(cxt->mrq->stop);
	kfree(cxt->mrq->cmd);
	kfree(cxt->mrq);
	cxt->card = NULL;
}
