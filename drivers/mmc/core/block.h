/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _MMC_CORE_BLOCK_H
#define _MMC_CORE_BLOCK_H

struct mmc_queue;
struct request;

void mmc_blk_cqe_recovery(struct mmc_queue *mq);

enum mmc_issued;

enum mmc_issued mmc_blk_mq_issue_rq(struct mmc_queue *mq, struct request *req);
void mmc_blk_mq_complete(struct request *req);
void mmc_blk_mq_recovery(struct mmc_queue *mq);

struct work_struct;

void mmc_blk_mq_complete_work(struct work_struct *work);
#if IS_ENABLED(CONFIG_MMC_PSTORE)
int mmc_blk_needs_part_switch(struct mmc_card *card);
sector_t mmc_blk_get_part(struct mmc_card *card, int part_num, sector_t *size);
void mmcpstore_card_set(struct mmc_card *card, const char *disk_name);
void unregister_mmcpstore(void);
#else
static inline void mmcpstore_card_set(struct mmc_card *card,
					const char *disk_name) {}
static inline void unregister_mmcpstore(void) {}
#endif

#endif
