// SPDX-License-Identifier: GPL-2.0
/* Marvell OcteonTx2 RVU Ethernet driver
 *
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/ethtool.h>

#include "otx2_reg.h"
#include "otx2_common.h"
#include "otx2_struct.h"

/* Sync MAC address with RVU */
int otx2_hw_set_mac_addr(struct otx2_nic *pfvf, struct net_device *netdev)
{
	struct nix_set_mac_addr *req;

	req = otx2_mbox_alloc_msg_nix_set_mac_addr(&pfvf->mbox);
	if (!req)
		return -ENOMEM;

	ether_addr_copy(req->mac_addr, netdev->dev_addr);

	return otx2_sync_mbox_msg(&pfvf->mbox);
}

int otx2_set_mac_address(struct net_device *netdev, void *p)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);

	otx2_hw_set_mac_addr(pfvf, netdev);

	return 0;
}

int otx2_hw_set_mtu(struct otx2_nic *pfvf, int mtu)
{
	struct nix_frs_cfg *req;

	req = otx2_mbox_alloc_msg_nix_set_hw_frs(&pfvf->mbox);
	if (!req)
		return -ENOMEM;

	req->update_smq = true;
	req->maxlen = mtu + OTX2_ETH_HLEN;
	return otx2_sync_mbox_msg(&pfvf->mbox);
}

int otx2_change_mtu(struct net_device *netdev, int new_mtu)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	int err;

	if (netif_running(netdev)) {
		err = otx2_hw_set_mtu(pfvf, new_mtu);
		if (err)
			return err;
	}
	netdev_info(netdev, "Changing MTU from %d to %d\n",
		    netdev->mtu, new_mtu);
	netdev->mtu = new_mtu;
	return 0;
}

int otx2_set_flowkey_cfg(struct otx2_nic *pfvf)
{
	struct otx2_rss_info *rss = &pfvf->hw.rss_info;
	struct nix_rss_flowkey_cfg *req;

	req = otx2_mbox_alloc_msg_nix_rss_flowkey_cfg(&pfvf->mbox);
	if (!req)
		return -ENOMEM;
	req->mcam_index = -1; /* Default or reserved index */
	req->flowkey_cfg = rss->flowkey_cfg;
	req->group = DEFAULT_RSS_CONTEXT_GROUP;

	return otx2_sync_mbox_msg(&pfvf->mbox);
}

int otx2_set_rss_table(struct otx2_nic *pfvf)
{
	struct otx2_rss_info *rss = &pfvf->hw.rss_info;
	struct mbox *mbox = &pfvf->mbox;
	struct nix_aq_enq_req *aq;
	int idx, err;

	/* Get memory to put this msg */
	for (idx = 0; idx < rss->rss_size; idx++) {
		aq = otx2_mbox_alloc_msg_nix_aq_enq(mbox);
		if (!aq) {
			/* The shared memory buffer can be full.
			 * Flush it and retry
			 */
			err = otx2_sync_mbox_msg(mbox);
			if (err)
				return err;
			aq = otx2_mbox_alloc_msg_nix_aq_enq(mbox);
			if (!aq)
				return -ENOMEM;
		}

		aq->rss.rq = rss->ind_tbl[idx];

		/* Fill AQ info */
		aq->qidx = idx;
		aq->ctype = NIX_AQ_CTYPE_RSS;
		aq->op = NIX_AQ_INSTOP_INIT;
	}
	return otx2_sync_mbox_msg(mbox);
}

void otx2_set_rss_key(struct otx2_nic *pfvf)
{
	struct otx2_rss_info *rss = &pfvf->hw.rss_info;
	u64 *key = (u64 *)&rss->key[4];
	int idx;

	/* 352bit or 44byte key needs to be configured as below
	 * NIX_LF_RX_SECRETX0 = key<351:288>
	 * NIX_LF_RX_SECRETX1 = key<287:224>
	 * NIX_LF_RX_SECRETX2 = key<223:160>
	 * NIX_LF_RX_SECRETX3 = key<159:96>
	 * NIX_LF_RX_SECRETX4 = key<95:32>
	 * NIX_LF_RX_SECRETX5<63:32> = key<31:0>
	 */
	otx2_write64(pfvf, NIX_LF_RX_SECRETX(5),
		     (u64)(*((u32 *)&rss->key)) << 32);
	idx = sizeof(rss->key) / sizeof(u64);
	while (idx > 0) {
		idx--;
		otx2_write64(pfvf, NIX_LF_RX_SECRETX(idx), *key++);
	}
}

int otx2_rss_init(struct otx2_nic *pfvf)
{
	struct otx2_rss_info *rss = &pfvf->hw.rss_info;
	int idx, ret = 0;

	/* Enable RSS */
	rss->enable = true;
	rss->rss_size = sizeof(rss->ind_tbl);

	/* Init RSS key here */
	netdev_rss_key_fill(rss->key, sizeof(rss->key));
	otx2_set_rss_key(pfvf);

	/* Default indirection table */
	for (idx = 0; idx < rss->rss_size; idx++)
		rss->ind_tbl[idx] =
			ethtool_rxfh_indir_default(idx, pfvf->hw.rx_queues);

	ret = otx2_set_rss_table(pfvf);
	if (ret)
		return ret;

	/* Default flowkey or hash config to be used for generating flow tag */
	rss->flowkey_cfg = NIX_FLOW_KEY_TYPE_IPV4 | NIX_FLOW_KEY_TYPE_IPV6 |
			   NIX_FLOW_KEY_TYPE_TCP | NIX_FLOW_KEY_TYPE_UDP |
			   NIX_FLOW_KEY_TYPE_SCTP;

	return otx2_set_flowkey_cfg(pfvf);
}

dma_addr_t otx2_alloc_rbuf(struct otx2_nic *pfvf, struct otx2_pool *pool,
			   gfp_t gfp)
{
	dma_addr_t iova;

	/* Check if request can be accommodated in previous allocated page */
	if (pool->page &&
	    ((pool->page_offset + pool->rbsize) <= PAGE_SIZE)) {
		pool->pageref++;
		goto ret;
	}

	otx2_get_page(pool);

	/* Allocate a new page */
	pool->page = alloc_pages(gfp | __GFP_COMP | __GFP_NOWARN, 0);
	if (!pool->page)
		return -ENOMEM;

	pool->page_offset = 0;
ret:
	iova = (u64)dma_map_page_attrs(pfvf->dev, pool->page,
				       pool->page_offset, pool->rbsize,
				       DMA_FROM_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
	if (dma_mapping_error(pfvf->dev, iova)) {
		if (!pool->page_offset)
			__free_pages(pool->page, 0);
		pool->page = NULL;
		return -ENOMEM;
	}
	pool->page_offset += pool->rbsize;
	return iova;
}

void otx2_tx_timeout(struct net_device *netdev)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);

	schedule_work(&pfvf->reset_task);
}

static int otx2_get_link(struct otx2_nic *pfvf)
{
	int link = 0;
	u16 map;

	/* cgx lmac link */
	if (pfvf->tx_chan_base >= CGX_CHAN_BASE) {
		map = pfvf->tx_chan_base & 0x7FF;
		link = 4 * ((map >> 8) & 0xF) + ((map >> 4) & 0xF);
	}
	/* LBK channel */
	if (pfvf->tx_chan_base < SDP_CHAN_BASE)
		link = 12;

	return link;
}

int otx2_txschq_config(struct otx2_nic *pfvf, int lvl)
{
	struct nix_txschq_config *req;
	struct otx2_hw *hw = &pfvf->hw;
	u64 schq, parent;

	req = otx2_mbox_alloc_msg_nix_txschq_cfg(&pfvf->mbox);
	if (!req)
		return -ENOMEM;

	req->lvl = lvl;
	req->num_regs = 1;

	schq = hw->txschq_list[lvl][0];
	/* Set topology e.t.c configuration */
	if (lvl == NIX_TXSCH_LVL_SMQ) {
		/* Set min and max Tx packet lengths */
		req->reg[0] = NIX_AF_SMQX_CFG(schq);
		req->regval[0] = ((pfvf->netdev->mtu  + OTX2_ETH_HLEN) << 8) |
				   OTX2_MIN_MTU;

		req->regval[0] |= (0x20ULL << 51) | (0x80ULL << 39);
		req->num_regs++;
		/* MDQ config */
		parent =  hw->txschq_list[NIX_TXSCH_LVL_TL4][0];
		req->reg[1] = NIX_AF_MDQX_PARENT(schq);
		req->regval[1] = parent << 16;
		req->num_regs++;
		/* Set DWRR quantum */
		req->reg[2] = NIX_AF_MDQX_SCHEDULE(schq);
		req->regval[2] = pfvf->netdev->mtu;
	} else if (lvl == NIX_TXSCH_LVL_TL4) {
		parent =  hw->txschq_list[NIX_TXSCH_LVL_TL3][0];
		req->reg[0] = NIX_AF_TL4X_PARENT(schq);
		req->regval[0] = parent << 16;
	} else if (lvl == NIX_TXSCH_LVL_TL3) {
		parent = hw->txschq_list[NIX_TXSCH_LVL_TL2][0];
		req->reg[0] = NIX_AF_TL3X_PARENT(schq);
		req->regval[0] = parent << 16;
	} else if (lvl == NIX_TXSCH_LVL_TL2) {
		parent =  hw->txschq_list[NIX_TXSCH_LVL_TL1][0];
		req->reg[0] = NIX_AF_TL2X_PARENT(schq);
		req->regval[0] = parent << 16;

		req->num_regs++;
		req->reg[1] = NIX_AF_TL2X_SCHEDULE(schq);
		req->regval[1] = TXSCH_TL1_DFLT_RR_PRIO << 24;

		req->num_regs++;
		req->reg[2] = NIX_AF_TL3_TL2X_LINKX_CFG(schq,
							otx2_get_link(pfvf));
		/* Enable this queue and backpressure */
		req->regval[2] = BIT_ULL(13) | BIT_ULL(12);

	} else if (lvl == NIX_TXSCH_LVL_TL1) {
		/* Default config for TL1.
		 * For VF this is always ignored.
		 */

		/* Set DWRR quantum */
		req->reg[0] = NIX_AF_TL1X_SCHEDULE(schq);
		req->regval[0] = TXSCH_TL1_DFLT_RR_QTM;

		req->num_regs++;
		req->reg[1] = NIX_AF_TL1X_TOPOLOGY(schq);
		req->regval[1] = (TXSCH_TL1_DFLT_RR_PRIO << 1);

		req->num_regs++;
		req->reg[2] = NIX_AF_TL1X_CIR(schq);
		req->regval[2] = 0;
	}

	return otx2_sync_mbox_msg(&pfvf->mbox);
}

int otx2_txsch_alloc(struct otx2_nic *pfvf)
{
	struct nix_txsch_alloc_req *req;
	int lvl;

	/* Get memory to put this msg */
	req = otx2_mbox_alloc_msg_nix_txsch_alloc(&pfvf->mbox);
	if (!req)
		return -ENOMEM;

	/* Request one schq per level */
	for (lvl = 0; lvl < NIX_TXSCH_LVL_CNT; lvl++)
		req->schq[lvl] = 1;

	return otx2_sync_mbox_msg(&pfvf->mbox);
}

int otx2_txschq_stop(struct otx2_nic *pfvf)
{
	struct nix_txsch_free_req *free_req;
	int lvl, schq;

	/* Free the transmit schedulers */
	free_req = otx2_mbox_alloc_msg_nix_txsch_free(&pfvf->mbox);
	if (!free_req)
		return -ENOMEM;

	free_req->flags = TXSCHQ_FREE_ALL;
	WARN_ON(otx2_sync_mbox_msg(&pfvf->mbox));

	/* Clear the txschq list */
	for (lvl = 0; lvl < NIX_TXSCH_LVL_CNT; lvl++) {
		for (schq = 0; schq < MAX_TXSCHQ_PER_FUNC; schq++)
			pfvf->hw.txschq_list[lvl][schq] = 0;
	}
	return 0;
}

static int otx2_rq_init(struct otx2_nic *pfvf, u16 qidx, u16 lpb_aura)
{
	struct nix_aq_enq_req *aq;

	/* Get memory to put this msg */
	aq = otx2_mbox_alloc_msg_nix_aq_enq(&pfvf->mbox);
	if (!aq)
		return -ENOMEM;

	aq->rq.cq = qidx;
	aq->rq.ena = 1;
	aq->rq.pb_caching = 1;
	aq->rq.lpb_aura = lpb_aura; /* Use large packet buffer aura */
	aq->rq.lpb_sizem1 = (DMA_BUFFER_LEN / 8) - 1;
	aq->rq.xqe_imm_size = 0; /* Copying of packet to CQE not needed */
	aq->rq.flow_tagw = 32; /* Copy full 32bit flow_tag to CQE header */

	/* Fill AQ info */
	aq->qidx = qidx;
	aq->ctype = NIX_AQ_CTYPE_RQ;
	aq->op = NIX_AQ_INSTOP_INIT;

	return otx2_sync_mbox_msg(&pfvf->mbox);
}

static int otx2_sq_init(struct otx2_nic *pfvf, u16 qidx, u16 sqb_aura)
{
	struct otx2_qset *qset = &pfvf->qset;
	struct otx2_snd_queue *sq;
	struct nix_aq_enq_req *aq;
	struct otx2_pool *pool;
	int err;

	pool = &pfvf->qset.pool[sqb_aura];
	sq = &qset->sq[qidx];
	sq->sqe_size = NIX_SQESZ_W16 ? 64 : 128;
	sq->sqe_cnt = qset->sqe_cnt;

	err = qmem_alloc(pfvf->dev, &sq->sqe, 1, sq->sqe_size);
	if (err)
		return err;

	sq->sqe_base = sq->sqe->base;
	sq->sg = kcalloc(qset->sqe_cnt, sizeof(struct sg_list), GFP_KERNEL);
	if (!sq->sg)
		return -ENOMEM;

	sq->head = 0;
	sq->num_sqbs = (pfvf->hw.sqb_size / sq->sqe_size) - 1;
	sq->num_sqbs = (qset->sqe_cnt + sq->num_sqbs) / sq->num_sqbs;
	sq->aura_id = sqb_aura;
	sq->aura_fc_addr = pool->fc_addr->base;
	sq->lmt_addr = (__force u64 *)(pfvf->reg_base + LMT_LF_LMTLINEX(qidx));
	sq->io_addr = (__force u64)(pfvf->reg_base + NIX_LF_OP_SENDX(0));

	/* Get memory to put this msg */
	aq = otx2_mbox_alloc_msg_nix_aq_enq(&pfvf->mbox);
	if (!aq)
		return -ENOMEM;

	aq->sq.cq = pfvf->hw.rx_queues + qidx;
	aq->sq.max_sqe_size = NIX_MAXSQESZ_W16; /* 128 byte */
	aq->sq.cq_ena = 1;
	aq->sq.ena = 1;
	/* Only one SMQ is allocated, map all SQ's to that SMQ  */
	aq->sq.smq = pfvf->hw.txschq_list[NIX_TXSCH_LVL_SMQ][0];
	aq->sq.smq_rr_quantum = OTX2_MAX_MTU;
	aq->sq.default_chan = pfvf->tx_chan_base;
	aq->sq.sqe_stype = NIX_STYPE_STF; /* Cache SQB */
	aq->sq.sqb_aura = sqb_aura;

	/* Fill AQ info */
	aq->qidx = qidx;
	aq->ctype = NIX_AQ_CTYPE_SQ;
	aq->op = NIX_AQ_INSTOP_INIT;

	return otx2_sync_mbox_msg(&pfvf->mbox);
}

static int otx2_cq_init(struct otx2_nic *pfvf, u16 qidx)
{
	struct otx2_qset *qset = &pfvf->qset;
	struct nix_aq_enq_req *aq;
	struct otx2_cq_queue *cq;
	int err, pool_id;

	cq = &qset->cq[qidx];
	cq->cqe_cnt = (qidx < pfvf->hw.rx_queues) ? qset->rqe_cnt
			: qset->sqe_cnt;
	cq->cqe_size = pfvf->qset.xqe_size;

	/* Allocate memory for CQEs */
	err = qmem_alloc(pfvf->dev, &cq->cqe, cq->cqe_cnt, cq->cqe_size);
	if (err)
		return err;

	/* Save CQE CPU base for faster reference */
	cq->cqe_base = cq->cqe->base;
	/* In case where all RQs auras point to single pool,
	 * all CQs receive buffer pool also point to same pool.
	 */
	pool_id = ((qidx < pfvf->hw.rx_queues) &&
		   (pfvf->hw.rqpool_cnt != pfvf->hw.rx_queues)) ? 0 : qidx;
	cq->rbpool = &qset->pool[pool_id];

	cq->cq_idx = qidx;

	/* Get memory to put this msg */
	aq = otx2_mbox_alloc_msg_nix_aq_enq(&pfvf->mbox);
	if (!aq)
		return -ENOMEM;

	aq->cq.ena = 1;
	aq->cq.qsize = Q_SIZE(cq->cqe_cnt, 4);
	aq->cq.caching = 1;
	aq->cq.base = cq->cqe->iova;
	aq->cq.cint_idx = (qidx < pfvf->hw.rx_queues) ? qidx
				: (qidx - pfvf->hw.rx_queues);
	cq->cint_idx = aq->cq.cint_idx;

	/* Fill AQ info */
	aq->qidx = qidx;
	aq->ctype = NIX_AQ_CTYPE_CQ;
	aq->op = NIX_AQ_INSTOP_INIT;

	return otx2_sync_mbox_msg(&pfvf->mbox);
}

int otx2_config_nix_queues(struct otx2_nic *pfvf)
{
	int qidx, err;

	/* Initialize RX queues */
	for (qidx = 0; qidx < pfvf->hw.rx_queues; qidx++) {
		u16 lpb_aura = otx2_get_pool_idx(pfvf, AURA_NIX_RQ, qidx);

		err = otx2_rq_init(pfvf, qidx, lpb_aura);
		if (err)
			return err;
	}

	/* Initialize TX queues */
	for (qidx = 0; qidx < pfvf->hw.tx_queues; qidx++) {
		u16 sqb_aura = otx2_get_pool_idx(pfvf, AURA_NIX_SQ, qidx);

		err = otx2_sq_init(pfvf, qidx, sqb_aura);
		if (err)
			return err;
	}

	/* Initialize completion queues */
	for (qidx = 0; qidx < pfvf->qset.cq_cnt; qidx++) {
		err = otx2_cq_init(pfvf, qidx);
		if (err)
			return err;
	}

	return 0;
}

int otx2_config_nix(struct otx2_nic *pfvf)
{
	struct nix_lf_alloc_req  *nixlf;

	pfvf->qset.xqe_size = NIX_XQESZ_W16 ? 128 : 512;

	/* Get memory to put this msg */
	nixlf = otx2_mbox_alloc_msg_nix_lf_alloc(&pfvf->mbox);
	if (!nixlf)
		return -ENOMEM;

	/* Set RQ/SQ/CQ counts */
	nixlf->rq_cnt = pfvf->hw.rx_queues;
	nixlf->sq_cnt = pfvf->hw.tx_queues;
	nixlf->cq_cnt = pfvf->qset.cq_cnt;
	nixlf->rss_sz = MAX_RSS_INDIR_TBL_SIZE;
	nixlf->rss_grps = 1; /* Single RSS indir table supported, for now */
	nixlf->xqe_sz = NIX_XQESZ_W16;
	/* We don't know absolute NPA LF idx attached.
	 * AF will replace 'RVU_DEFAULT_PF_FUNC' with
	 * NPA LF attached to this RVU PF/VF.
	 */
	nixlf->npa_func = RVU_DEFAULT_PF_FUNC;
	/* Disable alignment pad, enable L2 length check,
	 * enable L4 TCP/UDP checksum verification.
	 */
	nixlf->rx_cfg = BIT_ULL(33) | BIT_ULL(35) | BIT_ULL(37);

	return otx2_sync_mbox_msg(&pfvf->mbox);
}

void otx2_free_aura_ptr(struct otx2_nic *pfvf, int type)
{
	int pool_id, pool_start = 0, pool_end = 0, size = 0;
	u64 iova, pa;

	if (type == AURA_NIX_SQ) {
		pool_start = otx2_get_pool_idx(pfvf, type, 0);
		pool_end =  pool_start + pfvf->hw.sqpool_cnt;
		size = pfvf->hw.sqb_size;
	}
	if (type == AURA_NIX_RQ) {
		pool_start = otx2_get_pool_idx(pfvf, type, 0);
		pool_end = pfvf->hw.rqpool_cnt;
		size = RCV_FRAG_LEN;
	}

	/* Free SQB and RQB pointers from the aura pool */
	for (pool_id = pool_start; pool_id < pool_end; pool_id++) {
		iova = otx2_aura_allocptr(pfvf, pool_id);
		while (iova) {
			if (type == AURA_NIX_RQ)
				iova -= OTX2_HEAD_ROOM;

			pa = otx2_iova_to_phys(pfvf->iommu_domain, iova);
			dma_unmap_page_attrs(pfvf->dev, iova, size,
					     DMA_FROM_DEVICE,
					     DMA_ATTR_SKIP_CPU_SYNC);
			put_page(virt_to_page(phys_to_virt(pa)));
			iova = otx2_aura_allocptr(pfvf, pool_id);
		}
	}
}

void otx2_aura_pool_free(struct otx2_nic *pfvf)
{
	struct otx2_pool *pool;
	int pool_id;

	if (!pfvf->qset.pool)
		return;

	for (pool_id = 0; pool_id < pfvf->hw.pool_cnt; pool_id++) {
		pool = &pfvf->qset.pool[pool_id];
		qmem_free(pfvf->dev, pool->stack);
		qmem_free(pfvf->dev, pool->fc_addr);
	}
	devm_kfree(pfvf->dev, pfvf->qset.pool);
}

static int otx2_aura_init(struct otx2_nic *pfvf, int aura_id,
			  int pool_id, int numptrs)
{
	struct npa_aq_enq_req *aq;
	struct otx2_pool *pool;
	int err;

	pool = &pfvf->qset.pool[pool_id];

	/* Allocate memory for HW to update Aura count.
	 * Alloc one cache line, so that it fits all FC_STYPE modes.
	 */
	if (!pool->fc_addr) {
		err = qmem_alloc(pfvf->dev, &pool->fc_addr, 1, OTX2_ALIGN);
		if (err)
			return err;
	}

	/* Initialize this aura's context via AF */
	aq = otx2_mbox_alloc_msg_npa_aq_enq(&pfvf->mbox);
	if (!aq) {
		/* Shared mbox memory buffer is full, flush it and retry */
		err = otx2_sync_mbox_msg(&pfvf->mbox);
		if (err)
			return err;
		aq = otx2_mbox_alloc_msg_npa_aq_enq(&pfvf->mbox);
		if (!aq)
			return -ENOMEM;
	}

	aq->aura_id = aura_id;
	/* Will be filled by AF with correct pool context address */
	aq->aura.pool_addr = pool_id;
	aq->aura.pool_caching = 1;
	aq->aura.shift = ilog2(numptrs) - 8;
	aq->aura.count = numptrs;
	aq->aura.limit = numptrs;
	aq->aura.ena = 1;
	aq->aura.fc_ena = 1;
	aq->aura.fc_addr = pool->fc_addr->iova;
	aq->aura.fc_hyst_bits = 0; /* Store count on all updates */

	/* Fill AQ info */
	aq->ctype = NPA_AQ_CTYPE_AURA;
	aq->op = NPA_AQ_INSTOP_INIT;

	return 0;
}

static int otx2_pool_init(struct otx2_nic *pfvf, u16 pool_id,
			  int stack_pages, int numptrs, int buf_size)
{
	struct npa_aq_enq_req *aq;
	struct otx2_pool *pool;
	int err;

	pool = &pfvf->qset.pool[pool_id];
	/* Alloc memory for stack which is used to store buffer pointers */
	err = qmem_alloc(pfvf->dev, &pool->stack,
			 stack_pages, pfvf->hw.stack_pg_bytes);
	if (err)
		return err;

	pool->rbsize = buf_size;

	/* Initialize this pool's context via AF */
	aq = otx2_mbox_alloc_msg_npa_aq_enq(&pfvf->mbox);
	if (!aq) {
		/* Shared mbox memory buffer is full, flush it and retry */
		err = otx2_sync_mbox_msg(&pfvf->mbox);
		if (err) {
			qmem_free(pfvf->dev, pool->stack);
			return err;
		}
		aq = otx2_mbox_alloc_msg_npa_aq_enq(&pfvf->mbox);
		if (!aq) {
			qmem_free(pfvf->dev, pool->stack);
			return -ENOMEM;
		}
	}

	aq->aura_id = pool_id;
	aq->pool.stack_base = pool->stack->iova;
	aq->pool.stack_caching = 1;
	aq->pool.ena = 1;
	aq->pool.buf_size = buf_size / 128;
	aq->pool.stack_max_pages = stack_pages;
	aq->pool.shift = ilog2(numptrs) - 8;
	aq->pool.ptr_start = 0;
	aq->pool.ptr_end = ~0ULL;

	/* Fill AQ info */
	aq->ctype = NPA_AQ_CTYPE_POOL;
	aq->op = NPA_AQ_INSTOP_INIT;

	return 0;
}

int otx2_sq_aura_pool_init(struct otx2_nic *pfvf)
{
	int sq, pool_id, stack_pages, num_sqbs;
	struct otx2_qset *qset = &pfvf->qset;
	struct otx2_hw *hw = &pfvf->hw;
	struct otx2_pool *pool;
	int err, ptr;
	s64 bufptr;

	/* Calculate number of SQBs needed.
	 *
	 * For a 128byte SQE, and 4K size SQB, 31 SQEs will fit in one SQB.
	 * Last SQE is used for pointing to next SQB.
	 */
	num_sqbs = (hw->sqb_size / 128) - 1;
	num_sqbs = (qset->sqe_cnt + num_sqbs) / num_sqbs;

	/* Get no of stack pages needed */
	stack_pages =
		(num_sqbs + hw->stack_pg_ptrs - 1) / hw->stack_pg_ptrs;

	for (sq = 0; sq < hw->tx_queues; sq++) {
		pool_id = otx2_get_pool_idx(pfvf, AURA_NIX_SQ, sq);
		/* Initialize aura context */
		err = otx2_aura_init(pfvf, pool_id, pool_id, num_sqbs);
		if (err)
			goto fail;

		/* Initialize pool context */
		err = otx2_pool_init(pfvf, pool_id, stack_pages,
				     num_sqbs, hw->sqb_size);
		if (err)
			goto fail;
	}

	/* Flush accumulated messages */
	err = otx2_sync_mbox_msg(&pfvf->mbox);
	if (err)
		goto fail;

	/* Allocate pointers and free them to aura/pool */
	for (sq = 0; sq < hw->tx_queues; sq++) {
		pool_id = otx2_get_pool_idx(pfvf, AURA_NIX_SQ, sq);
		pool = &pfvf->qset.pool[pool_id];
		for (ptr = 0; ptr < num_sqbs; ptr++) {
			bufptr = otx2_alloc_rbuf(pfvf, pool, GFP_KERNEL);
			if (bufptr <= 0)
				return bufptr;
			otx2_aura_freeptr(pfvf, pool_id, bufptr);
		}
		otx2_get_page(pool);
	}

	return 0;
fail:
	otx2_aura_pool_free(pfvf);
	return err;
}

int otx2_rq_aura_pool_init(struct otx2_nic *pfvf)
{
	struct otx2_hw *hw = &pfvf->hw;
	int stack_pages, pool_id, rq;
	struct otx2_pool *pool;
	int err, ptr, num_ptrs;
	s64 bufptr;

	num_ptrs = pfvf->qset.rqe_cnt;

	stack_pages =
		(num_ptrs + hw->stack_pg_ptrs - 1) / hw->stack_pg_ptrs;

	for (rq = 0; rq < hw->rx_queues; rq++) {
		pool_id = otx2_get_pool_idx(pfvf, AURA_NIX_RQ, rq);
		/* Initialize aura context */
		err = otx2_aura_init(pfvf, pool_id, pool_id, num_ptrs);
		if (err)
			goto fail;
	}
	for (pool_id = 0; pool_id < hw->rqpool_cnt; pool_id++) {
		err = otx2_pool_init(pfvf, pool_id, stack_pages,
				     num_ptrs, RCV_FRAG_LEN);
		if (err)
			goto fail;
	}

	/* Flush accumulated messages */
	err = otx2_sync_mbox_msg(&pfvf->mbox);
	if (err)
		goto fail;

	/* Allocate pointers and free them to aura/pool */
	for (pool_id = 0; pool_id < hw->rqpool_cnt; pool_id++) {
		pool = &pfvf->qset.pool[pool_id];
		for (ptr = 0; ptr < num_ptrs; ptr++) {
			bufptr = otx2_alloc_rbuf(pfvf, pool, GFP_KERNEL);
			if (bufptr <= 0)
				return bufptr;
			otx2_aura_freeptr(pfvf, pool_id,
					  bufptr + OTX2_HEAD_ROOM);
		}
		otx2_get_page(pool);
	}

	return 0;
fail:
	otx2_aura_pool_free(pfvf);
	return err;
}

int otx2_config_npa(struct otx2_nic *pfvf)
{
	struct otx2_qset *qset = &pfvf->qset;
	struct npa_lf_alloc_req  *npalf;
	struct otx2_hw *hw = &pfvf->hw;
	int aura_cnt;

	/* Pool - Stack of free buffer pointers
	 * Aura - Alloc/frees pointers from/to pool for NIX DMA.
	 */

	if (!hw->pool_cnt)
		return -EINVAL;

	qset->pool = devm_kzalloc(pfvf->dev, sizeof(struct otx2_pool) *
				  hw->pool_cnt, GFP_KERNEL);
	if (!qset->pool)
		return -ENOMEM;

	/* Get memory to put this msg */
	npalf = otx2_mbox_alloc_msg_npa_lf_alloc(&pfvf->mbox);
	if (!npalf)
		return -ENOMEM;

	/* Set aura and pool counts */
	npalf->nr_pools = hw->pool_cnt;
	aura_cnt = ilog2(roundup_pow_of_two(hw->pool_cnt));
	npalf->aura_sz = (aura_cnt >= ilog2(128)) ? (aura_cnt - 6) : 1;

	return otx2_sync_mbox_msg(&pfvf->mbox);
}

int otx2_detach_resources(struct mbox *mbox)
{
	struct rsrc_detach *detach;

	detach = otx2_mbox_alloc_msg_detach_resources(mbox);
	if (!detach)
		return -ENOMEM;

	/* detach all */
	detach->partial = false;

	/* Send detach request to AF */
	otx2_mbox_msg_send(&mbox->mbox, 0);
	return 0;
}

int otx2_attach_npa_nix(struct otx2_nic *pfvf)
{
	struct rsrc_attach *attach;
	struct msg_req *msix;
	int err;

	/* Get memory to put this msg */
	attach = otx2_mbox_alloc_msg_attach_resources(&pfvf->mbox);
	if (!attach)
		return -ENOMEM;

	attach->npalf = true;
	attach->nixlf = true;

	/* Send attach request to AF */
	err = otx2_sync_mbox_msg(&pfvf->mbox);
	if (err)
		return err;

	/* Get NPA and NIX MSIX vector offsets */
	msix = otx2_mbox_alloc_msg_msix_offset(&pfvf->mbox);
	if (!msix)
		return -ENOMEM;

	err = otx2_sync_mbox_msg(&pfvf->mbox);
	if (err)
		return err;

	if (pfvf->hw.npa_msixoff == MSIX_VECTOR_INVALID ||
	    pfvf->hw.nix_msixoff == MSIX_VECTOR_INVALID) {
		dev_err(pfvf->dev,
			"RVUPF: Invalid MSIX vector offset for NPA/NIX\n");
		return -EINVAL;
	}
	return 0;
}

void otx2_ctx_disable(struct mbox *mbox, int type, bool npa)
{
	struct hwctx_disable_req *req;

	/* Request AQ to disable this context */
	if (npa)
		req = otx2_mbox_alloc_msg_npa_hwctx_disable(mbox);
	else
		req = otx2_mbox_alloc_msg_nix_hwctx_disable(mbox);

	if (!req)
		return;

	req->ctype = type;

	WARN_ON(otx2_sync_mbox_msg(mbox));
}

/* Mbox message handlers */
void mbox_handler_nix_txsch_alloc(struct otx2_nic *pf,
				  struct nix_txsch_alloc_rsp *rsp)
{
	int lvl, schq;

	/* Setup transmit scheduler list */
	for (lvl = 0; lvl < NIX_TXSCH_LVL_CNT; lvl++)
		for (schq = 0; schq < rsp->schq[lvl]; schq++)
			pf->hw.txschq_list[lvl][schq] =
				rsp->schq_list[lvl][schq];
}

void mbox_handler_npa_lf_alloc(struct otx2_nic *pfvf,
			       struct npa_lf_alloc_rsp *rsp)
{
	pfvf->hw.stack_pg_ptrs = rsp->stack_pg_ptrs;
	pfvf->hw.stack_pg_bytes = rsp->stack_pg_bytes;
}

void mbox_handler_nix_lf_alloc(struct otx2_nic *pfvf,
			       struct nix_lf_alloc_rsp *rsp)
{
	pfvf->hw.sqb_size = rsp->sqb_size;
	pfvf->rx_chan_base = rsp->rx_chan_base;
	pfvf->tx_chan_base = rsp->tx_chan_base;
	ether_addr_copy(pfvf->netdev->dev_addr, rsp->mac_addr);
	pfvf->hw.lso_tsov4_idx = rsp->lso_tsov4_idx;
	pfvf->hw.lso_tsov6_idx = rsp->lso_tsov6_idx;
}

void mbox_handler_msix_offset(struct otx2_nic *pfvf,
			      struct msix_offset_rsp *rsp)
{
	pfvf->hw.npa_msixoff = rsp->npa_msixoff;
	pfvf->hw.nix_msixoff = rsp->nix_msixoff;
}

void otx2_free_cints(struct otx2_nic *pfvf, int n)
{
	struct otx2_qset *qset = &pfvf->qset;
	struct otx2_hw *hw = &pfvf->hw;
	int irq, qidx;

	for (qidx = 0, irq = hw->nix_msixoff + NIX_LF_CINT_VEC_START;
	     qidx < n;
	     qidx++, irq++) {
		int vector = pci_irq_vector(pfvf->pdev, irq);

		irq_set_affinity_hint(vector, NULL);
		free_cpumask_var(hw->affinity_mask[irq]);
		free_irq(vector, &qset->napi[qidx]);
	}
}

void otx2_set_cints_affinity(struct otx2_nic *pfvf)
{
	struct otx2_hw *hw = &pfvf->hw;
	int vec, cpu, irq, cint;

	vec = hw->nix_msixoff + NIX_LF_CINT_VEC_START;
	cpu = cpumask_first(cpu_online_mask);

	/* CQ interrupts */
	for (cint = 0; cint < pfvf->hw.cint_cnt; cint++, vec++) {
		if (!alloc_cpumask_var(&hw->affinity_mask[vec], GFP_KERNEL))
			return;

		cpumask_set_cpu(cpu, hw->affinity_mask[vec]);

		irq = pci_irq_vector(pfvf->pdev, vec);
		irq_set_affinity_hint(irq, hw->affinity_mask[vec]);

		cpu = cpumask_next(cpu, cpu_online_mask);
		if (unlikely(cpu >= nr_cpu_ids))
			cpu = 0;
	}
}
