// SPDX-License-Identifier: GPL-2.0
/* Marvell OcteonTx2 RVU Ethernet driver
 *
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pci.h>
#include <linux/net_tstamp.h>
#include <linux/ethtool.h>
#include <linux/stddef.h>
#include <linux/etherdevice.h>
#include <linux/log2.h>

#include "otx2_common.h"

#define DRV_NAME	"octeontx2-nicpf"
#define DRV_VERSION	"1.0"

struct otx2_stat {
	char name[ETH_GSTRING_LEN];
	unsigned int index;
};

#define OTX2_DEV_STAT(stat) { \
	.name = #stat, \
	.index = offsetof(struct otx2_dev_stats, stat) / sizeof(u64), \
}

static const struct otx2_stat otx2_dev_stats[] = {
	OTX2_DEV_STAT(rx_bytes),
	OTX2_DEV_STAT(rx_frames),
	OTX2_DEV_STAT(rx_ucast_frames),
	OTX2_DEV_STAT(rx_bcast_frames),
	OTX2_DEV_STAT(rx_mcast_frames),
	OTX2_DEV_STAT(rx_drops),

	OTX2_DEV_STAT(tx_bytes),
	OTX2_DEV_STAT(tx_frames),
	OTX2_DEV_STAT(tx_ucast_frames),
	OTX2_DEV_STAT(tx_bcast_frames),
	OTX2_DEV_STAT(tx_mcast_frames),
	OTX2_DEV_STAT(tx_drops),
};

static const struct otx2_stat otx2_queue_stats[] = {
	{ "bytes", 0 },
	{ "frames", 1 },
};

static const unsigned int otx2_n_dev_stats = ARRAY_SIZE(otx2_dev_stats);
static const unsigned int otx2_n_queue_stats = ARRAY_SIZE(otx2_queue_stats);

static void otx2_get_drvinfo(struct net_device *netdev,
			     struct ethtool_drvinfo *info)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);

	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
	strlcpy(info->bus_info, pci_name(pfvf->pdev), sizeof(info->bus_info));
}

static void otx2_get_qset_strings(struct otx2_nic *pfvf, u8 **data, int qset)
{
	int start_qidx = qset * pfvf->hw.rx_queues;
	int qidx, stats;

	for (qidx = 0; qidx < pfvf->hw.rx_queues; qidx++) {
		for (stats = 0; stats < otx2_n_queue_stats; stats++) {
			sprintf(*data, "rxq%d: %s", qidx + start_qidx,
				otx2_queue_stats[stats].name);
			*data += ETH_GSTRING_LEN;
		}
	}
	for (qidx = 0; qidx < pfvf->hw.tx_queues; qidx++) {
		for (stats = 0; stats < otx2_n_queue_stats; stats++) {
			sprintf(*data, "txq%d: %s", qidx + start_qidx,
				otx2_queue_stats[stats].name);
			*data += ETH_GSTRING_LEN;
		}
	}
}

static void otx2_get_strings(struct net_device *netdev, u32 sset, u8 *data)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	int stats;

	if (sset != ETH_SS_STATS)
		return;

	for (stats = 0; stats < otx2_n_dev_stats; stats++) {
		memcpy(data, otx2_dev_stats[stats].name, ETH_GSTRING_LEN);
		data += ETH_GSTRING_LEN;
	}
	otx2_get_qset_strings(pfvf, &data, 0);

	for (stats = 0; stats < CGX_RX_STATS_COUNT; stats++) {
		sprintf(data, "cgx_rxstat%d: ", stats);
		data += ETH_GSTRING_LEN;
	}

	for (stats = 0; stats < CGX_TX_STATS_COUNT; stats++) {
		sprintf(data, "cgx_txstat%d: ", stats);
		data += ETH_GSTRING_LEN;
	}
}

static void otx2_get_qset_stats(struct otx2_nic *pfvf,
				struct ethtool_stats *stats, u64 **data)
{
	int stat, qidx;

	if (!pfvf)
		return;
	for (qidx = 0; qidx < pfvf->hw.rx_queues; qidx++) {
		if (!otx2_update_rq_stats(pfvf, qidx)) {
			for (stat = 0; stat < otx2_n_queue_stats; stat++)
				*((*data)++) = 0;
			continue;
		}
		for (stat = 0; stat < otx2_n_queue_stats; stat++)
			*((*data)++) = ((u64 *)&pfvf->qset.rq[qidx].stats)
				[otx2_queue_stats[stat].index];
	}

	for (qidx = 0; qidx < pfvf->hw.tx_queues; qidx++) {
		if (!otx2_update_sq_stats(pfvf, qidx)) {
			for (stat = 0; stat < otx2_n_queue_stats; stat++)
				*((*data)++) = 0;
			continue;
		}
		for (stat = 0; stat < otx2_n_queue_stats; stat++)
			*((*data)++) = ((u64 *)&pfvf->qset.sq[qidx].stats)
				[otx2_queue_stats[stat].index];
	}
}

/* Get device and per queue statistics */
static void otx2_get_ethtool_stats(struct net_device *netdev,
				   struct ethtool_stats *stats, u64 *data)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	int stat;

	otx2_get_dev_stats(pfvf);
	for (stat = 0; stat < otx2_n_dev_stats; stat++)
		*(data++) = ((u64 *)&pfvf->hw.dev_stats)
				[otx2_dev_stats[stat].index];
	otx2_get_qset_stats(pfvf, stats, &data);
	otx2_update_lmac_stats(pfvf);
	for (stat = 0; stat < CGX_RX_STATS_COUNT; stat++)
		*(data++) = pfvf->hw.cgx_rx_stats[stat];
	for (stat = 0; stat < CGX_TX_STATS_COUNT; stat++)
		*(data++) = pfvf->hw.cgx_tx_stats[stat];
}

static int otx2_get_sset_count(struct net_device *netdev, int sset)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	int qstats_count;

	if (sset != ETH_SS_STATS)
		return -EINVAL;

	qstats_count = otx2_n_queue_stats *
		       (pfvf->hw.rx_queues + pfvf->hw.tx_queues);
	return otx2_n_dev_stats + qstats_count +
		CGX_RX_STATS_COUNT + CGX_TX_STATS_COUNT;
}

/* Get no of queues device supports and current queue count */
static void otx2_get_channels(struct net_device *dev,
			      struct ethtool_channels *channel)
{
	struct otx2_nic *pfvf = netdev_priv(dev);

	memset(channel, 0, sizeof(*channel));
	channel->max_rx = pfvf->hw.max_queues;
	channel->max_tx = pfvf->hw.max_queues;

	channel->rx_count = pfvf->hw.rx_queues;
	channel->tx_count = pfvf->hw.tx_queues;
}

/* Set no of Tx, Rx queues to be used */
static int otx2_set_channels(struct net_device *dev,
			     struct ethtool_channels *channel)
{
	struct otx2_nic *pfvf = netdev_priv(dev);
	bool if_up = netif_running(dev);
	int err = 0;

	if (!channel->rx_count || !channel->tx_count)
		return -EINVAL;
	if (channel->rx_count > pfvf->hw.max_queues)
		return -EINVAL;
	if (channel->tx_count > pfvf->hw.max_queues)
		return -EINVAL;

	if (if_up)
		otx2_stop(dev);

	pfvf->hw.rx_queues = channel->rx_count;
	pfvf->hw.tx_queues = channel->tx_count;
	err = otx2_set_real_num_queues(dev, pfvf->hw.tx_queues,
				       pfvf->hw.rx_queues);
	pfvf->qset.cq_cnt = pfvf->hw.tx_queues +  pfvf->hw.rx_queues;
	if (err)
		return err;

	if (if_up)
		otx2_open(dev);

	netdev_info(dev, "Setting num Tx rings to %d, Rx rings to %d success\n",
		    pfvf->hw.tx_queues, pfvf->hw.rx_queues);

	return err;
}

static void otx2_get_ringparam(struct net_device *netdev,
			       struct ethtool_ringparam *ring)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	struct otx2_qset *qs = &pfvf->qset;

	ring->rx_max_pending = Q_COUNT(Q_SIZE_MAX);
	ring->rx_pending = qs->rqe_cnt;
	ring->tx_max_pending = Q_COUNT(Q_SIZE_MAX);
	ring->tx_pending = qs->sqe_cnt;
}

static int otx2_set_ringparam(struct net_device *netdev,
			      struct ethtool_ringparam *ring)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	bool if_up = netif_running(netdev);
	struct otx2_qset *qs = &pfvf->qset;
	u32 rx_count, tx_count;
	u32 tx_size, rx_size;

	if (ring->rx_mini_pending || ring->rx_jumbo_pending)
		return -EINVAL;

	rx_count = clamp_t(u32, ring->rx_pending,
			   Q_COUNT(Q_SIZE_MIN), Q_COUNT(Q_SIZE_MAX));
	tx_count = clamp_t(u32, ring->tx_pending,
			   Q_COUNT(Q_SIZE_MIN), Q_COUNT(Q_SIZE_MAX));

	if (tx_count == qs->sqe_cnt && rx_count == qs->rqe_cnt)
		return 0;

	/* Permitted lengths are 16 64 256 1K 4K 16K 64K 256K 1M  */
	tx_size = Q_SIZE(tx_count, 3);
	rx_size = Q_SIZE(rx_count, 3);

	/* Due to HW errata #34934 & #34873 RQ.CQ.size >= 1K
	 * and SQ.CQ.size >= 4K to avoid CQ overflow.
	 */
	if ((is_9xxx_pass1_silicon(pfvf->pdev)) &&
	    (tx_size < 0x4 || rx_size < 0x3))
		return 0;

	if (if_up)
		otx2_stop(netdev);

	/* Assigned to the nearest possible exponent. */
	qs->sqe_cnt = Q_COUNT(tx_size);
	qs->rqe_cnt = Q_COUNT(rx_size);

	if (if_up)
		otx2_open(netdev);
	return 0;
}

static int otx2_get_coalesce(struct net_device *netdev,
			     struct ethtool_coalesce *cmd)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);

	cmd->rx_coalesce_usecs = pfvf->cq_time_wait / 10;
	cmd->rx_max_coalesced_frames = pfvf->cq_ecount_wait + 1;
	cmd->tx_coalesce_usecs = pfvf->cq_time_wait / 10;
	cmd->tx_max_coalesced_frames = pfvf->cq_ecount_wait + 1;

	return 0;
}

static int otx2_set_coalesce(struct net_device *netdev,
			     struct ethtool_coalesce *ec)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	bool if_up = netif_running(netdev);

	if (ec->use_adaptive_rx_coalesce || ec->use_adaptive_tx_coalesce ||
	    ec->rx_coalesce_usecs_irq || ec->rx_max_coalesced_frames_irq ||
	    ec->tx_coalesce_usecs_irq || ec->tx_max_coalesced_frames_irq ||
	    ec->stats_block_coalesce_usecs || ec->pkt_rate_low ||
	    ec->rx_coalesce_usecs_low || ec->rx_max_coalesced_frames_low ||
	    ec->tx_coalesce_usecs_low || ec->tx_max_coalesced_frames_low ||
	    ec->pkt_rate_high || ec->rx_coalesce_usecs_high ||
	    ec->rx_max_coalesced_frames_high || ec->tx_coalesce_usecs_high ||
	    ec->tx_max_coalesced_frames_high || ec->rate_sample_interval)
		return -EOPNOTSUPP;

	if (!ec->rx_max_coalesced_frames || !ec->tx_max_coalesced_frames)
		return 0;

	if (if_up)
		otx2_stop(netdev);

	/* RQ and SQ are tied to CQ setting, so any of the below
	 * values reflects on CQ.
	 * cq_time_wait is in multiple of 100ns, rx_coalesce_usecs is in usecs
	 * hence cq_time_wait should be 10 times of rx/tx_coalesce_usecs.
	 */
	if (ec->rx_coalesce_usecs >= CQ_TIMER_THRESH_MAX)
		ec->rx_coalesce_usecs = CQ_TIMER_THRESH_MAX;
	if (ec->tx_coalesce_usecs >= CQ_TIMER_THRESH_MAX)
		ec->tx_coalesce_usecs = CQ_TIMER_THRESH_MAX;

	if (ec->tx_coalesce_usecs == ec->rx_coalesce_usecs) {
		pfvf->cq_time_wait = (u8)ec->rx_coalesce_usecs * 10;
	} else {
		/* If both the values are supplied and is different from
		 * previously set values arbitrarly taking the rx_coalesce_usecs
		 * if any of the value is same as previous value the different
		 * value is taken.
		 */
		pfvf->cq_time_wait = (pfvf->cq_time_wait ==
				      (u8)ec->rx_coalesce_usecs * 10) ?
			(u8)ec->tx_coalesce_usecs * 10 :
			(u8)ec->rx_coalesce_usecs * 10;
	}

	/* @rx_max_coalesced_frames: Maximum number of packets to receive
	 * before an RX interrupt.
	 * A completion interrupt is generated when
	 * NIX_LF_CINT(0..63)_CNT[ECOUNT] > NIX_LF_CINT(0..63)_WAIT[ECOUNT_WAIT]
	 * after either  value is updated. So cq_ecount_wait =
	 * rx/tx_max_coalesced frames -1
	 */
	if (ec->rx_max_coalesced_frames == ec->tx_max_coalesced_frames) {
		pfvf->cq_ecount_wait = ec->rx_max_coalesced_frames - 1;
	} else {
		/* same as above */
		pfvf->cq_ecount_wait = (pfvf->cq_ecount_wait ==
				      ec->rx_max_coalesced_frames - 1) ?
			 ec->tx_max_coalesced_frames - 1 :
			 ec->rx_max_coalesced_frames - 1;
	}

	if (if_up)
		otx2_open(netdev);

	return 0;
}

static const struct ethtool_ops otx2_ethtool_ops = {
	.get_drvinfo		= otx2_get_drvinfo,
	.get_strings		= otx2_get_strings,
	.get_ethtool_stats	= otx2_get_ethtool_stats,
	.get_sset_count		= otx2_get_sset_count,
	.set_channels		= otx2_set_channels,
	.get_channels		= otx2_get_channels,
	.get_ringparam		= otx2_get_ringparam,
	.set_ringparam		= otx2_set_ringparam,
	.get_coalesce		= otx2_get_coalesce,
	.set_coalesce		= otx2_set_coalesce,
};

void otx2_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &otx2_ethtool_ops;
}
