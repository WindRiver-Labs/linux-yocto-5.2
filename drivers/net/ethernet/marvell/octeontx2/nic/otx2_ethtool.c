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
#include "otx2_ptp.h"

#define DRV_NAME	"octeontx2-nicpf"
#define DRV_VERSION	"1.0"
#define DRV_VF_NAME	"octeontx2-nicvf"
#define DRV_VF_VERSION	"1.0"

#define OTX2_DEFAULT_ACTION	0x1

static const char otx2_priv_flags_strings[][ETH_GSTRING_LEN] = {
#define OTX2_PRIV_FLAGS_PAM4 BIT(0)
	"pam4",
};

struct otx2_stat {
	char name[ETH_GSTRING_LEN];
	unsigned int index;
};

/* HW device stats */
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

/* Driver level stats */
#define OTX2_DRV_STAT(stat) { \
	.name = #stat, \
	.index = offsetof(struct otx2_drv_stats, stat) / sizeof(atomic_t), \
}

static const struct otx2_stat otx2_drv_stats[] = {
	OTX2_DRV_STAT(rx_fcs_errs),
	OTX2_DRV_STAT(rx_oversize_errs),
	OTX2_DRV_STAT(rx_undersize_errs),
	OTX2_DRV_STAT(rx_csum_errs),
	OTX2_DRV_STAT(rx_len_errs),
	OTX2_DRV_STAT(rx_other_errs),
};

static const struct otx2_stat otx2_queue_stats[] = {
	{ "bytes", 0 },
	{ "frames", 1 },
};

static const unsigned int otx2_n_dev_stats = ARRAY_SIZE(otx2_dev_stats);
static const unsigned int otx2_n_drv_stats = ARRAY_SIZE(otx2_drv_stats);
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

	if (sset == ETH_SS_PRIV_FLAGS) {
		memcpy(data, otx2_priv_flags_strings,
		       ARRAY_SIZE(otx2_priv_flags_strings) * ETH_GSTRING_LEN);
		return;
	}

	if (sset != ETH_SS_STATS)
		return;

	for (stats = 0; stats < otx2_n_dev_stats; stats++) {
		memcpy(data, otx2_dev_stats[stats].name, ETH_GSTRING_LEN);
		data += ETH_GSTRING_LEN;
	}

	for (stats = 0; stats < otx2_n_drv_stats; stats++) {
		memcpy(data, otx2_drv_stats[stats].name, ETH_GSTRING_LEN);
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

	strcpy(data, "reset_count");
	data += ETH_GSTRING_LEN;
	if (pfvf->linfo.fec) {
		sprintf(data, "Fec Corrected Errors: ");
		data += ETH_GSTRING_LEN;
		sprintf(data, "Fec Uncorrected Errors: ");
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

	for (stat = 0; stat < otx2_n_drv_stats; stat++)
		*(data++) = atomic_read(&((atomic_t *)&pfvf->hw.drv_stats)
						[otx2_drv_stats[stat].index]);

	otx2_get_qset_stats(pfvf, stats, &data);
	otx2_update_lmac_stats(pfvf);
	for (stat = 0; stat < CGX_RX_STATS_COUNT; stat++)
		*(data++) = pfvf->hw.cgx_rx_stats[stat];
	for (stat = 0; stat < CGX_TX_STATS_COUNT; stat++)
		*(data++) = pfvf->hw.cgx_tx_stats[stat];
	*(data++) = pfvf->reset_count;
	if (pfvf->linfo.fec) {
		*(data++) = pfvf->hw.cgx_fec_corr_blks;
		*(data++) = pfvf->hw.cgx_fec_uncorr_blks;
	}
}

static int otx2_get_sset_count(struct net_device *netdev, int sset)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	int qstats_count, fec_stats_count = 0;
	bool if_up = netif_running(netdev);

	if (sset == ETH_SS_PRIV_FLAGS)
		return ARRAY_SIZE(otx2_priv_flags_strings);

	if (sset != ETH_SS_STATS)
		return -EINVAL;

	qstats_count = otx2_n_queue_stats *
		       (pfvf->hw.rx_queues + pfvf->hw.tx_queues);

	if (!if_up || !pfvf->linfo.fec) {
		return otx2_n_dev_stats + qstats_count +
			CGX_RX_STATS_COUNT + CGX_TX_STATS_COUNT + 1;
	}
	fec_stats_count = 2;
	otx2_update_lmac_fec_stats(pfvf);
	return otx2_n_dev_stats + qstats_count +
		CGX_RX_STATS_COUNT + CGX_TX_STATS_COUNT + 1 +
		fec_stats_count;
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

static void otx2_get_pauseparam(struct net_device *netdev,
				struct ethtool_pauseparam *pause)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	struct cgx_pause_frm_cfg *req, *rsp;

	req = otx2_mbox_alloc_msg_cgx_cfg_pause_frm(&pfvf->mbox);
	if (!req)
		return;

	if (!otx2_sync_mbox_msg(&pfvf->mbox)) {
		rsp = (struct cgx_pause_frm_cfg *)
		       otx2_mbox_get_rsp(&pfvf->mbox.mbox, 0, &req->hdr);
		pause->rx_pause = rsp->rx_pause;
		pause->tx_pause = rsp->tx_pause;
	}
}

static int otx2_set_pauseparam(struct net_device *netdev,
			       struct ethtool_pauseparam *pause)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	struct cgx_pause_frm_cfg *req;

	if (pause->autoneg)
		return -EOPNOTSUPP;

	req = otx2_mbox_alloc_msg_cgx_cfg_pause_frm(&pfvf->mbox);
	if (!req)
		return -EAGAIN;

	req->set = 1;
	req->rx_pause = pause->rx_pause;
	req->tx_pause = pause->tx_pause;

	return otx2_sync_mbox_msg(&pfvf->mbox);
}

static void otx2_get_ringparam(struct net_device *netdev,
			       struct ethtool_ringparam *ring)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	struct otx2_qset *qs = &pfvf->qset;

	ring->rx_max_pending = Q_COUNT(Q_SIZE_MAX);
	ring->rx_pending = qs->rqe_cnt ? qs->rqe_cnt : Q_COUNT(Q_SIZE_256);
	ring->tx_max_pending = Q_COUNT(Q_SIZE_MAX);
	ring->tx_pending = qs->sqe_cnt ? qs->sqe_cnt : Q_COUNT(Q_SIZE_4K);
}

static int otx2_set_ringparam(struct net_device *netdev,
			      struct ethtool_ringparam *ring)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	bool if_up = netif_running(netdev);
	struct otx2_qset *qs = &pfvf->qset;
	u32 rx_count, tx_count;

	if (ring->rx_mini_pending || ring->rx_jumbo_pending)
		return -EINVAL;

	/* Permitted lengths are 16 64 256 1K 4K 16K 64K 256K 1M  */
	rx_count = clamp_t(u32, ring->rx_pending,
			   Q_COUNT(Q_SIZE_MIN), Q_COUNT(Q_SIZE_MAX));
	/* On some silicon variants a skid or reserved CQEs are
	 * needed to avoid CQ overflow.
	 */
	if (rx_count < pfvf->rq_skid)
		rx_count =  pfvf->rq_skid;
	rx_count = Q_COUNT(Q_SIZE(rx_count, 3));

	/* Due pipelining impact minimum 2000 unused SQ CQE's
	 * need to maintain to avoid CQ overflow, hence the
	 * minimum 4K size.
	 */
	tx_count = clamp_t(u32, ring->tx_pending,
			   Q_COUNT(Q_SIZE_4K), Q_COUNT(Q_SIZE_MAX));
	tx_count = Q_COUNT(Q_SIZE(tx_count, 3));

	if (tx_count == qs->sqe_cnt && rx_count == qs->rqe_cnt)
		return 0;

	if (if_up)
		otx2_stop(netdev);

	/* Assigned to the nearest possible exponent. */
	qs->sqe_cnt = tx_count;
	qs->rqe_cnt = rx_count;

	if (if_up)
		otx2_open(netdev);
	return 0;
}

static int otx2_get_coalesce(struct net_device *netdev,
			     struct ethtool_coalesce *cmd)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);

	cmd->rx_coalesce_usecs = pfvf->cq_time_wait;
	cmd->rx_max_coalesced_frames = pfvf->cq_ecount_wait;
	cmd->tx_coalesce_usecs = pfvf->cq_time_wait;
	cmd->tx_max_coalesced_frames = pfvf->cq_ecount_wait;

	return 0;
}

static int otx2_set_coalesce(struct net_device *netdev,
			     struct ethtool_coalesce *ec)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	int qidx;

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

	/* 'cq_time_wait' is 8bit and is in multiple of 100ns,
	 * so clamp the user given value to the range of 1 to 25usec.
	 */
	ec->rx_coalesce_usecs = clamp_t(u32, ec->rx_coalesce_usecs,
					1, CQ_TIMER_THRESH_MAX);
	ec->tx_coalesce_usecs = clamp_t(u32, ec->tx_coalesce_usecs,
					1, CQ_TIMER_THRESH_MAX);

	/* Rx and Tx are mapped to same CQ, check which one
	 * is changed, if both then choose the min.
	 */
	if (pfvf->cq_time_wait == ec->rx_coalesce_usecs)
		pfvf->cq_time_wait = ec->tx_coalesce_usecs;
	else if (pfvf->cq_time_wait == ec->tx_coalesce_usecs)
		pfvf->cq_time_wait = ec->rx_coalesce_usecs;
	else
		pfvf->cq_time_wait = min_t(u8, ec->rx_coalesce_usecs,
					   ec->tx_coalesce_usecs);

	/* Max ecount_wait supported is 16bit,
	 * so clamp the user given value to the range of 1 to 64k.
	 */
	ec->rx_max_coalesced_frames = clamp_t(u32, ec->rx_max_coalesced_frames,
					      1, U16_MAX);
	ec->tx_max_coalesced_frames = clamp_t(u32, ec->tx_max_coalesced_frames,
					      1, U16_MAX);

	/* Rx and Tx are mapped to same CQ, check which one
	 * is changed, if both then choose the min.
	 */
	if (pfvf->cq_ecount_wait == ec->rx_max_coalesced_frames)
		pfvf->cq_ecount_wait = ec->tx_max_coalesced_frames;
	else if (pfvf->cq_ecount_wait == ec->tx_max_coalesced_frames)
		pfvf->cq_ecount_wait = ec->rx_max_coalesced_frames;
	else
		pfvf->cq_ecount_wait = min_t(u16, ec->rx_max_coalesced_frames,
					     ec->tx_max_coalesced_frames);

	if (netif_running(netdev)) {
		for (qidx = 0; qidx < pfvf->hw.cint_cnt; qidx++)
			otx2_config_irq_coalescing(pfvf, qidx);
	}

	return 0;
}

static int otx2_get_rss_hash_opts(struct otx2_nic *pfvf,
				  struct ethtool_rxnfc *nfc)
{
	struct otx2_rss_info *rss = &pfvf->hw.rss_info;

	if (!(rss->flowkey_cfg &
	    (NIX_FLOW_KEY_TYPE_IPV4 | NIX_FLOW_KEY_TYPE_IPV6)))
		return 0;

	/* Mimimum is IPv4 and IPv6, SIP/DIP */
	nfc->data = RXH_IP_SRC | RXH_IP_DST;

	switch (nfc->flow_type) {
	case TCP_V4_FLOW:
	case TCP_V6_FLOW:
		if (rss->flowkey_cfg & NIX_FLOW_KEY_TYPE_TCP)
			nfc->data |= RXH_L4_B_0_1 | RXH_L4_B_2_3;
		break;
	case UDP_V4_FLOW:
	case UDP_V6_FLOW:
		if (rss->flowkey_cfg & NIX_FLOW_KEY_TYPE_UDP)
			nfc->data |= RXH_L4_B_0_1 | RXH_L4_B_2_3;
		break;
	case SCTP_V4_FLOW:
	case SCTP_V6_FLOW:
		if (rss->flowkey_cfg & NIX_FLOW_KEY_TYPE_SCTP)
			nfc->data |= RXH_L4_B_0_1 | RXH_L4_B_2_3;
		break;
	case AH_ESP_V4_FLOW:
	case AH_V4_FLOW:
	case ESP_V4_FLOW:
	case IPV4_FLOW:
	case AH_ESP_V6_FLOW:
	case AH_V6_FLOW:
	case ESP_V6_FLOW:
	case IPV6_FLOW:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int otx2_set_rss_hash_opts(struct otx2_nic *pfvf,
				  struct ethtool_rxnfc *nfc)
{
	struct otx2_rss_info *rss = &pfvf->hw.rss_info;
	u32 rss_cfg = rss->flowkey_cfg;
	u32 rxh_l4 = RXH_L4_B_0_1 | RXH_L4_B_2_3;

	if (!rss->enable)
		netdev_err(pfvf->netdev, "RSS is disabled, cmd ignored\n");

	/* Mimimum is IPv4 and IPv6, SIP/DIP */
	if (!(nfc->data & RXH_IP_SRC) || !(nfc->data & RXH_IP_DST))
		return -EINVAL;

	switch (nfc->flow_type) {
	case TCP_V4_FLOW:
	case TCP_V6_FLOW:
		/* Different config for v4 and v6 is not supported.
		 * Both of them have to be either 4-tuple or 2-tuple.
		 */
		if ((nfc->data & rxh_l4) == rxh_l4)
			rss_cfg |= NIX_FLOW_KEY_TYPE_TCP;
		else
			rss_cfg &= ~NIX_FLOW_KEY_TYPE_TCP;
		break;
	case UDP_V4_FLOW:
	case UDP_V6_FLOW:
		if ((nfc->data & rxh_l4) == rxh_l4)
			rss_cfg |= NIX_FLOW_KEY_TYPE_UDP;
		else
			rss_cfg &= ~NIX_FLOW_KEY_TYPE_UDP;
		break;
	case SCTP_V4_FLOW:
	case SCTP_V6_FLOW:
		if ((nfc->data & rxh_l4) == rxh_l4)
			rss_cfg |= NIX_FLOW_KEY_TYPE_SCTP;
		else
			rss_cfg &= ~NIX_FLOW_KEY_TYPE_SCTP;
		break;
	case AH_ESP_V4_FLOW:
	case AH_V4_FLOW:
	case ESP_V4_FLOW:
	case IPV4_FLOW:
	case AH_ESP_V6_FLOW:
	case AH_V6_FLOW:
	case ESP_V6_FLOW:
	case IPV6_FLOW:
		rss_cfg = NIX_FLOW_KEY_TYPE_IPV4 | NIX_FLOW_KEY_TYPE_IPV6;
		break;
	default:
		return -EINVAL;
	}

	rss->flowkey_cfg = rss_cfg;
	otx2_set_flowkey_cfg(pfvf);
	return 0;
}

static int otx2_get_rxnfc(struct net_device *dev,
			  struct ethtool_rxnfc *nfc, u32 *rules)
{
	struct otx2_nic *pfvf = netdev_priv(dev);
	int ret = -EOPNOTSUPP;

	switch (nfc->cmd) {
	case ETHTOOL_GRXRINGS:
		nfc->data = pfvf->hw.rx_queues;
		ret = 0;
		break;
	case ETHTOOL_GRXCLSRLCNT:
		nfc->rule_cnt = pfvf->nr_flows;
		ret = 0;
		break;
	case ETHTOOL_GRXCLSRULE:
		ret = otx2_get_flow(pfvf, nfc,  nfc->fs.location);
		break;
	case ETHTOOL_GRXCLSRLALL:
		ret = otx2_get_all_flows(pfvf, nfc, rules);
		break;
	case ETHTOOL_GRXFH:
		return otx2_get_rss_hash_opts(pfvf, nfc);
	default:
		break;
	}
	return ret;
}

int otx2_prepare_flow_request(struct ethtool_rx_flow_spec *fsp,
			      struct npc_install_flow_req *req)
{
	struct ethtool_tcpip4_spec *l4_mask = &fsp->m_u.tcp_ip4_spec;
	struct ethtool_tcpip4_spec *l4_hdr = &fsp->h_u.tcp_ip4_spec;
	struct ethhdr *eth_mask = &fsp->m_u.ether_spec;
	struct ethhdr *eth_hdr = &fsp->h_u.ether_spec;
	struct flow_msg *pmask = &req->mask;
	struct flow_msg *pkt = &req->packet;
	u32 flow_type;

	flow_type = fsp->flow_type & ~(FLOW_EXT | FLOW_MAC_EXT);
	switch (flow_type) {
	/* bits not set in mask are don't care */
	case ETHER_FLOW:
		if (!is_zero_ether_addr(eth_mask->h_source)) {
			ether_addr_copy(pkt->smac, eth_hdr->h_source);
			ether_addr_copy(pmask->smac, eth_mask->h_source);
			req->features |= BIT_ULL(NPC_SMAC);
		}
		if (!is_zero_ether_addr(eth_mask->h_dest)) {
			ether_addr_copy(pkt->dmac, eth_hdr->h_dest);
			ether_addr_copy(pmask->dmac, eth_mask->h_dest);
			req->features |= BIT_ULL(NPC_DMAC);
		}
		if (eth_mask->h_proto) {
			memcpy(&pkt->etype, &eth_hdr->h_proto,
			       sizeof(pkt->etype));
			memcpy(&pmask->etype, &eth_mask->h_proto,
			       sizeof(pmask->etype));
			req->features |= BIT_ULL(NPC_ETYPE);
		}
		break;
	case TCP_V4_FLOW:
	case UDP_V4_FLOW:
		if (l4_mask->ip4src) {
			memcpy(&pkt->ip4src, &l4_hdr->ip4src,
			       sizeof(pkt->ip4src));
			memcpy(&pmask->ip4src, &l4_mask->ip4src,
			       sizeof(pmask->ip4src));
			req->features |= BIT_ULL(NPC_SIP_IPV4);
		}
		if (l4_mask->ip4dst) {
			memcpy(&pkt->ip4dst, &l4_hdr->ip4dst,
			       sizeof(pkt->ip4dst));
			memcpy(&pmask->ip4dst, &l4_mask->ip4dst,
			       sizeof(pmask->ip4dst));
			req->features |= BIT_ULL(NPC_DIP_IPV4);
		}
		if (l4_mask->psrc) {
			memcpy(&pkt->sport, &l4_hdr->psrc, sizeof(pkt->sport));
			memcpy(&pmask->sport, &l4_mask->psrc,
			       sizeof(pmask->sport));
			if (flow_type == UDP_V4_FLOW)
				req->features |= BIT_ULL(NPC_SPORT_UDP);
			else
				req->features |= BIT_ULL(NPC_SPORT_TCP);
		}
		if (l4_mask->pdst) {
			memcpy(&pkt->dport, &l4_hdr->pdst, sizeof(pkt->dport));
			memcpy(&pmask->dport, &l4_mask->pdst,
			       sizeof(pmask->dport));
			if (flow_type == UDP_V4_FLOW)
				req->features |= BIT_ULL(NPC_DPORT_UDP);
			else
				req->features |= BIT_ULL(NPC_DPORT_TCP);
		}
		break;
	default:
		return -ENOTSUPP;
	}
	if (fsp->flow_type & FLOW_EXT) {
		if (fsp->m_ext.vlan_etype)
			return -EINVAL;
		if (fsp->m_ext.vlan_tci) {
			if (fsp->m_ext.vlan_tci != cpu_to_be16(VLAN_VID_MASK))
				return -EINVAL;
			if (be16_to_cpu(fsp->h_ext.vlan_tci) >= VLAN_N_VID)
				return -EINVAL;
			memcpy(&pkt->vlan_tci, &fsp->h_ext.vlan_tci,
			       sizeof(pkt->vlan_tci));
			memcpy(&pmask->vlan_tci, &fsp->m_ext.vlan_tci,
			       sizeof(pmask->vlan_tci));
			req->features |= BIT_ULL(NPC_OUTER_VID);
		}
		/* Not Drop/Direct to queue but use action in default entry */
		if (fsp->m_ext.data[1] &&
		    fsp->h_ext.data[1] == cpu_to_be32(OTX2_DEFAULT_ACTION))
			req->op = NIX_RX_ACTION_DEFAULT;
	}
	if (fsp->flow_type & FLOW_MAC_EXT &&
	    !is_zero_ether_addr(fsp->m_ext.h_dest)) {
		ether_addr_copy(pkt->dmac, fsp->h_ext.h_dest);
		ether_addr_copy(pmask->dmac, fsp->m_ext.h_dest);
		req->features |= BIT_ULL(NPC_DMAC);
	}

	if (!req->features)
		return -ENOTSUPP;

	return 0;
}

static int otx2_set_rxnfc(struct net_device *dev, struct ethtool_rxnfc *nfc)
{
	bool ntuple = !!(dev->features & NETIF_F_NTUPLE);
	struct otx2_nic *pfvf = netdev_priv(dev);
	int ret = -EOPNOTSUPP;

	switch (nfc->cmd) {
	case ETHTOOL_SRXFH:
		ret = otx2_set_rss_hash_opts(pfvf, nfc);
		break;
	case ETHTOOL_SRXCLSRLINS:
		if (netif_running(dev) && ntuple)
			ret = otx2_add_flow(pfvf, &nfc->fs);
		break;
	case ETHTOOL_SRXCLSRLDEL:
		if (netif_running(dev) && ntuple)
			ret = otx2_remove_flow(pfvf, nfc->fs.location);
		break;
	default:
		break;
	}

	return ret;
}

static int otx2vf_get_rxnfc(struct net_device *dev,
			    struct ethtool_rxnfc *nfc, u32 *rules)
{
	struct otx2_nic *pfvf = netdev_priv(dev);
	int ret = -EOPNOTSUPP;

	switch (nfc->cmd) {
	case ETHTOOL_GRXRINGS:
		nfc->data = pfvf->hw.rx_queues;
		ret = 0;
		break;
	case ETHTOOL_GRXFH:
		return otx2_get_rss_hash_opts(pfvf, nfc);
	default:
		break;
	}
	return ret;
}

static int otx2vf_set_rxnfc(struct net_device *dev, struct ethtool_rxnfc *nfc)
{
	struct otx2_nic *pfvf = netdev_priv(dev);
	int ret = -EOPNOTSUPP;

	switch (nfc->cmd) {
	case ETHTOOL_SRXFH:
		ret = otx2_set_rss_hash_opts(pfvf, nfc);
		break;
	default:
		break;
	}

	return ret;
}

static u32 otx2_get_rxfh_key_size(struct net_device *netdev)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	struct otx2_rss_info *rss = &pfvf->hw.rss_info;

	return sizeof(rss->key);
}

static u32 otx2_get_rxfh_indir_size(struct net_device *dev)
{
	struct otx2_nic *pfvf = netdev_priv(dev);

	return pfvf->hw.rss_info.rss_size;
}

/* Get RSS configuration*/
static int otx2_get_rxfh(struct net_device *dev, u32 *indir,
			 u8 *hkey, u8 *hfunc)
{
	struct otx2_nic *pfvf = netdev_priv(dev);
	struct otx2_rss_info *rss = &pfvf->hw.rss_info;
	int idx;

	if (indir) {
		for (idx = 0; idx < rss->rss_size; idx++)
			indir[idx] = rss->ind_tbl[idx];
	}

	if (hkey)
		memcpy(hkey, rss->key, sizeof(rss->key));

	if (hfunc)
		*hfunc = ETH_RSS_HASH_TOP;

	return 0;
}

/* Configure RSS table and hash key*/
static int otx2_set_rxfh(struct net_device *dev, const u32 *indir,
			 const u8 *hkey, const u8 hfunc)
{
	struct otx2_nic *pfvf = netdev_priv(dev);
	struct otx2_rss_info *rss = &pfvf->hw.rss_info;
	int idx;

	if (hfunc != ETH_RSS_HASH_NO_CHANGE && hfunc != ETH_RSS_HASH_TOP)
		return -EOPNOTSUPP;

	if (!rss->enable) {
		netdev_err(dev, "RSS is disabled, cannot change settings\n");
		return -EIO;
	}

	if (indir) {
		for (idx = 0; idx < rss->rss_size; idx++)
			rss->ind_tbl[idx] = indir[idx];
	}

	if (hkey) {
		memcpy(rss->key, hkey, sizeof(rss->key));
		otx2_set_rss_key(pfvf);
	}

	otx2_set_rss_table(pfvf);
	return 0;
}

static int otx2_get_ts_info(struct net_device *netdev,
			    struct ethtool_ts_info *info)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);

	if (!pfvf->ptp)
		return ethtool_op_get_ts_info(netdev, info);

	info->so_timestamping = SOF_TIMESTAMPING_TX_SOFTWARE |
				SOF_TIMESTAMPING_RX_SOFTWARE |
				SOF_TIMESTAMPING_SOFTWARE |
				SOF_TIMESTAMPING_TX_HARDWARE |
				SOF_TIMESTAMPING_RX_HARDWARE |
				SOF_TIMESTAMPING_RAW_HARDWARE;

	info->phc_index = otx2_ptp_clock_index(pfvf);

	info->tx_types = (1 << HWTSTAMP_TX_OFF) | (1 << HWTSTAMP_TX_ON);

	info->rx_filters = (1 << HWTSTAMP_FILTER_NONE) |
			   (1 << HWTSTAMP_FILTER_ALL);

	return 0;
}

static u32 otx2_get_msglevel(struct net_device *netdev)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);

	return pfvf->msg_enable;
}

static void otx2_set_msglevel(struct net_device *netdev, u32 val)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);

	pfvf->msg_enable = val;
}

static void otx2_get_fec_info(u64 index, int mode, struct ethtool_link_ksettings
			      *link_ksettings)
{
	switch (index) {
	case OTX2_FEC_NONE:
		if (mode)
			ethtool_link_ksettings_add_link_mode(link_ksettings,
							     advertising,
							     FEC_NONE);
		else
			ethtool_link_ksettings_add_link_mode(link_ksettings,
							     supported,
							     FEC_NONE);
		break;
	case OTX2_FEC_BASER:
		if (mode)
			ethtool_link_ksettings_add_link_mode(link_ksettings,
							     advertising,
							     FEC_BASER);
		else
			ethtool_link_ksettings_add_link_mode(link_ksettings,
							     supported,
							     FEC_BASER);
		break;
	case OTX2_FEC_RS:
		if (mode)
			ethtool_link_ksettings_add_link_mode(link_ksettings,
							     advertising,
							     FEC_RS);
		else
			ethtool_link_ksettings_add_link_mode(link_ksettings,
							     supported,
							     FEC_RS);
		break;
	case OTX2_FEC_BASER | OTX2_FEC_RS:
		if (mode) {
			ethtool_link_ksettings_add_link_mode(link_ksettings,
							     advertising,
							     FEC_BASER);
			ethtool_link_ksettings_add_link_mode(link_ksettings,
							     advertising,
							     FEC_RS);
		} else {
			ethtool_link_ksettings_add_link_mode(link_ksettings,
							     supported,
							     FEC_BASER);
			ethtool_link_ksettings_add_link_mode(link_ksettings,
							     supported,
							     FEC_RS);
		}

		break;
	}
}

static struct cgx_fw_data *otx2_get_fwdata(struct otx2_nic *pfvf)
{
	struct cgx_fw_data *rsp = NULL;
	struct msg_req *req;
	int err = 0;

	otx2_mbox_lock(&pfvf->mbox);
	req = otx2_mbox_alloc_msg_cgx_get_aux_link_info(&pfvf->mbox);
	if (!req) {
		otx2_mbox_unlock(&pfvf->mbox);
		return ERR_PTR(-ENOMEM);
	}

	err = otx2_sync_mbox_msg(&pfvf->mbox);
	if (!err) {
		rsp = (struct cgx_fw_data *)
			otx2_mbox_get_rsp(&pfvf->mbox.mbox, 0, &req->hdr);
	} else {
		rsp = ERR_PTR(err);
	}

	otx2_mbox_unlock(&pfvf->mbox);
	return rsp;
}

static int otx2_get_module_info(struct net_device *netdev,
				struct ethtool_modinfo *modinfo)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	struct cgx_fw_data *rsp;

	rsp = otx2_get_fwdata(pfvf);
	if (IS_ERR(rsp))
		return PTR_ERR(rsp);

	modinfo->type = rsp->fwdata.sfp_eeprom.sff_id;
	modinfo->eeprom_len = SFP_EEPROM_SIZE;
	return 0;
}

static int otx2_get_module_eeprom(struct net_device *netdev,
				  struct ethtool_eeprom *ee,
				  u8 *data)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	struct cgx_fw_data *rsp;

	rsp = otx2_get_fwdata(pfvf);
	if (IS_ERR(rsp))
		return PTR_ERR(rsp);

	memcpy(data, &rsp->fwdata.sfp_eeprom.buf, ee->len);

	return 0;
}

static int otx2_get_link_ksettings(struct net_device *netdev,
				   struct ethtool_link_ksettings *cmd)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	u32 supported = 0, advertising = 0;
	struct cgx_fw_data *rsp = NULL;

	cmd->base.duplex = pfvf->linfo.full_duplex;
	cmd->base.speed = pfvf->linfo.speed;
	cmd->base.autoneg = pfvf->linfo.an;
	cmd->base.port = pfvf->linfo.port;

	rsp = otx2_get_fwdata(pfvf);
	if (IS_ERR(rsp))
		return PTR_ERR(rsp);

	if (rsp->fwdata.supported_an)
		supported |= SUPPORTED_Autoneg;
	ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.advertising,
						advertising);
	otx2_get_fec_info(rsp->fwdata.advertised_fec, 1, cmd);

	ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.supported,
						supported);
	otx2_get_fec_info(rsp->fwdata.supported_fec, 0, cmd);

	return 0;
}

static int otx2_set_link_ksettings(struct net_device *netdev,
				   const struct ethtool_link_ksettings *cmd)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	struct cgx_set_link_mode_req *req;
	struct cgx_set_link_mode_rsp *rsp;
	int err = 0;

	otx2_mbox_lock(&pfvf->mbox);
	req = otx2_mbox_alloc_msg_cgx_set_link_mode(&pfvf->mbox);
	if (!req) {
		otx2_mbox_unlock(&pfvf->mbox);
		return -EAGAIN;
	}
	req->args.speed = cmd->base.speed;
	/*The full_duplex variable in linkinfo takes 1 for full duplex and 0
	 * for half duplex. But the set_link_mode command in atf requires the
	 * argument to map 1 for half duplex and 0 for full duplex. Toggling to
	 * the current value in linkinfo for the purpose.
	 */
	if (cmd->base.duplex == DUPLEX_UNKNOWN)
		req->args.duplex = pfvf->linfo.full_duplex ^ 0x1;
	else
		req->args.duplex = cmd->base.duplex ^ 0x1;
	req->args.an =  cmd->base.autoneg;

	err =  otx2_sync_mbox_msg(&pfvf->mbox);
	if (!err) {
		rsp = (struct cgx_set_link_mode_rsp *)
			otx2_mbox_get_rsp(&pfvf->mbox.mbox, 0, &req->hdr);
		if (rsp->status)
			err =  rsp->status;
	}
	otx2_mbox_unlock(&pfvf->mbox);
	return err;
}

static u32 otx2_get_link(struct net_device *netdev)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);

	if (is_otx2_lbkvf(pfvf->pdev))
		return 1;
	return pfvf->linfo.link_up;
}

static int otx2_get_fecparam(struct net_device *netdev,
			     struct ethtool_fecparam *fecparam)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	struct cgx_fw_data *rsp;
	int fec[] = {
		ETHTOOL_FEC_OFF,
		ETHTOOL_FEC_BASER,
		ETHTOOL_FEC_RS,
		ETHTOOL_FEC_BASER | ETHTOOL_FEC_RS};
#define FEC_MAX_INDEX 3
	if (pfvf->linfo.fec < FEC_MAX_INDEX)
		fecparam->active_fec = fec[pfvf->linfo.fec];

	rsp = otx2_get_fwdata(pfvf);
	if (IS_ERR(rsp))
		return PTR_ERR(rsp);

	if (rsp->fwdata.supported_fec <= FEC_MAX_INDEX) {
		if (!rsp->fwdata.supported_fec)
			fecparam->fec = ETHTOOL_FEC_NONE;
		else
			fecparam->fec = fec[rsp->fwdata.supported_fec];
	}
	return 0;
}

static int otx2_set_fecparam(struct net_device *netdev,
			     struct ethtool_fecparam *fecparam)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	struct fec_mode *req, *rsp;
	int err = 0, fec = 0;

	switch (fecparam->fec) {
	case ETHTOOL_FEC_OFF:
		fec = OTX2_FEC_NONE;
		break;
	case ETHTOOL_FEC_RS:
		fec = OTX2_FEC_RS;
		break;
	case ETHTOOL_FEC_BASER:
		fec = OTX2_FEC_BASER;
		break;
	default:
		fec = OTX2_FEC_NONE;
		break;
	}

	if (fec == pfvf->linfo.fec)
		return 0;

	otx2_mbox_lock(&pfvf->mbox);
	req = otx2_mbox_alloc_msg_cgx_set_fec_param(&pfvf->mbox);
	if (!req) {
		err = -EAGAIN;
		goto end;
	}
	req->fec = fec;
	err = otx2_sync_mbox_msg(&pfvf->mbox);
	if (err)
		goto end;

	rsp = (struct fec_mode *)otx2_mbox_get_rsp(&pfvf->mbox.mbox,
						   0, &req->hdr);
	if (rsp->fec >= 0) {
		pfvf->linfo.fec = rsp->fec;
		pfvf->hw.cgx_fec_corr_blks = 0;
		pfvf->hw.cgx_fec_uncorr_blks = 0;

	} else {
		err = rsp->fec;
	}

end:	otx2_mbox_unlock(&pfvf->mbox);
	return err;
}

static struct ethtool_ops otx2_ethtool_ops = {
	.get_link		= otx2_get_link,
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
	.get_rxnfc		= otx2_get_rxnfc,
	.set_rxnfc              = otx2_set_rxnfc,
	.get_rxfh_key_size	= otx2_get_rxfh_key_size,
	.get_rxfh_indir_size	= otx2_get_rxfh_indir_size,
	.get_rxfh		= otx2_get_rxfh,
	.set_rxfh		= otx2_set_rxfh,
	.get_ts_info		= otx2_get_ts_info,
	.get_msglevel		= otx2_get_msglevel,
	.set_msglevel		= otx2_set_msglevel,
	.get_link_ksettings     = otx2_get_link_ksettings,
	.set_link_ksettings     = otx2_set_link_ksettings,
	.get_pauseparam		= otx2_get_pauseparam,
	.set_pauseparam		= otx2_set_pauseparam,
	.get_fecparam		= otx2_get_fecparam,
	.set_fecparam		= otx2_set_fecparam,
	.get_module_info	= otx2_get_module_info,
	.get_module_eeprom	= otx2_get_module_eeprom,
};

static int otx2_set_priv_flags(struct net_device *netdev, u32 priv_flags)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	struct cgx_phy_mod_type *req, *rsp;
	int rc = 0;

	otx2_mbox_lock(&pfvf->mbox);
	req = otx2_mbox_alloc_msg_cgx_set_phy_mod_type(&pfvf->mbox);
	if (!req) {
		rc = -EAGAIN;
		goto end;
	}
	req->mod = priv_flags & OTX2_PRIV_FLAGS_PAM4;
	rc = otx2_sync_mbox_msg(&pfvf->mbox);
	if (rc)
		goto end;

	rsp = (struct cgx_phy_mod_type *)otx2_mbox_get_rsp(&pfvf->mbox.mbox, 0,
							   &req->hdr);
	if (IS_ERR(rsp)) {
		rc = PTR_ERR(rsp);
		goto end;
	}
	if (rsp->hdr.rc) {
		rc = rsp->hdr.rc;
		goto end;
	}

end:	otx2_mbox_unlock(&pfvf->mbox);
	return rc;
}

static u32 otx2_get_priv_flags(struct net_device *netdev)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	struct cgx_fw_data *rsp;
	u32 priv_flags = 0;

	rsp = otx2_get_fwdata(pfvf);

	if (IS_ERR(rsp))
		return 0;

	if (rsp->fwdata.phy.mod_type)
		priv_flags |= OTX2_PRIV_FLAGS_PAM4;

	return priv_flags;
}

void otx2_set_ethtool_ops(struct net_device *netdev)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);
	struct cgx_fw_data *rsp;

	rsp = otx2_get_fwdata(pfvf);

	if (!IS_ERR(rsp) && rsp->fwdata.phy.can_change_mod_type) {
		otx2_ethtool_ops.set_priv_flags = otx2_set_priv_flags;
		otx2_ethtool_ops.get_priv_flags = otx2_get_priv_flags;
	}

	netdev->ethtool_ops = &otx2_ethtool_ops;
}

/* VF's ethtool APIs */
static void otx2vf_get_drvinfo(struct net_device *netdev,
			       struct ethtool_drvinfo *info)
{
	struct otx2_nic *vf = netdev_priv(netdev);

	strlcpy(info->driver, DRV_VF_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VF_VERSION, sizeof(info->version));
	strlcpy(info->bus_info, pci_name(vf->pdev), sizeof(info->bus_info));
}

static void otx2vf_get_strings(struct net_device *netdev, u32 sset, u8 *data)
{
	struct otx2_nic *vf = netdev_priv(netdev);
	int stats;

	if (sset != ETH_SS_STATS)
		return;

	for (stats = 0; stats < otx2_n_dev_stats; stats++) {
		memcpy(data, otx2_dev_stats[stats].name, ETH_GSTRING_LEN);
		data += ETH_GSTRING_LEN;
	}

	otx2_get_qset_strings(vf, &data, 0);
}

static void otx2vf_get_ethtool_stats(struct net_device *netdev,
				     struct ethtool_stats *stats, u64 *data)
{
	struct otx2_nic *vf = netdev_priv(netdev);
	int stat;

	otx2_get_dev_stats(vf);

	for (stat = 0; stat < otx2_n_dev_stats; stat++) {
		*data = ((u64 *)&vf->hw.dev_stats)[otx2_dev_stats[stat].index];
		data++;
	}

	otx2_get_qset_stats(vf, stats, &data);
}

static int otx2vf_get_sset_count(struct net_device *netdev, int sset)
{
	struct otx2_nic *vf = netdev_priv(netdev);

	if (sset != ETH_SS_STATS)
		return -EINVAL;

	return otx2_n_dev_stats +
	       otx2_n_queue_stats * (vf->hw.rx_queues + vf->hw.tx_queues);
}

static int otx2vf_get_link_ksettings(struct net_device *netdev,
				     struct ethtool_link_ksettings *cmd)
{
	struct otx2_nic *pfvf = netdev_priv(netdev);

	if (is_otx2_lbkvf(pfvf->pdev)) {
		cmd->base.port = PORT_OTHER;
		cmd->base.duplex = DUPLEX_FULL;
		cmd->base.speed = SPEED_100000;
	} else {
		return	otx2_get_link_ksettings(netdev, cmd);
	}
	return 0;
}
static const struct ethtool_ops otx2vf_ethtool_ops = {
	.get_link		= otx2_get_link,
	.get_drvinfo		= otx2vf_get_drvinfo,
	.get_strings		= otx2vf_get_strings,
	.get_ethtool_stats	= otx2vf_get_ethtool_stats,
	.get_sset_count		= otx2vf_get_sset_count,
	.set_channels		= otx2_set_channels,
	.get_channels		= otx2_get_channels,
	.get_rxnfc		= otx2vf_get_rxnfc,
	.set_rxnfc              = otx2vf_set_rxnfc,
	.get_rxfh_key_size	= otx2_get_rxfh_key_size,
	.get_rxfh_indir_size	= otx2_get_rxfh_indir_size,
	.get_rxfh		= otx2_get_rxfh,
	.set_rxfh		= otx2_set_rxfh,
	.get_ringparam		= otx2_get_ringparam,
	.set_ringparam		= otx2_set_ringparam,
	.get_coalesce		= otx2_get_coalesce,
	.set_coalesce		= otx2_set_coalesce,
	.get_pauseparam		= otx2_get_pauseparam,
	.set_pauseparam		= otx2_set_pauseparam,
	.get_link_ksettings     = otx2vf_get_link_ksettings,
};

void otx2vf_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &otx2vf_ethtool_ops;
}
EXPORT_SYMBOL(otx2vf_set_ethtool_ops);
