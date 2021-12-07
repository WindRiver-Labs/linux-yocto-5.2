/* SPDX-License-Identifier: GPL-2.0
 * Marvell OcteonTx2 RVU Ethernet driver
 *
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef OTX2_COMMON_H
#define OTX2_COMMON_H

#include <linux/pci.h>

#include <mbox.h>
#include "otx2_reg.h"
#include "otx2_txrx.h"

/* PCI device IDs */
#define PCI_DEVID_OCTEONTX2_RVU_PF              0xA063

/* PCI BAR nos */
#define PCI_CFG_REG_BAR_NUM                     2
#define PCI_MBOX_BAR_NUM                        4

#define NAME_SIZE                               32

enum arua_mapped_qtypes {
	AURA_NIX_RQ,
	AURA_NIX_SQ,
};

/* NIX LF interrupts range*/
#define NIX_LF_QINT_VEC_START			0x00
#define NIX_LF_CINT_VEC_START			0x40
#define NIX_LF_GINT_VEC				0x80
#define NIX_LF_ERR_VEC				0x81
#define NIX_LF_POISON_VEC			0x82

struct  mbox {
	struct otx2_mbox	mbox;
	struct work_struct	mbox_wrk;
	struct otx2_mbox	mbox_up;
	struct work_struct	mbox_up_wrk;
	struct otx2_nic		*pfvf;
	void *bbuf_base; /* Bounce buffer for mbox memory */
};

struct otx2_hw {
	struct pci_dev		*pdev;
	u16                     rx_queues;
	u16                     tx_queues;
	u16			max_queues;
	u16			pool_cnt;
	u16			rqpool_cnt;
	u16			sqpool_cnt;

	/* NPA */
	u32			stack_pg_ptrs;  /* No of ptrs per stack page */
	u32			stack_pg_bytes; /* Size of stack page */
	u16			sqb_size;

	/* MSI-X*/
	u16			npa_msixoff; /* Offset of NPA vectors */
	u16			nix_msixoff; /* Offset of NIX vectors */
	char			*irq_name;
	cpumask_var_t           *affinity_mask;

	u8			cint_cnt; /* CQ interrupt count */
	u16		txschq_list[NIX_TXSCH_LVL_CNT][MAX_TXSCHQ_PER_FUNC];
};

struct otx2_nic {
	void __iomem		*reg_base;
	struct pci_dev		*pdev;
	struct device		*dev;
	struct net_device	*netdev;
	void			*iommu_domain;

	struct otx2_qset	qset;
	struct otx2_hw		hw;
	struct mbox		mbox;
	struct workqueue_struct *mbox_wq;
	u8			intf_down;
	u16			pcifunc;
	u16			rx_chan_base;
	u16			tx_chan_base;
};

/* Register read/write APIs */
static inline void otx2_write64(struct otx2_nic *nic, u64 offset, u64 val)
{
	writeq(val, nic->reg_base + offset);
}

static inline u64 otx2_read64(struct otx2_nic *nic, u64 offset)
{
	return readq(nic->reg_base + offset);
}

/* Mbox bounce buffer APIs */
static inline int otx2_mbox_bbuf_init(struct mbox *mbox, struct pci_dev *pdev)
{
	struct otx2_mbox_dev *mdev;
	struct otx2_mbox *otx2_mbox;

	mbox->bbuf_base = devm_kmalloc(&pdev->dev, MBOX_SIZE, GFP_KERNEL);
	if (!mbox->bbuf_base)
		return -ENOMEM;

	/* Overwrite mbox mbase to point to bounce buffer, so that PF/VF
	 * prepare all mbox messages in bounce buffer instead of directly
	 * in hw mbox memory.
	 */
	otx2_mbox = &mbox->mbox;
	mdev = &otx2_mbox->dev[0];
	mdev->mbase = mbox->bbuf_base;

	return 0;
}

static inline void otx2_sync_mbox_bbuf(struct otx2_mbox *mbox, int devid)
{
	u16 msgs_offset = ALIGN(sizeof(struct mbox_hdr), MBOX_MSG_ALIGN);
	void *hw_mbase = mbox->hwbase + (devid * MBOX_SIZE);
	struct otx2_mbox_dev *mdev = &mbox->dev[devid];
	struct mbox_hdr *hdr;
	u64 msg_size;

	if (mdev->mbase == hw_mbase)
		return;

	hdr = hw_mbase + mbox->rx_start;
	msg_size = hdr->msg_size;

	if (msg_size > mbox->rx_size - msgs_offset)
		msg_size = mbox->rx_size - msgs_offset;

	/* Copy mbox messages from mbox memory to bounce buffer */
	memcpy(mdev->mbase + mbox->rx_start,
	       hw_mbase + mbox->rx_start, msg_size + msgs_offset);
}

/* With the absence of API for 128-bit IO memory access for arm64,
 * implement required operations at place.
 */
#ifdef __BIG_ENDIAN
#define otx2_high(high, low)   (low)
#define otx2_low(high, low)    (high)
#else
#define otx2_high(high, low)   (high)
#define otx2_low(high, low)    (low)
#endif

static inline void otx2_write128(__uint128_t val, void __iomem *addr)
{
	__uint128_t *__addr = (__force __uint128_t *)addr;
	u64 h, l;

	otx2_low(h, l) = (__force u64)cpu_to_le64(val);
	otx2_high(h, l) = (__force u64)cpu_to_le64(val >> 64);

	asm volatile("stp %x[x0], %x[x1], %x[p1]"
		: [p1]"=Ump"(*__addr)
		: [x0]"r"(l), [x1]"r"(h));
}

static inline __uint128_t otx2_read128(const void __iomem *addr)
{
	__uint128_t *__addr = (__force __uint128_t *)addr;
	u64 h, l;

	asm volatile("ldp %x[x0], %x[x1], %x[p1]"
		: [x0]"=r"(l), [x1]"=r"(h)
		: [p1]"Ump"(*__addr));

	return (__uint128_t)le64_to_cpu(otx2_low(h, l)) |
		(((__uint128_t)le64_to_cpu(otx2_high(h, l))) << 64);
}

/* Free pointer to a pool/aura */
static inline void otx2_aura_freeptr(struct otx2_nic *pfvf,
				     int aura, s64 buf)
{
	__uint128_t val;

	val = (__uint128_t)buf;
	val |= ((__uint128_t)aura | BIT_ULL(63)) << 64;

	otx2_write128(val, pfvf->reg_base + NPA_LF_AURA_OP_FREE0);
}

/* Update page ref count */
static inline void otx2_get_page(struct otx2_pool *pool)
{
	if (!pool->page)
		return;

	if (pool->pageref)
		page_ref_add(pool->page, pool->pageref);
	pool->pageref = 0;
	pool->page = NULL;
}

static inline int otx2_get_pool_idx(struct otx2_nic *pfvf, int type, int idx)
{
	if (type == AURA_NIX_SQ)
		return pfvf->hw.rqpool_cnt + idx;

	 /* AURA_NIX_RQ */
	return idx;
}

/* Mbox APIs */
static inline int otx2_sync_mbox_msg(struct mbox *mbox)
{
	int err;

	if (!otx2_mbox_nonempty(&mbox->mbox, 0))
		return 0;
	otx2_mbox_msg_send(&mbox->mbox, 0);
	err = otx2_mbox_wait_for_rsp(&mbox->mbox, 0);
	if (err)
		return err;

	return otx2_mbox_check_rsp_msgs(&mbox->mbox, 0);
}

/* Use this API to send mbox msgs in atomic context
 * where sleeping is not allowed
 */
static inline int otx2_sync_mbox_msg_busy_poll(struct mbox *mbox)
{
	int err;

	if (!otx2_mbox_nonempty(&mbox->mbox, 0))
		return 0;
	otx2_mbox_msg_send(&mbox->mbox, 0);
	err = otx2_mbox_busy_poll_for_rsp(&mbox->mbox, 0);
	if (err)
		return err;

	return otx2_mbox_check_rsp_msgs(&mbox->mbox, 0);
}

#define M(_name, _id, _fn_name, _req_type, _rsp_type)                   \
static struct _req_type __maybe_unused					\
*otx2_mbox_alloc_msg_ ## _fn_name(struct mbox *mbox)                    \
{									\
	struct _req_type *req;						\
									\
	req = (struct _req_type *)otx2_mbox_alloc_msg_rsp(		\
		&mbox->mbox, 0, sizeof(struct _req_type),		\
		sizeof(struct _rsp_type));				\
	if (!req)							\
		return NULL;						\
	req->hdr.sig = OTX2_MBOX_REQ_SIG;				\
	req->hdr.id = _id;						\
	return req;							\
}

MBOX_MESSAGES
#undef M

#define	RVU_PFVF_PF_SHIFT	10
#define	RVU_PFVF_PF_MASK	0x3F
#define	RVU_PFVF_FUNC_SHIFT	0
#define	RVU_PFVF_FUNC_MASK	0x3FF

static inline int rvu_get_pf(u16 pcifunc)
{
	return (pcifunc >> RVU_PFVF_PF_SHIFT) & RVU_PFVF_PF_MASK;
}

/* RVU block related APIs */
int otx2_attach_npa_nix(struct otx2_nic *pfvf);
int otx2_detach_resources(struct mbox *mbox);
int otx2_config_npa(struct otx2_nic *pfvf);
int otx2_sq_aura_pool_init(struct otx2_nic *pfvf);
int otx2_rq_aura_pool_init(struct otx2_nic *pfvf);
int otx2_config_nix(struct otx2_nic *pfvf);
int otx2_config_nix_queues(struct otx2_nic *pfvf);
int otx2_txschq_config(struct otx2_nic *pfvf, int lvl);
int otx2_txsch_alloc(struct otx2_nic *pfvf);
dma_addr_t otx2_alloc_rbuf(struct otx2_nic *pfvf, struct otx2_pool *pool,
			   gfp_t gfp);
int otx2_rxtx_enable(struct otx2_nic *pfvf, bool enable);

/* Mbox handlers */
void mbox_handler_msix_offset(struct otx2_nic *pfvf,
			      struct msix_offset_rsp *rsp);
void mbox_handler_npa_lf_alloc(struct otx2_nic *pfvf,
			       struct npa_lf_alloc_rsp *rsp);
void mbox_handler_nix_lf_alloc(struct otx2_nic *pfvf,
			       struct nix_lf_alloc_rsp *rsp);
void mbox_handler_nix_txsch_alloc(struct otx2_nic *pf,
				  struct nix_txsch_alloc_rsp *rsp);

#endif /* OTX2_COMMON_H */
