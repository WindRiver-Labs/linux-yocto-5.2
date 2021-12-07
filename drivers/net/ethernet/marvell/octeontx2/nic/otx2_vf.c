// SPDX-License-Identifier: GPL-2.0
/* Marvell OcteonTx2 RVU Virtual Function ethernet driver
 *
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "otx2_common.h"
#include "otx2_reg.h"

#define DRV_NAME	"octeontx2-nicvf"
#define DRV_STRING	"Marvell OcteonTX2 NIC Virtual Function Driver"
#define DRV_VERSION	"1.0"

static const struct pci_device_id otx2_vf_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, PCI_DEVID_OCTEONTX2_RVU_VF) },
	{ }
};

MODULE_AUTHOR("Marvell International Ltd.");
MODULE_DESCRIPTION(DRV_STRING);
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
MODULE_DEVICE_TABLE(pci, otx2_vf_id_table);

/**
 * RVU VF Interrupt Vector Enumeration
 */
enum {
	RVU_VF_INT_VEC_MBOX = 0x0,
};

static void otx2vf_process_vfaf_mbox_msg(struct otx2_nic *vf,
					 struct mbox_msghdr *msg)
{
	if (msg->id >= MBOX_MSG_MAX) {
		dev_err(vf->dev,
			"Mbox msg with unknown ID %d\n", msg->id);
		return;
	}

	if (msg->sig != OTX2_MBOX_RSP_SIG) {
		dev_err(vf->dev,
			"Mbox msg with wrong signature %x, ID %d\n",
			msg->sig, msg->id);
		return;
	}

	if (msg->rc == MBOX_MSG_INVALID) {
		dev_err(vf->dev,
			"PF/AF says the sent msg(s) %d were invalid\n",
			msg->id);
		return;
	}

	switch (msg->id) {
	default:
		if (msg->rc)
			dev_err(vf->dev,
				"Mbox msg response has err %d, ID %d\n",
				msg->rc, msg->id);
	}
}

static void otx2vf_vfaf_mbox_handler(struct work_struct *work)
{
	struct otx2_mbox_dev *mdev;
	struct mbox_hdr *rsp_hdr;
	struct mbox_msghdr *msg;
	struct otx2_mbox *mbox;
	struct mbox *af_mbox;
	int offset, id;

	af_mbox = container_of(work, struct mbox, mbox_wrk);
	mbox = &af_mbox->mbox;
	mdev = &mbox->dev[0];
	rsp_hdr = (struct mbox_hdr *)(mdev->mbase + mbox->rx_start);
	if (af_mbox->num_msgs == 0)
		return;
	offset = mbox->rx_start + ALIGN(sizeof(*rsp_hdr), MBOX_MSG_ALIGN);

	for (id = 0; id < af_mbox->num_msgs; id++) {
		msg = (struct mbox_msghdr *)(mdev->mbase + offset);
		otx2vf_process_vfaf_mbox_msg(af_mbox->pfvf, msg);
		offset = mbox->rx_start + msg->next_msgoff;
		mdev->msgs_acked++;
	}

	otx2_mbox_reset(mbox, 0);

	/* Clear the IRQ */
	smp_wmb();
	otx2_write64(af_mbox->pfvf, RVU_VF_INT, BIT_ULL(0));
}

static int otx2vf_process_mbox_msg_up(struct otx2_nic *vf,
				      struct mbox_msghdr *req)
{
	/* Check if valid, if not reply with a invalid msg */
	if (req->sig != OTX2_MBOX_REQ_SIG) {
		otx2_reply_invalid_msg(&vf->mbox.mbox_up, 0, 0, req->id);
		return -ENODEV;
	}

	switch (req->id) {
	default:
		otx2_reply_invalid_msg(&vf->mbox.mbox_up, 0, 0, req->id);
		return -ENODEV;
	}
	return 0;
}

static void otx2vf_vfaf_mbox_up_handler(struct work_struct *work)
{
	struct otx2_mbox_dev *mdev;
	struct mbox_hdr *rsp_hdr;
	struct mbox_msghdr *msg;
	struct otx2_mbox *mbox;
	struct mbox *vf_mbox;
	struct otx2_nic *vf;
	int offset, id;

	vf_mbox = container_of(work, struct mbox, mbox_up_wrk);
	vf =  vf_mbox->pfvf;
	mbox = &vf_mbox->mbox_up;
	mdev = &mbox->dev[0];

	rsp_hdr = (struct mbox_hdr *)(mdev->mbase + mbox->rx_start);
	if (vf_mbox->up_num_msgs == 0)
		return;

	offset = mbox->rx_start + ALIGN(sizeof(*rsp_hdr), MBOX_MSG_ALIGN);

	for (id = 0; id < vf_mbox->up_num_msgs; id++) {
		msg = (struct mbox_msghdr *)(mdev->mbase + offset);
		otx2vf_process_mbox_msg_up(vf, msg);
		offset = mbox->rx_start + msg->next_msgoff;
	}

	otx2_mbox_msg_send(mbox, 0);
}

static irqreturn_t otx2vf_vfaf_mbox_intr_handler(int irq, void *vf_irq)
{
	struct otx2_nic *vf = (struct otx2_nic *)vf_irq;
	struct otx2_mbox_dev *mdev;
	struct otx2_mbox *mbox;
	struct mbox_hdr *hdr;

	/* Read latest mbox data */
	smp_rmb();

	/* Check for PF => VF response messages */
	mbox = &vf->mbox.mbox;
	mdev = &mbox->dev[0];
	otx2_sync_mbox_bbuf(mbox, 0);

	hdr = (struct mbox_hdr *)(mdev->mbase + mbox->rx_start);
	if (hdr->num_msgs) {
		vf->mbox.num_msgs = hdr->num_msgs;
		hdr->num_msgs = 0;
		memset(mbox->hwbase + mbox->rx_start, 0,
		       ALIGN(sizeof(struct mbox_hdr), sizeof(u64)));
		queue_work(vf->mbox_wq, &vf->mbox.mbox_wrk);
	}
	/* Check for PF => VF notification messages */
	mbox = &vf->mbox.mbox_up;
	mdev = &mbox->dev[0];
	otx2_sync_mbox_bbuf(mbox, 0);

	hdr = (struct mbox_hdr *)(mdev->mbase + mbox->rx_start);
	if (hdr->num_msgs) {
		vf->mbox.up_num_msgs = hdr->num_msgs;
		hdr->num_msgs = 0;
		memset(mbox->hwbase + mbox->rx_start, 0,
		       ALIGN(sizeof(struct mbox_hdr), sizeof(u64)));
		queue_work(vf->mbox_wq, &vf->mbox.mbox_up_wrk);
	}
	/* Clear the IRQ */
	otx2_write64(vf, RVU_VF_INT, BIT_ULL(0));

	return IRQ_HANDLED;
}

static void otx2vf_disable_mbox_intr(struct otx2_nic *vf)
{
	int vector = pci_irq_vector(vf->pdev, RVU_VF_INT_VEC_MBOX);

	/* Disable VF => PF mailbox IRQ */
	otx2_write64(vf, RVU_VF_INT_ENA_W1C, BIT_ULL(0));
	free_irq(vector, vf);
}

static int otx2vf_register_mbox_intr(struct otx2_nic *vf)
{
	struct otx2_hw *hw = &vf->hw;
	struct msg_req *req;
	char *irq_name;
	int err;

	/* Register mailbox interrupt handler */
	irq_name = &hw->irq_name[RVU_VF_INT_VEC_MBOX * NAME_SIZE];
	snprintf(irq_name, NAME_SIZE, "RVUVFAF Mbox");
	err = request_irq(pci_irq_vector(vf->pdev, RVU_VF_INT_VEC_MBOX),
			  otx2vf_vfaf_mbox_intr_handler, 0, irq_name, vf);
	if (err) {
		dev_err(vf->dev,
			"RVUPF: IRQ registration failed for VFAF mbox irq\n");
		return err;
	}

	/* Enable mailbox interrupt for msgs coming from PF.
	 * First clear to avoid spurious interrupts, if any.
	 */
	otx2_write64(vf, RVU_VF_INT, BIT_ULL(0));
	otx2_write64(vf, RVU_VF_INT_ENA_W1S, BIT_ULL(0));

	/* Check mailbox communication with PF */
	req = otx2_mbox_alloc_msg_ready(&vf->mbox);
	if (!req) {
		otx2vf_disable_mbox_intr(vf);
		return -ENOMEM;
	}

	err = otx2_sync_mbox_msg(&vf->mbox);
	if (err) {
		dev_warn(vf->dev,
			 "AF not responding to mailbox, deferring probe\n");
		otx2vf_disable_mbox_intr(vf);
		return -EPROBE_DEFER;
	}
	return 0;
}

static void otx2vf_vfaf_mbox_destroy(struct otx2_nic *vf)
{
	struct mbox *mbox = &vf->mbox;

	if (vf->mbox_wq) {
		flush_workqueue(vf->mbox_wq);
		destroy_workqueue(vf->mbox_wq);
		vf->mbox_wq = NULL;
	}

	if (mbox->mbox.hwbase)
		iounmap((void __iomem *)mbox->mbox.hwbase);

	otx2_mbox_destroy(&mbox->mbox);
	otx2_mbox_destroy(&mbox->mbox_up);
}

static int otx2vf_vfaf_mbox_init(struct otx2_nic *vf)
{
	struct mbox *mbox = &vf->mbox;
	void __iomem *hwbase;
	int err;

	mbox->pfvf = vf;
	vf->mbox_wq = alloc_workqueue("otx2_vfaf_mailbox",
				      WQ_UNBOUND | WQ_HIGHPRI |
				      WQ_MEM_RECLAIM, 1);
	if (!vf->mbox_wq)
		return -ENOMEM;

	/* Mailbox is a reserved memory (in RAM) region shared between
	 * admin function (i.e PF0) and this VF, shouldn't be mapped as
	 * device memory to allow unaligned accesses.
	 */
	hwbase = ioremap_wc(pci_resource_start(vf->pdev, PCI_MBOX_BAR_NUM),
			    pci_resource_len(vf->pdev, PCI_MBOX_BAR_NUM));
	if (!hwbase) {
		dev_err(vf->dev, "Unable to map VFAF mailbox region\n");
		err = -ENOMEM;
		goto exit;
	}

	err = otx2_mbox_init(&mbox->mbox, hwbase, vf->pdev, vf->reg_base,
			     MBOX_DIR_VFPF, 1);
	if (err)
		goto exit;

	err = otx2_mbox_init(&mbox->mbox_up, hwbase, vf->pdev, vf->reg_base,
			     MBOX_DIR_VFPF_UP, 1);
	if (err)
		goto exit;

	err = otx2_mbox_bbuf_init(mbox, vf->pdev);
	if (err)
		goto exit;

	INIT_WORK(&mbox->mbox_wrk, otx2vf_vfaf_mbox_handler);
	INIT_WORK(&mbox->mbox_up_wrk, otx2vf_vfaf_mbox_up_handler);
	otx2_mbox_lock_init(&vf->mbox);

	return 0;
exit:
	destroy_workqueue(vf->mbox_wq);
	return err;
}

static const struct net_device_ops otx2vf_netdev_ops = {
};

static int otx2vf_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int num_vec = pci_msix_vec_count(pdev);
	struct device *dev = &pdev->dev;
	struct net_device *netdev;
	struct otx2_nic *vf;
	struct otx2_hw *hw;
	int err, qcount;

	err = pcim_enable_device(pdev);
	if (err) {
		dev_err(dev, "Failed to enable PCI device\n");
		return err;
	}

	err = pci_request_regions(pdev, DRV_NAME);
	if (err) {
		dev_err(dev, "PCI request regions failed 0x%x\n", err);
		return err;
	}

	err = pci_set_dma_mask(pdev, DMA_BIT_MASK(48));
	if (err) {
		dev_err(dev, "Unable to set DMA mask\n");
		goto err_release_regions;
	}

	err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(48));
	if (err) {
		dev_err(dev, "Unable to set consistent DMA mask\n");
		goto err_release_regions;
	}

	pci_set_master(pdev);

	qcount = num_online_cpus();
	netdev = alloc_etherdev_mqs(sizeof(*vf), qcount, qcount);
	if (!netdev) {
		err = -ENOMEM;
		goto err_release_regions;
	}

	pci_set_drvdata(pdev, netdev);
	SET_NETDEV_DEV(netdev, &pdev->dev);
	vf = netdev_priv(netdev);
	vf->netdev = netdev;
	vf->pdev = pdev;
	vf->dev = dev;
	vf->iommu_domain = iommu_get_domain_for_dev(dev);
	hw = &vf->hw;
	hw->pdev = vf->pdev;
	hw->rx_queues = qcount;
	hw->tx_queues = qcount;
	hw->max_queues = qcount;

	hw->irq_name = devm_kmalloc_array(&hw->pdev->dev, num_vec, NAME_SIZE,
					  GFP_KERNEL);
	if (!hw->irq_name)
		goto err_free_netdev;

	hw->affinity_mask = devm_kcalloc(&hw->pdev->dev, num_vec,
					 sizeof(cpumask_var_t), GFP_KERNEL);
	if (!hw->affinity_mask)
		goto err_free_netdev;

	err = pci_alloc_irq_vectors(hw->pdev, num_vec, num_vec, PCI_IRQ_MSIX);
	if (err < 0)
		goto err_free_netdev;

	vf->reg_base = pcim_iomap(pdev, PCI_CFG_REG_BAR_NUM, 0);
	if (!vf->reg_base) {
		dev_err(dev, "Unable to map physical function CSRs, aborting\n");
		err = -ENOMEM;
		goto err_free_irq_vectors;
	}

	/* Init VF <=> PF mailbox stuff */
	err = otx2vf_vfaf_mbox_init(vf);
	if (err)
		goto err_free_irq_vectors;

	/* Register mailbox interrupt */
	err = otx2vf_register_mbox_intr(vf);
	if (err)
		goto err_mbox_destroy;

	err = otx2_set_real_num_queues(netdev, qcount, qcount);
	if (err)
		goto err_disable_mbox_intr;

	netdev->hw_features = NETIF_F_RXCSUM | NETIF_F_IP_CSUM |
			      NETIF_F_IPV6_CSUM | NETIF_F_RXHASH |
			      NETIF_F_SG | NETIF_F_TSO | NETIF_F_TSO6;
	netdev->features = netdev->hw_features;

	netdev->netdev_ops = &otx2vf_netdev_ops;

	/* MTU range: 68 - 9190 */
	netdev->min_mtu = OTX2_MIN_MTU;
	netdev->max_mtu = OTX2_MAX_MTU;

	err = register_netdev(netdev);
	if (err) {
		dev_err(dev, "Failed to register netdevice\n");
		goto err_disable_mbox_intr;
	}

	return 0;

err_disable_mbox_intr:
	otx2vf_disable_mbox_intr(vf);
err_mbox_destroy:
	otx2vf_vfaf_mbox_destroy(vf);
err_free_irq_vectors:
	pci_free_irq_vectors(hw->pdev);
err_free_netdev:
	pci_set_drvdata(pdev, NULL);
	free_netdev(netdev);
err_release_regions:
	pci_release_regions(pdev);
	return err;
}

static void otx2vf_remove(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct otx2_nic *vf;

	if (!netdev)
		return;

	vf = netdev_priv(netdev);
	unregister_netdev(netdev);

	otx2vf_disable_mbox_intr(vf);

	otx2vf_vfaf_mbox_destroy(vf);
	pci_free_irq_vectors(vf->pdev);
	pci_set_drvdata(pdev, NULL);
	free_netdev(netdev);

	pci_release_regions(pdev);
}

static struct pci_driver otx2vf_driver = {
	.name = DRV_NAME,
	.id_table = otx2_vf_id_table,
	.probe = otx2vf_probe,
	.remove = otx2vf_remove,
};

static int __init otx2vf_init_module(void)
{
	pr_info("%s: %s\n", DRV_NAME, DRV_STRING);

	return pci_register_driver(&otx2vf_driver);
}

static void __exit otx2vf_cleanup_module(void)
{
	pci_unregister_driver(&otx2vf_driver);
}

module_init(otx2vf_init_module);
module_exit(otx2vf_cleanup_module);
