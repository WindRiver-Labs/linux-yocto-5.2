// SPDX-License-Identifier: GPL-2.0
/* Marvell OcteonTx2 RVU Admin Function driver
 *
 * Copyright (C) 2019 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pci.h>
#include "rvu.h"

/* SDP PF device id */
#define PCI_DEVID_OTX2_SDP_PF   0xA0F6

/* Maximum SDP blocks in a chip */
#define MAX_SDP		2

/* SDP PF number */
static int sdp_pf_num[MAX_SDP] = {-1, -1};

bool is_sdp_pfvf(u16 pcifunc)
{
	u16 pf = rvu_get_pf(pcifunc);
	u32 found = 0, i = 0;

	while (i < MAX_SDP) {
		if (pf == sdp_pf_num[i])
			found = 1;
		i++;
	}

	if (!found)
		return false;

	return true;
}

bool is_sdp_pf(u16 pcifunc)
{
	return (is_sdp_pfvf(pcifunc) &&
		!(pcifunc & RVU_PFVF_FUNC_MASK));
}

bool is_sdp_vf(u16 pcifunc)
{
	return (is_sdp_pfvf(pcifunc) &&
		!!(pcifunc & RVU_PFVF_FUNC_MASK));
}

int rvu_sdp_init(struct rvu *rvu)
{
	struct pci_dev *pdev = NULL;
	struct rvu_pfvf *pfvf;
	u32 i = 0;

	while ((i < MAX_SDP) && (pdev = pci_get_device(PCI_VENDOR_ID_CAVIUM,
						       PCI_DEVID_OTX2_SDP_PF,
						       pdev)) != NULL) {
		/* The RVU PF number is one less than bus number */
		sdp_pf_num[i] = pdev->bus->number - 1;
		pfvf = &rvu->pf[sdp_pf_num[i]];

		pfvf->sdp_info = devm_kzalloc(rvu->dev,
					      sizeof(struct sdp_node_info),
					      GFP_KERNEL);
		if (!pfvf->sdp_info)
			return -ENOMEM;

		dev_info(rvu->dev, "SDP PF number:%d\n", sdp_pf_num[i]);

		put_device(&pdev->dev);
		i++;
	}

	return 0;
}

int
rvu_mbox_handler_set_sdp_chan_info(struct rvu *rvu,
				   struct sdp_chan_info_msg *req,
				   struct msg_rsp *rsp)
{
	struct rvu_pfvf *pfvf = rvu_get_pfvf(rvu, req->hdr.pcifunc);

	memcpy(pfvf->sdp_info, &req->info, sizeof(struct sdp_node_info));
	dev_info(rvu->dev, "AF: SDP%d max_vfs %d num_pf_rings %d pf_srn %d\n",
		 req->info.node_id, req->info.max_vfs, req->info.num_pf_rings,
		 req->info.pf_srn);
	return 0;
}
