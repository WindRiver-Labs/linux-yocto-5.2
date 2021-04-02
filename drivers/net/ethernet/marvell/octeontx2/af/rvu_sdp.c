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
	struct pci_dev *pdev;
	int i;

	for (i = 0; i < rvu->hw->total_pfs; i++) {
		pdev = pci_get_domain_bus_and_slot(
				pci_domain_nr(rvu->pdev->bus), i + 1, 0);
		if (!pdev)
			continue;

		if (pdev->device == PCI_DEVID_OTX2_SDP_PF) {
			sdp_pf_num[i] = pdev->bus->number - 1;
			put_device(&pdev->dev);
			break;
		}

		put_device(&pdev->dev);
		i++;
	}

	return 0;
}
