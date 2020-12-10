// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Supports OcteonTX2 Generic Hardware Error Source[s] (GHES).
 *
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/arm_sdei.h>
#include <linux/uuid.h>
#include <linux/acpi.h>
#include <acpi/apei.h>
#include <linux/pci.h>
#include <linux/crash_dump.h>
#include "otx2-sdei-ghes.h"

#define DRV_NAME       "sdei-ghes"

/* The initialization function does not have a device ptr; use 'pr_xxx' */
#define initerrmsg(fmt, ...) pr_err(DRV_NAME ":" fmt, __VA_ARGS__)

#ifdef CONFIG_OCTEONTX2_SDEI_GHES_DEBUG
#  define initdbgmsg(fmt, ...) pr_info(DRV_NAME ":" fmt, __VA_ARGS__)
#  define dbgmsg(dev, ...) dev_info((dev), __VA_ARGS__)
#else
#  define initdbgmsg(fmt, ...) (void)(fmt)
#  define dbgmsg(dev, ...) (void)(dev)
#endif // CONFIG_OCTEONTX2_SDEI_GHES_DEBUG

static struct acpi_table_hest *hest;
/* A list of all GHES producers, allocated during module initialization. */
static struct otx2_ghes_source *ghes_source_list;

#define PCI_VENDOR_ID_CAVIUM            0x177d
#define PCI_DEVICE_ID_OCTEONTX2_LMC     0xa022
#define PCI_DEVICE_ID_OCTEONTX2_MCC     0xa070
#define PCI_DEVICE_ID_OCTEONTX2_MDC     0xa073
static const struct pci_device_id sdei_ghes_otx2_pci_tbl[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, PCI_DEVICE_ID_OCTEONTX2_LMC) },
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, PCI_DEVICE_ID_OCTEONTX2_MCC) },
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, PCI_DEVICE_ID_OCTEONTX2_MDC) },
	{ 0, },
};

/* SDEI event notification callback. */
static int sdei_ghes_callback(u32 event_id, struct pt_regs *regs, void *arg)
{
	struct acpi_hest_generic_data *hest_gen_data;
	struct acpi_hest_generic_status *estatus;
	struct otx2_ghes_err_record *err_rec;
	struct cper_sec_mem_err_old *mem_err;
	struct otx2_sdei_ghes_drv *ghes_drv;
	struct otx2_ghes_source *event;
	u32 head, tail;
	size_t idx;

	ghes_drv = arg;

	for (idx = 0; idx < ghes_drv->source_count; idx++) {
		event = &ghes_drv->source_list[idx];
		if (event->id != event_id)
			continue;

		head = event->ring->head;
		tail = event->ring->tail;

		if (head == tail) {
			initerrmsg("event 0x%x err_rec ring is empty!\n",
				   event_id);
			break;
		}

		err_rec = &event->ring->records[tail];

		estatus = event->estatus;

		estatus->raw_data_length = 0;
		/* Implementation note: 'data_length' must equal
		 * 'hest_gen_entry->error_block_length' MINUS
		 * sizeof(struct acpi_hest_generic_status).
		 * See 'sdei_ghes_init()'.
		 */

		/* Initialize 'data_length'; also see modifications below. */
		estatus->data_length = sizeof(*hest_gen_data);
		estatus->error_severity = err_rec->severity;

		/* generic data follows header */
		hest_gen_data = (struct acpi_hest_generic_data *)(estatus + 1);
		memset(hest_gen_data, 0, sizeof(*hest_gen_data));

		hest_gen_data->revision = 0x201; /* ACPI 4.x */
		if (err_rec->fru_text[0]) {
			hest_gen_data->validation_bits =
				ACPI_HEST_GEN_VALID_FRU_STRING;
			strncpy(hest_gen_data->fru_text, err_rec->fru_text,
				sizeof(hest_gen_data->fru_text));
		}
		/* copy severity from generic status */
		hest_gen_data->error_severity = estatus->error_severity;
		guid_copy((guid_t *)hest_gen_data->section_type,
			  &CPER_SEC_PLATFORM_MEM);

		hest_gen_data->error_data_length =
			sizeof(struct cper_sec_mem_err_old);
		estatus->data_length += sizeof(struct cper_sec_mem_err_old);
		/* memory error follows generic data */
		mem_err = (struct cper_sec_mem_err_old *)(hest_gen_data + 1);
		/* copy error record from ring */
		memcpy(mem_err, &err_rec->u.mcc, sizeof(*mem_err));

		/* Ensure that estatus is committed to memory prior to
		 * setting block_status.
		 */
		wmb();

		/*
		 * This simply needs the entry count to be non-zero.
		 * Set entry count to one (see ACPI_HEST_ERROR_ENTRY_COUNT).
		 */
		estatus->block_status = (1 << 4); /* i.e. one entry */

		if (++tail >= event->ring->size)
			tail = 0;
		event->ring->tail = tail;
		break;
	}

	return 0;
}

/*
 * Main initialization function for ghes_drv device instance.
 *
 * returns:
 *   0 if no error
 *   -ENODEV if error occurred initializing device
 *   ENODEV if device should not be used (not an error per se)
 */
static int sdei_ghes_init(struct platform_device *pdev)
{
	struct otx2_sdei_ghes_drv *ghes_drv;
	struct device *dev = &pdev->dev;
	struct otx2_ghes_source *event;
	size_t idx;
	int ret;

	dbgmsg(dev, "%s: entry\n", __func__);

	ghes_drv = platform_get_drvdata(pdev);
	ret = -ENODEV;

	/* Allocated during initialization (see sdei_ghes_driver_init) */
	ghes_drv->source_list = ghes_source_list;
	ghes_drv->source_count = hest->error_source_count;

	/* Register & enable each SDEI event */
	for (idx = 0; idx < ghes_drv->source_count; idx++) {
		event = &ghes_drv->source_list[idx];

		/* register the event */
		ret = sdei_event_register(event->id, sdei_ghes_callback,
					  ghes_drv);
		if (ret < 0) {
			dev_err(dev, "Error %d registering event 0x%x (%s)\n",
				ret, event->id, event->name);
			break;
		}

		/* enable the event */
		ret = sdei_event_enable(event->id);
		if (ret < 0) {
			dev_err(dev, "Error %d enabling event 0x%x (%s)\n",
				ret, event->id, event->name);
			break;
		}
	}

	if (idx != ghes_drv->source_count) {
		ret = -ENODEV;
		goto exit;
	}

	dbgmsg(dev, "Registered & enabled %ld events\n",
	       ghes_drv->source_count);

	ret = 0;

exit:
	return ret;
}

/* Main de-initialization function for ghes_drv device instance. */
static int sdei_ghes_de_init(struct platform_device *pdev)
{
	struct otx2_sdei_ghes_drv *ghes_drv;
	struct device *dev = &pdev->dev;
	struct otx2_ghes_source *event;
	int ret, idx;

	dbgmsg(dev, "%s: entry\n", __func__);

	ghes_drv = platform_get_drvdata(pdev);

	for (idx = 0; idx < ghes_drv->source_count; idx++) {
		event = &ghes_drv->source_list[idx];

		ret = sdei_event_disable(event->id);
		if (ret < 0)
			dev_err(dev,
				"Error %d disabling SDEI event 0x%x (%s)\n",
				ret, event->id, event->name);

		ret = sdei_event_unregister(event->id);
		if (ret < 0)
			dev_err(dev,
				"Error %d unregistering SDEI event 0x%x (%s)\n",
				ret, event->id, event->name);
	}

	return 0;
}

/* Linux driver framework probe function. */
static int sdei_ghes_probe(struct platform_device *pdev)
{
	struct otx2_sdei_ghes_drv *ghes_drv;
	struct device *dev = &pdev->dev;
	int ret;

	dbgmsg(dev, "%s: entry\n", __func__);

	ghes_drv = NULL;

	ret = -ENODEV;

	/* allocate device structure */
	ghes_drv = devm_kzalloc(dev, sizeof(*ghes_drv), GFP_KERNEL);

	if (ghes_drv == NULL) {
		ret = -ENOMEM;
		dev_err(dev, "Unable to allocate drv context.\n");
		goto exit;
	}

	platform_set_drvdata(pdev, ghes_drv);

	ret = sdei_ghes_init(pdev);

	/* a negative value indicates an error */
	if (ret < 0)
		dev_err(dev, "Error initializing SDEI GHES support.\n");

exit:
	if (ret) {
		sdei_ghes_de_init(pdev);

		if (ghes_drv != NULL)
			devm_kfree(dev, ghes_drv);
	}

	return ret ? -ENODEV : 0;
}

static void sdei_ghes_shutdown(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	dbgmsg(dev, "%s: entry\n", __func__);
}

static int sdei_ghes_remove(struct platform_device *pdev)
{
	struct otx2_sdei_ghes_drv *ghes_drv;
	struct device *dev = &pdev->dev;

	ghes_drv = platform_get_drvdata(pdev);

	dbgmsg(dev, "%s: entry\n", __func__);

	sdei_ghes_de_init(pdev);

	devm_kfree(dev, ghes_drv);

	return 0;
}

static const struct of_device_id sdei_ghes_of_match[] = {
	{ .compatible = "marvell,sdei-ghes", },
	{},
};
MODULE_DEVICE_TABLE(of, sdei_ghes_of_match);

static const struct platform_device_id sdei_ghes_pdev_match[] = {
	{ .name = DRV_NAME, },
	{},
};
MODULE_DEVICE_TABLE(platform, sdei_ghes_pdev_match);

static struct platform_driver sdei_ghes_drv = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = sdei_ghes_of_match,
	},
	.probe = sdei_ghes_probe,
	.remove = sdei_ghes_remove,
	.shutdown = sdei_ghes_shutdown,
	.id_table = sdei_ghes_pdev_match,
};

/*
 * Allocates and initializes Hardware Error Source Table (HEST), then
 * registers it with kernel.
 */
static int __init sdei_ghes_hest_init(struct device_node *of_node)
{
	const __be32 *of_base0, *of_base1, *of_base2;
	struct acpi_hest_generic *hest_gen_entry;
	struct device_node *child_node;
	struct otx2_ghes_source *event;
	size_t event_cnt, size, idx;
	const u32 *evt_id_prop;
	int ret, prop_sz;
	void *memblock;

	initdbgmsg("%s: entry\n", __func__);

	ret = -ENODEV;

	/* enumerate [GHES] producers available for subscription */
	event_cnt = 0;
	for_each_available_child_of_node(of_node, child_node) {
		of_base0 = of_get_address(child_node, 0, NULL, NULL);
		if ((of_base0 == NULL) ||
		    (of_translate_address(child_node, of_base0) == OF_BAD_ADDR))
			continue;
		of_base1 = of_get_address(child_node, 1, NULL, NULL);
		if ((of_base1 == NULL) ||
		    (of_translate_address(child_node, of_base1) == OF_BAD_ADDR))
			continue;
		of_base2 = of_get_address(child_node, 2, NULL, NULL);
		if ((of_base2 == NULL) ||
		    (of_translate_address(child_node, of_base2) == OF_BAD_ADDR))
			continue;
		evt_id_prop = of_get_property(child_node, "event-id", &prop_sz);
		if (!evt_id_prop && (prop_sz != sizeof(*evt_id_prop)))
			continue;

		event_cnt++;
		initdbgmsg("Found child %s/%s 0x%llx/0x%llx/0x%llx, ID:0x%x)\n",
		       child_node->name, child_node->full_name,
		       (long long)of_translate_address(child_node, of_base0),
		       (long long)of_translate_address(child_node, of_base1),
		       (long long)of_translate_address(child_node, of_base2),
		       be32_to_cpu(*evt_id_prop));
	}

	/* allocate room for HEST */
	size = sizeof(struct acpi_table_hest);
	/* each error source is of type ACPI_HEST_TYPE_GENERIC_ERROR */
	size += event_cnt * sizeof(struct acpi_hest_generic);
	/* align event list on 8-byte boundary */
	size = roundup(size, 8);

	/* allocate room for list of available events */
	size += event_cnt * sizeof(struct otx2_ghes_source);

	/* allocate everything in one block, ordered as:
	 *   HEST table
	 *   event list
	 */
	memblock = kzalloc(size, GFP_KERNEL);
	if (memblock == NULL) {
		initerrmsg("Unable to allocate HEST & event memory (0x%lx B)\n",
			   size);
		ret = -ENOMEM;
		goto exit;
	}

	/* HEST is at start of allocated block */
	hest = memblock;

	/* event table is after HEST */
	size = sizeof(struct acpi_table_hest);
	size += event_cnt * sizeof(struct acpi_hest_generic);
	/* align event list on 8-byte boundary (see allocation above) */
	size = roundup(size, 8);
	ghes_source_list = memblock + size;

	/* populate HEST header */
	strncpy(hest->header.signature, ACPI_SIG_HEST,
		sizeof(hest->header.signature));
	hest->header.length =
		sizeof(struct acpi_table_hest) +
		       (event_cnt * sizeof(struct acpi_hest_generic));
	hest->header.revision = 1;
#define OTX2_HEST_OEM_ID "MRVL  "
	strncpy(hest->header.oem_id, OTX2_HEST_OEM_ID,
		sizeof(hest->header.oem_id));
	strncpy(hest->header.oem_table_id, "OTX2    ",
		sizeof(hest->header.oem_table_id));
	hest->header.oem_revision = 1;
	strncpy(hest->header.asl_compiler_id, OTX2_HEST_OEM_ID,
		sizeof(hest->header.asl_compiler_id));
	hest->header.asl_compiler_revision = 1;
#pragma message "HEST checksum should be calculated"
	hest->header.checksum = 0;

	hest->error_source_count = event_cnt;

	/* retrieve/init event IDs from DeviceTree & populate HEST entries */
	idx = 0;
	hest_gen_entry = (struct acpi_hest_generic *)(hest + 1);
	for_each_available_child_of_node(of_node, child_node) {
		of_base0 = of_get_address(child_node, 0, NULL, NULL);
		if ((of_base0 == NULL) ||
		    (of_translate_address(child_node, of_base0) == OF_BAD_ADDR))
			continue;
		of_base1 = of_get_address(child_node, 1, NULL, NULL);
		if ((of_base1 == NULL) ||
		    (of_translate_address(child_node, of_base1) == OF_BAD_ADDR))
			continue;
		of_base2 = of_get_address(child_node, 2, NULL, NULL);
		if ((of_base2 == NULL) ||
		    (of_translate_address(child_node, of_base2) == OF_BAD_ADDR))
			continue;
		evt_id_prop = of_get_property(child_node, "event-id", &prop_sz);
		if (!evt_id_prop && (prop_sz != sizeof(*evt_id_prop)))
			continue;

		event = &ghes_source_list[idx];

		/* name is already terminated by 'kzalloc' */
		strncpy(event->name, child_node->name,
			sizeof(event->name) - 1);
		event->id = be32_to_cpu(*evt_id_prop);

		hest_gen_entry->header.type = ACPI_HEST_TYPE_GENERIC_ERROR;
		hest_gen_entry->header.source_id = idx;
		hest_gen_entry->related_source_id =
			hest_gen_entry->header.source_id;
		hest_gen_entry->reserved = 0;
		hest_gen_entry->enabled = 1;
		hest_gen_entry->records_to_preallocate = 1;
		hest_gen_entry->max_sections_per_record = 1;
		hest_gen_entry->max_raw_data_length = 0;

		hest_gen_entry->error_status_address.space_id =
			ACPI_ADR_SPACE_SYSTEM_MEMORY;
		hest_gen_entry->error_status_address.bit_width = 64;
		hest_gen_entry->error_status_address.bit_offset = 0;
		hest_gen_entry->error_status_address.access_width = 4;
		hest_gen_entry->error_status_address.address =
			of_translate_address(child_node, of_base0);

		hest_gen_entry->notify.type = ACPI_HEST_NOTIFY_POLLED;
		hest_gen_entry->notify.length = sizeof(hest_gen_entry->notify);
		hest_gen_entry->notify.config_write_enable = 0;
		hest_gen_entry->notify.poll_interval = 1000; /* i.e. 1 sec */
		hest_gen_entry->notify.vector = event->id;
		hest_gen_entry->notify.error_threshold_value = 1;
		hest_gen_entry->notify.error_threshold_window = 1;

		hest_gen_entry->error_block_length =
			sizeof(struct acpi_hest_generic_status) +
			sizeof(struct acpi_hest_generic_data) +
			sizeof(struct cper_sec_mem_err_old);

		event->estatus_address = phys_to_virt(
				hest_gen_entry->error_status_address.address);
		if (event->estatus_address == NULL) {
			initerrmsg("Unable to access estatus_address 0x%llx\n",
				   hest_gen_entry->error_status_address.address)
				   ;
			goto exit;
		}

		event->estatus_pa = of_translate_address(child_node, of_base1);
		event->estatus = phys_to_virt(event->estatus_pa);
		if (event->estatus == NULL) {
			initerrmsg("Unable to access estatus block 0x%llx\n",
				   of_translate_address(child_node, of_base1));
			goto exit;
		}

		/* Event ring buffer in memory */
		event->ring = phys_to_virt(of_translate_address(child_node,
								of_base2));
		if (event->ring == NULL) {
			initerrmsg("Unable to access event 0x%x ring buffer\n",
				   event->id);
			goto exit;
		}

		/* clear status */
		event->estatus->block_status = 0;

		/* set event status address */
		*event->estatus_address = event->estatus_pa;

		idx++;
		hest_gen_entry++;
	}

	if (idx != event_cnt) {
		ret = -ENODEV;
		goto exit;
	}

	initdbgmsg("%s: registering HEST\n", __func__);
	hest_table_set(hest);
	acpi_hest_init();

	ret = 0;

exit:
	return ret;
}

/*
 * Enable MSIX at the device level (MSIX_CAPABILITIES Header).
 *
 * NOTE: We SHOULD be able to use PCCPV_XXX_VSEC_SCTL[MSIX_SEC_EN]
 * to enable our SECURE IRQs, but for errata PCC-34263...
 */
static void dev_enable_msix(struct pci_dev *pdev)
{
	u16 ctrl;

	if ((pdev->msi_enabled) || (pdev->msix_enabled)) {
		initerrmsg("MSI(%d) or MSIX(%d) already enabled\n",
			    pdev->msi_enabled, pdev->msix_enabled);
		return;
	}

	/* enable MSIX delivery for this device; we handle [secure] MSIX ints */
	pdev->msix_cap = pci_find_capability(pdev, PCI_CAP_ID_MSIX);
	if (pdev->msix_cap) {
		pci_read_config_word(pdev, pdev->msix_cap + PCI_MSIX_FLAGS,
				     &ctrl);
		ctrl |= PCI_MSIX_FLAGS_ENABLE;
		pci_write_config_word(pdev, pdev->msix_cap + PCI_MSIX_FLAGS,
				      ctrl);
		initdbgmsg("Set MSI-X Enable for PCI dev %04d:%02d.%d\n",
			   pdev->bus->number, PCI_SLOT(pdev->devfn),
			   PCI_FUNC(pdev->devfn));
	} else {
		initerrmsg("PCI dev %04d:%02d.%d missing MSIX capabilities\n",
			   pdev->bus->number, PCI_SLOT(pdev->devfn),
			   PCI_FUNC(pdev->devfn));
	}
}

#ifdef CONFIG_OCTEONTX2_SDEI_GHES_BERT
static const struct of_device_id bed_bert_of_match[] = {
	{ .compatible = "marvell,bed-bert", },
	{},
};

struct bed_bert_mem_entry {
	union {
		/* These are identical; both are listed here for clarity */
		struct acpi_hest_generic_status hest;
		struct acpi_bert_region         bert;
	} estatus;
	struct acpi_hest_generic_data   gen_data;
	struct cper_sec_mem_err_old     mem_err;
} __packed;

/*
 * Allocates and initializes Boot Error Record Table (BERT), then
 * registers it with kernel.
 */
static int __init sdei_ghes_bert_init(struct device_node *of_node)
{
	const __be32 *of_base0, *of_base1, *of_base2;
	uint64_t bert_size, bert_rgn_size, of_size2;
	const char *bert_tbl_oem_id = "OTX2    ";
	struct bed_bert_mem_entry *bert_entries;
	struct otx2_ghes_err_ring *bed_ring;
	const char *bert_oem_id = "MRVL  ";
	struct acpi_bert_region *bert_rgn;
	struct acpi_table_bert *bert_tbl;
	void *bert_va, *firmware_bert_va;
	struct device_node *child_node;
	size_t error_cnt, size, idx;
	phys_addr_t memblock_phys;
	phys_addr_t bert_phys;
	void *memblock_va;
	int ret;
	u8 sum;
	u8 *p;

	initdbgmsg("%s: entry\n", __func__);

	ret = -ENODEV;
	bed_ring = NULL;
	error_cnt = 0;

	child_node = of_get_next_available_child(of_node, NULL);
	if (!child_node)
		goto exit;

	of_base0 = of_get_address(child_node, 0, NULL, NULL);
	if ((of_base0 == NULL) ||
	    (of_translate_address(child_node, of_base0) == OF_BAD_ADDR)) {
		pr_err("Bad or missing device tree entry #0");
		goto exit;
	}

	of_base1 = of_get_address(child_node, 1, NULL, NULL);
	if ((of_base1 == NULL) ||
	    (of_translate_address(child_node, of_base1) == OF_BAD_ADDR)) {
		pr_err("Bad or missing device tree entry #1");
		goto exit;
	}

	of_base2 = of_get_address(child_node, 2, &of_size2, NULL);
	if (!of_base2 ||
	    (of_translate_address(child_node, of_base2) == OF_BAD_ADDR)) {
		pr_err("Missing device tree entry #2");
		goto exit;
	}
	memblock_phys = of_translate_address(child_node, of_base2);

	if (!request_mem_region(memblock_phys, of_size2,
				"boot_error_data_BERT")) {
		pr_err("request mem region (0x%llx@0x%llx) failed\n",
		(unsigned long long)of_size2,
		(unsigned long long)memblock_phys);
		goto exit;
	}

	memblock_va = ioremap(memblock_phys, of_size2);
	if (memblock_va == NULL) {
		pr_err("Unable to access Boot Error Data memory\n");
		goto exit;
	}

	bed_ring = memblock_va;

	/*
	 * This indicates that the firmware does not support BERT.
	 * This is not an error, exit with success.
	 */
	if (!bed_ring->size) {
		pr_info("BERT support disabled by firmware\n");
		ret = 0;
		goto exit;
	}

	/* TODO: handle wrap-around */
	if (bed_ring->head >= bed_ring->tail)
		error_cnt = bed_ring->head - bed_ring->tail;
	else
		error_cnt = bed_ring->size -
			(bed_ring->tail - bed_ring->head);

	initdbgmsg("Node '%s' BED mem @ %llx (%llx PA), %llu B, entries %ld\n",
		   child_node->name, (long long)memblock_va, memblock_phys,
		   (long long)of_size2, error_cnt);

	if (!error_cnt) {
		ret = 0;
		goto exit;
	}

	/*
	 * The memory block contains the boot error data ring; beyond the ring
	 * is room for the BERT.  Calculate size of BERT area.
	 * Note: the ring structure definition already contains one entry,
	 * so subtract one from the 'size' member.
	 */
	size = sizeof(*bed_ring) +
		(sizeof(struct otx2_ghes_err_record) * (bed_ring->size - 1));
	/* round up to 8B */
	size = roundup(size, 8);
	if (size > of_size2) {
		pr_err("Insufficient memory for ring (0x%lx / 0x%lx)\n",
		       (long)size, (long)of_size2);
		goto exit;
	}

	bert_phys = memblock_phys + size;
	bert_size = of_size2 - size;

	/*
	 * BERT is contiguous to the boot error data ring and is organized as:
	 *   BERT header
	 *   BERT region (which includes error records)
	 */

	size = sizeof(*bert_tbl);
	size += sizeof(*bert_entries) * error_cnt;

	if (size > bert_size) {
		pr_err("Insufficient memory for BERT data (0x%lx / 0x%lx)\n",
		       (long)size, (long)bert_size);
		goto exit;
	}

	firmware_bert_va = memblock_va + (bert_phys - memblock_phys);

	bert_va = kzalloc(bert_size, GFP_KERNEL);
	if (!bert_va) {
		pr_err("Unable to allocate suitable BERT data area (0x%x B)\n",
		       (unsigned int)bert_size);
		goto exit;
	}

	/* BERT Table is at start of BERT block */
	bert_tbl = bert_va;
	/* BERT region follows Table (table is 8B-aligned, see above) */
	size = sizeof(*bert_tbl);
	bert_rgn = (struct acpi_bert_region *)(bert_va + size);
	bert_rgn_size = bert_size - size;
	/* entries begin at BERT region */
	bert_entries = (struct bed_bert_mem_entry *)bert_rgn;

	initdbgmsg("BERT memory at %llx (0x%llx PA), %llu B\n",
		   (long long)bert_va, bert_phys, (long long)bert_size);
	initdbgmsg("BERT header at %llx, %llu B\n", (long long)bert_tbl,
		   (long long)sizeof(*bert_tbl));
	initdbgmsg("BERT region (error recs) at %llx (%llu B)\n",
		   (long long)bert_rgn, bert_rgn_size);

	/* populate BERT header */
	strncpy(bert_tbl->header.signature, ACPI_SIG_BERT,
			  sizeof(bert_tbl->header.signature));
	bert_tbl->header.length = sizeof(*bert_tbl);
	bert_tbl->header.revision = 1;
	bert_tbl->header.oem_revision = 1;

	strncpy(bert_tbl->header.oem_id, bert_oem_id,
		sizeof(bert_tbl->header.oem_id));
	strncpy(bert_tbl->header.oem_table_id, bert_tbl_oem_id,
		sizeof(bert_tbl->header.oem_table_id));
	strncpy(bert_tbl->header.asl_compiler_id, bert_oem_id,
		sizeof(bert_tbl->header.asl_compiler_id));
	bert_tbl->header.asl_compiler_revision = 1;

	sum = 0;
	for (p = (u8 *)&bert_tbl->header; p < (u8 *)(&bert_tbl->header + 1);
	     p++)
		sum += *p;
	bert_tbl->header.checksum -= sum;
	bert_tbl->region_length = (error_cnt * sizeof(*bert_entries));
	bert_tbl->address = ((void *)bert_rgn - bert_va) + bert_phys;

	for (idx = 0; idx < error_cnt; idx++) {
		struct acpi_hest_generic_data *hest_gen_data;
		struct bed_bert_mem_entry *bert_mem_entry;
		struct acpi_hest_generic_status *estatus;
		struct cper_sec_mem_err_old *mem_err;
		struct otx2_ghes_err_record *err_rec;

		bert_mem_entry = &bert_entries[idx];

		estatus = &bert_mem_entry->estatus.hest;
		err_rec = &bed_ring->records[bed_ring->tail];

		estatus->raw_data_length = 0;
		estatus->raw_data_offset = 0;
		estatus->data_length = sizeof(bert_mem_entry->gen_data);
		estatus->error_severity = err_rec->severity;

		hest_gen_data = &bert_mem_entry->gen_data;

		hest_gen_data->revision = 0x201; /* ACPI 4.x */
		if (err_rec->fru_text[0]) {
			hest_gen_data->validation_bits =
				ACPI_HEST_GEN_VALID_FRU_STRING;
			strncpy(hest_gen_data->fru_text,
				err_rec->fru_text,
				sizeof(hest_gen_data->fru_text));
		}
		/* copy severity from generic status */
		hest_gen_data->error_severity = estatus->error_severity;
		memcpy((guid_t *)hest_gen_data->section_type,
		       &CPER_SEC_PLATFORM_MEM, sizeof(guid_t));

		hest_gen_data->error_data_length = sizeof(*mem_err);
		estatus->data_length += hest_gen_data->error_data_length;

		mem_err = &bert_mem_entry->mem_err;
		/* copy error record from ring */
		memcpy(mem_err, &err_rec->u.mcc, sizeof(*mem_err));

		/*
		 * This simply needs the entry count to be non-zero.
		 * Set entry count to one (see ACPI_HEST_ERROR_ENTRY_COUNT).
		 */
		estatus->block_status = (1 << 4); /* i.e. one entry */

		if (++bed_ring->tail >= bed_ring->size)
			bed_ring->tail = 0;
	}

	memcpy(firmware_bert_va, bert_va, bert_size);
	kfree(bert_va);

	initdbgmsg("%s: registering BERT\n", __func__);
	bert_table_set(firmware_bert_va);

	ret = 0;

exit:
	return ret;
}
#endif /* CONFIG_OCTEONTX2_SDEI_GHES_BERT */

/* Driver entry point */
static int __init sdei_ghes_driver_init(void)
{
	const struct pci_device_id *pdevid;
	struct device_node *of_node;
	struct pci_dev *pdev;
	int i, rc;

	initdbgmsg("%s: entry\n", __func__);

	rc = -ENODEV;

	/* Operation under kdump crash kernel is not desired nor supported. */
#ifdef CONFIG_CRASH_DUMP
	if (is_kdump_kernel())
#else
#pragma message "CONFIG_CRASH_DUMP setting is rquired for this module"
	if (true)
#endif
		return rc;

#ifdef CONFIG_OCTEONTX2_SDEI_GHES_BERT
	of_node = of_find_matching_node_and_match(NULL, bed_bert_of_match,
						  NULL);
	if (!of_node)
		goto skip_bert;

	/* Initialize Boot Error Record Table (BERT) */
	rc = sdei_ghes_bert_init(of_node);
	if (rc)
		initerrmsg("BERT initialization error %d\n", rc);

skip_bert:
#endif /* CONFIG_OCTEONTX2_SDEI_GHES_BERT */

	of_node = of_find_matching_node_and_match(NULL, sdei_ghes_of_match,
						  NULL);
	if (!of_node)
		return rc;

	/* Initialize Hardware Error Source Table (HEST) */
	rc = sdei_ghes_hest_init(of_node);
	if (rc) {
		initerrmsg("HEST initialization error %d\n", rc);
		return rc;
	}

	platform_driver_register(&sdei_ghes_drv);

	/* Enable MSIX for devices whose [secure] IRQ's we control.
	 * These IRQs have been initialized by ATF.
	 * This is required due to an errata against
	 * PCCPV_XXX_VSEC_SCTL[MSIX_SEC_EN].
	 */
	for (i = 0; i < ARRAY_SIZE(sdei_ghes_otx2_pci_tbl); i++) {
		pdevid = &sdei_ghes_otx2_pci_tbl[i];
		pdev = NULL;
		while ((pdev = pci_get_device(pdevid->vendor, pdevid->device,
			pdev))) {
			dev_enable_msix(pdev);
		}
	}

	return rc;
}

device_initcall(sdei_ghes_driver_init);

MODULE_DESCRIPTION("OcteonTX2 SDEI GHES Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
