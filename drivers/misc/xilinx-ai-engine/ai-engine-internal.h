/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Xilinx AI Engine driver internal header
 *
 * Copyright (C) 2020 Xilinx, Inc.
 */

#ifndef AIE_INTERNAL_H
#define AIE_INTERNAL_H

#include <linux/bitfield.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fpga/fpga-bridge.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <uapi/linux/xlnx-ai-engine.h>

enum aie_tile_type {
	AIE_TILE_TYPE_TILE = 1,
	AIE_TILE_TYPE_SHIMPL,
	AIE_TILE_TYPE_SHIMNOC
};

/*
 * Macros for attribute property of AI engine registers accessed by kernel
 * 0 - 7 bits: tile type bits
 * 8 - 15 bits: permission bits. If it is 1, it allows write from userspace
 */
#define AIE_REGS_ATTR_TILE_TYPE_SHIFT	0U
#define AIE_REGS_ATTR_PERM_SHIFT	8U
#define AIE_REGS_ATTR_TILE_TYPE_MASK	GENMASK(AIE_REGS_ATTR_PERM_SHIFT - 1, \
						AIE_REGS_ATTR_TILE_TYPE_SHIFT)
#define AIE_REGS_ATTR_PERM_MASK		GENMASK(15, \
						AIE_REGS_ATTR_PERM_SHIFT)

#define AIE_PART_STATUS_BRIDGE_DISABLED	0x1U

/**
 * struct aie_tile_regs - contiguous range of AI engine register
 *			  within an AI engine tile
 * @soff: start offset of the range
 * @eoff: end offset of the range
 * @attribute: registers attribute. It uses AIE_REGS_ATTR_* macros defined
 *	       above.
 */
struct aie_tile_regs {
	size_t soff;
	size_t eoff;
	u32 attribute;
};

struct aie_device;
struct aie_partition;

/**
 * struct aie_tile_operations - AI engine device operations
 * @get_tile_type: get type of tile based on tile operation
 *
 * Different AI engine device version has its own device
 * operation.
 */
struct aie_tile_operations {
	u32 (*get_tile_type)(struct aie_location *loc);
};

/**
 * struct aie_resource - AI engine resource structure
 * @bitmap: resource bitmap
 * @total: total number of resource
 */
struct aie_resource {
	unsigned long *bitmap;
	u32 total;
};

/**
 * struct aie_device - AI engine device structure
 * @partitions: list of partitions requested
 * @cdev: cdev for the AI engine
 * @dev: device for the AI engine device
 * @mlock: protection for AI engine device operations
 * @base: AI engine device base virtual address
 * @res: memory resource of AI engine device
 * @kernel_regs: array of kernel only registers
 * @ops: tile operations
 * @size: size of the AI engine address space
 * @array_shift: array address shift
 * @col_shift: column address shift
 * @row_shift: row address shift
 * @cols_res: AI engine columns resources to indicate
 *	      while columns are occupied by partitions.
 * @num_kernel_regs: number of kernel only registers range
 * @version: AI engine device version
 */
struct aie_device {
	struct list_head partitions;
	struct cdev cdev;
	struct device dev;
	struct mutex mlock; /* protection for AI engine partitions */
	void __iomem *base;
	struct resource *res;
	const struct aie_tile_regs *kernel_regs;
	const struct aie_tile_operations *ops;
	size_t size;
	struct aie_resource cols_res;
	u32 array_shift;
	u32 col_shift;
	u32 row_shift;
	u32 num_kernel_regs;
	int version;
};

/**
 * struct aie_part_bridge - AI engine FPGA bridge
 * @name: name of the FPGA bridge
 * @br: pointer to FPGA bridge
 */
struct aie_part_bridge {
	char name[32];
	struct fpga_bridge *br;
};

/**
 * struct aie_partition - AI engine partition structure
 * @node: list node
 * @adev: pointer to AI device instance
 * @br: AI engine FPGA bridge
 * @range: range of partition
 * @mlock: protection for AI engine partition operations
 * @dev: device for the AI engine partition
 * @partition_id: partition id. Partition ID is the identifier
 *		  of the AI engine partition in the system.
 * @status: indicate if the partition is in use
 */
struct aie_partition {
	struct list_head node;
	struct aie_part_bridge br;
	struct aie_device *adev;
	struct aie_range range;
	struct mutex mlock; /* protection for AI engine partition operations */
	struct device dev;
	u32 partition_id;
	u32 status;
};

extern struct class *aie_class;
extern const struct file_operations aie_part_fops;

#define cdev_to_aiedev(i_cdev) container_of((i_cdev), struct aie_device, cdev)
#define dev_to_aiedev(_dev) container_of((_dev), struct aie_device, dev)
#define dev_to_aiepart(_dev) container_of((_dev), struct aie_partition, dev)

#define aie_col_mask(adev) ({ \
	struct aie_device *_adev = (adev); \
	GENMASK_ULL(_adev->array_shift - 1, _adev->col_shift);  \
	})

#define aie_row_mask(adev) ({ \
	struct aie_device *_adev = (adev); \
	GENMASK_ULL(_adev->col_shift - 1, _adev->row_shift);  \
	})

#define aie_tile_reg_mask(adev) ({ \
	struct aie_device *_adev = (adev); \
	GENMASK_ULL(_adev->row_shift - 1, 0);  \
	})

/*
 * Need to define field get, as AI engine shift mask is not constant.
 * Cannot use FIELD_GET()
 */
#define aie_tile_reg_field_get(mask, shift, regoff) ( \
	((regoff) & (mask)) >> (shift))

#define aie_cal_tile_reg(adev, regoff) ( \
	aie_tile_reg_field_get(aie_tile_reg_mask(adev), 0, regoff))

/**
 * aie_validate_location() - validate tile location within an AI engine
 *			     partition
 * @apart: AI engine partition
 * @loc: AI engine tile location
 * @return: return 0 if it it is valid, negative value for errors.
 *
 * This function checks if the AI engine location is within the AI engine
 * partition.
 */
static inline int aie_validate_location(struct aie_partition *apart,
					struct aie_location loc)
{
	if (loc.col < apart->range.start.col ||
	    loc.col >= apart->range.start.col + apart->range.size.col ||
	    loc.row < apart->range.start.row ||
	    loc.row >= apart->range.start.row + apart->range.size.row)
		return -EINVAL;

	return 0;
}

int aie_resource_initialize(struct aie_resource *res, int count);
void aie_resource_uninitialize(struct aie_resource *res);
int aie_resource_check_region(struct aie_resource *res, u32 start,
			      u32 count);
int aie_resource_get_region(struct aie_resource *res, u32 start,
			    u32 count);
void aie_resource_put_region(struct aie_resource *res, int start, u32 count);

const struct file_operations *aie_part_get_fops(void);
u8 aie_part_in_use(struct aie_partition *apart);
struct aie_partition *aie_get_partition_from_id(struct aie_device *adev,
						u32 partition_id);
struct aie_partition *aie_request_partition_from_id(struct aie_device *adev,
						    u32 partition_id);
struct aie_partition *of_aie_part_probe(struct aie_device *adev,
					struct device_node *nc);
void aie_part_remove(struct aie_partition *apart);

int aie_fpga_create_bridge(struct aie_partition *apart);
void aie_fpga_free_bridge(struct aie_partition *apart);

int aiev1_device_init(struct aie_device *adev);
#endif /* AIE_INTERNAL_H */
