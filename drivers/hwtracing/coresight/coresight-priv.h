/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
 */

#ifndef _CORESIGHT_PRIV_H
#define _CORESIGHT_PRIV_H

#include <linux/amba/bus.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/coresight.h>
#include <linux/pm_runtime.h>

/*
 * Coresight management registers (0xf00-0xfcc)
 * 0xfa0 - 0xfa4: Management	registers in PFTv1.0
 *		  Trace		registers in PFTv1.1
 */
#define CORESIGHT_ITCTRL	0xf00
#define CORESIGHT_CLAIMSET	0xfa0
#define CORESIGHT_CLAIMCLR	0xfa4
#define CORESIGHT_LAR		0xfb0
#define CORESIGHT_LSR		0xfb4
#define CORESIGHT_AUTHSTATUS	0xfb8
#define CORESIGHT_DEVID		0xfc8
#define CORESIGHT_DEVTYPE	0xfcc


/*
 * Coresight device CLAIM protocol.
 * See PSCI - ARM DEN 0022D, Section: 6.8.1 Debug and Trace save and restore.
 */
#define CORESIGHT_CLAIM_SELF_HOSTED	BIT(1)

#define TIMEOUT_US		100
#define BMVAL(val, lsb, msb)	((val & GENMASK(msb, lsb)) >> lsb)

#define ETM_MODE_EXCL_KERN	BIT(30)
#define ETM_MODE_EXCL_USER	BIT(31)

typedef u32 (*coresight_read_fn)(const struct device *, u32 offset);
#define __coresight_simple_func(type, func, name, lo_off, hi_off)	\
static ssize_t name##_show(struct device *_dev,				\
			   struct device_attribute *attr, char *buf)	\
{									\
	type *drvdata = dev_get_drvdata(_dev->parent);			\
	coresight_read_fn fn = func;					\
	u64 val;							\
	pm_runtime_get_sync(_dev->parent);				\
	if (fn)								\
		val = (u64)fn(_dev->parent, lo_off);			\
	else								\
		val = coresight_read_reg_pair(drvdata->base,		\
						 lo_off, hi_off);	\
	pm_runtime_put_sync(_dev->parent);				\
	return scnprintf(buf, PAGE_SIZE, "0x%llx\n", val);		\
}									\
static DEVICE_ATTR_RO(name)

#define coresight_simple_func(type, func, name, offset)			\
	__coresight_simple_func(type, func, name, offset, -1)
#define coresight_simple_reg32(type, name, offset)			\
	__coresight_simple_func(type, NULL, name, offset, -1)
#define coresight_simple_reg64(type, name, lo_off, hi_off)		\
	__coresight_simple_func(type, NULL, name, lo_off, hi_off)

extern const u32 barrier_pkt[4];
#define CORESIGHT_BARRIER_PKT_SIZE (sizeof(barrier_pkt))

/* Marvell OcteonTx CN9xxx device */
#define OCTEONTX_CN9XXX_ETR		0x000cc213
/* Marvell OcteonTx CN9xxx ETM device */
#define OCTEONTX_CN9XXX_ETM		0x000cc210

/* Coresight Hardware quirks */
#define CSETR_QUIRK_BUFFSIZE_8BX	(0x1U << 0) /* 8 byte size multiplier */
#define CSETR_QUIRK_SECURE_BUFF		(0x1U << 1) /* Trace buffer is Secure */
#define CSETR_QUIRK_RESET_CTL_REG	(0x1U << 2) /* Reset CTL on reset */
#define CSETR_QUIRK_NO_STOP_FLUSH	(0x1U << 3) /* No Stop on flush */
#define CSETM_QUIRK_SW_SYNC		(0x1U << 4) /* No Hardware sync */
#define CSETM_QUIRK_TREAT_ETMv43	(0x1U << 5) /* ETMv4.2 as ETMv4.3 */

/* ETM sync insertion modes
 * 1. MODE_HW
 *    Sync insertion is done by hardware without any software intervention
 *
 * 2. MODE_SW_GLOBAL
 *    sync insertion runs from common timer handler on primary core
 *
 * 3. MODE_SW_PER_CORE
 *    sync insertion runs from per core timer handler
 *
 * When hardware doesn't support sync insertion, we fall back to software based
 * ones. Typically, GLOBAL mode would be preferred when the traced cores are
 * running performance critical applications and cannot be interrupted,
 * but at the same time there would be a small loss of trace data during the
 * insertion sequence as well.
 *
 * For the sake of simplicity, in GLOBAL mode, common timer handler is
 * always expected to run on primary core(core 0).
 */
#define SYNC_GLOBAL_CORE	0 /* Core 0 */

enum etm_sync_mode {
	SYNC_MODE_INVALID,
	SYNC_MODE_HW,
	SYNC_MODE_SW_GLOBAL,
	SYNC_MODE_SW_PER_CORE,
};

enum etm_addr_type {
	ETM_ADDR_TYPE_NONE,
	ETM_ADDR_TYPE_SINGLE,
	ETM_ADDR_TYPE_RANGE,
	ETM_ADDR_TYPE_START,
	ETM_ADDR_TYPE_STOP,
};

enum cs_mode {
	CS_MODE_DISABLED,
	CS_MODE_SYSFS,
	CS_MODE_PERF,
	CS_MODE_READ_PREVBOOT,
};

/**
 * struct cs_buffer - keep track of a recording session' specifics
 * @cur:	index of the current buffer
 * @nr_pages:	max number of pages granted to us
 * @offset:	offset within the current buffer
 * @data_size:	how much we collected in this run
 * @snapshot:	is this run in snapshot mode
 * @data_pages:	a handle the ring buffer
 */
struct cs_buffers {
	unsigned int		cur;
	unsigned int		nr_pages;
	unsigned long		offset;
	local_t			data_size;
	bool			snapshot;
	void			**data_pages;
};

static inline void coresight_insert_barrier_packet(void *buf)
{
	if (buf)
		memcpy(buf, barrier_pkt, CORESIGHT_BARRIER_PKT_SIZE);
}


static inline void CS_LOCK(void __iomem *addr)
{
	do {
		/* Wait for things to settle */
		mb();
		writel_relaxed(0x0, addr + CORESIGHT_LAR);
	} while (0);
}

static inline void CS_UNLOCK(void __iomem *addr)
{
	do {
		writel_relaxed(CORESIGHT_UNLOCK, addr + CORESIGHT_LAR);
		/* Make sure everyone has seen this */
		mb();
	} while (0);
}

static inline u64
coresight_read_reg_pair(void __iomem *addr, s32 lo_offset, s32 hi_offset)
{
	u64 val;

	val = readl_relaxed(addr + lo_offset);
	val |= (hi_offset < 0) ? 0 :
	       (u64)readl_relaxed(addr + hi_offset) << 32;
	return val;
}

static inline void coresight_write_reg_pair(void __iomem *addr, u64 val,
						 s32 lo_offset, s32 hi_offset)
{
	writel_relaxed((u32)val, addr + lo_offset);
	if (hi_offset >= 0)
		writel_relaxed((u32)(val >> 32), addr + hi_offset);
}

void coresight_disable_path(struct list_head *path);
int coresight_enable_path(struct list_head *path, u32 mode, void *sink_data);
struct coresight_device *coresight_get_sink(struct list_head *path);
struct coresight_device *coresight_get_enabled_sink(struct coresight_device *cs,
						    bool reset);
struct coresight_device *coresight_get_sink_by_id(u32 id);
struct list_head *coresight_build_path(struct coresight_device *csdev,
				       struct coresight_device *sink);
void coresight_release_path(struct list_head *path);

#ifdef CONFIG_CORESIGHT_SOURCE_ETM3X
extern int etm_readl_cp14(u32 off, unsigned int *val);
extern int etm_writel_cp14(u32 off, u32 val);
#else
static inline int etm_readl_cp14(u32 off, unsigned int *val) { return 0; }
static inline int etm_writel_cp14(u32 off, u32 val) { return 0; }
#endif

/*
 * Macros and inline functions to handle CoreSight UCI data and driver
 * private data in AMBA ID table entries, and extract data values.
 */

/* coresight AMBA ID, no UCI, no driver data: id table entry */
#define CS_AMBA_ID(pid)			\
	{				\
		.id	= pid,		\
		.mask	= 0x000fffff,	\
	}

/* coresight AMBA ID, UCI with driver data only: id table entry. */
#define CS_AMBA_ID_DATA(pid, dval)				\
	{							\
		.id	= pid,					\
		.mask	= 0x000fffff,				\
		.data	=  (void *)&(struct amba_cs_uci_id)	\
			{				\
				.data = (void *)dval,	\
			}				\
	}

/* coresight AMBA ID, full UCI structure: id table entry. */
#define CS_AMBA_UCI_ID(pid, uci_ptr)	\
	{				\
		.id	= pid,		\
		.mask	= 0x000fffff,	\
		.data	= uci_ptr	\
	}

/* extract the data value from a UCI structure given amba_id pointer. */
static inline void *coresight_get_uci_data(const struct amba_id *id)
{
	if (id->data)
		return ((struct amba_cs_uci_id *)(id->data))->data;
	return 0;
}

/* Coresight ETM/ETR hardware quirks */
u32 coresight_get_etr_quirks(unsigned int id);
u32 coresight_get_etm_quirks(unsigned int id);

/* ETM software sync insertion */
bool is_etm_sync_mode_hw(void);
bool is_etm_sync_mode_sw_global(void);

void coresight_etm_active_enable(int cpu);
void coresight_etm_active_disable(int cpu);
cpumask_t coresight_etm_active_list(void);
#endif
