/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */
/*
 * Driver for the IDT ClockMatrix(TM) and 82p33xxx families of
 * timing and synchronization devices.
 *
 * Copyright (C) 2019 Integrated Device Technology, Inc., a Renesas Company.
 */

#ifndef __UAPI_LINUX_RSMU_CDEV_H
#define __UAPI_LINUX_RSMU_CDEV_H

#include <linux/types.h>
#include <linux/ioctl.h>

/* Set dpll combomode */
struct rsmu_combomode {
	__u8 dpll;
	__u8 mode;
};

/* Get dpll state */
struct rsmu_get_state {
	__u8 dpll;
	__u8 state;
};

/* Get dpll ffo (fractional frequency offset) in ppqt*/
struct rsmu_get_ffo {
	__u8 dpll;
	__s64 ffo;
};

/*
 * RSMU IOCTL List
 */
#define RSMU_MAGIC '?'

/**
 * @Description
 * ioctl to set SMU combo mode.
 *
 * @Parameters
 * pointer to struct rsmu_combomode that contains dpll combomode setting
 */
#define RSMU_SET_COMBOMODE  _IOW(RSMU_MAGIC, 1, struct rsmu_combomode)

/**
 * @Description
 * ioctl to get SMU dpll state.
 *
 * @Parameters
 * pointer to struct rsmu_get_state that contains dpll state
 */
#define RSMU_GET_STATE  _IOR(RSMU_MAGIC, 2, struct rsmu_get_state)

/**
 * @Description
 * ioctl to get SMU dpll ffo.
 *
 * @Parameters
 * pointer to struct rsmu_get_ffo that contains dpll ffo in ppqt
 */
#define RSMU_GET_FFO  _IOR(RSMU_MAGIC, 3, struct rsmu_get_ffo)
#endif /* __UAPI_LINUX_RSMU_CDEV_H */
