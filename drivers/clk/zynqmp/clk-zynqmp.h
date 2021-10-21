/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  Copyright (C) 2016-2018 Xilinx
 */

#ifndef __LINUX_CLK_ZYNQMP_H_
#define __LINUX_CLK_ZYNQMP_H_

#include <linux/spinlock.h>

#include <linux/firmware/xlnx-zynqmp.h>

/* Common Flags */
/* must be gated across rate change */
#define ZYNQMP_CLK_SET_RATE_GATE	BIT(0)
/* must be gated across re-parent */
#define ZYNQMP_CLK_SET_PARENT_GATE	BIT(1)
/* propagate rate change up one level */
#define ZYNQMP_CLK_SET_RATE_PARENT	BIT(2)
/* do not gate even if unused */
#define ZYNQMP_CLK_IGNORE_UNUSED	BIT(3)
/* do not use the cached clk rate */
#define ZYNQMP_CLK_GET_RATE_NOCACHE	BIT(6)
/* don't re-parent on rate change */
#define ZYNQMP_CLK_SET_RATE_NO_REPARENT	BIT(7)
/* do not use the cached clk accuracy */
#define ZYNQMP_CLK_GET_ACCURACY_NOCACHE	BIT(8)
/* recalc rates after notifications */
#define ZYNQMP_CLK_RECALC_NEW_RATES	BIT(9)
/* clock needs to run to set rate */
#define ZYNQMP_CLK_SET_RATE_UNGATE	BIT(10)
/* do not gate, ever */
#define ZYNQMP_CLK_IS_CRITICAL		BIT(11)

enum topology_type {
	TYPE_INVALID,
	TYPE_MUX,
	TYPE_PLL,
	TYPE_FIXEDFACTOR,
	TYPE_DIV1,
	TYPE_DIV2,
	TYPE_GATE,
};

/**
 * struct clock_topology - Clock topology
 * @type:	Type of topology
 * @flag:	Topology flags
 * @type_flag:	Topology type specific flag
 * @custom_type_flag: Topology type specific custome flag
 */
struct clock_topology {
	u32 type;
	u32 flag;
	u32 type_flag;
	u8 custom_type_flag;
};

struct clk_hw *zynqmp_clk_register_pll(const char *name, u32 clk_id,
				       const char * const *parents,
				       u8 num_parents,
				       const struct clock_topology *nodes);

struct clk_hw *zynqmp_clk_register_gate(const char *name, u32 clk_id,
					const char * const *parents,
					u8 num_parents,
					const struct clock_topology *nodes);

struct clk_hw *zynqmp_clk_register_divider(const char *name,
					   u32 clk_id,
					   const char * const *parents,
					   u8 num_parents,
					   const struct clock_topology *nodes);

struct clk_hw *zynqmp_clk_register_mux(const char *name, u32 clk_id,
				       const char * const *parents,
				       u8 num_parents,
				       const struct clock_topology *nodes);

struct clk_hw *zynqmp_clk_register_fixed_factor(const char *name,
					u32 clk_id,
					const char * const *parents,
					u8 num_parents,
					const struct clock_topology *nodes);

#endif
