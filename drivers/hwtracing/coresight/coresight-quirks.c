// SPDX-License-Identifier: GPL-2.0
//
#include <linux/coresight.h>
#include "coresight-priv.h"

u32 coresight_get_etr_quirks(unsigned int id)
{
	u32 options = 0; /* reset */

	if (midr_is_cpu_model_range(read_cpuid_id(),
				     MIDR_MRVL_OCTEONTX2_96XX,
				     MIDR_CPU_VAR_REV(0, 0),
				     MIDR_CPU_VAR_REV(3, 0)) ||
	    midr_is_cpu_model_range(read_cpuid_id(),
				     MIDR_MRVL_OCTEONTX2_95XX,
				     MIDR_CPU_VAR_REV(0, 0),
				     MIDR_CPU_VAR_REV(2, 0)))
		options |= CSETR_QUIRK_BUFFSIZE_8BX |
			CSETR_QUIRK_RESET_CTL_REG;

	if (id == OCTEONTX_CN9XXX_ETR)
		options |= CSETR_QUIRK_SECURE_BUFF;

	return options;
}
