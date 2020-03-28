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
				     MIDR_CPU_VAR_REV(3, 1)) ||
	    midr_is_cpu_model_range(read_cpuid_id(),
				     MIDR_MRVL_OCTEONTX2_95XX,
				     MIDR_CPU_VAR_REV(0, 0),
				     MIDR_CPU_VAR_REV(2, 0)))
		options |= CSETR_QUIRK_RESET_CTL_REG |
			CSETR_QUIRK_BUFFSIZE_8BX |
			CSETR_QUIRK_NO_STOP_FLUSH;

	/* Common across all Chip variants and revisions */
	if (id == OCTEONTX_CN9XXX_ETR)
		options |= CSETR_QUIRK_SECURE_BUFF |
			CSETR_QUIRK_FORCE_64B_DBA_RW;

	return options;
}

bool is_etm_has_hw_sync(void)
{
	/* Check if hardware supports sync insertion */
	if (midr_is_cpu_model_range(read_cpuid_id(),
				     MIDR_MRVL_OCTEONTX2_96XX,
				     MIDR_CPU_VAR_REV(0, 0),
				     MIDR_CPU_VAR_REV(3, 1)) ||
	    midr_is_cpu_model_range(read_cpuid_id(),
				     MIDR_MRVL_OCTEONTX2_95XX,
				     MIDR_CPU_VAR_REV(0, 0),
				     MIDR_CPU_VAR_REV(2, 0)))
		return false;
	else
		return true;
}

u32 coresight_get_etm_quirks(unsigned int id)
{
	u32 options = 0; /* reset */

	if (id == OCTEONTX_CN9XXX_ETM)
		options |= CSETM_QUIRK_TREAT_ETMv43;

	if (!is_etm_has_hw_sync())
		options |= CSETM_QUIRK_SW_SYNC;

	return options;
}

/* APIs for choosing the sync insertion mode */
static int coresight_get_etm_sync_mode(void)
{
	/* Check if hardware supports sync insertion */
	if (is_etm_has_hw_sync())
		return SYNC_MODE_HW;

	/* Find the software based sync insertion mode */
#ifdef CONFIG_TASK_ISOLATION
	return SYNC_MODE_SW_GLOBAL;
#else
	return SYNC_MODE_SW_PER_CORE;
#endif
}

/* APIs for enabling fixes for CSETR_QUIRK_SW_SYNC
 *
 * Note: Driver options are not used as done in other quirks,
 * since the fix is spread across multiple(ETM/ETR) driver files.
 */

bool is_etm_sync_mode_hw(void)
{
	return coresight_get_etm_sync_mode() == SYNC_MODE_HW;
}

bool is_etm_sync_mode_sw_global(void)
{
	return coresight_get_etm_sync_mode() == SYNC_MODE_SW_GLOBAL;
}

/* Support functions for managing active ETM list used by
 * global mode sync insertion.
 *
 * Note: It is assumed that all accessor functions
 * on etm_active_list should be called in a atomic context
 */

static cpumask_t etm_active_list; /* Bitmap of active ETMs cpu wise */

void coresight_etm_active_enable(int cpu)
{
	cpumask_set_cpu(cpu, &etm_active_list);
}

void coresight_etm_active_disable(int cpu)
{
	cpumask_clear_cpu(cpu, &etm_active_list);
}

cpumask_t coresight_etm_active_list(void)
{
	return etm_active_list;
}
