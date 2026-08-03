/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * AmebaSmart (RTL8730E) CA32 clock-gating entry point.
 *
 * The CG mechanism itself (SOCPS_SleepCG_LIB) is in lib_pmc.a; this file holds
 * the sleep_param instance it consumes and the SOCPS_SleepCG wrapper that sets
 * the CG sleep parameters and reports AP running again on wake.
 */

#include "ameba_soc.h"

/* AP sleep parameters, shared with the LP core over IPC.  Cacheline aligned so
 * a DCache clean covers exactly one line for the LP/NP to observe.
 */
SLEEP_ParamDef sleep_param ALIGNMTO(32);

/* clock-gate the CA32 (hand off to LP over IPC, gate the clock) — lib_pmc.a. */
extern void SOCPS_SleepCG_LIB(void);

void SOCPS_SleepCG(void)
{
	sleep_param.sleep_type = SLEEP_CG;
	sleep_param.sleep_time = 0; /* wake purely by configured wake events */
	sleep_param.dlps_enable = DISABLE;
	DCache_CleanInvalidate((u32)&sleep_param, sizeof(SLEEP_ParamDef));

	RTK_LOGS(NOTAG, RTK_LOG_INFO, "APCG\n");

	SOCPS_SleepCG_LIB();

	RTK_LOGS(NOTAG, RTK_LOG_INFO, "APCW\n");

	/* Indicate AP is running again so the LP core stops treating it as
	 * suspended.
	 */
	HAL_WRITE8(SYSTEM_CTRL_BASE_LP, REG_LSYS_AP_STATUS_SW,
		   HAL_READ8(SYSTEM_CTRL_BASE_LP, REG_LSYS_AP_STATUS_SW) | LSYS_BIT_AP_RUNNING);
}
