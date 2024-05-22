/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>
#include "ameba_soc.h"


#define SECTION(_name)      __attribute__ ((__section__(_name)))

#define IMAGE2_ENTRY_SECTION                     \
        SECTION(".image2.entry.data")


void z_arm_reset(void);

IMAGE2_ENTRY_SECTION
RAM_START_FUNCTION Img2EntryFun0 = {
	z_arm_reset,
	NULL,//BOOT_RAM_WakeFromPG,
	(uint32_t)NewVectorTable
};

static int amebadplus_init(void)
{
	/* Update CMSIS SystemCoreClock variable (HCLK) */
	/* At reset, system core clock is set to 4 MHz from MSI */
	// SystemCoreClock = 4000000;

	/* do xtal/osc clk init */
	SystemCoreClockUpdate();

	XTAL_INIT();

	if (SYSCFG_CHIPType_Get() == CHIP_TYPE_ASIC_POSTSIM) { /* Only Asic need OSC Calibration */
		OSC4M_Init();
		OSC4M_Calibration(30000);
		if ((((BOOT_Reason()) & AON_BIT_RSTF_DSLP) == FALSE) && (RTCIO_IsEnabled() == FALSE)) {
			OSC131K_Calibration(30000); /* PPM=30000=3% *//* 7.5ms */
		}
	}
	return 0;
}

SYS_INIT(amebadplus_init, PRE_KERNEL_1, 0);
