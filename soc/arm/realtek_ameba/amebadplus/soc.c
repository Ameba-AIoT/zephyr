/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <soc.h>
#include <ameba_soc.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>


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


static int amebadplus_app_init(void)
{
	/* Register IPC interrupt */
	IRQ_CONNECT(IPC_KM4_IRQ, INT_PRI5, IPC_INTHandler, (uint32_t)IPCKM4_DEV, 0);
	irq_enable(IPC_KM4_IRQ);

	/* IPC table initialization */
	ipc_table_init(IPCKM4_DEV);

	return 0;
}

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
SYS_INIT(amebadplus_app_init, APPLICATION, 0);
