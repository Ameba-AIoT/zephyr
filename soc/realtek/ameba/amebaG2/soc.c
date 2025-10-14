/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>

void z_arm_reset(void);

IMAGE2_ENTRY_SECTION
RAM_START_FUNCTION Img2EntryFun0 = {z_arm_reset, NULL, /* BOOT_RAM_WakeFromPG, */
				    (uint32_t)RomVectorTable};

u32 SOC_OSC131_Enable(void)
{
	u32 temp;

	if (RCC_PeriphClockEnableChk(APBPeriph_RTC_CLOCK)) {
		return 0;
	}

	while (0 == RTC_GetDetintr()) {
	}
	RTC_ClearDetINT();

	SDM32K_Enable(); /* SDK32 shall enable after RTC_DET_IRQ irq, which means osc131 is ready */
	SYSTIMER_Init(); /* 0.2ms */
	RCC_PeriphClockCmd(APBPeriph_NULL, APBPeriph_RTC_CLOCK, ENABLE);

	temp = Get_OSC131_STATE();
	if ((temp & RTC_BIT_FIRST_PON) == 0) {
		/*set first_pon to 1, this indicate RTC first pon state*/
		Set_OSC131_STATE(temp | RTC_BIT_FIRST_PON);

		/*before 131k calibratopn, cke_rtc should be enabled*/
		if (SYSCFG_CHIPType_Get() == CHIP_TYPE_ASIC_POSTSIM) {
			OSC131K_Calibration(30000); /* PPM=30000=3% */ /* 7.5ms */
		}
	}

	RTC_ClkSource_Select(SDM32K);

	return 0;
}

void soc_early_init_hook(void)
{
	/*
	 * Cache is enabled by default at reset, disable it before
	 * sys_cache*-functions can enable them.
	 */
	Cache_Enable(DISABLE);
	sys_cache_instr_enable();
	sys_cache_data_enable();

	RBSS_UDELAY_DIV = 5;

	XTAL_INIT();

	if ((SYSCFG_RLVersion()) >= SYSCFG_CUT_VERSION_B) {
		if (SYSCFG_CHIPType_Get() == CHIP_TYPE_ASIC_POSTSIM) {
			OSC4M_Init();
			OSC4M_Calibration(30000);
		}
	}

	SOC_OSC131_Enable();

	/* IPC table initialization */
	ipc_table_init(IPCAP_DEV);
	IRQ_CONNECT(IPC_CPU0_IRQ, INT_PRI_MIDDLE, IPC_INTHandler, (uint32_t)IPCAP_DEV, 0);
	irq_enable(IPC_CPU0_IRQ);

#ifdef CONFIG_AMEBA_PSRAM
	ameba_init_psram();
#endif
}
