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
#include <zephyr/cache.h>

void z_arm_reset(void);

IMAGE2_ENTRY_SECTION
RAM_START_FUNCTION Img2EntryFun0 = {z_arm_reset, NULL, /* BOOT_RAM_WakeFromPG, */
				    (uint32_t)RomVectorTable};

void app_rtc_init(void)
{
	RTC_InitTypeDef RTC_InitStruct;
	RTC_TimeTypeDef RTC_TimeStruct;

	RTC_TimeStructInit(&RTC_TimeStruct);
	RTC_TimeStruct.RTC_Year = 2021;
	RTC_TimeStruct.RTC_Hours = 10;
	RTC_TimeStruct.RTC_Minutes = 20;
	RTC_TimeStruct.RTC_Seconds = 30;

	RTC_StructInit(&RTC_InitStruct);
	/*enable RTC*/
	RTC_Enable(ENABLE);
	RTC_Init(&RTC_InitStruct);
	RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct);
}

u32 rtc_irq_init(void *Data)
{
	/* To avoid gcc warnings */
	(void)Data;
	u32 temp;

	RTC_ClearDetINT();
	SDM32K_Enable(); /* SDK32 shall enable after RTC_DET_IRQ irq, which means osc131 is ready */
	SYSTIMER_Init(); /* 0.2ms */
	RCC_PeriphClockCmd(APBPeriph_NULL, APBPeriph_RTC_CLOCK, ENABLE);

	if ((Get_OSC131_STATE() & RTC_BIT_FIRST_PON) == 0) {
		app_rtc_init();
		/* set first_pon to 1, this indicate RTC first pon state */
		temp = Get_OSC131_STATE() | RTC_BIT_FIRST_PON;
		Set_OSC131_STATE(temp);

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

	/* Register RTC_DET_IRQ callback function */
	IRQ_CONNECT(RTC_DET_IRQ, IRQ_PRIO_LOWEST, rtc_irq_init, (uint32_t)NULL, 0);
	irq_enable(RTC_DET_IRQ);

	/* IPC table initialization */
	ipc_table_init(IPCAP_DEV);
	IRQ_CONNECT(IPC_CPU0_IRQ, INT_PRI_MIDDLE, IPC_INTHandler, (uint32_t)IPCAP_DEV, 0);
	irq_enable(IPC_CPU0_IRQ);
}
