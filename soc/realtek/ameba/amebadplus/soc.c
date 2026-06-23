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

extern void SOCPS_WakeFromPG_KM4(void);
extern void z_arm_reset(void);

IMAGE2_ENTRY_SECTION
RAM_START_FUNCTION Img2EntryFun0 = {
	z_arm_reset,
#if defined(CONFIG_PM) && !defined(CONFIG_BOOTLOADER_MCUBOOT)
	/* For ameba loader wake from PG, BOOT_WakeFromPG jump to SOCPS_WakeFromPG */
	SOCPS_WakeFromPG_KM4,
#else
	/* For mcuboot wake from PG, BOOT_WakeFromPG jump to app's z_arm_reset */
	z_arm_reset,
#endif
	(uint32_t)NewVectorTable};

static void app_vdd1833_detect(void)
{
	ADC_InitTypeDef ADC_InitStruct;
	u32 buf[16], i = 0;
	u32 temp = 0;
	u32 data = 0;

	RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, ENABLE);

	ADC_StructInit(&ADC_InitStruct);
	ADC_InitStruct.ADC_OpMode = ADC_AUTO_MODE;
	ADC_InitStruct.ADC_CvlistLen = 0;
	ADC_InitStruct.ADC_Cvlist[0] = ADC_CH9; /* AVDD33 */
	ADC_Init(&ADC_InitStruct);

	ADC_Cmd(ENABLE);
	ADC_ReceiveBuf(buf, 16);
	ADC_Cmd(DISABLE);

	while (i < 16) {
		data += ADC_GET_DATA_GLOBAL(buf[i++]);
	}
	data >>= 4;

	temp = HAL_READ32(SYSTEM_CTRL_BASE, REG_AON_RSVD_FOR_SW1);
	if (data > 3000) { /* 3000: about 2.4V */
		temp |= AON_BIT_WIFI_RF_1833_SEL;
	} else {
		temp &= ~AON_BIT_WIFI_RF_1833_SEL;
	}
	HAL_WRITE32(SYSTEM_CTRL_BASE, REG_AON_RSVD_FOR_SW1, temp);
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

	XTAL_INIT();

	if (SYSCFG_CHIPType_Get() == CHIP_TYPE_ASIC_POSTSIM) { /* Only Asic need OSC Calibration */
		OSC4M_Init();
		OSC4M_Calibration(30000);
		if ((((BOOT_Reason()) & AON_BIT_RSTF_DSLP) == FALSE) &&
		    (RTCIO_IsEnabled() == FALSE)) {
			OSC131K_Calibration(30000); /* PPM=30000=3% */ /* 7.5ms */
		}
	}

	/* Register IPC interrupt */
	IRQ_CONNECT(IPC_KM4_IRQ, INT_PRI_MIDDLE, IPC_INTHandler, (uint32_t)IPCKM4_DEV, 0);
	irq_enable(IPC_KM4_IRQ);

	/* IPC table initialization */
	ipc_table_init(IPCKM4_DEV);

	app_vdd1833_detect();

#ifdef CONFIG_AMEBA_PSRAM
	ameba_init_psram();
#endif

#ifdef CONFIG_PM
	/* PMC init */
	SOCPS_SleepInit();
	pmu_init_wakeup_timer();
	pmu_set_sleep_type(SLEEP_PG);

	/* Clear SENONPEND bit otherwise WFE would be wake up by pending interrupts.
	 * The AP is designed to be wake up by sev from NP
	 */
	SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;

	/* In zephyr, whether enter sleep is decided by policy like power-state, custom policy
	 * instead of user api control, so keep OS lock released and no need to acuqire it again.
	 */
	pmu_release_wakelock(PMU_OS);
#endif
}
