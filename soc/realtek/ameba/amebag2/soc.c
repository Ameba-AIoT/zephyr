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

extern void SOCPS_WakeFromPG_AP(void);
extern void z_arm_reset(void);

/* Load z_arm_reset to Img2EntryFun0 is only required by ameba bootloader.
 * In mcuboot z_arm_reset is accessed througth flash layout
 * (refer to bootloader/mcuboot/boot/zephyr/main.c)
 */
IMAGE2_ENTRY_SECTION
RAM_START_FUNCTION Img2EntryFun0 = {z_arm_reset,
#ifdef CONFIG_PM
				    SOCPS_WakeFromPG_AP, /* BOOT_RAM_WakeFromPG, */
#else
				    NULL,
#endif
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
	memset((void *)__rom_bss_start_ns__, 0, (__rom_bss_end_ns__ - __rom_bss_start_ns__));

#ifdef CONFIG_BOOTLOADER_MCUBOOT
	/* Copy Img2EntryFun0 from flash to ram when using mcuboot. */
	extern u8 __image2_entry_func_end__[];
	extern u8 __image2_entry_func_load_start__[];

	memcpy(&__image2_entry_func__, &__image2_entry_func_load_start__,
	       __image2_entry_func_end__ - __image2_entry_func__);
#endif

	RBSS_UDELAY_DIV = 5;

	XTAL_INIT();

	if ((EFUSE_GetChipVersion()) >= SYSCFG_CUT_VERSION_B) {
		if (SYSCFG_CHIPType_Get() == CHIP_TYPE_ASIC_POSTSIM) {
			OSC4M_Init();
			OSC4M_Calibration(30000);
		}
	} else {
		assert_param(FALSE);
	}

	SOC_OSC131_Enable();

	/* IPC table initialization */
	ipc_table_init(IPCAP_DEV);
	IRQ_CONNECT(IPC_CPU0_IRQ, INT_PRI_MIDDLE, IPC_INTHandler, (uint32_t)IPCAP_DEV, 0);
	irq_enable(IPC_CPU0_IRQ);

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
