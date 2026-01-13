/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <soc.h>
#include <ameba_soc.h>
#include <boot_security_km4tz.h>
#include <bootloader_km4tz.h>
#include <fault_injection_hardening.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_MCUBOOT

LOG_MODULE_REGISTER(loader, CONFIG_MCUBOOT_LOG_LEVEL);
#define IMG_TZ_PATITION DT_NODELABEL(slot0_partition)
#define IMG_NS_PATITION DT_NODELABEL(slot2_partition)

_LONG_CALL_ void RCC_PeriphClockCmd(u32 APBPeriph, u32 APBPeriph_Clock, u8 NewState);
extern void BOOT_ROM_Copy(void *__restrict dst0, const void *__restrict src0, size_t len0);
extern void Peripheral_Reset(void);
extern void BOOT_Log_Init(void);

extern MCM_MemTypeDef meminfo;

static const u32 ImagePattern[2] = {
	0x35393138,
	0x31313738,
};

void BOOT_RSIP_MMU_Config(void)
{
	/* FIXME: Hard code address */

	/* NOTE: Mapping start address should take mcuboot header(0x200) into consideration to make
	 * actual code start at 0x02000000
	 */
	RSIP_MMU_Config(MMU_ID1, 0x02000000 - 0x200, 0x04000000,
			0x08000000 + DT_REG_ADDR(IMG_NS_PATITION));
	RSIP_MMU_Cmd(MMU_ID1, ENABLE);

	/* NOTE: Make sure code start from 0x04000000 which is determined by km4tz code address */
	/* KM4TZ's code start address is determing by:
	 * CONFIG_FLASH_BASE_ADDRESS(NOTE: in app but mcuboot: 0x02000000) +
	 * DT_REG_ADDR(IMG_TZ_PATITION)
	 */
	RSIP_MMU_Config(MMU_ID2, 0x04000000 + DT_REG_ADDR(IMG_TZ_PATITION), 0x06000000,
			0x08000000 + DT_REG_ADDR(IMG_TZ_PATITION));
	RSIP_MMU_Cmd(MMU_ID2, ENABLE);

	RSIP_MMU_Cache_Clean();
}

void Boot_Copy_NP_Image(void)
{
	static const char *const NpLabel[] = {"NP XIP IMG", "NP SRAM", "NP PSRAM"};
	u32 StartAddr = 0x08000000 + DT_REG_ADDR(IMG_NS_PATITION) + 0x200;
	IMAGE_HEADER ImgHdr;
	u32 DstAddr, Len;
	u32 i;

	for (i = 0; i < 3; i++) {
		BOOT_ROM_Copy((void *)&ImgHdr, (void *)StartAddr, IMAGE_HEADER_LEN);
		if (_memcmp(ImgHdr.signature, ImagePattern, sizeof(ImagePattern)) != 0) {
			return;
		}

		DstAddr = ImgHdr.image_addr - IMAGE_HEADER_LEN;
		Len = ImgHdr.image_size + IMAGE_HEADER_LEN;

		/* np rom code jump address is from NP_BOOT_INDEX */
		if (ImgHdr.boot_index == NP_BOOT_INDEX) {
			LOG_DBG("write ns: %x\n", ImgHdr.image_addr);
			HAL_WRITE32(SYSTEM_CTRL_BASE, REG_LSYS_BOOT_ADDR_NS, ImgHdr.image_addr);
		}

		/* If not XIP sub-image, copy it to specific address(include the IMAGE_HEADER)*/
		LOG_DBG("copy %s: %x <- %x, %x\n", NpLabel[i], DstAddr, StartAddr, Len);
		if ((!IS_FLASH_ADDR(DstAddr)) && (Len > IMAGE_HEADER_LEN)) {
			LOG_DBG("BOOT_ImgCopy\n");
			BOOT_ROM_Copy((void *)DstAddr, (void *)StartAddr, Len);
			DCache_CleanInvalidate(DstAddr, Len);
		}

		/* empty Image, Just put in flash, for image hash later */
		if (Len == IMAGE_HEADER_LEN) {
			DstAddr = StartAddr;
		}
		StartAddr += Len;
	}
}

#else /* CONFIG_BOOTLOADER_MCUBOOT */

LOG_MODULE_REGISTER(loader, CONFIG_SOC_LOG_LEVEL);

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
#endif

#ifdef CONFIG_MCUBOOT
void __ameba_mcuboot_soc_early_init_hook(void)
{
	FIH_DECLARE(fih_rc, FIH_FAILURE);

	BOOT_ReasonSet();

	/* For debug reset: when debugger reset cpu, it's required to reset other cpus and some
	 * peripherals
	 */
	if (BOOT_Reason() & (AON_BIT_RSTF_WARM_KM4NS | AON_BIT_RSTF_WARM_KM4TZ)) {
		Peripheral_Reset();
	}

	BOOT_VerCheck();
	BOOT_SOC_ClkSet();
	BOOT_Log_Init();
	BOOT_RccConfig();
	BOOT_ResetMask_Config();

	meminfo = ChipInfo_MCMInfo();
	int ret = BOOT_PSRAM_Init();

	if (ret == -1) {
		/* psram initial fail or non-psram chip，close psram LDO(mem_ldo 1.8V)*/
		LDO_MemSetInNormal(MLDO_OFF);
#ifdef CONFIG_PSRAM_USED
		assert_param(0); /*Code Can only XIP When No Psram*/
#endif
	} else {
		LDO_MemSetInNormal(MLDO_NORMAL);
		LDO_MemSetInSleep(MLDO_SLEEP);
	}

	BOOT_RSIP_MMU_Config();

	Boot_Copy_NP_Image();
	FIH_CALL(BOOT_RAM_TZCfg, fih_rc);
	if (FIH_NOT_EQ(fih_rc, FIH_SUCCESS)) {
		LOG_ERR("Boot tz config failed");
		FIH_PANIC;
	}
	BOOT_Enable_NP();
}
#else
void __ameba_app_soc_early_init_hook(void)
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
#endif
