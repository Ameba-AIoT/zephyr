/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <ameba_soc.h>
#include <boot_security_km4.h>
#include <bootloader_km4.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/logging/log.h>

#include "bootutil/bootutil.h"
#include "bootutil/bootutil_log.h"
#include "bootutil/mcuboot_status.h"
#include "sysflash/sysflash.h"

LOG_MODULE_REGISTER(loader, CONFIG_MCUBOOT_LOG_LEVEL);
#define IMG_APP_PATITION DT_NODELABEL(slot0_partition)

#define IMG_KM4_LOGIC_ADDR  DT_PROP(DT_PATH(zephyr_user), primary_logic_addr)
#define IMG_KM0_LOGIC_ADDR  DT_PROP(DT_PATH(zephyr_user), secondary_logic_addr)
#define FLASH_BASE_PHY_ADDR DT_PROP(DT_PATH(zephyr_user), flash_base_phy)

_LONG_CALL_ void RCC_PeriphClockCmd(uint32_t APBPeriph, uint32_t APBPeriph_Clock, uint8_t NewState);
extern void BOOT_ROM_Copy(void *__restrict dst0, const void *__restrict src0, size_t len0);
extern void Peripheral_Reset(void);
extern bool BOOT_RRAM_InfoValid(void);
extern void BOOT_Log_Init(void);
extern void RSIP_IV_Set(uint8_t index, uint8_t *IV);
extern void BOOT_OTFCheck(uint32_t start_addr, uint32_t end_addr, uint32_t IV_index,
			  uint32_t OTF_index);

extern MCM_MemTypeDef meminfo;

static const uint32_t ImagePattern[2] = {
	0x35393138,
	0x31313738,
};

static uint32_t PrevIrqStatus;

/* Strong symbol: override weak HAL implementation. */
void FLASH_Write_Lock(void)
{
	PrevIrqStatus = __get_PRIMASK();
	__disable_irq();
}

/* Strong symbol: override weak HAL implementation. */
void FLASH_Write_Unlock(void)
{
	__set_PRIMASK(PrevIrqStatus);
}

int BOOT_RSIP_Load_Image(uint8_t id, uint8_t *iv, struct image_header *hdr)
{
	uint32_t off = 0;
	const struct flash_area *fap = NULL;
	int rc;

	LOG_INF("Attempting to parse IV from TLV...");

	rc = flash_area_open(id, &fap);
	if (rc != 0) {
		return rc;
	}

	rc = flash_area_read(fap, off, hdr, sizeof(*hdr));
	if (rc != 0) {
		goto end;
	}

	/* Traverse through the TLV area to find the image hash TLV. */
	struct image_tlv_iter it = {0};
	uint16_t type;
	uint16_t len;
#if defined(MCUBOOT_SWAP_USING_OFFSET)
	it.start_off = 0;
#endif
	rc = bootutil_tlv_iter_begin(&it, hdr, fap, IMAGE_TLV_ANY, false);

	if (rc != 0) {
		goto end;
	}

	while (true) {
		rc = bootutil_tlv_iter_next(&it, &off, &len, &type);
		if (rc != 0) {
			goto end;
		}

		if (type == CONFIG_AMEBA_RSIP_IV_TYPE_IN_TLV) {
			if (len != 8) {
				rc = -1;
				goto end;
			}

			rc = flash_area_read(fap, off, iv, len);
			break;
		}
	}
end:
	flash_area_close(fap);
	return rc;
}

uint32_t Boot_Copy_KM0_Image(void)
{
	static const char *const km0_label[] = {"KM0 XIP IMG", "KM0 SRAM", "KM0 PSRAM",
						"KM0 ENTRY"};
	IMAGE_HEADER ImgHdr;
	uint32_t StartAddr = IMG_KM0_LOGIC_ADDR;
	uint32_t DstAddr, Len;
	uint32_t Cnt;

	Cnt = sizeof(km0_label) / sizeof(char *);
	for (int i = 0; i < Cnt; i++) {
		_memcpy((void *)&ImgHdr, (void *)StartAddr, IMAGE_HEADER_LEN);
		if (_memcmp(ImgHdr.signature, ImagePattern, sizeof(ImagePattern)) != 0) {
			LOG_ERR("%s Invalid", km0_label[i]);
			return 0;
		}

		DstAddr = ImgHdr.image_addr - IMAGE_HEADER_LEN;
		Len = ImgHdr.image_size + IMAGE_HEADER_LEN;

		/* If not XIP sub-image, copy it to specific address(include the IMAGE_HEADER)*/
		LOG_DBG("try copy %s: %x <- %x, %x", km0_label[i], DstAddr, StartAddr, Len);
		if ((!IS_FLASH_ADDR(DstAddr)) && (Len > IMAGE_HEADER_LEN)) {
			_memcpy((void *)DstAddr, (void *)StartAddr, Len);
			DCache_CleanInvalidate(DstAddr, Len);
			LOG_DBG("copy done");
		}

		/* empty Image, Just put in flash, for image hash later */
		if (Len == IMAGE_HEADER_LEN) {
			DstAddr = StartAddr;
		}
		StartAddr += Len;
	}

	return StartAddr - IMG_KM0_LOGIC_ADDR;
}

void BOOT_RSIP_MMU_Config(void)
{
	struct image_header hdr_img;
	uint32_t km0_size, km0_total, km4_size;
	uint32_t start, end;
	uint8_t iv[8];

	BOOT_RSIP_Load_Image(FLASH_AREA_IMAGE_PRIMARY(0), iv, &hdr_img);

	if (SYSCFG_OTP_RSIPEn() == TRUE) {
		LOG_DBG("SYSCFG_OTP_RSIPEn TRUE");
		RSIP_IV_Set(1, iv);
	}

	/* NOTE: Mapping start address should take mcuboot header(0x200) into consideration to make
	 * actual code start at IMG_KM0_LOGIC_ADDR
	 */
	RSIP_MMU_Config(MMU_ID1, IMG_KM0_LOGIC_ADDR - hdr_img.ih_hdr_size, IMG_KM4_LOGIC_ADDR,
			FLASH_BASE_PHY_ADDR + DT_REG_ADDR(IMG_APP_PATITION));
	RSIP_MMU_Cmd(MMU_ID1, ENABLE);
	RSIP_MMU_Cache_Clean();

	BOOT_OTFCheck(IMG_KM0_LOGIC_ADDR, IMG_KM0_LOGIC_ADDR + DT_REG_SIZE(IMG_APP_PATITION), 1, 1);

	km0_size = Boot_Copy_KM0_Image();

	/* add 4K-align padding info */
	km0_total = (((km0_size - 1) >> 12) + 1) << 12;
	km4_size = hdr_img.ih_img_size - km0_total;

	RSIP_MMU_Config(MMU_ID2, IMG_KM4_LOGIC_ADDR + hdr_img.ih_hdr_size,
			IMG_KM4_LOGIC_ADDR + 0x01000000,
			FLASH_BASE_PHY_ADDR + DT_REG_ADDR(IMG_APP_PATITION) + km0_total +
				hdr_img.ih_hdr_size);
	RSIP_MMU_Cmd(MMU_ID2, ENABLE);
	RSIP_MMU_Cache_Clean();

	start = IMG_KM4_LOGIC_ADDR + hdr_img.ih_hdr_size;
	end = start + km4_size;

	BOOT_OTFCheck(start, end, 1, 2);
}

void mcuboot_status_change(mcuboot_status_type_t status)
{
	if (status != MCUBOOT_STATUS_BOOTABLE_IMAGE_FOUND) {
		return;
	}

	RRAM_TypeDef *rram = RRAM_DEV;

	BOOT_ReasonSet();

	Peripheral_Reset();

	if ((BOOT_Reason() == 0) || (!BOOT_RRAM_InfoValid())) {
		OSC131K_Reset();

		_memset(RRAM_DEV, 0, sizeof(RRAM_TypeDef));
		RRAM_DEV->MAGIC_NUMBER = 0x6969A5A5;
	}

	BOOT_SOC_ClkSet();

	/* Mem_LDO in normal0 mode by default, change dummy load from 500uA to 200uA */
	LDO_MemDummyCtrl(LDO_MEM_DUMMY_200UA);

	BOOT_TRNG_ParaSet();

	BOOT_Log_Init();

	LOG_DBG("IMG1 ENTER MSP:[%08X]", __get_MSP());
	LOG_DBG("Build Time: %s %s", __DATE__, __TIME__);

	BOOT_RccConfig();

	BOOT_Disable_PadSlewRate();

	flash_highspeed_setup();

	/* backup flash_init_para address for KM0 */
	DCache_Clean((uint32_t)&flash_init_para, sizeof(FLASH_InitTypeDef));
	HAL_WRITE32(SYSTEM_CTRL_BASE, REG_LSYS_FLASH_PARA_ADDR, (uint32_t)&flash_init_para);

	/* If No PLL CLK, ComboSPIC can't glitch-free mux to XTAL */
	BOOT_ComboSpic_PLL_Open();

	MCM_MemTypeDef meminfo = ChipInfo_MCMInfo();

	rram->Memory_Type = meminfo.mem_type;
	if (meminfo.mem_type == MCM_TYPE_PSRAM) {
		BOOT_PSRAM_Init();
		/* PA5 is psram cs pin, need PU for sleep power consideration */
		PAD_PullCtrl(_PA_5, GPIO_PuPd_UP);
		PAD_SleepPullCtrl(_PA_5, GPIO_PuPd_UP);
	} else if (meminfo.mem_type == MCM_TYPE_NOR_FLASH) {
#ifdef CONFIG_PSRAM_USED
		assert_param(0); /*Code Can only XIP When No Psram*/
#endif
		BOOT_Data_Flash_Init();

		/* PA5 is flash clk pin, need PD for sleep power consideration */
		PAD_PullCtrl(_PA_5, GPIO_PuPd_DOWN);
		PAD_SleepPullCtrl(_PA_5, GPIO_PuPd_DOWN);
	}

	BOOT_Share_Memory_Patch();

	BOOT_ResetMask_Config();

	BOOT_VerCheck();

	/* set sw patch reg to 0 for security, this reg seure access only */
	HAL_WRITE32(SYSTEM_CTRL_BASE, REG_LSYS_SW_PATCH, 0);

	BOOT_RSIP_MMU_Config();

	/**
	 * @brief If there are any functions placed on the flash (defined by BOOT_XIP_TEXT_SECTION)
	 * that need to be executed, they must be run before BOOT_RAM_TZcfg().
	 */
	/* Config Non-Security World Registers */
	BOOT_RAM_TZCfg();

	/*KM0 shall wait MPC setting for non-secure access*/
	BOOT_Enable_KM0();
}
