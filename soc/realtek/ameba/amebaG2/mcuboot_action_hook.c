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

#include "bootutil/bootutil.h"
#include "bootutil/bootutil_log.h"
#include "bootutil/mcuboot_status.h"
#include "sysflash/sysflash.h"

LOG_MODULE_REGISTER(loader, CONFIG_MCUBOOT_LOG_LEVEL);
#define IMG_TZ_PATITION DT_NODELABEL(slot0_partition)
#define IMG_NS_PATITION DT_NODELABEL(slot2_partition)

#define IMG_TZ_LOGIC_ADDR   DT_PROP(DT_PATH(zephyr_user), primary_logic_addr)   /* 0x04000000 */
#define IMG_NS_LOGIC_ADDR   DT_PROP(DT_PATH(zephyr_user), secondary_logic_addr) /* 0x02000000 */
#define FLASH_BASE_PHY_ADDR DT_PROP(DT_PATH(zephyr_user), flash_base_phy)       /* 0x08000000 */

_LONG_CALL_ void RCC_PeriphClockCmd(u32 APBPeriph, u32 APBPeriph_Clock, u8 NewState);
extern void BOOT_ROM_Copy(void *__restrict dst0, const void *__restrict src0, size_t len0);
extern void Peripheral_Reset(void);
extern void BOOT_Log_Init(void);
extern void RSIP_IV_Set(uint8_t index, uint8_t *IV);
extern fih_ret BOOT_OTFCheck(uint32_t start_addr, uint32_t end_addr, uint32_t IV_index,
			     uint32_t OTF_index);

extern MCM_MemTypeDef meminfo;

static const u32 ImagePattern[2] = {
	0x35393138,
	0x31313738,
};

int BOOT_RSIP_Load_Image(uint8_t id, uint8_t *iv, struct image_header *hdr)
{
	uint32_t off = 0;
	const struct flash_area *fap = NULL;
	int rc;

	BOOT_LOG_INF("Attempting to parse IV from TLV...");

	rc = flash_area_open(id, &fap);
	if (rc != 0) {
		return rc;
	}

	rc = flash_area_read(fap, off, hdr, sizeof(*hdr));
	if (rc) {
		goto end;
	}

	if (iv) {
		/* Traverse through the TLV area to find the image hash TLV. */
		struct image_tlv_iter it = {0};
		uint16_t type;
		uint16_t len;

		it.start_off = 0;
		rc = bootutil_tlv_iter_begin(&it, hdr, fap, IMAGE_TLV_ANY, false);

		if (rc) {
			goto end;
		}

		while (true) {
			rc = bootutil_tlv_iter_next(&it, &off, &len, &type);
			if (rc != 0) {
				goto end;
			}

			if (type == CONFIG_AMEBA_RSIP_IV_TYPE_IN_TLV) {
				/* Get the image's hash value from the manifest section. */
				if (len != 8) {
					rc = -1;
					goto end;
				}

				rc = flash_area_read(fap, off, iv, len);
				if (rc) {
					goto end;
				}
				rc = 0;
				break;
			}
		}
	}
end:
	flash_area_close(fap);
	return rc;
}

void BOOT_RSIP_MMU_Config(void)
{
	/* FIXME: Hard code address in this function */
	FIH_DECLARE(fih_rc, FIH_FAILURE);

	struct image_header hdr_img0;
	struct image_header hdr_img1;
	uint8_t iv[8];

	BOOT_RSIP_Load_Image(FLASH_AREA_IMAGE_PRIMARY(0), iv, &hdr_img0);
	BOOT_RSIP_Load_Image(FLASH_AREA_IMAGE_PRIMARY(1), NULL, &hdr_img1);

	if (SYSCFG_OTP_RSIPEn() == TRUE) {
		RSIP_IV_Set(1, iv);
	}

	/* NOTE: Mapping start address should take mcuboot header(0x200) into consideration to make
	 * actual code start at IMG_NS_LOGIC_ADDR
	 */
	RSIP_MMU_Config(MMU_ID1, IMG_NS_LOGIC_ADDR - hdr_img1.ih_hdr_size, IMG_TZ_LOGIC_ADDR,
			FLASH_BASE_PHY_ADDR + DT_REG_ADDR(IMG_NS_PATITION));
	RSIP_MMU_Cmd(MMU_ID1, ENABLE);
	RSIP_MMU_Cache_Clean();

	FIH_CALL(BOOT_OTFCheck, fih_rc, IMG_NS_LOGIC_ADDR, IMG_NS_LOGIC_ADDR + hdr_img1.ih_img_size,
		 1, 1);

	/* NOTE: KM4TZ's code start address is determing by:
	 *   CONFIG_FLASH_BASE_ADDRESS(like:0x04000000) + DT_REG_ADDR(IMG_TZ_PATITION)
	 *   CONFIG_FLASH_BASE_ADDRESS is configured to primary_logic_addr_base
	 *   primary_logic_addr_base equals to IMG0_LOGIC_ADDR - IMG0_SLOT0_OFFSET
	 #   IMG0_SLOT0_OFFSET equals to DT_REG_ADDR(IMG_TZ_PATITION)
	 */
	RSIP_MMU_Config(MMU_ID2, IMG_TZ_LOGIC_ADDR, IMG_TZ_LOGIC_ADDR + 0x02000000,
			FLASH_BASE_PHY_ADDR + DT_REG_ADDR(IMG_TZ_PATITION));
	RSIP_MMU_Cmd(MMU_ID2, ENABLE);
	RSIP_MMU_Cache_Clean();

	uint32_t start, end;

	start = IMG_TZ_LOGIC_ADDR + hdr_img0.ih_hdr_size;
	end = start + hdr_img0.ih_img_size;
	FIH_CALL(BOOT_OTFCheck, fih_rc, start, end, 1, 2);
}

void Boot_Copy_NP_Image(void)
{
	static const char *const NpLabel[] = {"NP XIP IMG", "NP SRAM", "NP PSRAM"};
	u32 StartAddr = IMG_NS_LOGIC_ADDR;
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
			LOG_DBG("NP_BOOT_INDEX: %x", ImgHdr.image_addr);
			HAL_WRITE32(SYSTEM_CTRL_BASE, REG_LSYS_BOOT_ADDR_NS, ImgHdr.image_addr);
		}

		/* If not XIP sub-image, copy it to specific address(include the IMAGE_HEADER)*/
		LOG_DBG("try copy %s: %x <- %x, %x", NpLabel[i], DstAddr, StartAddr, Len);
		if ((!IS_FLASH_ADDR(DstAddr)) && (Len > IMAGE_HEADER_LEN)) {
			LOG_DBG("  copy");
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

void mcuboot_status_change(mcuboot_status_type_t status)
{
	if (status != MCUBOOT_STATUS_BOOTABLE_IMAGE_FOUND) {
		return;
	}

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
