
/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "boot_prepare.h"

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <sysflash/sysflash.h>
#include "bootutil/mcuboot_status.h"

#define IMG_APP_PATITION DT_NODELABEL(slot0_partition)

#define APP_IMAGE_AREA_ID     FLASH_AREA_IMAGE_PRIMARY(0)
#define APP_IMAGE_AREA_OFFSET DT_REG_ADDR(IMG_APP_PATITION)
#define APP_IMAGE_AREA_SIZE   DT_REG_SIZE(IMG_APP_PATITION)
#define IMG_AP_LOGIC_ADDR     DT_PROP(DT_PATH(zephyr_user), primary_logic_addr)
#define IMG_NP_LOGIC_ADDR     DT_PROP(DT_PATH(zephyr_user), secondary_logic_addr)
#define FLASH_BASE_PHY_ADDR   DT_PROP(DT_PATH(zephyr_user), flash_base_phy)       /* 0x08000000 */

void mcuboot_status_change(mcuboot_status_type_t status)
{
	/* NOTE: Only for last step before do_boot */
	if (status != MCUBOOT_STATUS_BOOTABLE_IMAGE_FOUND) {
		return;
	}

#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), tertiary_logic_addr)
	/* AmebaSmart: tri-core (KM0/KM4/CA32) — the CA32 XIP window needs its own
	 * logic base, so boot_prepare() takes a 7th argument here.
	 */
	boot_prepare(FLASH_BASE_PHY_ADDR, APP_IMAGE_AREA_ID, APP_IMAGE_AREA_OFFSET,
		     APP_IMAGE_AREA_SIZE, IMG_AP_LOGIC_ADDR, IMG_NP_LOGIC_ADDR,
		     DT_PROP(DT_PATH(zephyr_user), tertiary_logic_addr));
#else
	/* AmebaG2 / AmebaDplus: dual-image (KM0/KM4), 6-arg boot_prepare(). */
	boot_prepare(FLASH_BASE_PHY_ADDR, APP_IMAGE_AREA_ID, APP_IMAGE_AREA_OFFSET,
		     APP_IMAGE_AREA_SIZE, IMG_AP_LOGIC_ADDR, IMG_NP_LOGIC_ADDR);
#endif
}
