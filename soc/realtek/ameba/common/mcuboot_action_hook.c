
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
#define IMG_AP_LOGIC_ADDR     DT_PROP(DT_PATH(zephyr_user), primary_logic_addr)   /* 0x04000000 */
#define IMG_NP_LOGIC_ADDR     DT_PROP(DT_PATH(zephyr_user), secondary_logic_addr) /* 0x02000000 */
#define FLASH_BASE_PHY_ADDR   DT_PROP(DT_PATH(zephyr_user), flash_base_phy)       /* 0x08000000 */

void mcuboot_status_change(mcuboot_status_type_t status)
{
	/* NOTE: Only for last step before do_boot */
	if (status != MCUBOOT_STATUS_BOOTABLE_IMAGE_FOUND) {
		return;
	}

	boot_prepare(FLASH_BASE_PHY_ADDR, APP_IMAGE_AREA_ID, APP_IMAGE_AREA_OFFSET,
		     APP_IMAGE_AREA_SIZE, IMG_AP_LOGIC_ADDR, IMG_NP_LOGIC_ADDR);
}
