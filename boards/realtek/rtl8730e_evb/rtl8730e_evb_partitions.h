/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared flash-partition / XIP-logic-address macros for the RTL8730E EVB.
 *
 */

#ifndef RTL8730E_EVB_PARTITIONS_H_
#define RTL8730E_EVB_PARTITIONS_H_

#include <mem.h>

/* Physical flash base (SPI_FLASH_BASE) */
#define FLASH_BASE_PHY    0x08000000

/*
 * RSIP MMU window base for boot_prepare()
 *   KM0 : KM0_IMG2_XIP  ORIGIN 0x0C000020 - 0x20 = 0x0C000000  (MMU_LP_IDX, np_logic)
 *   KM4 : KM4_IMG2_XIP  ORIGIN 0x0D000020 - 0x20 = 0x0D000000  (MMU_HP_IDX, ap_logic)
 *   CA32: CA32_IMG2_XIP ORIGIN 0x0E000020 - 0x20 = 0x0E000000  (MMU_AP_IDX, ca32_logic)
 */
#define KM4_LOGIC_ADDR    0x0D000000
#define KM0_LOGIC_ADDR    0x0C000000
#define CA32_LOGIC_ADDR   0x0E000000

/* Partition sizes */
#define BOOT_SLOT_BASE    0x0
#define BOOT_SLOT_SIZE    DT_SIZE_K(256)   /* 0x00040000 - holds MCUboot (~93 KB) */
#define APP_SLOT_SIZE     DT_SIZE_K(1024)

/* Partition offsets (calculated) */
#define APP_SLOT0_OFFSET  (BOOT_SLOT_BASE   + BOOT_SLOT_SIZE)  /* 0x00040000 */
#define APP_SLOT1_OFFSET  (APP_SLOT0_OFFSET + APP_SLOT_SIZE)   /* 0x00140000 */
#define STORAGE_OFFSET    (APP_SLOT1_OFFSET + APP_SLOT_SIZE)   /* 0x00240000 */

#endif /* RTL8730E_EVB_PARTITIONS_H_ */
