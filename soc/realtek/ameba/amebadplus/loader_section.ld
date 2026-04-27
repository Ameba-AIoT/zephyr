/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

. = ALIGN(0x20);

/* Used to adding paddings to align code map to match rom entry under requirement of MCUBOOT partition layout
 * In merge_bin.py these paddings is dropped before generating final image
 */
. = . + 0x1020;

/* Used in merge_bin.py to calculate the padding length */
__km4_boot_text_start__ = ABSOLUTE(.);
*(.non.dram.text)

/* This symbol is required in lib_chipinfo */
__non_secure_psram_end__ = ABSOLUTE(DT_REG_ADDR(DT_NODELABEL(psram)) + DT_REG_SIZE(DT_NODELABEL(psram)));

__km4_bd_psram_start__ = DT_REG_ADDR(DT_NODELABEL(psram));
