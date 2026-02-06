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
PROVIDE(__ram_image1_text_start__ = ABSOLUTE(.));

/* Used in merge_bin.py to calculate the padding length */
__km4tz_boot_text_start__ = ABSOLUTE(.);

KEEP(*(SORT_BY_NAME(.image1.entry.data*)))
KEEP(*(.image1.validate.rodata*))
*(.image1.export.symb)
*(.non.dram.text)

/* This symbol is required in lib_chipinfo */
__non_secure_psram_end__ = ABSOLUTE(DT_REG_ADDR(DT_NODELABEL(psram)) + DT_REG_SIZE(DT_NODELABEL(psram)));