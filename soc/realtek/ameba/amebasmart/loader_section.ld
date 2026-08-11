/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * amebasmart KM4 MCUboot: all-SRAM-resident image1 entry layout.
 *
 * With CONFIG_XIP=n, ROMABLE_REGION = RAM = sram0 (0x30003020).
 * This file is injected first (ROM_START SORT_KEY 0x0), so the
 * entry table lands at the very start of SRAM = 0x30003020.
 *
 * ROM reads (from the fixed constant __ram_start_table_start__ = 0x30003020):
 *   [+0x00]  RamStartTable (32 B)     → calls FlashStartFun (SRAM address) → MCUboot
 *   [+0x20]  validate bytes            → ROM_IMG1_VALID_PATTEN check
 *   [+0x28]  export symbol
 *
 * NOTE: No 0x1020 XIP padding here.  For SRAM-resident boot, the ROM copies
 * the whole image to SRAM and there is no XIP alignment requirement.
 * __km4_boot_text_start__ = start of SRAM = image_addr used by merge_bin.py
 * to build the image header (ROM copies to image_addr - IMAGE_HEADER_LEN = 0x30003000).
 */

. = ALIGN(0x20);

__ram_image1_text_start__ = .;
__ram_start_table_start__ = .;

/* __km4_boot_text_start__ is used by mcuboot_image.cmake to set image_addr.
 * For amebasmart all-SRAM, it equals __ram_start_table_start__ (no XIP offset). */
__km4_boot_text_start__ = ABSOLUTE(.);

/* Entry table → validate bytes → export symbol (order is critical for ROM) */
KEEP(*(SORT(.image1.entry.data*)))
KEEP(*(.image1.validate.rodata*))
KEEP(*(.image1.export.symb))

/* SRAM-only code (e.g. flash-write routines that must not execute from XIP) */
*(.non.dram.text)

/* Required by the chipinfo library's ChipInfo_PsramBoundary(). */
__non_secure_psram_end__ = ABSOLUTE(DT_REG_ADDR(DT_NODELABEL(psram)) + DT_REG_SIZE(DT_NODELABEL(psram)));

/*
 * NP (KM4) image2 entry = KM4_BD_DRAM origin, i.e. the DRAM base plus the
 * Realtek sub-image header (IMAGE_HEADER_LEN, 0x20) -- see ameba_layout.ld.
 * The MCUboot image1 has no __image2_entry_func__ of its own; BOOT_WakeFromPG
 * (PG warm-boot handler) needs it to resume the loaded NP image2's RamWakeupFun.
 */
__image2_entry_func__ = ABSOLUTE(DT_REG_ADDR(DT_NODELABEL(psram)) + 0x20);
