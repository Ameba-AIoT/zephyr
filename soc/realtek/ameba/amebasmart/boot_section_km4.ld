/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * amebasmart KM4 MCUboot: landing spot for HAL routines marked
 * SRAMDRAM_ONLY_TEXT_SECTION (ameba_flash_ram.c / ameba_flashclk.c — flash
 * write/clock routines that must not execute out of flash XIP), plus the
 * numbered `.sramdram.only.text.<N>` variants gcc emits for the same section
 * inside the prebuilt km4/lib_chipinfo.a (ameba_rom_patch.o).
 *
 * This is deliberately NOT the CA32 boot_section.ld in this same directory:
 * that one 4 KB-aligns the section for the CA32 app's MMU R+X remap and
 * defines a CA32-only IPC symbol. KM4 MCUboot is already fully SRAM-resident
 * (CONFIG_XIP=n, see loader_section.ld) with an extremely tight ROM boot-copy
 * window, so it needs none of that — just an explicit home for this section
 * instead of letting the linker orphan-place it.
 */

SECTION_PROLOGUE(.sramdram_only_text,,)
{
	*(*.sramdram.only.text*)
} GROUP_LINK_IN(ROMABLE_REGION)
