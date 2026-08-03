/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * amebasmart KM4 MCUboot: relocate z_main_stack (zephyr/kernel/init.c) into
 * KM4_MSP_RAM_S [MSPLIM_RAM_HP, HS_BOOT_ADDR_START) -- ROM-reserved secure-MSP space,
 * outside the tight ROM boot-copy window -- freeing 8 KB from that window.
 *
 * z_main_stack gets its own noinit input section, ".noinit.<file>.<N>", N
 * being a per-file declaration-order counter (verified: index 0 here).
 * FRAGILE: a future Zephyr change could shift that index -- the ASSERTs
 * below turn silent drift into a build failure instead of a boot hang.
 * Must land ahead of the generic `.noinit.*` catch-all in common-noinit.ld;
 * RAM_SECTIONS is included earlier in linker_cortex_m.ld, so it is.
 */

#include "hal_platform.h"

.km4_msp_stack MSPLIM_RAM_HP (NOLOAD) :
{
	KEEP(*(".noinit.WEST_TOPDIR/zephyr/kernel/init.c.0"))
}

/* Catches the ".0" index drifting to capture the wrong noinit global. */
ASSERT(SIZEOF(.km4_msp_stack) == CONFIG_MAIN_STACK_SIZE,
       "km4_msp_stack.ld captured the wrong section -- re-check the .noinit index")

ASSERT(ADDR(.km4_msp_stack) + SIZEOF(.km4_msp_stack) <= HS_BOOT_ADDR_START,
       "z_main_stack overruns the MCUboot ROM boot-copy window (HS_BOOT_ADDR_START)")
