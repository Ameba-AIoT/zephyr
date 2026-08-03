/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 * SPDX-License-Identifier: Apache-2.0
 *
 * ld.bfd 2.38 (asdk-12.3.1) places .intList* as orphan sections in FLASH
 * before reaching the standard intlist.ld rule in snippets-sections.ld.
 * Injecting this rule at SORT_KEY 0x0 (before rom_start) ensures the section
 * is claimed before any orphan placement occurs.
 */
#include <zephyr/linker/intlist.ld>
