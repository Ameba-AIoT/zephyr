/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <ameba_psram.h>
#include <zephyr/multi_heap/shared_multi_heap.h>

extern int _psram_bss_start;
extern int _psram_bss_end;
extern int _psram_heap_start;

#ifdef CONFIG_PSRAM_HEAP_SHARED_MULTI
static struct shared_multi_heap_region smh_psram = {
	.addr = (uintptr_t)&_psram_heap_start,
	.size = CONFIG_AMEBA_PSRAM_HEAP_SIZE,
	.attr = SMH_REG_ATTR_CACHEABLE,
};
#endif

#ifdef CONFIG_PSRAM_HEAP_SINGLE
static struct k_heap ameba_psram_heap;
struct k_heap *g_ameba_psram_heap = &ameba_psram_heap;
static uint8_t PSRAM_ONLY_DATA_SECTION psram_heap_area[CONFIG_AMEBA_PSRAM_HEAP_SIZE];
#endif

void ameba_init_psram(void)
{
#if defined(CONFIG_PSRAM_HEAP_SHARED_MULTI)
	shared_multi_heap_pool_init();
	if (shared_multi_heap_add(&smh_psram, NULL)) {
		printk("Failed to Initialize PSRAM, aborting.\n");
		return;
	}
#elif defined(CONFIG_PSRAM_HEAP_SINGLE)
	k_heap_init(g_ameba_psram_heap, psram_heap_area, CONFIG_AMEBA_PSRAM_HEAP_SIZE);
#endif
}
