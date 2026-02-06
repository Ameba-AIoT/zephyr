/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef CONFIG_AMEBA_PSRAM
GROUP_START(PSRAM)
  SECTION_DATA_PROLOGUE(AMEBA_PSRAM_BSS,(NOLOAD),)
  {
    . = ALIGN(32);
    _psram_bss_start = ABSOLUTE(.);
    *(.psram.only.data*)
    *(.lvgl_heap)
    *(.lvgl_buf)
    . = ALIGN(32);
    _psram_bss_end = ABSOLUTE(.);
    _psram_heap_start = ABSOLUTE(.);
  } GROUP_LINK_IN(PSRAM)
GROUP_END(PSRAM)

__psram_region_end = ORIGIN(PSRAM) + CONFIG_AMEBA_PSRAM_SIZE;
PROVIDE(_psram_heap_end = __psram_region_end);
#endif