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
    . = ALIGN(32);
    _psram_bss_end = ABSOLUTE(.);
    _psram_heap_start = ABSOLUTE(.);
  } GROUP_LINK_IN(PSRAM)
GROUP_END(PSRAM)
#endif