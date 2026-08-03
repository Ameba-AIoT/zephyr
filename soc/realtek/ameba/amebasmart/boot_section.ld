/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * KM0 IPC SRAM ring base.  Used by HAL ameba_ipc_api.c.
 * Mirrors SDK's ameba_layout.ld:73 (`__km0_ipc_memory_start__ = ORIGIN(KM0_IPC_RAM)`),
 * but Zephyr does not consume that ld file as a linker script (it is image
 * post-processing metadata only -- see merge_bin.py / image_process/context.py),
 * so the symbol must be provided here for the Zephyr link to resolve.
 * Address is fixed by silicon layout and matches ATF platform_def.h::KM0_IPC_RAM.
 */
__km0_ipc_memory_start__ = 0x2301FD00;

  SECTION_PROLOGUE(.psram_image2.text.data,,)
  {
      *(*.sramdram.only.data)
      /*
       * Pad to 4 KB so the executable block below starts on a fresh MMU
       * page.  mmu_regions.c maps __image2_backtrace_start__ ..
       * __image2_backtrace_end__ as R+X; this alignment is what lets that
       * 4 KB-granular L2 override sit cleanly on top of the surrounding
       * R+W mapping.
       */
      . = ALIGN(4096);
      __image2_backtrace_start__ = .;
      *(*.sramdram.only.text)
      *(.wlan.critical.text*)

      /* lib_pmc.a a32_pm.o (cpu_suspend/cpu_resume/vPortBackupRegs/...) is in a
       * .boot section; place it in this R+X, flat-mapped, DRAM-retained block
       * (REGION_PSRAM_IMAGE2) so it executes and cpu_resume's warm-boot entry
       * resolves to its physical link address with the MMU off.
       */
      KEEP(*(.boot))

      . = ALIGN (4);
      __ipc_table_start__ = .;
      KEEP(*(*.ipc.table.data*))
      __ipc_table_end__ = .;
      __image2_backtrace_end__ = .;

      /* Pad to 4 KB so the next section (data/BSS) starts on a fresh page. */
      . = ALIGN(4096);
  } GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)
