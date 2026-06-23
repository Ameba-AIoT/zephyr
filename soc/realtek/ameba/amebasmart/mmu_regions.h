/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Private API between mmu_regions.c (static MMU table) and soc.c
 * (soc_prep_hook patches runtime-only fields at boot).
 */

#ifndef ZEPHYR_SOC_REALTEK_AMEBA_AMEBASMART_MMU_REGIONS_H_
#define ZEPHYR_SOC_REALTEK_AMEBA_AMEBASMART_MMU_REGIONS_H_

#include <stddef.h>
#include <stdint.h>
#include <zephyr/devicetree.h>

/* CA32 NS DRAM base and end, derived from the DTS memory@60300000 node.
 * Current values (TF-A platform_def.h):
 *   AMEBASMART_DRAM0_BASE = 0x60300000 (NS_DRAM0_BASE)
 *   AMEBASMART_DRAM0_SIZE = 0x500000   (NS_DRAM0_SIZE = 5 MB)
 *   AMEBASMART_DRAM_END   = 0x60800000 (PSRAM_END)
 * All three stay in sync when the DTS reg changes.
 */
#define AMEBASMART_DRAM0_BASE  DT_REG_ADDR(DT_CHOSEN(zephyr_sram))
#define AMEBASMART_DRAM0_SIZE  DT_REG_SIZE(DT_CHOSEN(zephyr_sram))
#define AMEBASMART_DRAM_END    (AMEBASMART_DRAM0_BASE + AMEBASMART_DRAM0_SIZE)

void amebasmart_mmu_set_psram_image2_size(size_t size);

void amebasmart_mmu_set_dram_beyond_size(uintptr_t image_ram_end);

#endif /* ZEPHYR_SOC_REALTEK_AMEBA_AMEBASMART_MMU_REGIONS_H_ */
