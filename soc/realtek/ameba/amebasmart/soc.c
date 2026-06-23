/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/init.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/interrupt_controller/gic.h>
#include <zephyr/linker/linker-defs.h>
#include <cmsis_core.h>

#include "mmu_regions.h"

#define VECTOR_ADDRESS ((uintptr_t)_vector_start)

/*
 * Patch runtime-only MMU region fields before z_arm_mmu_init walks
 * the table.  Linker-symbol subtraction is not a constant expression
 * and cannot appear in mmu_regions.c's static initialiser.
 */
extern char _image_ram_end[];
extern char __image2_backtrace_start__[];
extern char __image2_backtrace_end__[];

void soc_prep_hook(void)
{
	uintptr_t start = (uintptr_t)__image2_backtrace_start__;
	uintptr_t end = (uintptr_t)__image2_backtrace_end__;

	__ASSERT_NO_MSG((start & 0xfff) == 0);
	__ASSERT_NO_MSG(end > start);

	if ((start & 0xfff) != 0 || end <= start) {
		return;
	}

	amebasmart_mmu_set_psram_image2_size((size_t)(end - start));

	uintptr_t img_end = (uintptr_t)_image_ram_end;

	__ASSERT_NO_MSG((img_end & 0xfff) == 0);
	__ASSERT_NO_MSG(img_end < AMEBASMART_DRAM_END);

	if ((img_end & 0xfff) != 0 || img_end >= AMEBASMART_DRAM_END) {
		return;
	}

	amebasmart_mmu_set_dram_beyond_size(img_end);
}

/*
 * Zephyr-side implementation of HAL irq_disable_save / irq_enable_restore.
 *
 * The HAL declares these in fwlib/include/ameba_arch.h but does not provide
 * a CA32 implementation -- ram_common/ameba_arch.c is excluded from CA32
 * build (it carries Cortex-M SysTick code), and the FreeRTOS SDK supplies
 * its own CA32 version in ap_core/ameba_irq.c.  Zephyr fills this slot
 * here, mapped onto irq_lock / irq_unlock.
 *
 * The lock key flows through the caller's stack via the return value /
 * parameter pair, exactly like irq_lock()/irq_unlock() and like the HAL
 * contract for these wrappers.  Do NOT cache the key in a file-static
 * variable: nested critical sections (e.g. FLASH_Write_Lock entered from
 * an outer driver path that is itself inside a critical section) would
 * overwrite the outer key on the inner irq_disable_save and lose the
 * outer lock state when the inner irq_enable_restore unlocks.
 */
uint32_t irq_disable_save(void)
{
	return (uint32_t)irq_lock();
}

void irq_enable_restore(uint32_t PrevStatus)
{
	irq_unlock((unsigned int)PrevStatus);
}

/*
 * Empty stubs for HAL ameba_flash_ram.c FLASH_Write_Lock/Unlock helpers.
 *
 * Strong-overriding FLASH_Write_Lock/Unlock in this file would be the
 * cleaner expression of intent, but cannot drop these stubs: HAL puts
 * FLASH_Write_Lock alongside FLASH_*XIP into the same explicit section
 * (SRAMDRAM_ONLY_TEXT_SECTION = .sramdram.only.text).  --gc-sections
 * works at section granularity, and the FLASH_*XIP entry points are
 * called by the flash driver, so the whole section -- including the
 * weak FLASH_Write_Lock body that references these helpers -- is kept.
 * Hence the symbols still need to resolve.
 *
 * Functionally these are no-ops because Zephyr boots amebasmart in UP
 * mode and runs from DRAM: there is no second CA32 core to gate, and
 * no XIP region whose MMU window needs flipping.
 */
void vPortGateOtherCore(void)
{
}

void vPortWakeOtherCore(void)
{
}

void xlat_flash_region_device(void)
{
}

void xlat_flash_region_xip(void)
{
}

void relocate_vector_table(void)
{
	__set_VBAR(VECTOR_ADDRESS & ~0x1f);
	__ISB();
}

/*
 * IPC AP interrupt bring-up must run at PRE_KERNEL_2, after the GIC driver's
 * PRE_KERNEL_1 init.  gic_dist_init() writes 0xffffffff to GICD_ICENABLERn for
 * every SPI, so a GIC enable done earlier (e.g. in soc_early_init_hook) would
 * be wiped.  ipc_table_init() only programs IPCAP_IMR (peripheral MMIO), but is
 * kept together with the IRQ wiring so the whole IPC bring-up lives in one
 * place and is ordered like the vendor SDK (connect/enable, then table init).
 *
 * IPC_AP_IRQ is a GIC SPI index (0-based); the Zephyr IRQ number is the full
 * GIC INTID, i.e. SPI index + GIC_SPI_INT_BASE.
 */
static int soc_ipc_irq_init(void)
{
	ipc_table_init(IPCAP_DEV);
	IRQ_CONNECT(IPC_AP_IRQ + GIC_SPI_INT_BASE, INT_PRI_MIDDLE, IPC_INTHandler,
		    (uint32_t)IPCAP_DEV, 0);
	irq_enable(IPC_AP_IRQ + GIC_SPI_INT_BASE);

	return 0;
}
SYS_INIT(soc_ipc_irq_init, PRE_KERNEL_2, 0);

void soc_early_init_hook(void)
{
	uint32_t flash_para_addr = HAL_READ32(SYSTEM_CTRL_BASE_LP,
					      REG_LSYS_FLASH_PARA_ADDR);

	if (flash_para_addr == 0U) {
		return;
	}

	DCache_Invalidate(flash_para_addr, sizeof(FLASH_InitTypeDef));
	_memcpy(&flash_init_para, (const void *)flash_para_addr,
		sizeof(FLASH_InitTypeDef));
}
