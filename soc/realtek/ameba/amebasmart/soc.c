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
#include <zephyr/sys/barrier.h>

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

uint64_t vGetGenericTimerFreq(void)
{
	extern unsigned int z_clock_hw_cycles_per_sec;

	return (uint64_t)z_clock_hw_cycles_per_sec;
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
 * The Cortex-A/R arch layer's default sys_arch_reboot() (arch/arm/core/
 * cortex_a_r/reboot.c) is a no-op __weak stub, unlike Cortex-M's
 * NVIC_SystemReset(). Without this override sys_reboot() silently never
 * resets on CA32. System_Reset() is the same Ameba fwlib warm-reset trigger
 * common/reset_ameba_shell.c's "reboot uartburn" command already uses.
 */
void sys_arch_reboot(int type)
{
	ARG_UNUSED(type);

	System_Reset();
}

/*
 * IPC AP interrupt bring-up must run at PRE_KERNEL_2, after the GIC driver's
 * PRE_KERNEL_1 init.  gic_dist_init() writes 0xffffffff to GICD_ICENABLERn for
 * every SPI, so a GIC enable done earlier (e.g. in soc_early_init_hook) would
 * be wiped.  ipc_table_init() only programs IPCAP_IMR (peripheral MMIO), but is
 * kept together with the IRQ wiring so the whole IPC bring-up lives in one
 * place and is ordered like the vendor SDK (connect/enable, then table init).
 */
static int soc_ipc_irq_init(void)
{
	ipc_table_init(IPCAP_DEV);
	IRQ_CONNECT(IPC_AP_IRQ, INT_PRI_MIDDLE, IPC_INTHandler, (uint32_t)IPCAP_DEV, 0);
	irq_enable(IPC_AP_IRQ);

	return 0;
}
SYS_INIT(soc_ipc_irq_init, PRE_KERNEL_2, 0);

#if CONFIG_MP_MAX_NUM_CPUS > 1
/*
 * Pin ALL SPI (external peripheral) IRQs to Core0 — same policy the
 * FreeRTOS SDK uses on this silicon.
 *
 * gic_dist_init() (arm_gic v2) writes every SPI's ITARGETSR byte with
 * (Core0 | Core1) bits set — the "1-of-N" model then delivers each
 * pending SPI IRQ to whichever CPU is available.  Under CONFIG_SMP this
 * gives peripheral ISRs a non-deterministic CPU affinity and opens up a
 * class of races between writers running in thread context on one core
 * and their matching ISR firing on the other core.  Concrete case
 * observed on RTL8730E: the LogUART shell TX ISR racing shell_uart
 * irq_write occasionally truncates the warm-reset boot burst (banner /
 * "Secondary CPU 1 is up" / "Test thread: loop 1#" silently lost from
 * UART, verified via JLink halt — main/test_task both reach
 * _THREAD_DEAD normally, no fatal, just missing output).
 *
 * FreeRTOS avoids this class of bug by binding every SPI to Core0.  Do
 * the same here: at PRE_KERNEL_2 (after gic_dist_init at PRE_KERNEL_1
 * has written the default all-CPUs mask, and before any SYS_INIT enables
 * a peripheral IRQ), rewrite every SPI's ITARGETSR byte to Core0-only.
 *
 * SGIs (0-15) and PPIs (16-31) are per-CPU by construction and are not
 * touched.  Core1's per-CPU IRQs (arch timer, IPIs, banked GIC state)
 * remain unaffected and continue to run on Core1.
 */
static int soc_pin_spi_irqs_to_core0(void)
{
	const uint32_t gic_irqs_field = sys_read32(GICD_TYPER) & 0x1f;
	const unsigned int gic_irqs = MIN((gic_irqs_field + 1) * 32U, 1020U);
	/* ITARGETSRn is byte-per-IRQ, word-accessible.  SGI/PPI target
	 * bytes (IRQ 0..31) are read-only in v2, so we start writing from
	 * SPI base (IRQ 32) — write in 32-bit words, four IRQs per word.
	 */
	const uint32_t core0_mask = BIT(0) | (BIT(0) << 8) | (BIT(0) << 16) | (BIT(0) << 24);
	for (unsigned int i = GIC_SPI_INT_BASE; i < gic_irqs; i += 4) {
		sys_write32(core0_mask, GICD_ITARGETSRn + i);
	}
	barrier_dmem_fence_full();

	return 0;
}
/* Priority 1: run after soc_ipc_irq_init (prio 0) at PRE_KERNEL_2, and
 * before any POST_KERNEL SYS_INIT that would enable a peripheral IRQ.
 */
SYS_INIT(soc_pin_spi_irqs_to_core0, PRE_KERNEL_2, 1);
#endif /* CONFIG_MP_MAX_NUM_CPUS > 1 */

void soc_early_init_hook(void)
{
	/* Program the ARM generic-timer rate from the live PLL configuration. */
	extern unsigned int z_clock_hw_cycles_per_sec;

	z_clock_hw_cycles_per_sec = PLL_GetHBUSClk() / 2;

	uint32_t flash_para_addr = HAL_READ32(SYSTEM_CTRL_BASE_LP, REG_LSYS_FLASH_PARA_ADDR);

	if (flash_para_addr == 0U) {
		return;
	}

	DCache_Invalidate(flash_para_addr, sizeof(FLASH_InitTypeDef));
	_memcpy(&flash_init_para, (const void *)flash_para_addr, sizeof(FLASH_InitTypeDef));
}

#ifdef CONFIG_SMP
/*
 * soc_cpu_power_on - called by arch_cpu_start() before PSCI CPU_ON.
 *
 * RTL8730E's ATF SP_MIN PSCI pwr_domain_on() only polls for Core1's
 * WFE-standby bit (CA32_STANDBYWFE_CORE1) and then writes the hold-base
 * to release it.  Core1 must already be powered on and waiting in WFE
 * before the PSCI call is made.
 *
 * Power-on sequence (mirrors rtk_core1_power_on in Green2_Z smp.c):
 *  1. Assert Core1 resets via CA32_C0_RST_CTRL
 *  2. Set isolation bits for Core1 power domains
 *  3. Enable power switch for Core1 (HSYS_HP_PWC)
 *  4. Release isolation
 *  5. De-assert Core1 resets → Core1 starts executing from its reset vector
 *     and will spin in ATF's secondary cold-boot WFE loop.
 *
 * A short delay after power-on allows Core1 to reach the WFE loop in
 * ATF before the PSCI cpu_on SMC fires (ATF's pwr_domain_on has a
 * 100 ms timeout).
 */
void soc_cpu_power_on(uint32_t cpu_mpid)
{
	uint32_t val;
	CA32_TypeDef *ca32 = CA32_BASE;

	/* Already powered on? (isolation released = cores running) */
	if ((HSYS_GET_ISO_HP_AP_CORE(HAL_READ32(SYSTEM_CTRL_BASE_HP, REG_HSYS_HP_ISO)) == 0)) {
		return;
	}

	/* 1. Assert Core1 reset */
	ca32->CA32_C0_RST_CTRL &= ~(CA32_NCOREPORESET(0x2) | CA32_NCORERESET(0x2));

	/* 2. Set isolation for Core1 power domains */
	val = HAL_READ32(SYSTEM_CTRL_BASE_HP, REG_HSYS_HP_ISO);
	val |= HSYS_ISO_HP_AP_CORE(0x2);
	HAL_WRITE32(SYSTEM_CTRL_BASE_HP, REG_HSYS_HP_ISO, val);
	DelayUs(50);

	/* 3a. Power on Core1 primary power switch */
	val = HAL_READ32(SYSTEM_CTRL_BASE_HP, REG_HSYS_HP_PWC);
	val |= HSYS_PSW_HP_AP_CORE(0x3);
	HAL_WRITE32(SYSTEM_CTRL_BASE_HP, REG_HSYS_HP_PWC, val);
	DelayUs(50);

	/* 3b. Power on Core1 secondary power switch */
	val = HAL_READ32(SYSTEM_CTRL_BASE_HP, REG_HSYS_HP_PWC);
	val |= HSYS_PSW_HP_AP_CORE_2ND(0x3);
	HAL_WRITE32(SYSTEM_CTRL_BASE_HP, REG_HSYS_HP_PWC, val);
	DelayUs(500);

	/* 4. Release isolation */
	val = HAL_READ32(SYSTEM_CTRL_BASE_HP, REG_HSYS_HP_ISO);
	val &= ~HSYS_ISO_HP_AP_CORE(0x3);
	HAL_WRITE32(SYSTEM_CTRL_BASE_HP, REG_HSYS_HP_ISO, val);
	DelayUs(50);

	/* 5. De-assert Core1 resets */
	ca32->CA32_C0_RST_CTRL |= (CA32_NCOREPORESET(0x2) | CA32_NCORERESET(0x2));

	/*
	 * Give Core1 time to reach ATF's WFE hold-loop.
	 * DRAM access takes ~7 µs, PSRAM ~15 µs; use 40 µs as margin.
	 */
	DelayUs(40);
}

/*
 * soc_per_core_init_hook - secondary core MMU / GIC bring-up.
 *
 * Called from arch_secondary_cpu_init() on Core1 after stack setup
 * and MMU initialisation.  Re-initialise the GIC CPU interface for
 * this core and enable the sched IPI SGI.
 */
void soc_per_core_init_hook(void)
{
#ifdef CONFIG_PM
	/*
	 * If this secondary core is warm-booting after cluster power-gating
	 * (CPU1 hotplug), longjmp back into the PM sleep path to resume its
	 * pre-sleep context instead of continuing as a fresh secondary CPU.
	 * Returns (falls through to a normal bring-up) if not a PG resume.
	 */
	extern bool amebasmart_cpu1_pg_resume_if_needed(void);

	(void)amebasmart_cpu1_pg_resume_if_needed();
#endif
	/* Nothing extra needed beyond arch_secondary_cpu_init() defaults */
}
#endif /* CONFIG_SMP */
