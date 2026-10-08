/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * AmebaG2 PMC entry points for a TF-M non-secure image.
 *
 * lib_pmc.a calls SOCPS_*Entry() for the parts of a sleep only the secure world
 * may perform. In the ameba TrustZone scheme those are NS_ENTRY veneers exported
 * by the ameba secure image, so each call crosses worlds; TF-M exports no such
 * veneer, so the calls reached the ROM's non-secure build of the same functions
 * and performed a secure operation from the non-secure world -- an
 * attribution-unit violation, measured as SFAR 0x5080a290 entering a clock-gate
 * and 0x30000690 entering a power-gate.
 *
 * So take the secure half away from lib_pmc.a and ask the secure world for it.
 * Peripheral and bit ownership go to the TF-M platform service, but from
 * common/pm.c rather than here: lib_pmc.a calls these entries with interrupts
 * locked and the caches on their way out, where the non-secure interface cannot
 * take its mutex. The secure world's own CPU state and RAM are its business
 * either way -- a non-secure caller could not have preserved them on its behalf
 * -- so those passes are dropped. The non-secure world's own state is still saved
 * and restored here.
 *
 * --wrap interceptions rather than edits to ameba_pmc.c, which is kept identical
 * to the FreeRTOS SDK where these functions must cross worlds; the wrapped
 * symbols are undefined references in lib_pmc.a, which is what --wrap acts on.
 */

#include <ameba_soc.h>

#include <zephyr/toolchain.h>

/*
 * Distance from a System Control Space block to its non-secure alias, e.g. the
 * SCS at 0xE000E000 and SCS_NS at 0xE002E000 (Armv8-M).
 */
#define PMC_SCS_NS_ALIAS_OFFSET 0x00020000U

/*
 * The non-secure half of the sequence is passed the non-secure aliases, which
 * only the secure world can reach. Running non-secure, the plain addresses are
 * already the non-secure banked registers, so fold the alias away -- otherwise
 * the accesses are dropped and the backup captures (and later restores) zeroes,
 * which leaves SysTick stopped and every interrupt disabled after wake.
 */
#define PMC_SCS_LOCAL(p) ((void *)((u32)(p) & ~PMC_SCS_NS_ALIAS_OFFSET))

/*
 * lib_pmc.a issues backup and restore twice: once for the non-secure world's
 * SysTick/NVIC/SCB/MPU, once for the secure world's. The pass meant for the secure
 * world is told apart by its buffer -- addressed through the secure alias
 * (MSPLIM_RAM_HP, 0x30000600) rather than the non-secure one -- and dropped, the
 * file comment above saying why. Not by TrustZone_IsSecure(), which returns
 * non-secure for both passes.
 */
static inline bool pmc_call_targets_secure_world(const struct CPU_BackUp_TypeDef *bk)
{
	return (u32)bk >= HP_SRAM0_BASE_S;
}

/*
 * Peripheral and bit ownership: secure-only registers, handed over around the
 * whole sleep from common/pm.c instead. Restoring the secure peripherals is
 * TF-M's own business as it comes back up.
 */
void __wrap_SOCPS_PeriPermissionEntry(uint32_t ip_mask, u32 enable)
{
	ARG_UNUSED(ip_mask);
	ARG_UNUSED(enable);
}

void __wrap_SOCPS_BitPermissionEntry(uint32_t ip_mask, u32 enable)
{
	ARG_UNUSED(ip_mask);
	ARG_UNUSED(enable);
}

void __wrap_SOCPS_PeriRestoreEntry(void)
{
}

void __wrap_SOCPS_NVICBackupEntry(struct CPU_BackUp_TypeDef *bk, SysTick_Type *systick,
				  NVIC_Type *nvic, SCB_Type *scb)
{
	if (pmc_call_targets_secure_world(bk)) {
		return;
	}

	SOCPS_NVICBackup(bk, PMC_SCS_LOCAL(systick), PMC_SCS_LOCAL(nvic), PMC_SCS_LOCAL(scb));
}

void __wrap_SOCPS_NVICReFillEntry(struct CPU_BackUp_TypeDef *bk, SysTick_Type *systick,
				  NVIC_Type *nvic, SCB_Type *scb)
{
	if (pmc_call_targets_secure_world(bk)) {
		return;
	}

	SOCPS_NVICReFill(bk, PMC_SCS_LOCAL(systick), PMC_SCS_LOCAL(nvic), PMC_SCS_LOCAL(scb));
}

void __wrap_SOCPS_MPUBackupEntry(struct CPU_BackUp_TypeDef *bk, MPU_Type *mpu)
{
	if (pmc_call_targets_secure_world(bk)) {
		return;
	}

	SOCPS_MPUBackup(bk, PMC_SCS_LOCAL(mpu));
}

void __wrap_SOCPS_MPUReFillEntry(struct CPU_BackUp_TypeDef *bk, MPU_Type *mpu)
{
	if (pmc_call_targets_secure_world(bk)) {
		return;
	}

	SOCPS_MPUReFill(bk, PMC_SCS_LOCAL(mpu));
}
