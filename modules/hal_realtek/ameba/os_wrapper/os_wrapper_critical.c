/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "os_wrapper.h"
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
LOG_MODULE_REGISTER(os_if_critical);

/*
 * Critical section implementation for SMP (CONFIG_SMP=y) and UP.
 *
 * Design:
 *   - One k_spinlock per RTOS_CRITICAL_LIST component provides fine-grained
 *     mutual exclusion: different subsystems (WiFi, BT, USB…) can proceed
 *     concurrently and only same-component callers serialise against each other.
 *   - k_spinlock disables local IRQs on acquisition and spins the other CPU
 *     on an independent atomic_t, ensuring cross-CPU exclusion.
 *   - Nesting is tracked in per-CPU arrays (outer index = CPU id) so a single
 *     CPU's data sits in the same cache line regardless of which component is
 *     accessed.  After k_spin_lock() local IRQs are off, so the per-CPU arrays
 *     are accessed exclusively by the owning CPU — no further synchronisation.
 *   - The saved IRQ key from k_spin_lock() is also stored per-CPU so that the
 *     unlock restores exactly the IRQ flags that were active when the outermost
 *     critical_enter was called.
 *
 * Cross-component nesting (Enter(A), Enter(B), Exit(B), Exit(A)):
 *   Each component has its own lock.  Enter(B) while holding A calls
 *   k_spin_lock(B) with IRQs already off (A's lock disabled them); the returned
 *   key records "IRQs were already off".  Exit(B) restores that — IRQs stay
 *   off.  Exit(A) restores the original "IRQs were on" key.  Correct.
 *
 * ISR safety:
 *   arch_curr_cpu()->id is valid in ISR context.  k_spin_lock() is safe from
 *   ISRs: it calls arch_irq_lock() (which is idempotent when already off) then
 *   spins.  The nesting check before k_spin_lock() is safe because Zephyr
 *   threads are CPU-pinned during execution and the per-CPU arrays are read
 *   before IRQs are disabled.  A thread cannot migrate CPUs between the read
 *   and the lock, so the CPU-id remains stable.
 *
 * UP builds (CONFIG_SMP=n):
 *   k_spinlock degenerates to arch_irq_lock()/arch_irq_unlock() with no
 *   atomic spin — identical overhead to the original irq_lock() implementation.
 */

#define NCPUS CONFIG_MP_MAX_NUM_CPUS

/*
 * Per-component locks (shared across CPUs — one per subsystem).
 * Indexed by RTOS_CRITICAL_LIST component_id.
 */
static struct k_spinlock critical_locks[RTOS_CRITICAL_MAX];

/*
 * Per-CPU state: outer index = CPU id so one CPU's nesting and key data
 * for all components fit in adjacent memory (cache-friendly access pattern).
 */
static k_spinlock_key_t critical_keys[NCPUS][RTOS_CRITICAL_MAX];
static uint32_t         ulCriticalNesting[NCPUS][RTOS_CRITICAL_MAX];

int rtos_critical_is_in_interrupt(void)
{
#ifdef CONFIG_ARM_CORE_CA32
	return (__get_mode() != CPSR_M_USR) && (__get_mode() != CPSR_M_SYS);
#elif CONFIG_ARM_CORE_CM4
	return (__get_xPSR() & 0x1FF) != 0;
#elif defined(CONFIG_RSICV_CORE_KR4)
	return plic_get_active_irq_id() != 0;
#else
	return __get_IPSR() != 0;
#endif
}

void rtos_critical_enter(uint32_t component_id)
{
	if (component_id >= RTOS_CRITICAL_MAX) {
		component_id = RTOS_CRITICAL_DEFAULT;
	}

	unsigned int cpu = arch_curr_cpu()->id;

	if (ulCriticalNesting[cpu][component_id] == 0) {
		critical_keys[cpu][component_id] =
			k_spin_lock(&critical_locks[component_id]);
	}
	ulCriticalNesting[cpu][component_id]++;
}

void rtos_critical_exit(uint32_t component_id)
{
	if (component_id >= RTOS_CRITICAL_MAX) {
		component_id = RTOS_CRITICAL_DEFAULT;
	}

	unsigned int cpu = arch_curr_cpu()->id;

	if (ulCriticalNesting[cpu][component_id] == 0) {
		/* Unbalanced exit — should not happen */
		return;
	}

	ulCriticalNesting[cpu][component_id]--;
	if (ulCriticalNesting[cpu][component_id] == 0) {
		k_spin_unlock(&critical_locks[component_id],
			      critical_keys[cpu][component_id]);
	}
}

uint32_t rtos_get_critical_state(void)
{
	unsigned int cpu = arch_curr_cpu()->id;
	uint32_t depth = 0;

	for (int i = 0; i < RTOS_CRITICAL_MAX; i++) {
		depth += ulCriticalNesting[cpu][i];
	}
	return depth;
}
