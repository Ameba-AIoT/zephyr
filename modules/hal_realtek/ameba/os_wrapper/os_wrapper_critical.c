/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "os_wrapper.h"
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
LOG_MODULE_REGISTER(os_if_critical);

/* SMP: per-component spinlock + per-CPU nesting.  UP: single irq_lock. */
#ifdef CONFIG_SMP

static struct k_spinlock critical_locks[RTOS_CRITICAL_MAX];
static k_spinlock_key_t critical_keys[CONFIG_MP_MAX_NUM_CPUS][RTOS_CRITICAL_MAX];
static uint32_t ulCriticalNesting[CONFIG_MP_MAX_NUM_CPUS][RTOS_CRITICAL_MAX];

static struct k_spinlock os_critical_lock;
static k_spinlock_key_t os_critical_key[CONFIG_MP_MAX_NUM_CPUS];
static uint32_t os_critical_nesting[CONFIG_MP_MAX_NUM_CPUS];

#else /* !CONFIG_SMP */

static unsigned int critical_key;
static uint32_t critical_nesting;

#endif /* CONFIG_SMP */

int rtos_critical_is_in_interrupt(void)
{
#if defined(CONFIG_CPU_AARCH32_CORTEX_A)
	return (__get_mode() != CPSR_M_USR) && (__get_mode() != CPSR_M_SYS);
#else
	return __get_IPSR() != 0;
#endif
}

#ifndef CONFIG_SMP
static inline void critical_enter(void)
{
	if (critical_nesting == 0) {
		critical_key = irq_lock();
	}
	critical_nesting++;
}

static inline void critical_exit(const char *who)
{
	if (critical_nesting == 0) {
		LOG_ERR("%s: unbalanced exit", who);
		return;
	}
	critical_nesting--;
	if (critical_nesting == 0) {
		irq_unlock(critical_key);
	}
}
#endif /* !CONFIG_SMP */

void rtos_critical_enter(uint32_t component_id)
{
#ifdef CONFIG_SMP
	unsigned int flags;
	unsigned int cpu;

	if (component_id >= RTOS_CRITICAL_MAX) {
		component_id = RTOS_CRITICAL_DEFAULT;
	}

	/* Disable IRQs before reading cpu_id to prevent migration.  Keep them
	 * disabled through k_spin_lock (avoids TOCTOU on cpu_id).
	 */
	flags = arch_irq_lock();
	cpu = arch_curr_cpu()->id;

	if (ulCriticalNesting[cpu][component_id] == 0) {
		/* First entry: take the spinlock and stash 'flags' (the caller's
		 * original IRQ state) in its key slot.  We discard the key that
		 * k_spin_lock returns — it saves "IRQ already disabled" from our
		 * arch_irq_lock above, not the caller's real state.  The outer-
		 * most exit will k_spin_unlock with our stashed key, restoring
		 * the caller's IRQ state via arch_irq_unlock(flags).
		 */
		(void)k_spin_lock(&critical_locks[component_id]);
		critical_keys[cpu][component_id] = (k_spinlock_key_t){.key = (int)flags};
	}
	/* Nested entry: IRQs are already disabled by the outer lock, so our
	 * arch_irq_lock above was a no-op.  Discard 'flags' — the outermost
	 * key already holds the caller's original IRQ state.
	 */
	ulCriticalNesting[cpu][component_id]++;
#else
	ARG_UNUSED(component_id);
	critical_enter();
#endif
}

void rtos_critical_exit(uint32_t component_id)
{
#ifdef CONFIG_SMP
	unsigned int cpu;

	if (component_id >= RTOS_CRITICAL_MAX) {
		component_id = RTOS_CRITICAL_DEFAULT;
	}

	cpu = arch_curr_cpu()->id; /* stable: IRQs disabled by held spinlock */

	if (ulCriticalNesting[cpu][component_id] == 0) {
		LOG_ERR("%s: unbalanced exit on CPU %u id=%u", __func__, cpu, component_id);
		return;
	}
	ulCriticalNesting[cpu][component_id]--;
	if (ulCriticalNesting[cpu][component_id] == 0) {
		k_spin_unlock(&critical_locks[component_id], critical_keys[cpu][component_id]);
	}
#else
	ARG_UNUSED(component_id);
	critical_exit(__func__);
#endif
}

uint32_t rtos_get_critical_state(void)
{
#ifdef CONFIG_SMP
	unsigned int flags = arch_irq_lock(); /* stabilize cpu_id */
	unsigned int cpu = arch_curr_cpu()->id;
	uint32_t depth = 0;

	for (int i = 0; i < RTOS_CRITICAL_MAX; i++) {
		depth += ulCriticalNesting[cpu][i];
	}
	depth += os_critical_nesting[cpu];
	arch_irq_unlock(flags);
	return depth;
#else
	return critical_nesting;
#endif
}

void __rtos_critical_enter_os(void)
{
#ifdef CONFIG_SMP
	unsigned int flags;
	unsigned int cpu;

	/* Same locking pattern as rtos_critical_enter — see there for why we
	 * stash 'flags' in the spinlock key and discard it on nested entry.
	 */
	flags = arch_irq_lock();
	cpu = arch_curr_cpu()->id;

	if (os_critical_nesting[cpu] == 0) {
		(void)k_spin_lock(&os_critical_lock);
		os_critical_key[cpu] = (k_spinlock_key_t){.key = (int)flags};
	}
	os_critical_nesting[cpu]++;
#else
	critical_enter();
#endif
}

void __rtos_critical_exit_os(void)
{
#ifdef CONFIG_SMP
	unsigned int cpu = arch_curr_cpu()->id;

	if (os_critical_nesting[cpu] == 0) {
		LOG_ERR("%s: unbalanced exit on CPU %u", __func__, cpu);
		return;
	}
	os_critical_nesting[cpu]--;
	if (os_critical_nesting[cpu] == 0) {
		k_spin_unlock(&os_critical_lock, os_critical_key[cpu]);
	}
#else
	critical_exit(__func__);
#endif
}
