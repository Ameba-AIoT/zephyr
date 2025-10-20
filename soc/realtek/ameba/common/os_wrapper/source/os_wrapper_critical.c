/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "os_wrapper.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(os_if_critical);

static int key;
static volatile uint32_t ulCriticalNesting;

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

/*-------------------------------critical------------------------------*/
/*
 * Zephyr does not have concept of Critical section
 * disable and enable irq to avoid interrupt and context switch
 */
void rtos_critical_enter(void)
{
	if (ulCriticalNesting == 0) {
		key = irq_lock();
	}
	ulCriticalNesting++;
}

void rtos_critical_exit(void)
{
	ulCriticalNesting--;
	if (ulCriticalNesting == 0) {
		irq_unlock(key);
	}
}

uint32_t rtos_get_critical_state(void)
{
	return ulCriticalNesting;
}
