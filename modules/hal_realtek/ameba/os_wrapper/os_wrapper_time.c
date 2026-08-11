/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "os_wrapper.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(os_if_time);

void rtos_time_delay_ms(uint32_t ms)
{
	/* Fall back to busy-wait when scheduler cannot switch. */
	if (!rtos_critical_is_in_interrupt() && rtos_sched_get_state() == RTOS_SCHED_RUNNING &&
	    rtos_get_critical_state() == 0) {
		k_msleep(ms);
	} else {
		DelayMs(ms);
	}
}

void rtos_time_delay_us(uint32_t us)
{
	DelayUs(us);
}

uint32_t rtos_time_get_current_system_time_ms(void)
{
	return k_uptime_get_32();
}

uint64_t rtos_time_get_current_system_time_us(void)
{
	/* Cycle counter gives sub-tick precision. */
	return k_cyc_to_us_floor64(k_cycle_get_64());
}

uint64_t rtos_time_get_current_system_time_ns(void)
{
	return k_cyc_to_ns_floor64(k_cycle_get_64());
}

uint32_t rtos_time_get_current_pended_time_ms(void)
{
	return 0;
}

uint64_t rtos_time_get_current_system_time_ms_64bit(void)
{
	return (uint64_t)k_uptime_get();
}
