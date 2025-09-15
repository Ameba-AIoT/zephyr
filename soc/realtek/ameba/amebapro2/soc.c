/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <hal_pinmux.h>
#include <hal_timer.h>
#include <hal_wdt.h>
#include <hal_gdma.h>

#include <zephyr/irq.h>

extern hal_pin_mux_mang_t pinmux_manager;
extern hal_timer_group_adapter_t timer_group1;
extern hal_timer_adapter_t system_timer;
extern hal_gdma_group_t hal_gdma_group;
extern uint32_t _vector_start;
extern void z_arm_reset(void);

SECTION(".ram.start.text")
void ram_start_func(void)
{
	__set_MSP(_vector_start);
	z_arm_reset();
}

void soc_early_init_hook(void)
{
	hal_wdt_all_disable();

	/* system timer (gtimer 1) for hal_delay_us */
	hal_timer_clock_init(1, ENABLE);
	hal_timer_group_init(&timer_group1, 1);
	hal_timer_group_sclk_sel(&timer_group1, GTimerSClk_31_25M);
	hal_start_systimer(&system_timer, CONFIG_SYS_TIMER_ID, GTimerCountUp, CONFIG_SYS_TICK_TIME,
			   1);

	hal_pinmux_manager_init(&pinmux_manager);

	hal_gdma_group_init(&hal_gdma_group);

	/* disable fcs timer irq */
	irq_disable(TimerGroup3_IRQn);
}
