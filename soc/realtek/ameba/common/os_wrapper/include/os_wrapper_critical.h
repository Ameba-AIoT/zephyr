/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __OS_WRAPPER_CRITCAL_H__
#define __OS_WRAPPER_CRITCAL_H__

/**
 * Define CA32 SMP spin lock id
 */
typedef enum {
	RTOS_CRITICAL_DEFAULT = 0,
	RTOS_CRITICAL_SOC,
	RTOS_CRITICAL_AUDIO,
	RTOS_CRITICAL_WIFI,
	RTOS_CRITICAL_NETWORK,
	RTOS_CRITICAL_LWIP,
	RTOS_CRITICAL_BT,
	RTOS_CRITICAL_USB,
	RTOS_CRITICAL_WPAN,
	RTOS_CRITICAL_SEMA,
	RTOS_CRITICAL_LOG,
	RTOS_CRITICAL_MAX
} RTOS_CRITICAL_LIST;

/**
 * @brief  Check if in task interrupt
 * @retval 1: interrupt; 0: context
 */
int rtos_critical_is_in_interrupt(void);

/**
 * @brief  Internally handles interrupt status (PRIMASK/CPSR) save
 */
void rtos_critical_enter(uint32_t component_id);

/**
 * @brief  Internally handles interrupt status(PRIMASK/CPSR) restore
 */
void rtos_critical_exit(uint32_t component_id);

/**
 * @brief  get task enter critical state
 * @retval >0: in critical state; 0: exit critical state
 */
uint32_t rtos_get_critical_state(void);
#endif
