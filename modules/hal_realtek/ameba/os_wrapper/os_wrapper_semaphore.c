/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "os_wrapper.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(os_if_sema);

int rtos_sema_create(rtos_sema_t *pp_handle, uint32_t init_count, uint32_t max_count)
{
	if (pp_handle == NULL) {
		return RTK_FAIL;
	}

#if (CONFIG_HEAP_MEM_POOL_SIZE > 0)
	*pp_handle = k_malloc(sizeof(struct k_sem));
	if (*pp_handle == NULL) {
		return RTK_FAIL;
	}
#else
	LOG_ERR("%s <<< k_malloc not support. >>>", __func__);
	return RTK_FAIL;
#endif

	if (k_sem_init(*pp_handle, init_count, max_count) == 0) {
		return RTK_SUCCESS;
	} else {
		k_free(*pp_handle);
		*pp_handle = NULL;
		return RTK_FAIL;
	}
}

int rtos_sema_create_binary(rtos_sema_t *pp_handle)
{
	return rtos_sema_create(pp_handle, 0, 1);
}

int rtos_sema_delete(rtos_sema_t p_handle)
{
	if (p_handle == NULL) {
		return RTK_FAIL;
	}

	/* k_sem_reset wakes waiters with -EAGAIN; they don't touch sem after. */
	k_sem_reset((struct k_sem *)p_handle);
	k_free(p_handle);
	return RTK_SUCCESS;
}

int rtos_sema_take(rtos_sema_t p_handle, uint32_t wait_ms)
{
	if (p_handle == NULL) {
		return RTK_FAIL;
	}

	if (rtos_critical_is_in_interrupt()) {
		/* ISR / critical: non-blocking only. */
		if (k_sem_take(p_handle, K_NO_WAIT) == 0) {
			return RTK_SUCCESS;
		} else {
			return RTK_FAIL;
		}
	} else {
		k_timeout_t ticks;

		if (rtos_get_critical_state() != 0) {
			ticks = K_NO_WAIT;
		} else if (wait_ms == 0xFFFFFFFFUL) {
			ticks = K_FOREVER;
		} else {
			ticks = K_MSEC(wait_ms);
		}

		if (k_sem_take(p_handle, ticks) == 0) {
			return RTK_SUCCESS;
		} else {
			return RTK_FAIL;
		}
	}
}

int rtos_sema_give(rtos_sema_t p_handle)
{
	if (p_handle == NULL) {
		return RTK_FAIL;
	}

	k_sem_give(p_handle); /* ISR-safe, clamps to limit internally */
	return RTK_SUCCESS;
}

uint32_t rtos_sema_get_count(rtos_sema_t p_handle)
{
	if (p_handle == NULL) {
		return (uint32_t)RTK_FAIL;
	}

	return k_sem_count_get(p_handle);
}

int rtos_sema_create_static(rtos_sema_t *pp_handle, uint32_t init_count, uint32_t max_count)
{
	return rtos_sema_create(pp_handle, init_count, max_count);
}

int rtos_sema_create_binary_static(rtos_sema_t *pp_handle)
{
	return rtos_sema_create_binary(pp_handle);
}

int rtos_sema_delete_static(rtos_sema_t p_handle)
{
	return rtos_sema_delete(p_handle);
}
