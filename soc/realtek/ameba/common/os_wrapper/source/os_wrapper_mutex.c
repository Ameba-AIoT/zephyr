/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "os_wrapper.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(os_if_mutex);

int rtos_mutex_create(rtos_mutex_t *pp_handle)
{
	if (pp_handle == NULL) {
		return RTK_FAIL;
	}

#if (CONFIG_HEAP_MEM_POOL_SIZE > 0)
	*pp_handle = k_malloc(sizeof(struct k_mutex));
	if (*pp_handle == NULL) {
		return RTK_FAIL;
	}
#else
	LOG_ERR("%s <<< k_malloc not support. >>>\n", __func__);
	return RTK_FAIL;
#endif

	if (0 == k_mutex_init(*pp_handle)) {
		return RTK_SUCCESS;
	} else {
		return RTK_FAIL;
	}
}

/**
 * @brief  For FreeRTOS, map to vSemaphoreDelete
 * @note   Do not delete mutex if held by a task
 * @param  p_handle:
 * @retval
 */
int rtos_mutex_delete(rtos_mutex_t p_handle)
{
	if (p_handle == NULL) {
		return RTK_FAIL;
	}

	if (0 == k_mutex_unlock(p_handle)) {
		LOG_ERR("%s <<< The mutex has not been released, but the mutex has been deleted. "
			">>>\n",
			__func__);
		k_free(p_handle);
		return RTK_FAIL;
	}

	k_free(p_handle);
	return RTK_SUCCESS;
}

/**
 * @brief  For FreeRTOS, map to xSemaphoreTake / xSemaphoreTakeFromISR
 *         The API internally determines whether it is in the interrupt state and calls the
 * corresponding RTOS interface.
 * @param  p_handle:
 * @param  wait_ms:
 * @retval
 */
int rtos_mutex_take(rtos_mutex_t p_handle, uint32_t wait_ms)
{
	int status;
	k_timeout_t wait_ticks;

	if (wait_ms == 0xFFFFFFFFUL) {
		wait_ticks = K_FOREVER;
	} else {
		wait_ticks = K_MSEC(wait_ms);
	}

	status = k_mutex_lock(p_handle, wait_ticks);
	if (status == 0) {
		return RTK_SUCCESS;
	} else {
		return RTK_FAIL;
	}
}

/**
 * @brief  For FreeRTOS, map to xSemaphoreGive / xSemaphoreGiveFromISR
 *         The API internally determines whether it is in the interrupt state and calls the
 * corresponding RTOS interface.
 * @param  p_handle:
 * @param  wait_ms:
 * @retval
 */
int rtos_mutex_give(rtos_mutex_t p_handle)
{
	int status;

	status = k_mutex_unlock(p_handle);
	if (status == 0) {
		return RTK_SUCCESS;
	} else {
		return RTK_FAIL;
	}
}

int rtos_mutex_create_static(rtos_mutex_t *pp_handle)
{
	return rtos_mutex_create(pp_handle);
}

int rtos_mutex_delete_static(rtos_mutex_t p_handle)
{
	return rtos_mutex_delete(p_handle);
}

int rtos_mutex_recursive_create(rtos_mutex_t *pp_handle)
{
	ARG_UNUSED(pp_handle);
	LOG_ERR("%s Not Support\n", __func__);
	return RTK_FAIL;
}

int rtos_mutex_recursive_delete(rtos_mutex_t p_handle)
{
	ARG_UNUSED(p_handle);
	LOG_ERR("%s Not Support\n", __func__);
	return RTK_FAIL;
}

int rtos_mutex_recursive_take(rtos_mutex_t p_handle, uint32_t wait_ms)
{
	ARG_UNUSED(p_handle);
	ARG_UNUSED(wait_ms);
	LOG_ERR("%s Not Support\n", __func__);
	return RTK_FAIL;
}

int rtos_mutex_recursive_give(rtos_mutex_t p_handle)
{
	ARG_UNUSED(p_handle);
	LOG_ERR("%s Not Support\n", __func__);
	return RTK_FAIL;
}

int rtos_mutex_recursive_create_static(rtos_mutex_t *pp_handle)
{
	ARG_UNUSED(pp_handle);
	LOG_ERR("%s Not Support\n", __func__);
	return RTK_FAIL;
}

int rtos_mutex_recursive_delete_static(rtos_mutex_t p_handle)
{
	ARG_UNUSED(p_handle);
	LOG_ERR("%s Not Support\n", __func__);
	return RTK_FAIL;
}
