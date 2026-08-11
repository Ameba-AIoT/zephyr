/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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
	LOG_ERR("%s <<< k_malloc not support. >>>", __func__);
	return RTK_FAIL;
#endif

	if (k_mutex_init(*pp_handle) == 0) {
		return RTK_SUCCESS;
	} else {
		k_free(*pp_handle);
		*pp_handle = NULL;
		return RTK_FAIL;
	}
}

int rtos_mutex_delete(rtos_mutex_t p_handle)
{
	if (p_handle == NULL) {
		return RTK_FAIL;
	}

	struct k_mutex *m = (struct k_mutex *)p_handle;

	if (m->owner != NULL) {
		LOG_ERR("%s: mutex still held by %p", __func__, (void *)m->owner);
		return RTK_FAIL;
	}
	k_free(p_handle);
	return RTK_SUCCESS;
}

int rtos_mutex_take(rtos_mutex_t p_handle, uint32_t wait_ms)
{
	if (p_handle == NULL) {
		return RTK_FAIL;
	}

	if (rtos_critical_is_in_interrupt()) {
		LOG_ERR("%s: called from ISR", __func__);
		return RTK_FAIL;
	}

	k_timeout_t ticks;

	if (rtos_sched_get_state() == RTOS_SCHED_NOT_STARTED) {
		return RTK_FAIL;
	} else if (rtos_get_critical_state() != 0) {
		ticks = K_NO_WAIT;
	} else if (wait_ms == 0xFFFFFFFFUL) {
		ticks = K_FOREVER;
	} else {
		ticks = K_MSEC(wait_ms);
	}

	if (k_mutex_lock(p_handle, ticks) == 0) {
		return RTK_SUCCESS;
	} else {
		return RTK_FAIL;
	}
}

int rtos_mutex_give(rtos_mutex_t p_handle)
{
	if (p_handle == NULL) {
		return RTK_FAIL;
	}

	if (rtos_critical_is_in_interrupt()) {
		LOG_ERR("%s: called from ISR", __func__);
		return RTK_FAIL;
	}

	if (k_mutex_unlock(p_handle) == 0) {
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

/* k_mutex is re-entrant; recursive variants map to regular mutex operations. */
int rtos_mutex_recursive_create(rtos_mutex_t *pp_handle)
{
	return rtos_mutex_create(pp_handle);
}

int rtos_mutex_recursive_delete(rtos_mutex_t p_handle)
{
	return rtos_mutex_delete(p_handle);
}

int rtos_mutex_recursive_take(rtos_mutex_t p_handle, uint32_t wait_ms)
{
	return rtos_mutex_take(p_handle, wait_ms);
}

int rtos_mutex_recursive_give(rtos_mutex_t p_handle)
{
	return rtos_mutex_give(p_handle);
}

int rtos_mutex_recursive_create_static(rtos_mutex_t *pp_handle)
{
	return rtos_mutex_create(pp_handle);
}

int rtos_mutex_recursive_delete_static(rtos_mutex_t p_handle)
{
	return rtos_mutex_delete(p_handle);
}
