/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "os_wrapper.h"
#include "os_wrapper_deferred.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(os_if_timer);

typedef struct {
	struct k_timer z_timer;
	uint32_t timer_id;
	uint32_t interval_ms;
	uint8_t reload;
	uint8_t started;
	void (*user_cb)(void *p);
} k_timer_wrapper_t;

/* Clear 'started' on one-shot expiry so is_timer_active() returns FALSE. */
static void timer_expiry_wrapper(struct k_timer *z_timer)
{
	k_timer_wrapper_t *w = CONTAINER_OF(z_timer, k_timer_wrapper_t, z_timer);

	if (!w->reload) {
		w->started = 0U;
	}
	if (w->user_cb) {
		w->user_cb(w);
	}
}

int rtos_timer_create(rtos_timer_t *pp_handle, const char *p_timer_name, uint32_t timer_id,
		      uint32_t interval_ms, uint8_t reload, void (*p_timer_callback)(void *))
{
	k_timer_wrapper_t *p_timer;

	ARG_UNUSED(p_timer_name);
	if (pp_handle == NULL || p_timer_callback == NULL) {
		return RTK_FAIL;
	}

#if (CONFIG_HEAP_MEM_POOL_SIZE > 0)
	p_timer = k_malloc(sizeof(k_timer_wrapper_t));
	if (p_timer == NULL) {
		return RTK_FAIL;
	}
#else
	LOG_ERR("%s <<< k_malloc not support. >>>", __func__);
	return RTK_FAIL;
#endif

	p_timer->user_cb = p_timer_callback;
	k_timer_init(&p_timer->z_timer, timer_expiry_wrapper, NULL);
	p_timer->timer_id = timer_id;
	p_timer->interval_ms = interval_ms;
	p_timer->reload = reload;
	p_timer->started = 0U;

	*pp_handle = p_timer;
	return RTK_SUCCESS;
}

/* k_timer_stop cancels future expiries but not an in-flight callback;
 * defer the free so it runs after any callback has unwound.
 */
static void finalize_timer_delete(void *timer, void *unused1, void *unused2)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	k_free(timer);
}

int rtos_timer_delete(rtos_timer_t p_handle, uint32_t wait_ms)
{
	ARG_UNUSED(wait_ms);
	if (p_handle == NULL) {
		return RTK_FAIL;
	}

	k_timer_stop((struct k_timer *)p_handle);

	if (deferred_submit(finalize_timer_delete, p_handle, NULL, NULL) != 0) {
		/* Cannot sync-free: k_timer_stop() does not wait for an in-flight
		 * expiry callback, so freeing here would UAF.  Leak instead; the
		 * pool should be sized so this is unreachable under normal load.
		 */
		LOG_ERR("%s: deferred_submit failed, timer leaked", __func__);
	}
	return RTK_SUCCESS;
}

int rtos_timer_create_static(rtos_timer_t *pp_handle, const char *p_timer_name, uint32_t timer_id,
			     uint32_t interval_ms, uint8_t reload, void (*p_timer_callback)(void *))
{
	return rtos_timer_create(pp_handle, p_timer_name, timer_id, interval_ms, reload,
				 p_timer_callback);
}

int rtos_timer_delete_static(rtos_timer_t p_handle, uint32_t wait_ms)
{
	return rtos_timer_delete(p_handle, wait_ms);
}

int rtos_timer_start(rtos_timer_t p_handle, uint32_t wait_ms)
{
	k_timer_wrapper_t *p_timer = p_handle;

	ARG_UNUSED(wait_ms);
	if (p_handle == NULL) {
		return RTK_FAIL;
	}

	if (p_timer->reload) {
		k_timer_start((struct k_timer *)p_timer, K_MSEC(p_timer->interval_ms),
			      K_MSEC(p_timer->interval_ms));
	} else {
		k_timer_start((struct k_timer *)p_timer, K_MSEC(p_timer->interval_ms), K_NO_WAIT);
	}
	p_timer->started = 1U;

	return RTK_SUCCESS;
}

int rtos_timer_stop(rtos_timer_t p_handle, uint32_t wait_ms)
{
	k_timer_wrapper_t *p_timer = p_handle;

	ARG_UNUSED(wait_ms);
	if (p_handle == NULL) {
		return RTK_FAIL;
	}

	k_timer_stop((struct k_timer *)p_timer);
	p_timer->started = 0U;
	return RTK_SUCCESS;
}

int rtos_timer_change_period(rtos_timer_t p_handle, uint32_t interval_ms, uint32_t wait_ms)
{
	k_timer_wrapper_t *p_timer = p_handle;

	ARG_UNUSED(wait_ms);
	if (p_handle == NULL) {
		return RTK_FAIL;
	}

	p_timer->interval_ms = interval_ms;

	/* Always (re)arm, even on a dormant timer. */
	if (p_timer->reload) {
		k_timer_start((struct k_timer *)p_timer, K_MSEC(interval_ms), K_MSEC(interval_ms));
	} else {
		k_timer_start((struct k_timer *)p_timer, K_MSEC(interval_ms), K_NO_WAIT);
	}
	p_timer->started = 1U;
	return RTK_SUCCESS;
}

uint32_t rtos_timer_is_timer_active(rtos_timer_t p_handle)
{
	k_timer_wrapper_t *p_timer = p_handle;

	if (p_handle == NULL) {
		return 0;
	}

	return p_timer->started ? TRUE : FALSE;
}

uint32_t rtos_timer_get_id(rtos_timer_t p_handle)
{
	k_timer_wrapper_t *p_timer = p_handle;

	if (p_handle == NULL) {
		return 0;
	}

	return p_timer->timer_id;
}

_WEAK void init_timer_wrapper(void)
{
	LOG_ERR("%s Not Support", __func__);
}

/* Trampoline: three-void* slot -> the caller's (void *, uint32_t) signature. */
static void invoke_pended_call(void *fn, void *p1, void *p2)
{
	void (*func)(void *, uint32_t) = (void (*)(void *, uint32_t))fn;

	if (func != NULL) {
		func(p1, (uint32_t)(uintptr_t)p2);
	}
}

int rtos_timer_pend_function_call(void (*p_func)(void *, uint32_t), void *pv_p1, uint32_t ul_p2,
				  uint32_t wait_ms)
{
	ARG_UNUSED(wait_ms);
	if (p_func == NULL) {
		return RTK_FAIL;
	}

	if (rtos_critical_is_in_interrupt()) {
		LOG_ERR("%s: called from ISR", __func__);
		return RTK_FAIL;
	}

	if (deferred_submit(invoke_pended_call, (void *)p_func, pv_p1,
			    (void *)(uintptr_t)ul_p2) != 0) {
		return RTK_FAIL;
	}
	return RTK_SUCCESS;
}
