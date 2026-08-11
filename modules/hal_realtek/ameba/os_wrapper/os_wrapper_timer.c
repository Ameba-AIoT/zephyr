/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "os_wrapper.h"
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

/* Defer free to sysworkq: k_timer_stop only cancels future expiries; a
 * currently-running expiry_fn (in ISR) may still touch the wrapper.
 * Heap-allocated ctx (contains k_work) is intentionally not freed here —
 * Zephyr writes work->flags after the handler returns.
 */
struct rtos_timer_free_ctx {
	struct k_work work;
	void *timer;
	atomic_t in_use;
};

/* Static slot used when k_malloc fails.  One is enough as sysworkq drains
 * quickly; if both heap and slot are unavailable we fall back to sync free.
 */
static struct rtos_timer_free_ctx fallback_ctx;

static void deferred_timer_free_handler(struct k_work *work)
{
	struct rtos_timer_free_ctx *ctx = CONTAINER_OF(work, struct rtos_timer_free_ctx, work);

	k_free(ctx->timer);
	if (ctx == &fallback_ctx) {
		atomic_set(&ctx->in_use, 0);
	}
}

int rtos_timer_delete(rtos_timer_t p_handle, uint32_t wait_ms)
{
	struct rtos_timer_free_ctx *ctx;

	ARG_UNUSED(wait_ms);
	if (p_handle == NULL) {
		return RTK_FAIL;
	}

	k_timer_stop((struct k_timer *)p_handle);

	ctx = k_malloc(sizeof(*ctx));
	if (ctx == NULL) {
		if (atomic_cas(&fallback_ctx.in_use, 0, 1)) {
			ctx = &fallback_ctx;
		} else {
			/* Heap full + fallback busy: sync free (rare UAF risk). */
			LOG_WRN("%s: OOM and fallback busy, freeing synchronously", __func__);
			k_free(p_handle);
			return RTK_SUCCESS;
		}
	}
	ctx->timer = p_handle;
	k_work_init(&ctx->work, deferred_timer_free_handler);
	if (k_work_submit(&ctx->work) < 0) {
		/* Sysworkq full: release fallback slot (if used) and free sync. */
		if (ctx == &fallback_ctx) {
			atomic_set(&ctx->in_use, 0);
		} else {
			k_free(ctx);
		}
		LOG_WRN("%s: k_work_submit failed, freeing synchronously", __func__);
		k_free(p_handle);
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

/* pw (contains k_work) intentionally leaks: Zephyr writes work->flags
 * after handler returns, so we cannot free it inside the handler.
 */
typedef struct {
	void (*func)(void *pv_p1, uint32_t ul_p2);
	void *param1;
	uint32_t param2;
} rtos_pended_call_t;

typedef struct {
	struct k_work work;
	rtos_pended_call_t *call;
} rtos_pended_work_t;

static void rtos_pended_work_handler(struct k_work *work)
{
	rtos_pended_work_t *pw = CONTAINER_OF(work, rtos_pended_work_t, work);
	rtos_pended_call_t *call = pw->call;

	if (call && call->func) {
		call->func(call->param1, call->param2);
	}
	k_free(call);
}

int rtos_timer_pend_function_call(void (*p_func)(void *, uint32_t), void *pv_p1, uint32_t ul_p2,
				  uint32_t wait_ms)
{
	rtos_pended_call_t *call;
	rtos_pended_work_t *pw;

	ARG_UNUSED(wait_ms);
	if (p_func == NULL) {
		return RTK_FAIL;
	}

	if (rtos_critical_is_in_interrupt()) {
		LOG_ERR("%s: called from ISR", __func__);
		return RTK_FAIL;
	}

#if (CONFIG_HEAP_MEM_POOL_SIZE > 0)
	call = k_malloc(sizeof(rtos_pended_call_t));
	if (!call) {
		return RTK_FAIL;
	}
	pw = k_malloc(sizeof(rtos_pended_work_t));
	if (!pw) {
		k_free(call);
		return RTK_FAIL;
	}
#else
	LOG_ERR("%s <<< k_malloc not support. >>>", __func__);
	return RTK_FAIL;
#endif

	call->func = p_func;
	call->param1 = pv_p1;
	call->param2 = ul_p2;

	pw->call = call;
	k_work_init(&pw->work, rtos_pended_work_handler);

	int ret = k_work_submit(&pw->work);

	if (ret < 0) {
		k_free(pw);
		k_free(call);
		return RTK_FAIL;
	}
	return RTK_SUCCESS;
}
