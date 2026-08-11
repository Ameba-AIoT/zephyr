/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "os_wrapper.h"
#include <zephyr/kernel_structs.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(os_if_task);

/* Per-thread metadata stored in k_thread.custom_data. */
struct rtos_task_meta {
	void *stack_orig;
	void *tls[1]; /* slot 0 = lwIP */
};

/* Self-delete via sysworkq: freeing the thread's own stack/k_thread while
 * still running on them is unsafe, and thread_abort_hook runs under the
 * scheduler spinlock (can't call k_free there).  So we submit a work item,
 * abort self; the handler runs on sysworkq, joins the dead thread, then
 * frees everything.  ctx (containing k_work) intentionally leaks: Zephyr
 * writes work->flags after the handler returns.
 */
struct self_delete_ctx {
	struct k_work work;
	k_tid_t thread;
	void *stack_orig;
	struct rtos_task_meta *meta;
	atomic_t in_use;
};

/* Static fallback for OOM path — avoids leaking meta/stack when heap is full. */
static struct self_delete_ctx self_delete_fallback;

static void self_delete_handler(struct k_work *work)
{
	struct self_delete_ctx *ctx = CONTAINER_OF(work, struct self_delete_ctx, work);

	k_thread_join(ctx->thread, K_FOREVER);
	k_free(ctx->stack_orig);
	k_free(ctx->meta);
	k_free(ctx->thread);
	if (ctx == &self_delete_fallback) {
		atomic_set(&ctx->in_use, 0);
	}
}

int rtos_sched_start(void)
{
	LOG_WRN("%s Not Support", __func__);
	return RTK_SUCCESS;
}

int rtos_sched_stop(void)
{
	LOG_ERR("%s Not Support", __func__);
	return RTK_FAIL;
}

int rtos_sched_suspend(void)
{
	if (k_is_in_isr()) {
		return RTK_FAIL;
	}
	k_sched_lock();
	return RTK_SUCCESS;
}

int rtos_sched_resume(void)
{
	if (k_is_in_isr()) {
		return RTK_FAIL;
	}
	k_sched_unlock();
	return RTK_SUCCESS;
}

int rtos_sched_get_state(void)
{
	if (k_is_pre_kernel()) {
		return RTOS_SCHED_NOT_STARTED;
	}

	if (_current->base.sched_locked != 0U) {
		return RTOS_SCHED_SUSPENDED;
	}

	return RTOS_SCHED_RUNNING;
}

int rtos_task_create(rtos_task_t *pp_handle, const char *p_name, void (*p_routine)(void *),
		     void *p_param, size_t stack_size_in_byte, uint16_t priority)
{
	k_tid_t p_thread;
	k_thread_stack_t *p_stack;
	struct rtos_task_meta *meta;
	/* Zephyr priority: 0 = highest, K_IDLE_PRIO = CONFIG_NUM_PREEMPT_PRIORITIES
	 * (reserved for idle).  Usable preemptible range is [0, MAX-1], so map
	 * caller priority p ∈ [0, MAX-1] to Zephyr (MAX-1) - p, giving the same
	 * MAX slots without ever landing on K_IDLE_PRIO.
	 */
	int switch_priority = (RTOS_TASK_MAX_PRIORITIES - 1) - priority;

	if (p_routine == NULL) {
		LOG_ERR("%s: NULL routine", __func__);
		return RTK_FAIL;
	}
	if (priority >= RTOS_TASK_MAX_PRIORITIES) {
		LOG_ERR("%s: priority %u >= max %d", __func__, priority, RTOS_TASK_MAX_PRIORITIES);
		return RTK_FAIL;
	}

#if (CONFIG_HEAP_MEM_POOL_SIZE > 0)
	p_thread = k_malloc(sizeof(struct k_thread));
	if (p_thread == NULL) {
		return RTK_FAIL;
	}

	p_stack = (k_thread_stack_t *)k_malloc(K_KERNEL_STACK_LEN(stack_size_in_byte));
	if (p_stack == NULL) {
		k_free(p_thread);
		LOG_ERR("Alloc stack fail for %s", p_name);
		return RTK_FAIL;
	}

	meta = k_malloc(sizeof(struct rtos_task_meta));
	if (meta == NULL) {
		k_free(p_stack);
		k_free(p_thread);
		return RTK_FAIL;
	}
	meta->stack_orig = p_stack;
	meta->tls[0] = NULL;
#else
	LOG_ERR("%s <<< k_malloc not support. >>>", __func__);
	return RTK_FAIL;
#endif

	k_thread_create(p_thread, p_stack, stack_size_in_byte, (k_thread_entry_t)p_routine, p_param,
			NULL, NULL, switch_priority, 0, K_FOREVER);
	k_thread_name_set(p_thread, p_name);

	p_thread->custom_data = meta;

	if (pp_handle) {
		*pp_handle = p_thread;
	}

	k_thread_start(p_thread);
	return RTK_SUCCESS;
}

int rtos_task_delete(rtos_task_t p_handle)
{
	k_tid_t p_free = (k_tid_t)p_handle;
	k_tid_t p_curr = k_current_get();
	bool is_self_delete = (p_free == NULL) || (p_curr == p_free);

	if (is_self_delete) {
		struct rtos_task_meta *meta = (struct rtos_task_meta *)p_curr->custom_data;
		struct self_delete_ctx *ctx = k_malloc(sizeof(*ctx));

		if (ctx == NULL) {
			/* Heap full: try the static fallback slot. */
			if (atomic_cas(&self_delete_fallback.in_use, 0, 1)) {
				ctx = &self_delete_fallback;
			}
		}
		if (ctx && meta) {
			ctx->thread = p_curr;
			ctx->stack_orig = meta->stack_orig;
			ctx->meta = meta;
			p_curr->custom_data = NULL;
			k_work_init(&ctx->work, self_delete_handler);
			if (k_work_submit(&ctx->work) < 0) {
				/* Sysworkq full: nothing safe we can do here.
				 * Restore in_use so the fallback stays available,
				 * and log — resources will leak.
				 */
				LOG_ERR("%s: k_work_submit failed, resources leak", __func__);
				if (ctx == &self_delete_fallback) {
					atomic_set(&ctx->in_use, 0);
				}
			}
		} else if (ctx) {
			/* meta is NULL (thread not from rtos_task_create): free ctx. */
			if (ctx == &self_delete_fallback) {
				atomic_set(&ctx->in_use, 0);
			} else {
				k_free(ctx);
			}
		} else {
			LOG_ERR("%s: no ctx (heap full + fallback busy), meta leaks", __func__);
		}
		k_thread_abort(p_curr);
		CODE_UNREACHABLE;
	} else {
		struct rtos_task_meta *meta = (struct rtos_task_meta *)p_free->custom_data;

		p_free->custom_data = NULL;
		k_thread_abort(p_free);
		if (meta) {
			k_free(meta->stack_orig);
			k_free(meta);
		}
		k_free(p_free);
	}

	return RTK_SUCCESS;
}

int rtos_task_suspend(rtos_task_t p_handle)
{
	k_tid_t tid = (p_handle != NULL) ? (k_tid_t)p_handle : k_current_get();

	k_thread_suspend(tid);
	return RTK_SUCCESS;
}

int rtos_task_resume(rtos_task_t p_handle)
{
	if (p_handle == NULL) {
		return RTK_FAIL;
	}
	k_thread_resume((k_tid_t)p_handle);
	return RTK_SUCCESS;
}

int rtos_task_yield(void)
{
	if (k_is_in_isr()) {
		return RTK_FAIL;
	}
	k_yield();
	return RTK_SUCCESS;
}

rtos_task_t rtos_task_handle_get(void)
{
	return (rtos_task_t)k_current_get();
}

int rtos_task_priority_set(rtos_task_t p_handle, uint16_t priority)
{
	if (priority >= RTOS_TASK_MAX_PRIORITIES) {
		LOG_ERR("%s: priority %u >= max %d", __func__, priority, RTOS_TASK_MAX_PRIORITIES);
		return RTK_FAIL;
	}

	int switch_priority = (RTOS_TASK_MAX_PRIORITIES - 1) - priority;

	if (p_handle == NULL) {
		p_handle = k_current_get();
	}

	k_thread_priority_set(p_handle, switch_priority);
	return RTK_SUCCESS;
}

uint32_t rtos_task_priority_get(rtos_task_t p_handle)
{
	if (p_handle == NULL) {
		p_handle = k_current_get();
	}
	int priority = k_thread_priority_get(p_handle);
	int rtk_prio = (RTOS_TASK_MAX_PRIORITIES - 1) - priority;

	/* Clamp for coop/idle threads outside [0, MAX-1]. */
	if (rtk_prio < 0) {
		rtk_prio = 0;
	} else if (rtk_prio >= RTOS_TASK_MAX_PRIORITIES) {
		rtk_prio = RTOS_TASK_MAX_PRIORITIES - 1;
	}
	return (uint32_t)rtk_prio;
}

void thread_abort_hook(struct k_thread *p_free)
{
	ARG_UNUSED(p_free);
}

#if defined(CONFIG_THREAD_MONITOR) && defined(CONFIG_INIT_STACKS) &&                               \
	defined(CONFIG_THREAD_STACK_INFO)
static void thread_status_cb(const struct k_thread *thread, void *user_data)
{
	const char *name = k_thread_name_get((k_tid_t)thread);
	int prio = k_thread_priority_get((k_tid_t)thread);
	unsigned int state = thread->base.thread_state;
	size_t unused;

	k_thread_stack_space_get(thread, &unused);

	LOG_INF("  %-16s  prio=%-4d  state=0x%x  stack_free=%u/%u", name ? name : "(anon)", prio,
		state, (unsigned int)unused, (unsigned int)thread->stack_info.size);
}
#endif /* CONFIG_THREAD_MONITOR && CONFIG_INIT_STACKS && CONFIG_THREAD_STACK_INFO */

void rtos_task_out_current_status(void)
{
	LOG_INF("=== Thread Status Dump ===");
#if defined(CONFIG_THREAD_MONITOR) && defined(CONFIG_INIT_STACKS) &&                               \
	defined(CONFIG_THREAD_STACK_INFO)
	k_thread_foreach(thread_status_cb, NULL);
#else
	LOG_WRN("Thread status dump requires CONFIG_THREAD_MONITOR, CONFIG_INIT_STACKS, "
		"CONFIG_THREAD_STACK_INFO");
#endif
}

void rtos_create_secure_context(uint32_t size)
{
	/* Zephyr manages TrustZone contexts implicitly. */
	(void)size;
}

char *rtos_task_name_get(rtos_task_t p_handle)
{
#ifdef CONFIG_THREAD_NAME
	k_tid_t tid = (p_handle != NULL) ? (k_tid_t)p_handle : k_current_get();

	return (char *)k_thread_name_get(tid);
#else
	ARG_UNUSED(p_handle);
	return NULL;
#endif
}

rtos_task_t rtos_task_handle_get_idle(uint32_t coreID)
{
	if (coreID >= (uint32_t)CONFIG_MP_MAX_NUM_CPUS) {
		LOG_WRN("%s: coreID %u >= CONFIG_MP_MAX_NUM_CPUS %u", __func__, coreID,
			(uint32_t)CONFIG_MP_MAX_NUM_CPUS);
		return NULL;
	}
	return (rtos_task_t)_kernel.cpus[coreID].idle_thread;
}

void rtos_task_set_affinity(rtos_task_t p_handle, uint32_t coreID)
{
#ifdef CONFIG_SCHED_CPU_MASK
	if (p_handle == NULL) {
		LOG_ERR("%s: NULL handle", __func__);
		return;
	}
	int ret = k_thread_cpu_pin((k_tid_t)p_handle, (int)coreID);

	if (ret != 0) {
		LOG_ERR("%s: k_thread_cpu_pin(core %u) failed: %d", __func__, coreID, ret);
	}
#else
	ARG_UNUSED(p_handle);
	ARG_UNUSED(coreID);
	LOG_WRN("%s: CONFIG_SCHED_CPU_MASK not set — no-op", __func__);
#endif
}

void rtos_task_set_time_out_state(rtos_time_out_t *const p_rtos_time_out)
{
	if (p_rtos_time_out == NULL) {
		return;
	}
	int64_t now = k_uptime_get();

	p_rtos_time_out->time_on_entering = (uint32_t)(now & 0xFFFFFFFFU);
	p_rtos_time_out->over_flow_count = (uint32_t)((uint64_t)now >> 32);
}

int rtos_task_check_for_time_out(rtos_time_out_t *const p_rtos_time_out, uint32_t *p_ms_to_wait)
{
	if (p_rtos_time_out == NULL || p_ms_to_wait == NULL) {
		return TRUE;
	}

	if (*p_ms_to_wait == 0xFFFFFFFFUL) {
		return FALSE; /* infinite wait, never times out */
	}

	int64_t start = (int64_t)(((uint64_t)p_rtos_time_out->over_flow_count << 32) |
				  (uint64_t)p_rtos_time_out->time_on_entering);
	int64_t elapsed_ms = k_uptime_get() - start;
	int64_t timeout_ms = (int64_t)(*p_ms_to_wait);

	if (elapsed_ms >= timeout_ms) {
		*p_ms_to_wait = 0U;
		return TRUE;
	}
	*p_ms_to_wait = (uint32_t)(timeout_ms - elapsed_ms);
	return FALSE;
}

void rtos_task_set_thread_local_storage_pointer(rtos_task_t p_handle, uint16_t index, void *p_param)
{
	if (index != RTOS_LOCAL_STORAGE_LWIP_INDEX) {
		LOG_WRN("%s: only index %u supported on Zephyr (got %u)", __func__,
			RTOS_LOCAL_STORAGE_LWIP_INDEX, index);
		return;
	}
	k_tid_t tid = (p_handle != NULL) ? (k_tid_t)p_handle : k_current_get();
	struct rtos_task_meta *meta = (struct rtos_task_meta *)tid->custom_data;

	if (meta == NULL) {
		LOG_WRN("%s: no meta (thread not via rtos_task_create)", __func__);
		return;
	}
	meta->tls[index] = p_param;
}

void *rtos_task_get_thread_local_storage_pointer(rtos_task_t p_handle, uint16_t index)
{
	if (index != RTOS_LOCAL_STORAGE_LWIP_INDEX) {
		LOG_WRN("%s: only index %u supported on Zephyr (got %u)", __func__,
			RTOS_LOCAL_STORAGE_LWIP_INDEX, index);
		return NULL;
	}
	k_tid_t tid = (p_handle != NULL) ? (k_tid_t)p_handle : k_current_get();
	struct rtos_task_meta *meta = (struct rtos_task_meta *)tid->custom_data;

	if (meta == NULL) {
		return NULL;
	}
	return meta->tls[index];
}

void rtos_start_affinity_idle_task(void)
{
#ifdef CONFIG_SCHED_CPU_MASK
	for (unsigned int cpu = 0; cpu < (unsigned int)CONFIG_MP_MAX_NUM_CPUS; cpu++) {
		rtos_task_t idle = rtos_task_handle_get_idle(cpu);

		if (idle != NULL) {
			rtos_task_set_affinity(idle, cpu);
		}
	}
#endif
}
