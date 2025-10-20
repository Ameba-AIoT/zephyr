/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "os_wrapper.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(os_if_task);
extern struct k_thread *z_swap_next_thread(void);

int rtos_sched_start(void)
{
	LOG_WRN("%s Not Support\n", __func__);
	return RTK_SUCCESS;
}

int rtos_sched_stop(void)
{
	LOG_ERR("%s Not Support\n", __func__);
	return RTK_SUCCESS;
}

int rtos_sched_suspend(void)
{
	k_sched_lock();
	return RTK_SUCCESS;
}

int rtos_sched_resume(void)
{
	k_sched_unlock();
	return RTK_SUCCESS;
}

int rtos_sched_get_state(void)
{
	int status = RTK_FAIL;

	if (z_swap_next_thread() == NULL) {
		status = RTOS_SCHED_NOT_STARTED;
	} else if (_current->base.sched_locked != 0U) {
		status = RTOS_SCHED_SUSPENDED;
	} else {
		status = RTOS_SCHED_RUNNING;
	}

	return status;
}

int rtos_task_create(rtos_task_t *pp_handle, const char *p_name, void (*p_routine)(void *),
		     void *p_param, uint16_t stack_size_in_byte, uint16_t priority)
{
	k_tid_t p_thread;
	k_thread_stack_t *p_stack;
	/* higher value, lower priority. see
	 * https://docs.zephyrproject.org/latest/kernel/services/threads/index.html#thread-priorities
	 */
	int switch_priority = RTOS_TASK_MAX_PRIORITIES - priority;

#if (CONFIG_HEAP_MEM_POOL_SIZE > 0)
	p_thread = k_malloc(sizeof(struct k_thread));
	if (p_thread == NULL) {
		return RTK_FAIL;
	}

	/* use k_malloc to avoid k_thread_stack_alloc is empty */
	p_stack = (k_thread_stack_t *)k_malloc(K_KERNEL_STACK_LEN(stack_size_in_byte));
	if (p_stack == NULL) {
		k_free(p_thread);
		LOG_ERR("Alloc stack fail for %s\n", p_name);
		return RTK_FAIL;
	}
#else
	LOG_ERR("%s <<< k_malloc not support. >>>\n", __func__);
	return RTK_FAIL;
#endif

	k_thread_create(p_thread, p_stack, stack_size_in_byte, (k_thread_entry_t)p_routine, p_param,
			NULL, NULL, switch_priority, 0, K_FOREVER);
	k_thread_name_set(p_thread, p_name);

	p_thread->custom_data = p_thread;

	if (pp_handle) {
		*pp_handle = p_thread;
	}

	k_thread_start(p_thread);
	return RTK_SUCCESS;
}

void thread_abort_hook(struct k_thread *p_free)
{
	k_tid_t p_curr = k_thread_custom_data_get();

	if (p_curr == p_free) {
		k_free((void *)p_curr->stack_info.start);
		k_free(p_curr);
	}
}

int rtos_task_delete(rtos_task_t p_handle)
{
	k_tid_t p_free = (k_tid_t)p_handle;
	k_tid_t p_curr = k_current_get();

	if ((p_free == NULL) || (p_curr == p_free)) {
		/* TODO: wait wifi use dynamic task create   */
		/* k_thread_custom_data_set((void *)p_curr); */

		k_thread_abort(p_curr);
		CODE_UNREACHABLE;
	} else {
		k_thread_abort(p_free);

		k_free((void *)p_free->stack_info.start);
		k_free(p_free);
	}

	return RTK_SUCCESS;
}

int rtos_task_suspend(rtos_task_t p_handle)
{
	k_thread_suspend((k_tid_t)p_handle);
	return RTK_SUCCESS;
}

int rtos_task_resume(rtos_task_t p_handle)
{
	k_thread_resume((k_tid_t)p_handle);
	return RTK_SUCCESS;
}

int rtos_task_yield(void)
{
	k_yield();
	return RTK_SUCCESS;
}

rtos_task_t rtos_task_handle_get(void)
{
	return (rtos_task_t)k_current_get();
}

int rtos_task_priority_set(rtos_task_t p_handle, uint16_t priority)
{
	int switch_priority = RTOS_TASK_MAX_PRIORITIES - priority;

	k_thread_priority_set(p_handle, switch_priority);
	return RTK_SUCCESS;
}

uint32_t rtos_task_priority_get(rtos_task_t p_handle)
{
	int priority = k_thread_priority_get(p_handle);

	return RTOS_TASK_MAX_PRIORITIES - priority;
}

void rtos_create_secure_context(uint32_t size)
{
	(void)size;
	LOG_ERR("%s Not Support\n", __func__);
}
