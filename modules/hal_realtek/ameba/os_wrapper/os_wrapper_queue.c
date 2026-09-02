/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "os_wrapper.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(os_if_queue);

int rtos_queue_create(rtos_queue_t *pp_handle, uint32_t msg_num, uint32_t msg_size)
{
	struct k_msgq *p_queue;

	if (pp_handle == NULL) {
		return RTK_FAIL;
	}

#if (CONFIG_HEAP_MEM_POOL_SIZE > 0)
	p_queue = (struct k_msgq *)k_malloc(sizeof(struct k_msgq));
	if (p_queue == NULL) {
		return RTK_FAIL;
	}
#else
	LOG_ERR("%s <<< k_malloc not support. >>>", __func__);
	return RTK_FAIL;
#endif

	if (k_msgq_alloc_init(p_queue, msg_size, msg_num) != 0) {
		k_free(p_queue);
		*pp_handle = NULL;
		return RTK_FAIL;
	}

	*pp_handle = p_queue;
	return RTK_SUCCESS;
}

/* k_msgq_cleanup returns -EBUSY while any thread is blocked on the queue;
 * k_msgq_purge wakes them but they must be scheduled out before cleanup
 * sees an empty wait_q.  Retry with a short sleep; if still EBUSY the
 * caller has a bug and we leak rather than free memory in use.
 */
#define QUEUE_DELETE_MAX_RETRIES 8

int rtos_queue_delete(rtos_queue_t p_handle)
{
	int ret;
	int i;

	if (p_handle == NULL) {
		return RTK_FAIL;
	}

	if (rtos_queue_message_waiting(p_handle) != 0) {
		LOG_WRN("%s: deleting non-empty queue", __func__);
	}

	for (i = 0; i < QUEUE_DELETE_MAX_RETRIES; i++) {
		k_msgq_purge(p_handle);
		ret = k_msgq_cleanup(p_handle);
		if (ret == 0) {
			k_free(p_handle);
			return RTK_SUCCESS;
		}
		if (ret != -EBUSY) {
			break;
		}
		k_msleep(1);
	}

	LOG_ERR("%s: cleanup failed (%d) after %d retries, leaking to avoid UAF "
		"— caller likely has a thread still blocking on this queue",
		__func__, ret, QUEUE_DELETE_MAX_RETRIES);
	return RTK_FAIL;
}

uint32_t rtos_queue_message_waiting(rtos_queue_t p_handle)
{
	if (p_handle == NULL) {
		return (uint32_t)RTK_FAIL;
	}

	return k_msgq_num_used_get(p_handle);
}

int rtos_queue_send(rtos_queue_t p_handle, void *p_msg, uint32_t wait_ms)
{
	if (p_handle == NULL) {
		return RTK_FAIL;
	}

	k_timeout_t ticks;

	if (rtos_critical_is_in_interrupt() || rtos_get_critical_state() != 0) {
		ticks = K_NO_WAIT;
	} else if (wait_ms == 0xFFFFFFFFUL) {
		ticks = K_FOREVER;
	} else {
		ticks = K_MSEC(wait_ms);
	}

	if (k_msgq_put(p_handle, p_msg, ticks) == 0) {
		return RTK_SUCCESS;
	} else {
		return RTK_FAIL;
	}
}

int rtos_queue_send_to_front(rtos_queue_t p_handle, void *p_msg, uint32_t wait_ms)
{
	if (p_handle == NULL) {
		return RTK_FAIL;
	}

	if (k_msgq_put_front(p_handle, p_msg) == 0) {
		return RTK_SUCCESS;
	}

	/* Queue full.  k_msgq_put_front is non-blocking; emulate blocking
	 * semantics by polling (matches the wait_ms contract for callers).
	 */
	if (rtos_critical_is_in_interrupt() || rtos_get_critical_state() != 0 ||
	    rtos_sched_get_state() != RTOS_SCHED_RUNNING || wait_ms == 0U) {
		return RTK_FAIL;
	}

	const uint32_t POLL_INTERVAL_MS = 1U;
	int64_t start = k_uptime_get();
	int64_t deadline = (wait_ms == 0xFFFFFFFFUL) ? INT64_MAX : (start + (int64_t)wait_ms);

	while (k_uptime_get() < deadline) {
		k_msleep(POLL_INTERVAL_MS);
		if (k_msgq_put_front(p_handle, p_msg) == 0) {
			return RTK_SUCCESS;
		}
	}
	return RTK_FAIL;
}

int rtos_queue_receive(rtos_queue_t p_handle, void *p_msg, uint32_t wait_ms)
{
	if (p_handle == NULL) {
		return RTK_FAIL;
	}

	k_timeout_t ticks;

	if (rtos_critical_is_in_interrupt() || rtos_get_critical_state() != 0) {
		ticks = K_NO_WAIT;
	} else if (wait_ms == 0xFFFFFFFFUL) {
		ticks = K_FOREVER;
	} else {
		ticks = K_MSEC(wait_ms);
	}

	if (k_msgq_get(p_handle, p_msg, ticks) == 0) {
		return RTK_SUCCESS;
	} else {
		return RTK_FAIL;
	}
}

int rtos_queue_peek(rtos_queue_t p_handle, void *p_msg, uint32_t wait_ms)
{
	if (p_handle == NULL || p_msg == NULL) {
		return RTK_FAIL;
	}

	if (k_msgq_peek(p_handle, p_msg) == 0) {
		return RTK_SUCCESS;
	}

	if (rtos_critical_is_in_interrupt() || rtos_get_critical_state() != 0 ||
	    rtos_sched_get_state() != RTOS_SCHED_RUNNING || wait_ms == 0U) {
		return RTK_FAIL;
	}

	/* k_msgq has no blocking peek — poll with 1 ms sleep. */
	const uint32_t POLL_INTERVAL_MS = 1U;
	int64_t start = k_uptime_get();
	int64_t deadline = (wait_ms == 0xFFFFFFFFUL) ? INT64_MAX : (start + (int64_t)wait_ms);

	while (k_uptime_get() < deadline) {
		k_msleep(POLL_INTERVAL_MS);
		if (k_msgq_peek(p_handle, p_msg) == 0) {
			return RTK_SUCCESS;
		}
	}
	return RTK_FAIL;
}
