/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Submit a piece of work to sysworkq that calls a user function with up to
 * three opaque pointer arguments.
 *
 * The naive pattern - k_malloc() a struct that embeds k_work, submit, k_free
 * in the handler - is unsafe: kernel/work.c writes work->flags AFTER the
 * handler returns.  Use a static slot pool instead; a slot is reusable only
 * when in_use==0 AND k_work_busy_get()==0 (busy_get reads under the same
 * spinlock kernel/work.c uses, so a 0 result guarantees the post-handler
 * flag write is done).
 */

#include "os_wrapper_deferred.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(os_if_deferred);

#define POOL_SIZE CONFIG_REALTEK_AMEBA_OS_WRAPPER_DEFERRED_POOL_SIZE

struct deferred_slot {
	struct k_work work;
	deferred_fn_t fn;
	void *a1;
	void *a2;
	void *a3;
	atomic_t in_use;
};

static struct deferred_slot pool[POOL_SIZE];
static struct k_spinlock pool_lock;
static atomic_t exhaustion_count;

static void deferred_slot_handler(struct k_work *work)
{
	struct deferred_slot *slot = CONTAINER_OF(work, struct deferred_slot, work);
	deferred_fn_t fn = slot->fn;
	void *a1 = slot->a1;
	void *a2 = slot->a2;
	void *a3 = slot->a3;

	/* Release the slot; k_work_busy_get() still guards reuse until the
	 * workqueue clears K_WORK_RUNNING_BIT after we return.
	 */
	atomic_set(&slot->in_use, 0);

	if (fn != NULL) {
		fn(a1, a2, a3);
	}
}

#define SLOT_ALLOC_RETRIES 4   /* total up to 3 ms of retry from thread ctx */

int deferred_submit(deferred_fn_t fn, void *a1, void *a2, void *a3)
{
	struct deferred_slot *slot = NULL;
	k_spinlock_key_t key;
	int retry;
	int i;

	if (fn == NULL) {
		return -EINVAL;
	}

	/* Retry with a short sleep to let sysworkq (coop -1) drain pending
	 * handlers and release slots.  ISR context cannot sleep so we make
	 * a single attempt there.
	 */
	for (retry = 0; retry < SLOT_ALLOC_RETRIES; retry++) {
		if (retry > 0) {
			if (k_is_in_isr()) {
				break;
			}
			k_msleep(1);
		}

		key = k_spin_lock(&pool_lock);
		for (i = 0; i < POOL_SIZE; i++) {
			if (!atomic_get(&pool[i].in_use) &&
			    k_work_busy_get(&pool[i].work) == 0) {
				atomic_set(&pool[i].in_use, 1);
				slot = &pool[i];
				break;
			}
		}
		k_spin_unlock(&pool_lock, key);

		if (slot != NULL) {
			break;
		}
	}

	if (slot == NULL) {
		atomic_inc(&exhaustion_count);
		LOG_ERR("deferred pool exhausted (total=%ld); consider raising "
			"CONFIG_REALTEK_AMEBA_OS_WRAPPER_DEFERRED_POOL_SIZE (=%d)",
			atomic_get(&exhaustion_count), POOL_SIZE);
		return -ENOMEM;
	}

	slot->fn = fn;
	slot->a1 = a1;
	slot->a2 = a2;
	slot->a3 = a3;
	k_work_init(&slot->work, deferred_slot_handler);

	if (k_work_submit(&slot->work) < 0) {
		/* Never queued, no kernel reference; release the slot. */
		atomic_set(&slot->in_use, 0);
		LOG_WRN("k_work_submit rejected work item");
		return -EAGAIN;
	}
	return 0;
}
