/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private to os_wrapper/.  See os_wrapper_deferred.c for design notes.
 */

#ifndef ZEPHYR_MODULES_HAL_REALTEK_AMEBA_OS_WRAPPER_DEFERRED_H_
#define ZEPHYR_MODULES_HAL_REALTEK_AMEBA_OS_WRAPPER_DEFERRED_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*deferred_fn_t)(void *a1, void *a2, void *a3);

/* Submit @fn(a1, a2, a3) to run once on sysworkq.  Returns 0 on success,
 * -EINVAL if fn is NULL, -ENOMEM if the slot pool is exhausted, or
 * -EAGAIN if sysworkq rejected the submit.
 */
int deferred_submit(deferred_fn_t fn, void *a1, void *a2, void *a3);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_MODULES_HAL_REALTEK_AMEBA_OS_WRAPPER_DEFERRED_H_ */
