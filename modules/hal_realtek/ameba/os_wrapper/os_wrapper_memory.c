/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "os_wrapper.h"
#include <zephyr/sys/math_extras.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(os_if_memory);

#if (K_HEAP_MEM_POOL_SIZE > 0)
extern struct k_heap _system_heap;
#define _SYSTEM_HEAP (&_system_heap.heap)
#endif

void rtos_mem_init(void)
{
	/* Zephyr initializes the system heap automatically. */
}

void rtos_mem_free(void *pbuf)
{
	if (pbuf == NULL) {
		return;
	}
	k_free(pbuf);
}

void *rtos_mem_malloc(uint32_t size)
{
#if (CONFIG_HEAP_MEM_POOL_SIZE > 0)
	return k_aligned_alloc(CACHE_LINE_SIZE, CACHE_LINE_ALIGNMENT(size));
#else
	LOG_ERR("%s <<< k_aligned_alloc not support. >>>", __func__);
	return NULL;
#endif
}

void *rtos_mem_zmalloc(uint32_t size)
{
	void *pbuf = NULL;

	pbuf = rtos_mem_malloc(size);
	if (pbuf != NULL) {
		memset(pbuf, 0, size);
	}

	return pbuf;
}

void *rtos_mem_calloc(uint32_t elementNum, uint32_t elementSize)
{
	size_t sz;

	if (size_mul_overflow(elementNum, elementSize, &sz)) {
		return NULL;
	}
	return rtos_mem_zmalloc((uint32_t)sz);
}

void *rtos_mem_realloc(void *pbuf, uint32_t size)
{
	struct k_heap **heap_ref;
	struct k_heap *heap;
	void *new_ptr;
	size_t old_size;

	if (size == 0) {
		rtos_mem_free(pbuf);
		return NULL;
	}
	if (pbuf == NULL) {
		return rtos_mem_malloc(size);
	}

	/* Emulate realloc as malloc + copy + free — sys_heap_aligned_realloc
	 * doesn't preserve the heap_ref header on relocation.  Read the heap
	 * pointer that z_alloc_helper stored just before pbuf; query the old
	 * usable size via pbuf so sys_heap_usable_size subtracts the header.
	 *
	 * NOTE: this assumes the Zephyr allocation layout established by
	 * z_alloc_helper (kernel/mempool.c).  See DEPEND for the compatible
	 * Zephyr version.
	 */
	heap_ref = (struct k_heap **)pbuf;
	heap = *(--heap_ref);
	old_size = sys_heap_usable_size(&heap->heap, pbuf);

	new_ptr = rtos_mem_malloc(size);
	if (new_ptr == NULL) {
		return NULL;
	}
	memcpy(new_ptr, pbuf, MIN(old_size, (size_t)size));
	rtos_mem_free(pbuf);
	return new_ptr;
}

uint32_t rtos_mem_get_free_heap_size(void)
{
	uint32_t size = 0;

#ifdef CONFIG_SYS_HEAP_RUNTIME_STATS
	struct sys_memory_stats stats;

	sys_heap_runtime_stats_get(_SYSTEM_HEAP, &stats);
	size = stats.free_bytes;
#else
	LOG_ERR("%s Not Support", __func__);
#endif
	return size;
}

uint32_t rtos_mem_get_minimum_ever_free_heap_size(void)
{
	uint32_t size = 0;

#ifdef CONFIG_SYS_HEAP_RUNTIME_STATS
	struct sys_memory_stats stats;

	sys_heap_runtime_stats_get(_SYSTEM_HEAP, &stats);
	size = K_HEAP_MEM_POOL_SIZE - stats.max_allocated_bytes;
#else
	LOG_ERR("%s Not Support", __func__);
#endif
	return size;
}

static void warn_if_non_dram_type(MALLOC_TYPES type, const char *func)
{
	if (type == TYPE_TCM || type == TYPE_SRAM) {
		LOG_WRN_ONCE("%s: TCM/SRAM requested but only DRAM heap available", func);
	}
}

void *rtos_heap_types_malloc(uint32_t size, MALLOC_TYPES type)
{
	warn_if_non_dram_type(type, __func__);
	return rtos_mem_malloc(size);
}

void *rtos_heap_types_zmalloc(uint32_t size, MALLOC_TYPES type)
{
	warn_if_non_dram_type(type, __func__);
	return rtos_mem_zmalloc(size);
}

void *rtos_heap_types_calloc(uint32_t elementNum, uint32_t elementSize, MALLOC_TYPES type)
{
	warn_if_non_dram_type(type, __func__);
	return rtos_mem_calloc(elementNum, elementSize);
}

void *rtos_heap_types_realloc(void *pbuf, uint32_t size, MALLOC_TYPES type)
{
	warn_if_non_dram_type(type, __func__);
	return rtos_mem_realloc(pbuf, size);
}
