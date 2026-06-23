/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include "test_buffers.h"

#if CONFIG_NOCACHE_MEMORY
__aligned(32) char tx_data[TEST_BUF_SIZE] __used __attribute__((__section__(".nocache_load"))) =
	"It is harder to be kind than to be wise........";
__aligned(32) char rx_data[TEST_BUF_SIZE] __used __attribute__((__section__(".nocache.dma")));
#else
__aligned(32) char tx_data[TEST_BUF_SIZE] = "It is harder to be kind than to be wise........";
__aligned(32) char rx_data[TEST_BUF_SIZE] = { 0 };
#endif
