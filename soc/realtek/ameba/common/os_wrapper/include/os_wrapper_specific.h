/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __OS_WRAPPER_SPECIFIC_H__
#define __OS_WRAPPER_SPECIFIC_H__

#define RTOS_CONVERT_MS_TO_TICKS(MS) k_ms_to_cyc_floor32(MS)

#endif
