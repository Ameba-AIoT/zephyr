/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_REALTEK_AMEBA_AMEBADPLUS_H_
#define ZEPHYR_SOC_REALTEK_AMEBA_AMEBADPLUS_H_

#ifndef _ASMLANGUAGE

#include <zephyr/sys/util.h>

#define __NVIC_PRIO_BITS               3         /**< Number of priority bits implemented in the NVIC */

#ifndef __MPU_PRESENT
#define __MPU_PRESENT             CONFIG_CPU_HAS_ARM_MPU
#endif

#ifndef __FPU_PRESENT
#define __FPU_PRESENT             CONFIG_CPU_HAS_FPU
#endif

#ifndef __DSP_PRESENT
#define __DSP_PRESENT             CONFIG_ARMV8_M_DSP
#endif

#define DCACHE_4WAY
#define s32                     int32_t
#define u32                     uint32_t
#ifndef BOOL
typedef unsigned char           BOOL;
#endif
#define _LONG_CALL_     __attribute__ ((long_call))
#include <ameba_vector.h>
#include "core_armv81mml.h"

#endif /* _ASMLANGUAGE */

#endif  /* ZEPHYR_SOC_REALTEK_AMEBA_AMEBADPLUS_H_ */
