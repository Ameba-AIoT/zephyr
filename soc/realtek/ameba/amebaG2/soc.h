/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_REALTEK_AMEBA_AMEBAG2_H_
#define ZEPHYR_SOC_REALTEK_AMEBA_AMEBAG2_H_

#ifndef _ASMLANGUAGE

#include <zephyr/sys/util.h>
#include "cmsis_cpu.h"
#include "ameba_psram.h"

void __ameba_app_soc_early_init_hook(void);
void __ameba_mcuboot_soc_early_init_hook(void);

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_SOC_REALTEK_AMEBA_AMEBAG2_H_ */
