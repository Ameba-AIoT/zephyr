/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_REALTEK_AMEBA_COMMON_AMEBA_PSRAM_H_
#define ZEPHYR_SOC_REALTEK_AMEBA_COMMON_AMEBA_PSRAM_H_

#ifndef _ASMLANGUAGE

#define PSRAM_ONLY_DATA_SECTION Z_GENERIC_SECTION(".psram.only.data")
void ameba_init_psram(void);

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_SOC_REALTEK_AMEBA_COMMON_AMEBA_PSRAM_H_ */
