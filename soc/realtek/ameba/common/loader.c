/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <soc.h>

void soc_early_init_hook(void)
{
#ifdef CONFIG_MCUBOOT
	__ameba_mcuboot_soc_early_init_hook();
#else
	__ameba_app_soc_early_init_hook();
#endif
}
