/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>
#include <zephyr/kernel.h>

bool irq_register(IRQ_FUN IrqFun, IRQn_Type IrqNum, u32 Data, u32 Priority)
{
	irq_connect_dynamic((unsigned int)(IrqNum), (unsigned int)(Priority),
			    (void (*)(const void *))(IrqFun), (const void *)(Data), (uint32_t)0);
	return TRUE;
}

bool irq_unregister(IRQn_Type IrqNum)
{
	irq_disable(IrqNum);
	return TRUE;
}
