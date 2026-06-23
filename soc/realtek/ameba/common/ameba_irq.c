/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>
#include <zephyr/kernel.h>

/*
 * CA32IRQn values are GIC SPI indices (0-based).
 * Zephyr ARM IRQ APIs use the full GIC INTID (SPI index + 32).
 * Cortex-M IRQn_Type maps directly to NVIC external IRQ numbers (no offset).
 */
#ifdef CONFIG_CPU_AARCH32_CORTEX_A
#define GIC_SPI_OFFSET 32
#else
#define GIC_SPI_OFFSET 0
#endif

bool irq_register(IRQ_FUN IrqFun, IRQn_Type IrqNum, u32 Data, u32 Priority)
{
	irq_connect_dynamic((unsigned int)(IrqNum + GIC_SPI_OFFSET), (unsigned int)(Priority),
			    (void (*)(const void *))(IrqFun), (const void *)(Data), (uint32_t)0);
	return TRUE;
}

/*
 * Zephyr defines irq_enable() as a macro expanding to arch_irq_enable().
 * #undef it here so we can define the real exported function that HAL blobs
 * link against.
 */
#undef irq_enable
void irq_enable(IRQn_Type IrqNum)
{
	arm_irq_enable((unsigned int)(IrqNum + GIC_SPI_OFFSET));
}

bool irq_unregister(IRQn_Type IrqNum)
{
	arm_irq_disable((unsigned int)(IrqNum + GIC_SPI_OFFSET));
	return TRUE;
}
