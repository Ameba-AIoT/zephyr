/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_POWER_AMEBADPLUS_POWER_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_POWER_AMEBADPLUS_POWER_H_

#define WAKE_SRC_WIFI_FISR_FESR_IRQ 0x00000001 /* ((u32)0x00000001 << 0) */
#define WAKE_SRC_WIFI_FTSR_MAILBOX  0x00000002 /* ((u32)0x00000001 << 1) */
#define WAKE_SRC_KM4_WAKE_IRQ       0x00000004 /* ((u32)0x00000001 << 2) */
#define WAKE_SRC_BT_WAKE_HOST       0x00000008 /* ((u32)0x00000001 << 3) */
#define WAKE_SRC_IPC_KM4            0x00000010 /* ((u32)0x00000001 << 4) */
#define WAKE_SRC_IWDG0              0x00000020 /* ((u32)0x00000001 << 5) */
#define WAKE_SRC_TIMER4             0x00000040 /* ((u32)0x00000001 << 6) */
#define WAKE_SRC_TIMER5             0x00000080 /* ((u32)0x00000001 << 7) */
#define WAKE_SRC_TIMER6             0x00000100 /* ((u32)0x00000001 << 8) */
#define WAKE_SRC_TIMER7             0x00000200 /* ((u32)0x00000001 << 9) */
#define WAKE_SRC_PMC_TIMER0         0x00000400 /* ((u32)0x00000001 << 10) */
#define WAKE_SRC_PMC_TIMER1         0x00000800 /* ((u32)0x00000001 << 11) */
#define WAKE_SRC_UART0              0x00001000 /* ((u32)0x00000001 << 12) */
#define WAKE_SRC_UART1              0x00002000 /* ((u32)0x00000001 << 13) */
#define WAKE_SRC_UART2_BT           0x00004000 /* ((u32)0x00000001 << 14) */
#define WAKE_SRC_UART_LOG           0x00008000 /* ((u32)0x00000001 << 15) */
#define WAKE_SRC_GPIOA              0x00010000 /* ((u32)0x00000001 << 16) */
#define WAKE_SRC_GPIOB              0x00020000 /* ((u32)0x00000001 << 17) */
#define WAKE_SRC_I2C0               0x00040000 /* ((u32)0x00000001 << 18) */
#define WAKE_SRC_I2C1               0x00080000 /* ((u32)0x00000001 << 19) */
#define WAKE_SRC_CTOUCH             0x00100000 /* ((u32)0x00000001 << 20) */
#define WAKE_SRC_RTC                0x00200000 /* ((u32)0x00000001 << 21) */
#define WAKE_SRC_ADC                0x00400000 /* ((u32)0x00000001 << 22) */
#define WAKE_SRC_ADC_COMP           0x00800000 /* ((u32)0x00000001 << 23) */
#define WAKE_SRC_BOR                0x01000000 /* ((u32)0x00000001 << 24) */
#define WAKE_SRC_PWR_DOWN           0x02000000 /* ((u32)0x00000001 << 25) */
#define WAKE_SRC_Keyscan            0x04000000 /* ((u32)0x00000001 << 26) */
#define WAKE_SRC_AON_TIM            0x08000000 /* ((u32)0x00000001 << 27) */
#define WAKE_SRC_AON_WAKEPIN        0x10000000 /* ((u32)0x00000001 << 28) */
#define WAKE_SRC_SDIO               0x20000000 /* ((u32)0x00000001 << 29) */

#endif
