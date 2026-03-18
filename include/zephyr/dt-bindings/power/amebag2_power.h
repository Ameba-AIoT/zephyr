/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_POWER_AMEBAG2_POWER_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_POWER_AMEBAG2_POWER_H_

#define WAKE_SRC_WIFI_FISR_FESR_IRQ    0x00000001 /* ((u32)0x00000001 << 0) */
#define WAKE_SRC_WIFI_FTSR_MAILBOX_IRQ 0x00000002 /* ((u32)0x00000001 << 1) */
#define WAKE_SRC_AP_WAKE_IRQ           0x00000004 /* ((u32)0x00000001 << 2) */
#define WAKE_SRC_IPC_CPU1              0x00000008 /* ((u32)0x00000001 << 3) */
#define WAKE_SRC_IPC_CPU0              0x00000010 /* ((u32)0x00000001 << 4) */
#define WAKE_SRC_IWDG                  0x00000020 /* ((u32)0x00000001 << 5) */
#define WAKE_SRC_TIMER0                0x00000040 /* ((u32)0x00000001 << 6) */
#define WAKE_SRC_TIMER1                0x00000080 /* ((u32)0x00000001 << 7) */
#define WAKE_SRC_TIMER2                0x00000100 /* ((u32)0x00000001 << 8) */
#define WAKE_SRC_TIMER3                0x00000200 /* ((u32)0x00000001 << 9) */
#define WAKE_SRC_PMC_TIMER0            0x00000400 /* ((u32)0x00000001 << 10) */
#define WAKE_SRC_PMC_TIMER1            0x00000800 /* ((u32)0x00000001 << 11) */
#define WAKE_SRC_UART0                 0x00001000 /* ((u32)0x00000001 << 12) */
#define WAKE_SRC_UART1                 0x00002000 /* ((u32)0x00000001 << 13) */
#define WAKE_SRC_UART2                 0x00004000 /* ((u32)0x00000001 << 14) */
#define WAKE_SRC_UART3                 0x00008000 /* ((u32)0x00000001 << 15) */
#define WAKE_SRC_UART_LOG              0x00010000 /* ((u32)0x00000001 << 16) */
#define WAKE_SRC_GPIOA                 0x00020000 /* ((u32)0x00000001 << 17) */
#define WAKE_SRC_GPIOB                 0x00040000 /* ((u32)0x00000001 << 18) */
#define WAKE_SRC_GPIOC                 0x00080000 /* ((u32)0x00000001 << 19) */
#define WAKE_SRC_RTC                   0x00400000 /* ((u32)0x00000001 << 22) */
#define WAKE_SRC_ADC                   0x00800000 /* ((u32)0x00000001 << 23) */
#define WAKE_SRC_CAPTOUCH              0x02000000 /* ((u32)0x00000001 << 25) */
#define WAKE_SRC_BOR                   0x04000000 /* ((u32)0x00000001 << 26) */
#define WAKE_SRC_PWR_DOWN              0x08000000 /* ((u32)0x00000001 << 27) */
#define WAKE_SRC_RMII                  0x10000000 /* ((u32)0x00000001 << 28) */
#define WAKE_SRC_AON_TIM               0x20000000 /* ((u32)0x00000001 << 29) */

#define WAKE_SRC_AON_WAKEPIN  0xC0000001 /* ((u32)(3 << 30) | (0x00000001 << 0)) */
#define WAKE_SRC_SDIO_WIFI    0xC0000002 /* ((u32)(3 << 30) | (0x00000001 << 1)) */
#define WAKE_SRC_SDIO_BT      0xC0000004 /* ((u32)(3 << 30) | (0x00000001 << 2)) */
#define WAKE_SRC_SDIO_HOST    0xC0000008 /* ((u32)(3 << 30) | (0x00000001 << 3)) */
#define WAKE_SRC_USB          0xC0000010 /* ((u32)(3 << 30) | (0x00000001 << 4)) */
#define WAKE_SRC_CAN0         0xC0000020 /* ((u32)(3 << 30) | (0x00000001 << 5)) */
#define WAKE_SRC_CAN1         0xC0000040 /* ((u32)(3 << 30) | (0x00000001 << 6)) */
#define WAKE_SRC_BT_SCB       0xC0000080 /* ((u32)(3 << 30) | (0x00000001 << 7)) */
#define WAKE_SRC_BT_WAKE_HOST 0xC0000100 /* ((u32)(3 << 30) | (0x00000001 << 8)) */

#endif
