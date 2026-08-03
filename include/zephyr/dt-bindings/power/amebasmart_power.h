/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * AmebaSmart (RTL8730E) AP (CA32) wake-source IDs.
 *
 * Values mirror modules/hal/realtek/ameba/amebasmart/source/fwlib/include/
 * sysreg_pmc.h (WAK_MASK0_AP / WAK_MASK1_AP).  SOCPS_SetAPWakeEvent() uses
 * bits 30-31 of the value to select the mask register: 0..2 -> WAK_MASK0_AP,
 * 3 (0xC0000000 prefix) -> WAK_MASK1_AP.
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_POWER_AMEBASMART_POWER_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_POWER_AMEBASMART_POWER_H_

/* WAK_MASK0_AP: bit N -> (1 << N) */
#define WAKE_SRC_WIFI_FISR_FESR    0x00000001 /* bit 0  */
#define WAKE_SRC_WIFI_FTSR_MAILBOX 0x00000002 /* bit 1  */
#define WAKE_SRC_AON_TIM           0x00000004 /* bit 2  */
#define WAKE_SRC_NP_WAKE           0x00000008 /* bit 3  */
#define WAKE_SRC_AP_WAKE           0x00000010 /* bit 4  */
#define WAKE_SRC_WDG0              0x00000020 /* bit 5  */
#define WAKE_SRC_TIMER0            0x00000040 /* bit 6  */
#define WAKE_SRC_TIMER1            0x00000080 /* bit 7  */
#define WAKE_SRC_TIMER2            0x00000100 /* bit 8  */
#define WAKE_SRC_TIMER3            0x00000200 /* bit 9  */
#define WAKE_SRC_TIMER4            0x00000400 /* bit 10 */
#define WAKE_SRC_TIMER5            0x00000800 /* bit 11 */
#define WAKE_SRC_TIMER6            0x00001000 /* bit 12 */
#define WAKE_SRC_TIMER7            0x00002000 /* bit 13 */
#define WAKE_SRC_UART_LOG          0x00004000 /* bit 14 */
#define WAKE_SRC_GPIOA             0x00008000 /* bit 15 */
#define WAKE_SRC_GPIOB             0x00010000 /* bit 16 */
#define WAKE_SRC_GPIOC             0x00020000 /* bit 17 */
#define WAKE_SRC_RTC               0x00040000 /* bit 18 */
#define WAKE_SRC_CTOUCH            0x00080000 /* bit 19 */
#define WAKE_SRC_ADC               0x00100000 /* bit 20 */
#define WAKE_SRC_ADC_COMP          0x00200000 /* bit 21 */
#define WAKE_SRC_BOR               0x00400000 /* bit 22 */
#define WAKE_SRC_PWR_DOWN          0x00800000 /* bit 23 */
#define WAKE_SRC_VADBT_OR_VADPC    0x01000000 /* bit 24 */
#define WAKE_SRC_IPC_NP            0x02000000 /* bit 25 */
#define WAKE_SRC_IPC_AP            0x04000000 /* bit 26 */
#define WAKE_SRC_USB_OTG           0x08000000 /* bit 27 */
#define WAKE_SRC_SPI0              0x10000000 /* bit 28 */
#define WAKE_SRC_SPI1              0x20000000 /* bit 29 */
#define WAKE_SRC_UART0             0x40000000 /* bit 30 */
#define WAKE_SRC_UART1             0x80000000 /* bit 31 */

/* WAK_MASK1_AP: 0xC0000000 (RegIndex 3) | (1 << N) */
#define WAKE_SRC_UART2        0xC0000001 /* bit 0  */
#define WAKE_SRC_WDG1         0xC0000002 /* bit 1  */
#define WAKE_SRC_WDG2         0xC0000004 /* bit 2  */
#define WAKE_SRC_WDG3         0xC0000008 /* bit 3  */
#define WAKE_SRC_WDG4         0xC0000010 /* bit 4  */
#define WAKE_SRC_AON_WAKEPIN  0xC0000020 /* bit 5  */
#define WAKE_SRC_BT_WAKE_HOST 0xC0000040 /* bit 6  */
#define WAKE_SRC_nFIQOUT0     0xC0000080 /* bit 7  */
#define WAKE_SRC_nFIQOUT1     0xC0000100 /* bit 8  */
#define WAKE_SRC_I2C1         0xC0000400 /* bit 10 */
#define WAKE_SRC_I2C2         0xC0000800 /* bit 11 */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_POWER_AMEBASMART_POWER_H_ */
