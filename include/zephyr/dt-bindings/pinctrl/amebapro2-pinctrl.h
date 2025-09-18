/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_AMEBAPRO2_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_AMEBAPRO2_PINCTRL_H_

/* IO port name */
#define PORT_A   0
#define PORT_B   1
#define PORT_C   2
#define PORT_D   3
#define PORT_E   4
#define PORT_F   5
#define PORT_S   6
#define PORT_DDR 7

/* PINMUX Function definitions */
#define FUNC_COMP_ADC  0x00
#define FUNC_VP        0x00
#define FUNC_USB       0x00
#define FUNC_FLASH     0x00
#define FUNC_MIPI      0x00
#define FUNC_SENSOR    0x00
#define FUNC_I2C3      0x00
#define FUNC_RF_DBG    0x00
#define FUNC_ADC       0x00
#define FUNC_SD_HOST   0x00
#define FUNC_I2C       0x01
#define FUNC_SIC       0x01
#define FUNC_DMIC      0x01
#define FUNC_SGPIO     0x01
#define FUNC_WLAN_LED  0x02
#define FUNC_WLAN_UART 0x02
#define FUNC_PWM       0x02
#define FUNC_JTAG      0x03
#define FUNC_RFE_CTRL  0x03
#define FUNC_BT_AOX    0x04
#define FUNC_SDIO_HOST 0x04
#define FUNC_I2S       0x05
#define FUNC_SPI_M     0x05
#define FUNC_UART      0x06
#define FUNC_TPIU      0x06
#define FUNC_BT_UART   0x07
#define FUNC_BT_GPIO   0x07
#define FUNC_BT_LOG    0x07
#define FUNC_BT_MP     0x07
#define FUNC_BT_JTAG   0x07
#define FUNC_SPI_S     0x08
#define FUNC_VOE_JTAG  0x08
#define FUNC_EXT_BT    0x09
#define FUNC_GPIO      0x0F
#define FUNC_DDR       0x0F

/* Peripheral function category 0x00 */
#define PID_COMP_ADC  ((FUNC_COMP_ADC << 28) + 2)
#define PID_VP        ((FUNC_VP << 28) + 4)
#define PID_USB       ((FUNC_USB << 28) + 6)
#define PID_FLASH     ((FUNC_FLASH << 28) + 8)
#define PID_MIPI      ((FUNC_MIPI << 28) + 10)
#define PID_SENSOR    ((FUNC_SENSOR << 28) + 12)
#define PID_I2C3      ((FUNC_I2C3 << 28) + 14)
#define PID_RF_DBG    ((FUNC_RF_DBG << 28) + 16)
#define PID_ADC0      ((FUNC_ADC << 28) + 18)
#define PID_ADC1      (PID_ADC0 + 1)
#define PID_ADC2      (PID_ADC0 + 2)
#define PID_ADC3      (PID_ADC0 + 3)
#define PID_SD_HOST   ((FUNC_SD_HOST << 28) | (1 << 5))
#define PID_SPI2_1    ((FUNC_COMP_ADC << 28) | (1 << 6))
/* Peripheral function category 0x01 */
#define PID_I2C0      (FUNC_I2C << 28)
#define PID_I2C1      (PID_I2C0 + 1)
#define PID_I2C2      (PID_I2C0 + 2)
#define PID_SIC       ((FUNC_SIC << 28) | (1 << 2))
#define PID_DMIC      ((FUNC_DMIC << 28) | (1 << 4))
#define PID_SGPIO     ((FUNC_SGPIO << 28) | (1 << 6))
#define PID_SPI2_2    ((FUNC_I2C << 28) | (1 << 8))
/* Peripheral function category 0x02 */
#define PID_WLAN_LED  (FUNC_WLAN_LED << 28)
#define PID_WLAN_UART ((FUNC_WLAN_UART << 28) | (1 << 2))
#define PID_PWM0      ((FUNC_PWM << 28) | (1 << 4))
#define PID_PWM1      (PID_PWM0 + 1)
#define PID_PWM2      (PID_PWM0 + 2)
#define PID_PWM3      (PID_PWM0 + 3)
#define PID_PWM4      (PID_PWM0 + 4)
#define PID_PWM5      (PID_PWM0 + 5)
#define PID_PWM6      (PID_PWM0 + 6)
#define PID_PWM7      (PID_PWM0 + 7)
#define PID_PWM8      (PID_PWM0 + 8)
#define PID_PWM9      (PID_PWM0 + 9)
#define PID_PWM10     (PID_PWM0 + 10)
#define PID_PWM11     (PID_PWM0 + 11)
/* Peripheral function category 0x03 */
#define PID_JTAG      (FUNC_JTAG << 28)
#define PID_RFE_CTRL  ((FUNC_RFE_CTRL << 28) | (1 << 2))
/* Peripheral function category 0x04 */
#define PID_BT_AOX    (FUNC_BT_AOX << 28)
#define PID_SDIO_HOST ((FUNC_SDIO_HOST << 28) | (1 << 2))
/* Peripheral function category 0x05 */
#define PID_I2S0      (FUNC_I2S << 28)
#define PID_I2S1      (PID_I2S0 + 1)
#define PID_SPI0      ((FUNC_SPI_M << 28) | (1 << 2))
#define PID_SPI1      (PID_SPI0 + 1)
/* Peripheral function category 0x06 */
#define PID_UART0     (FUNC_UART << 28)
#define PID_UART1     (PID_UART0 + 1)
#define PID_UART2     (PID_UART0 + 2)
#define PID_UART3     (PID_UART0 + 3)
#define PID_UART4     (PID_UART0 + 4)
#define PID_TPIU      ((FUNC_TPIU << 28) | (1 << 3))
/* Peripheral function category 0x07 */
#define PID_BT_UART   (FUNC_BT_UART << 28)
#define PID_BT_GPIO   ((FUNC_BT_GPIO << 28) + 2)
#define PID_BT_LOG    ((FUNC_BT_LOG << 28) + 4)
#define PID_BT_MP_I2C ((FUNC_BT_MP << 28) + 6)
#define PID_BT_JTAG   ((FUNC_BT_JTAG << 28) + 8)
/* Peripheral function category 0x08 */
#define PID_SPI2      (FUNC_SPI_S << 28)
#define PID_SPI3      (PID_SPI2 + 1)
#define PID_VOE_JTAG  ((FUNC_VOE_JTAG << 28) | (1 << 2))
/* Peripheral function category 0x09 */
#define PID_EXT_BT    (FUNC_EXT_BT << 28)
#define PID_SPI2_3    ((FUNC_EXT_BT << 28) | (1 << 2))
/* Peripheral function category 0x0F */
#define PID_GPIO      (FUNC_GPIO << 28)
#define PID_DDR       ((FUNC_DDR << 28) | (1 << 2))

/* Define pins number: bit[21:18] port, bit[17:13] pin, bit[12:9] + bit[8:0] function ID */
#define AMEBA_PIN_NAME(port, pin) (((port) << 5) | (pin))
#define AMEBA_PINMUX(port, pin, funcid)                                                            \
	((AMEBA_PIN_NAME(port, pin) << 13) | (((funcid) >> 28) << 9) | ((funcid) & 0x1FF))

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_AMEBAPRO2_PINCTRL_H_ */
