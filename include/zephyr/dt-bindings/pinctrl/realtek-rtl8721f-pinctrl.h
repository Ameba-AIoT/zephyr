/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_REALTEK_RTL8721F_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_REALTEK_RTL8721F_PINCTRL_H_

/* Define pin modes */
#define GPIO							0x0
#define LOG_UART					0x1
#define UART						0x1
#define SPIC0_FLASH					0x2
#define SPIC1_FLASH					0x3
#define SPIC1_PSRAM					0x4
#define QSPI_OPSI					0x5
#define CAPTOUCH_AUX_ADC			0x6
#define SIC							0x7
#define SPI							0x8
#define SWD							0x9
#define SDIO							0xA
#define ANT_DIV						0xB
#define EXT_BT						0xC
#define BT_IO						0xD
#define BT							0xE
#define EXT_ZIGBEE					0xF
#define TIMER						0x10
#define USB							0x11
#define DEBUG						0x12
#define UART0_TXD					0x13
#define UART0_RXD					0x14
#define UART0_CTS					0x15
#define UART0_RTS					0x16
#define UART1_TXD					0x17
#define UART1_RXD					0x18
#define UART2_TXD					0x19
#define UART2_RXD					0x1A
#define UART2_CTS					0x1B
#define UART2_RTS					0x1C
#define SPI1_CLK						0x1D
#define SPI1_MISO					0x1E
#define SPI1_MOSI					0x1F
#define SPI1_CS						0x20
#define LEDC							0x21
#define I2S0_MCLK					0x22
#define I2S0_BCLK					0x23
#define I2S0_WS						0x24
#define I2S0_DIO0					0x25
#define I2S0_DIO1					0x26
#define I2S0_DIO2					0x27
#define I2S0_DIO3					0x28
#define I2S1_MCLK					0x29
#define I2S1_BCLK					0x2A
#define I2S1_WS						0x2B
#define I2S1_DIO0					0x2C
#define I2S1_DIO1					0x2D
#define I2S1_DIO2					0x2E
#define I2S1_DIO3					0x2F
#define I2C0_SCL						0x30
#define I2C0_SDA						0x31
#define I2C1_SCL						0x32
#define I2C1_SDA						0x33
#define PWM0						0x34
#define PWM1						0x35
#define PWM2						0x36
#define PWM3						0x37
#define PWM4						0x38
#define PWM5						0x39
#define PWM6						0x3A
#define PWM7						0x3B
#define BT_UART_TXD				0x3C
#define BT_UART_RTS				0x3D
#define DMIC_CLK					0x3E
#define DMIC_DATA					0x3F
#define IR_TX						0x40
#define IR_RX						0x41
#define KEY_ROW0					0x42
#define KEY_ROW1					0x43
#define KEY_ROW2					0x44
#define KEY_ROW3					0x45
#define KEY_ROW4					0x46
#define KEY_ROW5					0x47
#define KEY_ROW6					0x48
#define KEY_ROW7					0x49
#define KEY_COL0					0x4A
#define KEY_COL1					0x4B
#define KEY_COL2					0x4C
#define KEY_COL3					0x4D
#define KEY_COL4					0x4E
#define KEY_COL5					0x4F
#define KEY_COL6					0x50
#define KEY_COL7					0x51

/* Define pins number */
#define PORT_PIN(port, line)	((((port) - 'A') << 5) + (line))

#define REALTEK_PINMUX(port, line, mode) (((PORT_PIN(port, line)) << 8) | (mode))

#define GET_PORT_NUM(pin_mux)		((pin_mux >> 13) & 0x03)
#define GET_PIN_NUM(pin_mux)		((pin_mux >> 8) & 0x1f)
#define GET_PIMNUX_ID(pin_mux)		(pin_mux & 0xFF)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_REALTEK_RTL8721F_PINCTRL_H_ */

