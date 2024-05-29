/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba LOGUART
 */

#include <ameba_soc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>

/*
 * Extract information from devicetree.
 *
 * This driver only supports one instance of this IP block, so the
 * instance number is always 0.
 */
#define DT_DRV_COMPAT	realtek_ameba_loguart

/* Device data structure */
struct ameba_loguart_data {
	struct uart_config config;
};

/**
 * @brief Poll the device for input.
 *
 * @param dev UART device struct
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer if empty.
 */

static int uart_ameba_loguart_poll_in(const struct device *dev, unsigned char *c)
{
	ARG_UNUSED(dev);
	*c = LOGUART_GetChar(true);
	return 0;
}

/**
 * @brief Output a character in polled mode.
 *
 * @param dev UART device struct
 * @param c Character to send
 */
static void uart_ameba_loguart_poll_out(const struct device *dev, unsigned char c)
{
	ARG_UNUSED(dev);
	LOGUART_PutChar(c);
}

/**
 * @brief Initialize UART channel
 *
 * This routine is called to reset the chip in a quiescent state.
 * It is assumed that this function is called only once per UART.
 *
 * @param dev UART device struct
 *
 * @return 0 on success
 */
static int uart_ameba_loguart_init(const struct device *dev)
{
	struct ameba_loguart_data *data = dev->data;
	LOGUART_InitTypeDef loguart_init_struct;

	RCC_PeriphClockCmd(APBPeriph_LOGUART, APBPeriph_LOGUART_CLOCK, ENABLE);

	Pinmux_UartLogCtrl(PINMUX_S0, ON);

	LOGUART_StructInit(&loguart_init_struct);

	/* Wait Log_UART tx done, otherwise redundant data will be tx when re-initialize Log_UART */
	LOGUART_WaitTxComplete();

	/* Initialize Log_UART */
	_LOGUART_Init(LOGUART_DEV, &loguart_init_struct);

	/* Set baudrate */
	LOGUART_RxCmd(LOGUART_DEV, DISABLE);
	LOGUART_SetBaud(LOGUART_DEV, data->config.baudrate);
	LOGUART_INTConfig(LOGUART_DEV, LOGUART_BIT_ERBI | LOGUART_BIT_ELSI, DISABLE);
	LOGUART_RxCmd(LOGUART_DEV, ENABLE);

	return 0;
}

static const struct uart_driver_api uart_ameba_loguart_driver_api = {
	.poll_in = uart_ameba_loguart_poll_in,
	.poll_out = uart_ameba_loguart_poll_out,
};

static struct ameba_loguart_data uart_ameba_loguart_data = {
	.config = {
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.baudrate = DT_INST_PROP(0, current_speed),
		.parity = UART_CFG_PARITY_NONE,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	}
};

DEVICE_DT_INST_DEFINE(0,
					  uart_ameba_loguart_init,
					  NULL,
					  &uart_ameba_loguart_data,
					  NULL,
					  PRE_KERNEL_1,
					  CONFIG_SERIAL_INIT_PRIORITY,
					  &uart_ameba_loguart_driver_api);
