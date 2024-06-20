/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba UART interface
 */
#define DT_DRV_COMPAT realtek_ameba_uart

#ifdef CONFIG_UART_ASYNC_API
#include <zephyr/drivers/dma.h>
#endif

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>
#include <soc.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/drivers/clock_control.h>
#include <errno.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_ameba, CONFIG_UART_LOG_LEVEL);

struct uart_ameba_config {
	const struct device *clock_dev;
	const struct pinctrl_dev_config *pcfg;
	const clock_control_subsys_t clock_subsys;
	int irq_source;
	int irq_priority;
#if CONFIG_UART_ASYNC_API
	const struct device *dma_dev;
	uint8_t tx_dma_channel;
	uint8_t rx_dma_channel;
	bool uart_id;
#endif
};

/* driver data */
struct uart_ameba_data {
	struct uart_config uart_config;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t irq_cb;
	void *irq_cb_data;
#endif
#if CONFIG_UART_ASYNC_API
	struct uart_ameba_async_data async;
	uhci_dev_t *uhci_dev;
	const struct device *uart_dev;
#endif
};

static int uart_ameba_poll_in(const struct device *dev, unsigned char *p_char)
{
	return 0;
}

static void uart_ameba_poll_out(const struct device *dev, unsigned char c)
{

}

static int uart_ameba_err_check(const struct device *dev)
{
	return 0;
}

static int uart_ameba_init(const struct device *dev)
{
	return 0;
}

static const struct uart_driver_api uart_ameba_api = {
	.poll_in = uart_ameba_poll_in,
	.poll_out = uart_ameba_poll_out,
	.err_check = uart_ameba_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	// .configure = uart_ameba_configure,
	// .config_get = uart_ameba_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_ameba_fifo_fill,
	.fifo_read = uart_ameba_fifo_read,
	.irq_tx_enable = uart_ameba_irq_tx_enable,
	.irq_tx_disable = uart_ameba_irq_tx_disable,
	.irq_tx_ready = uart_ameba_irq_tx_ready,
	.irq_rx_enable = uart_ameba_irq_rx_enable,
	.irq_rx_disable = uart_ameba_irq_rx_disable,
	.irq_tx_complete = uart_ameba_irq_tx_complete,
	.irq_rx_ready = uart_ameba_irq_rx_ready,
	.irq_err_enable = uart_ameba_irq_err_enable,
	.irq_err_disable = uart_ameba_irq_err_disable,
	.irq_is_pending = uart_ameba_irq_is_pending,
	.irq_update = uart_ameba_irq_update,
	.irq_callback_set = uart_ameba_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#if CONFIG_UART_ASYNC_API
	.callback_set = uart_ameba_async_callback_set,
	.tx = uart_ameba_async_tx,
	.tx_abort = uart_ameba_async_tx_abort,
	.rx_enable = uart_ameba_async_rx_enable,
	.rx_buf_rsp = uart_ameba_async_rx_buf_rsp,
	.rx_disable = uart_ameba_async_rx_disable,
#endif /*CONFIG_UART_ASYNC_API*/
};

#if CONFIG_UART_ASYNC_API
#define UART_IRQ_PRIORITY INT_PRI_MIDDLE
#else
#define UART_IRQ_PRIORITY (0)
#endif

#define AMEBA_UART_INIT(n)   \
     						 \
	static const struct uart_ameba_config uart_ameba_config##n = {              \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                     \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, idx),    \
		.irq_source = DT_INST_IRQN(n),                                          \
		.irq_priority = UART_IRQ_PRIORITY,                                      \
    };          \
				\
	static struct uart_ameba_data uart_ameba_data_##n = {                       \
		.uart_config = {.baudrate = DT_INST_PROP(n, current_speed),             \
				.parity = UART_CFG_PARITY_NONE,                                 \
				.stop_bits = UART_CFG_STOP_BITS_1,                              \
				.data_bits = UART_CFG_DATA_BITS_8,                              \
				.flow_ctrl = MAX(COND_CODE_1(DT_INST_PROP(n, hw_rs485_hd_mode), \
							     (UART_CFG_FLOW_CTRL_RS485),           \
							     (UART_CFG_FLOW_CTRL_NONE)),           \
						 COND_CODE_1(DT_INST_PROP(n, hw_flow_control), \
							     (UART_CFG_FLOW_CTRL_RTS_CTS),         \
							     (UART_CFG_FLOW_CTRL_NONE)))},         \
	};          \
				\
	DEVICE_DT_INST_DEFINE(n, &uart_ameba_init, NULL, &uart_ameba_data_##n,      \
			      &uart_ameba_config##n, PRE_KERNEL_1,                          \
			      CONFIG_SERIAL_INIT_PRIORITY, &uart_ameba_api);

DT_INST_FOREACH_STATUS_OKAY(AMEBA_UART_INIT);
