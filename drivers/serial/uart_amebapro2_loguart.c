/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba LOGUART
 */

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <hal_uart.h>
#include <stdio_port_func.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(loguart_ameba, CONFIG_UART_LOG_LEVEL);

/*
 * Extract information from devicetree.
 *
 * This driver only supports one instance of this IP block, so the
 * instance number is always 0.
 */
#define DT_DRV_COMPAT realtek_amebapro2_loguart

/* Device config structure */
struct loguart_ameba_config {
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	uart_irq_config_func_t irq_config_func;
#endif
};

/* Device data structure */
struct loguart_ameba_data {
	struct uart_config config;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
	bool tx_int_en;
	bool rx_int_en;
#endif
};

extern hal_uart_adapter_t log_uart;

/**
 * @brief Poll the device for input.
 *
 * @param dev UART device struct
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer if empty.
 */
static int loguart_ameba_poll_in(const struct device *dev, unsigned char *c)
{
	ARG_UNUSED(dev);

	if (!(log_uart.base_addr->rflvr_b.rx_fifo_lv > 0)) {
		return -1;
	}

	hal_uart_rgetc(&log_uart, (char *) c);
	return 0;
}

/**
 * @brief Output a character in polled mode.
 *
 * @param dev UART device struct
 * @param c Character to send
 */
static void loguart_ameba_poll_out(const struct device *dev, unsigned char c)
{
	ARG_UNUSED(dev);

	hal_uart_wputc(&log_uart, (uint8_t) c);
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int loguart_ameba_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	ARG_UNUSED(dev);

	uint8_t num_tx = 0U;
	unsigned int key;

	if (!(log_uart.base_addr->tflvr_b.tx_fifo_lv < Uart_Tx_FIFO_Size)) {
		return num_tx;
	}

	/* Lock interrupts to prevent nested interrupts or thread switch */

	key = irq_lock();

	while ((len - num_tx > 0) && (log_uart.base_addr->tflvr_b.tx_fifo_lv < Uart_Tx_FIFO_Size)) {
		hal_uart_wputc(&log_uart, (uint8_t) tx_data[num_tx++]);
	}

	irq_unlock(key);

	return num_tx;
}

static int loguart_ameba_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	ARG_UNUSED(dev);

	uint8_t num_rx = 0U;

	while ((size - num_rx > 0) && (log_uart.base_addr->rflvr_b.rx_fifo_lv > 0)) {
		hal_uart_rgetc(&log_uart, (char *) &rx_data[num_rx++]);
	}

	return num_rx;
}

static void loguart_ameba_irq_tx_enable(const struct device *dev)
{
	struct loguart_ameba_data *data = dev->data;

	data->tx_int_en = true;

	/* enable TX FIFO empty interrupt */
	log_uart.base_addr->ier_b.etbei = 1;
}

static void loguart_ameba_irq_tx_disable(const struct device *dev)
{
	struct loguart_ameba_data *data = dev->data;

	/* disable TX FIFO empty interrupt */
	log_uart.base_addr->ier_b.etbei = 0;

	data->tx_int_en = false;
}

static int loguart_ameba_irq_tx_ready(const struct device *dev)
{
	struct loguart_ameba_data *data = dev->data;

	return (log_uart.base_addr->tflvr_b.tx_fifo_lv < Uart_Tx_FIFO_Size) && data->tx_int_en;
}

static int loguart_ameba_irq_tx_complete(const struct device *dev)
{
	struct loguart_ameba_data *data = dev->data;

	return (log_uart.base_addr->tflvr_b.tx_fifo_lv == 0) && data->tx_int_en;
}

static void loguart_ameba_irq_rx_enable(const struct device *dev)
{
	struct loguart_ameba_data *data = dev->data;

	data->rx_int_en = true;

	/* enable RX data available interrupt */
	log_uart.base_addr->ier_b.erbi = 1;
}

static void loguart_ameba_irq_rx_disable(const struct device *dev)
{
	struct loguart_ameba_data *data = dev->data;

	data->rx_int_en = false;

	/* disable RX data available interrupt */
	log_uart.base_addr->ier_b.erbi = 0;
}

static int loguart_ameba_irq_rx_ready(const struct device *dev)
{
	struct loguart_ameba_data *data = dev->data;

	return (log_uart.base_addr->rflvr_b.rx_fifo_lv > 0) && data->rx_int_en;
}

static void loguart_ameba_irq_err_enable(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static void loguart_ameba_irq_err_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static int loguart_ameba_irq_is_pending(const struct device *dev)
{
	return (log_uart.base_addr->iir_b.int_pend == 0);
}

static int loguart_ameba_irq_update(const struct device *dev)
{
	return 1;
}

static void loguart_ameba_irq_callback_set(const struct device *dev,
		uart_irq_callback_user_data_t cb, void *cb_data)
{
	struct loguart_ameba_data *data = dev->data;

	data->user_cb = cb;
	data->user_data = cb_data;
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

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
static int loguart_ameba_init(const struct device *dev)
{
	hal_status_t ret;
	struct loguart_ameba_data *data = dev->data;

	ret = hal_uart_init(&log_uart, STDIO_UART_TX_PIN, STDIO_UART_RX_PIN, NULL);

	/* recover IRQ handler to zephyr ISR wrapper after hal_uart_init */
	__NVIC_SetVector((IRQn_Type) UART1_IRQn, (uint32_t) _isr_wrapper);

	if (ret == HAL_OK) {
		hal_uart_set_baudrate(&log_uart, data->config.baudrate);
		hal_uart_set_format(&log_uart, 8, UartParityNone, 1);
	} else {
		return -1;
	}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	const struct loguart_ameba_config *config = dev->config;

	config->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	return 0;
}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
#define AMEBA_LOGUART_IRQ_HANDLER_DECL                                                             \
	static void loguart_ameba_irq_config_func(const struct device *dev);
#define AMEBA_LOGUART_IRQ_HANDLER                                                                  \
	static void loguart_ameba_irq_config_func(const struct device *dev)                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), loguart_ameba_isr,          \
			    DEVICE_DT_INST_GET(0), 0);                                             \
		irq_enable(DT_INST_IRQN(0));                                                       \
	}
#define AMEBA_LOGUART_IRQ_HANDLER_FUNC .irq_config_func = loguart_ameba_irq_config_func,
#else
#define AMEBA_LOGUART_IRQ_HANDLER_DECL /* Not used */
#define AMEBA_LOGUART_IRQ_HANDLER      /* Not used */
#define AMEBA_LOGUART_IRQ_HANDLER_FUNC /* Not used */
#endif

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)

static void loguart_ameba_isr(const struct device *dev)
{
	struct loguart_ameba_data *data = dev->data;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	/* read IIR to clear pending */
	if (log_uart.base_addr->iir_b.int_pend != 0) {
		return;
	}

	if (data->user_cb) {
		data->user_cb(dev, data->user_data);
	}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api loguart_ameba_driver_api = {
	.poll_in = loguart_ameba_poll_in,
	.poll_out = loguart_ameba_poll_out,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = loguart_ameba_fifo_fill,
	.fifo_read = loguart_ameba_fifo_read,
	.irq_tx_enable = loguart_ameba_irq_tx_enable,
	.irq_tx_disable = loguart_ameba_irq_tx_disable,
	.irq_tx_ready = loguart_ameba_irq_tx_ready,
	.irq_rx_enable = loguart_ameba_irq_rx_enable,
	.irq_rx_disable = loguart_ameba_irq_rx_disable,
	.irq_tx_complete = loguart_ameba_irq_tx_complete,
	.irq_rx_ready = loguart_ameba_irq_rx_ready,
	.irq_err_enable = loguart_ameba_irq_err_enable,
	.irq_err_disable = loguart_ameba_irq_err_disable,
	.irq_is_pending = loguart_ameba_irq_is_pending,
	.irq_update = loguart_ameba_irq_update,
	.irq_callback_set = loguart_ameba_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

AMEBA_LOGUART_IRQ_HANDLER_DECL
AMEBA_LOGUART_IRQ_HANDLER

static const struct loguart_ameba_config loguart_config = {AMEBA_LOGUART_IRQ_HANDLER_FUNC};

static struct loguart_ameba_data loguart_data = {.config = {
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.baudrate = DT_INST_PROP(0, current_speed),
		.parity = UART_CFG_PARITY_NONE,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	}
};

DEVICE_DT_INST_DEFINE(0, loguart_ameba_init, NULL, &loguart_data, &loguart_config, PRE_KERNEL_1,
					  CONFIG_SERIAL_INIT_PRIORITY, &loguart_ameba_driver_api);
