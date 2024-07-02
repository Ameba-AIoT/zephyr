/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_i2s

#include <ameba_soc.h>
#include <string.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2s.h>
// #include <zephyr/drivers/dma/dma_ameba.h>
#include <zephyr/drivers/clock_control/ameba_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>

#include "i2s_ameba.h"
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(i2s_ameba);

#define MODULO_INC(val, max) { val = (++val < max) ? val : 0; }

static unsigned int div_round_closest(uint32_t dividend, uint32_t divisor)
{
	return (dividend + (divisor / 2U)) / divisor;
}

/*
 * Get data from the queue
 */
static int queue_get(struct ring_buf *rb, void **mem_block, size_t *size)
{
	unsigned int key;

	key = irq_lock();

	if (rb->tail == rb->head) {
		/* Ring buffer is empty */
		irq_unlock(key);
		return -ENOMEM;
	}

	*mem_block = rb->buf[rb->tail].mem_block;
	*size = rb->buf[rb->tail].size;
	// MODULO_INC(rb->tail, rb->len);

	irq_unlock(key);

	return 0;
}

/*
 * Put data in the queue
 */
static int queue_put(struct ring_buf *rb, void *mem_block, size_t size)
{
	uint16_t head_next;
	unsigned int key;

	key = irq_lock();

	head_next = rb->head;
	// MODULO_INC(head_next, rb->len);

	if (head_next == rb->tail) {
		/* Ring buffer is full */
		irq_unlock(key);
		return -ENOMEM;
	}

	rb->buf[rb->head].mem_block = mem_block;
	rb->buf[rb->head].size = size;
	rb->head = head_next;

	irq_unlock(key);

	return 0;
}

static int i2s_ameba_enable_clock(const struct device *dev)
{
	const struct i2s_ameba_cfg *cfg = dev->config;

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* enables I2S peripheral */
	if (clock_control_on(cfg->clock_dev, cfg->clock_subsys)) { //开启clock
		LOG_ERR("Could not enable I2S clock");
		return -EIO;
	}

	return 0;
}

static int i2s_ameba_configure(const struct device *dev, enum i2s_dir dir,
							   const struct i2s_config *i2s_cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(dir);
	ARG_UNUSED(i2s_cfg);
	return 0;
}

static int i2s_ameba_trigger(const struct device *dev, enum i2s_dir dir,
							 enum i2s_trigger_cmd cmd)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(dir);
	ARG_UNUSED(cmd);
	return 0;
}

static int i2s_ameba_read(const struct device *dev, void **mem_block,
						  size_t *size)
{
	struct i2s_ameba_data *const dev_data = dev->data;
	int ret;

	if (dev_data->rx.state == I2S_STATE_NOT_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}

	if (dev_data->rx.state != I2S_STATE_ERROR) {
		ret = k_sem_take(&dev_data->rx.sem,
						 SYS_TIMEOUT_MS(dev_data->rx.cfg.timeout));
		if (ret < 0) {
			return ret;
		}
	}

	/* Get data from the beginning of RX queue */
	ret = queue_get(&dev_data->rx.mem_block_queue, mem_block, size);
	if (ret < 0) {
		return -EIO;
	}

	return 0;
}

static int i2s_ameba_write(const struct device *dev, void *mem_block,
						   size_t size)
{
	struct i2s_ameba_data *const dev_data = dev->data;
	int ret;

	if (dev_data->tx.state != I2S_STATE_RUNNING &&
		dev_data->tx.state != I2S_STATE_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}

	ret = k_sem_take(&dev_data->tx.sem,
					 SYS_TIMEOUT_MS(dev_data->tx.cfg.timeout));
	if (ret < 0) {
		return ret;
	}

	/* Add data to the end of the TX queue */
	queue_put(&dev_data->tx.mem_block_queue, mem_block, size);

	return 0;
}

static const struct i2s_driver_api i2s_ameba_driver_api = {
	.configure = i2s_ameba_configure,
	.read = i2s_ameba_read,
	.write = i2s_ameba_write,
	.trigger = i2s_ameba_trigger,
};

static void i2s_ameba_isr(const struct device *dev)
{
	const struct i2s_ameba_cfg *cfg = dev->config;
	ARG_UNUSED(cfg);
}

static int i2s_ameba_initialize(const struct device *dev)
{
	const struct i2s_ameba_cfg *cfg = dev->config;
	struct i2s_ameba_data *const dev_data = dev->data;
	int ret;

	/* Enable I2S clock propagation */
	ret = i2s_ameba_enable_clock(dev);
	if (ret < 0) {
		LOG_ERR("%s: clock enabling failed: %d",  __func__, ret);
		return -EIO;
	}

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("I2S pinctrl setup failed (%d)", ret);
		return ret;
	}

	cfg->irq_config(dev);

	k_sem_init(&dev_data->rx.sem, 0, CONFIG_I2S_AMEBA_RX_BLOCK_COUNT);
	k_sem_init(&dev_data->tx.sem, CONFIG_I2S_AMEBA_TX_BLOCK_COUNT,
			   CONFIG_I2S_AMEBA_TX_BLOCK_COUNT);

	// for (i = 0; i < AMEBA_DMA_NUM_CHANNELS; i++) {
	// 	active_dma_rx_channel[i] = NULL;
	// 	active_dma_tx_channel[i] = NULL;
	// }

	/* Get the binding to the DMA device */
	if (!device_is_ready(dev_data->tx.dev_dma)) {
		LOG_ERR("%s device not ready", dev_data->tx.dev_dma->name);
		return -ENODEV;
	}
	if (!device_is_ready(dev_data->rx.dev_dma)) {
		LOG_ERR("%s device not ready", dev_data->rx.dev_dma->name);
		return -ENODEV;
	}

	LOG_INF("%s inited", dev->name);

	return 0;
}

/* src_dev and dest_dev should be 'MEMORY' or 'PERIPHERAL'. */
#define I2S_DMA_CHANNEL_INIT(index, dir, dir_cap, src_dev, dest_dev)			\


#define I2S_AMEBA_INIT(n)			\
									\
	static void i2s_ameba_irq_config_func_##n(const struct device *dev);	\
									\
	PINCTRL_DT_INST_DEFINE(n);		\
									\
	static const struct i2s_ameba_cfg i2s_ameba_config_##n = {		\
		.i2s = (SPI_TypeDef *)DT_INST_REG_ADDR(n),			\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),   \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, idx),   \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
		.irq_config = i2s_ameba_irq_config_func_##n,		\
				\
	};	/*.master_clk_sel = DT_INST_PROP(n, mck_enabled)*/							\
										\
	struct queue_item rx_##n##_ring_buf[CONFIG_I2S_AMEBA_RX_BLOCK_COUNT + 1];\
	struct queue_item tx_##n##_ring_buf[CONFIG_I2S_AMEBA_TX_BLOCK_COUNT + 1];\
										\
	static struct i2s_ameba_data i2s_ameba_data_##n = {				\
		/*UTIL_AND(DT_INST_DMAS_HAS_NAME(n, rx), I2S_DMA_CHANNEL_INIT(n, rx, RX, PERIPHERAL, MEMORY)),*/	\
		/*UTIL_AND(DT_INST_DMAS_HAS_NAME(n, tx), I2S_DMA_CHANNEL_INIT(n, tx, TX, MEMORY, PERIPHERAL)),*/	\
	};									\
	DEVICE_DT_INST_DEFINE(n,						\
				&i2s_ameba_initialize, NULL,			\
				&i2s_ameba_data_##n,				\
				&i2s_ameba_config_##n, POST_KERNEL,		\
				CONFIG_I2S_INIT_PRIORITY, &i2s_ameba_driver_api);	\
										\
	static void i2s_ameba_irq_config_func_##n(const struct device *dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
				DT_INST_IRQ(n, priority),			\
				i2s_ameba_isr, DEVICE_DT_INST_GET(n), 0);	\
		irq_enable(DT_INST_IRQN(n));				\
	}

DT_INST_FOREACH_STATUS_OKAY(I2S_AMEBA_INIT)
