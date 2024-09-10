/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_I2S_I2S_AMEBA_H_
#define ZEPHYR_DRIVERS_I2S_I2S_AMEBA_H_

#include <zephyr/drivers/dma.h>

/* Device constant configuration parameters */
struct i2s_ameba_cfg {
	AUDIO_SPORT_TypeDef *i2s;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pcfg;
	bool MultiIO;
	bool mono_stereo;
	uint8_t index;
	uint8_t tdmmode;
	uint8_t pll_tune;
	uint16_t chn_len;
	uint32_t mclk_multiple;
	uint32_t mclk_fixed_max;
	uint32_t clock_mode;
	int irq;
};

struct i2s_dma_stream {
	const struct device *dma_dev;
	uint32_t dma_channel;
	struct dma_config dma_cfg;
	uint8_t priority;
	uint8_t src_addr_increment;
	uint8_t dst_addr_increment;
	int fifo_threshold;
	struct dma_block_config blk_cfg;
	uint8_t *buffer;
	size_t buffer_length;
	size_t offset;
	volatile size_t counter;
	int32_t timeout;
	struct k_work_delayable timeout_work;
	bool enabled;
};

struct stream {
	int32_t state;
	struct i2s_config cfg;
	uint8_t free_tx_dma_blocks;
	bool last_block;
	struct k_msgq in_queue;
	struct k_msgq out_queue;
};

/* Device run time data */
struct i2s_ameba_data {
	uint8_t fifo_num;
	struct stream tx;
	struct stream rx;
	void *tx_out_msgs[1];
	void *rx_in_msgs[1];
	void *tx_in_msgs[CONFIG_I2S_TX_BLOCK_COUNT];
	void *rx_out_msgs[CONFIG_I2S_RX_BLOCK_COUNT];
	struct i2s_dma_stream dma_rx;//dma rx
	struct i2s_dma_stream dma_tx;//dma tx
#if defined(I2S_CHANNEL_EXT) && I2S_CHANNEL_EXT
	struct i2s_dma_stream dma_rx_ext;//dma rx_ext
	struct i2s_dma_stream dma_tx_ext;//dma tx_ext
#endif
};
#endif	/* _ameba_I2S_H_ */
