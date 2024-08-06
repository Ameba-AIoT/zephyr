/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_I2S_I2S_AMEBA_H_
#define ZEPHYR_DRIVERS_I2S_I2S_AMEBA_H_

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
	uint8_t fifo_num;
	uint8_t pll_tune;
	uint16_t chn_len;
	uint32_t mclk_multiple;
	uint32_t mclk_fixed_max;
	uint32_t clock_mode;
};

struct SP_GDMA_STRUCT {
	GDMA_InitTypeDef       	SpTxGdmaInitStruct;
	GDMA_InitTypeDef       	SpRxGdmaInitStruct;
	GDMA_InitTypeDef       	SpTxGdmaInitStructExt;
	GDMA_InitTypeDef       	SpRxGdmaInitStructExt;
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
	struct stream tx;
	struct stream rx;
	void *tx_in_msgs[CONFIG_I2S_TX_BLOCK_COUNT];
	void *tx_out_msgs[CONFIG_I2S_TX_BLOCK_COUNT];
	void *rx_in_msgs[CONFIG_I2S_RX_BLOCK_COUNT];
	void *rx_out_msgs[CONFIG_I2S_RX_BLOCK_COUNT];
	struct SP_GDMA_STRUCT sp;
};
#endif	/* _ameba_I2S_H_ */
