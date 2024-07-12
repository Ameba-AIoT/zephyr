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
	uint8_t index;
	uint32_t mclk_multiple;
	uint32_t mclk_fixed_max;
	uint8_t tdmmode;
	uint8_t fifo_num;
	bool MultiIO;
	uint16_t chn_len;
	bool mono_stereo;
	uint32_t clock_mode;
	uint8_t pll_tune;
};

struct AUDIO_ClockParams {
	u32 Clock;
	u32 MCLK_NI;
	u32 MCLK_MI;
};

struct AUDIO_InitParams {
	u32 sr;
	u32 chn_len;
	u32 chn_cnt;
	u32 codec_multiplier_with_rate;
	u32 sport_mclk_fixed_max;
};

#define PLL_CLOCK_45P1584M	45158400
#define PLL_CLOCK_98P304M	98304000
#define I2S_CLOCK_XTAL40M	40000000

struct SP_GDMA_STRUCT {
	GDMA_InitTypeDef       	SpTxGdmaInitStruct;
	GDMA_InitTypeDef       	SpRxGdmaInitStruct;
	GDMA_InitTypeDef       	SpTxGdmaInitStructExt;
	GDMA_InitTypeDef       	SpRxGdmaInitStructExt;
};

struct SP_GDMA_STRUCT audio_global;

struct stream {
	int32_t state;
	struct i2s_config cfg;
};

/* Device run time data */
struct i2s_ameba_data {
	struct stream tx;
	struct stream rx;
};

#endif	/* _ameba_I2S_H_ */
