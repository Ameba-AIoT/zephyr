/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_i2s

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/irq.h>
#include "i2s_ameba.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2s_ameba);

uint32_t AudioTxSingleHandler(void *Data)
{
	struct SP_GDMA_STRUCT *sp = (struct SP_GDMA_STRUCT *)Data;
	PGDMA_InitTypeDef GDMA_InitStruct;
	GDMA_InitStruct = &sp->SpTxGdmaInitStruct;

	/* Clear Pending ISR */
	GDMA_ClearINT(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);
	GDMA_Cmd(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum, DISABLE);
	GDMA_ChnlFree(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);
	return 0;
}

uint32_t AudioTxSingleHandlerExt(void *Data)
{
	struct SP_GDMA_STRUCT *sp = (struct SP_GDMA_STRUCT *)Data;
	PGDMA_InitTypeDef GDMA_InitStruct;
	GDMA_InitStruct = &sp->SpTxGdmaInitStructExt;

	/* Clear Pending ISR */
	GDMA_ClearINT(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);
	GDMA_Cmd(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum, DISABLE);
	GDMA_ChnlFree(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);
	return 0;

}

uint32_t AudioRxSingleHandler(void *Data)
{
	struct SP_GDMA_STRUCT *sp = (struct SP_GDMA_STRUCT *)Data;
	PGDMA_InitTypeDef GDMA_InitStruct;
	GDMA_InitStruct = &sp->SpRxGdmaInitStruct;

	/* Clear Pending ISR */
	GDMA_ClearINT(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);
	GDMA_Cmd(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum, DISABLE);
	GDMA_ChnlFree(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);
	return 0;
}

uint32_t AudioRxSingleHandlerExt(void *Data)
{
	struct SP_GDMA_STRUCT *sp = (struct SP_GDMA_STRUCT *)Data;
	PGDMA_InitTypeDef GDMA_InitStruct;
	GDMA_InitStruct = &sp->SpRxGdmaInitStructExt;

	/* Clear Pending ISR */
	GDMA_ClearINT(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);
	GDMA_Cmd(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum, DISABLE);
	GDMA_ChnlFree(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);
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
	if (clock_control_on(cfg->clock_dev, cfg->clock_subsys)) {
		LOG_ERR("Could not enable I2S clock");
		return -EIO;
	}

	return 0;
}

static int i2s_ameba_configure(const struct device *dev, enum i2s_dir dir,
							   const struct i2s_config *i2s_cfg)
{
	struct i2s_ameba_data *data = dev->data;
	const struct i2s_ameba_cfg *cfg = dev->config;

	static struct AUDIO_ClockParams Clock_Params;
	static struct AUDIO_InitParams Init_Params;

	bool slave;
	u32 sp_dir;

	if (dir == I2S_DIR_RX) {
		sp_dir = SP_DIR_RX;
	} else if (dir == I2S_DIR_TX) {
		sp_dir = SP_DIR_TX;
	} else {
		LOG_ERR("Unsupported I2S direction");
		return -EINVAL;
	}

	if ((i2s_cfg->options & I2S_OPT_PINGPONG) == I2S_OPT_PINGPONG) {
		LOG_ERR("Ping-pong mode not supported");
	}

	if (i2s_cfg->frame_clk_freq == 0U) {
		LOG_ERR("Invalid frame_clk_freq %u",
				i2s_cfg->frame_clk_freq);
		if (dir == I2S_DIR_TX) {
			data->tx.state = I2S_STATE_NOT_READY;
		} else {
			data->rx.state = I2S_STATE_NOT_READY;
		}
		return -EINVAL;
	}

	SP_InitTypeDef SP_InitStruct;

	uint16_t chn_len1;
	uint16_t xtal_enable;

	/* choose XTAL or PLL */
	switch (cfg->chn_len) {
	case SP_TXCL_16:
		chn_len1 = SP_CL_16;
		break;
	case SP_TXCL_20:
		chn_len1 = SP_CL_20;
		break;
	case SP_TXCL_24:
		chn_len1 = SP_CL_24;
		break;
	default:
		chn_len1 = SP_CL_32;
		break;
	}

	Init_Params.chn_len = chn_len1;
	Init_Params.chn_cnt = (cfg->tdmmode + 1U) * 2U;
	Init_Params.sr = i2s_cfg->frame_clk_freq;

	Init_Params.codec_multiplier_with_rate = cfg->mclk_multiple;
	Init_Params.sport_mclk_fixed_max = cfg->mclk_fixed_max;


	if (cfg->clock_mode == 40000000U) {
		xtal_enable = 1;
	} else {
		xtal_enable = 0;
	}

	/* get MCLK div */
	Audio_Clock_Choose(xtal_enable, &Init_Params, &Clock_Params);

	AUDIO_SP_Reset(cfg->index);
	AUDIO_SP_StructInit(&SP_InitStruct);

	if (i2s_cfg->word_size == 16U) {
		SP_InitStruct.SP_SelWordLen = 0;
	} else if (i2s_cfg->word_size == 24U) {
		SP_InitStruct.SP_SelWordLen = 2U;
	} else if (i2s_cfg->word_size == 32U) {
		SP_InitStruct.SP_SelWordLen = 4U;
	} else {
		LOG_ERR("invalid word size");
		return -EINVAL;
	}

	/* set I2S Standard */
	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		SP_InitStruct.SP_SelDataFormat = i2s_cfg->format;
		break;

	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
		SP_InitStruct.SP_SelDataFormat = i2s_cfg->format + 1U;
		break;

	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		LOG_ERR("Unsupported I2S data format");
		return -EINVAL;

	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		SP_InitStruct.SP_SelDataFormat = i2s_cfg->format - 2U;
		break;

	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		LOG_ERR("Unsupported I2S data format");
		return -EINVAL;

	default:
		LOG_ERR("Unsupported I2S data format");
		return -EINVAL;
	}

	SP_InitStruct.SP_SelI2SMonoStereo = cfg->mono_stereo;
	SP_InitStruct.SP_SelTDM = cfg->tdmmode;
	SP_InitStruct.SP_SelFIFO = cfg->fifo_num;
	SP_InitStruct.SP_SetMultiIO = cfg->MultiIO;
	SP_InitStruct.SP_SR = i2s_cfg->frame_clk_freq;
	SP_InitStruct.SP_SelChLen = cfg->chn_len;
	SP_InitStruct.SP_SelClk = cfg->clock_mode;

	/* set MCLK */
	AUDIO_SP_SetMclkDiv(cfg->index, Clock_Params.MCLK_NI, Clock_Params.MCLK_MI);

	if ((i2s_cfg->options & I2S_OPT_LOOPBACK) == I2S_OPT_LOOPBACK) {
		AUDIO_SP_SetSelfLPBK(cfg->index);
	} else {
		AUDIO_SP_Init(cfg->index, sp_dir, &SP_InitStruct);
	}

	if (i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE ||
		i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE) {
		slave = true;
	} else {
		slave = false;
	}

	AUDIO_SP_SetMasterSlave(cfg->index, slave);

	if (dir == I2S_DIR_RX) {
		switch (i2s_cfg->channels) {
		case 2:
			AUDIO_SP_SetPinMux(cfg->index, DIN0_FUNC);
			break;
		case 4:
			AUDIO_SP_SetPinMux(cfg->index, DIN0_FUNC);
			AUDIO_SP_SetPinMux(cfg->index, DIN1_FUNC);
			break;
		case 6:
			AUDIO_SP_SetPinMux(cfg->index, DIN0_FUNC);
			AUDIO_SP_SetPinMux(cfg->index, DIN1_FUNC);
			AUDIO_SP_SetPinMux(cfg->index, DIN2_FUNC);
			break;
		case 8:
			AUDIO_SP_SetPinMux(cfg->index, DIN0_FUNC);
			AUDIO_SP_SetPinMux(cfg->index, DIN1_FUNC);
			AUDIO_SP_SetPinMux(cfg->index, DIN2_FUNC);
			AUDIO_SP_SetPinMux(cfg->index, DIN3_FUNC);
			break;
		}
	} else {
		switch (i2s_cfg->channels) {
		case 2:
			AUDIO_SP_SetPinMux(cfg->index, DOUT0_FUNC);
			break;
		case 4:
			AUDIO_SP_SetPinMux(cfg->index, DOUT0_FUNC);
			AUDIO_SP_SetPinMux(cfg->index, DOUT1_FUNC);
			break;
		case 6:
			AUDIO_SP_SetPinMux(cfg->index, DOUT0_FUNC);
			AUDIO_SP_SetPinMux(cfg->index, DOUT1_FUNC);
			AUDIO_SP_SetPinMux(cfg->index, DOUT2_FUNC);
			break;
		case 8:
			AUDIO_SP_SetPinMux(cfg->index, DOUT0_FUNC);
			AUDIO_SP_SetPinMux(cfg->index, DOUT1_FUNC);
			AUDIO_SP_SetPinMux(cfg->index, DOUT2_FUNC);
			AUDIO_SP_SetPinMux(cfg->index, DOUT3_FUNC);
		default:
			LOG_ERR("Unsupported I2S channel");
			return -EINVAL;
		}
	}

	return 0;
}

static const struct i2s_config *i2s_ameba_config_get(const struct device *dev,
		enum i2s_dir dir)
{
	struct i2s_ameba_data *data = dev->data;

	if (dir == I2S_DIR_RX) {
		return &data->rx.cfg;
	}

	return &data->tx.cfg;
}

static int i2s_ameba_trigger(const struct device *dev, enum i2s_dir dir,
							 enum i2s_trigger_cmd cmd)
{
	const struct i2s_ameba_cfg *cfg = dev->config;

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (dir == I2S_DIR_TX) {
			/* Enable GMDA and SPORT handshake*/
			AUDIO_SP_DmaCmd(cfg->index, ENABLE);
			AUDIO_SP_TXStart(cfg->index, ENABLE);
		} else {
			/* Enable GMDA and SPORT handshake*/
			AUDIO_SP_DmaCmd(cfg->index, ENABLE);
			AUDIO_SP_RXStart(cfg->index, ENABLE);
		}
		break;

	case I2S_TRIGGER_DROP:
		if (dir == I2S_DIR_TX) {
			/* Enable GMDA and SPORT handshake*/
			AUDIO_SP_DmaCmd(cfg->index, DISABLE);
			AUDIO_SP_TXStart(cfg->index, DISABLE);

		} else {
			/* Enable GMDA and SPORT handshake*/
			AUDIO_SP_DmaCmd(cfg->index, DISABLE);
			AUDIO_SP_RXStart(cfg->index, DISABLE);
		}
		break;

	default:
		LOG_ERR("Unsupported cmd");
		return -EINVAL;
	}

	return 0;
}


static int i2s_ameba_read(const struct device *dev, void **mem_block,
						  size_t *size)
{
	/*struct i2s_ameba_data *data = dev->data;*/
	const struct i2s_ameba_cfg *cfg = dev->config;

	struct SP_GDMA_STRUCT *sp = &audio_global;
	memset(sp, 0, sizeof(struct SP_GDMA_STRUCT));

	if (cfg->fifo_num == 0 || cfg->fifo_num == 1) {
		AUDIO_SP_RXGDMA_Init(cfg->index, GDMA_INT, &sp->SpRxGdmaInitStruct, sp,
							 (IRQ_FUN)AudioRxSingleHandler, (u8 *)(*mem_block), (uint32_t)size);
		irq_connect_dynamic(33, 2, AudioRxSingleHandler, sp, 0);
		irq_enable(33);
	} else {
		AUDIO_SP_RXGDMA_Init(cfg->index, GDMA_INT, &sp->SpRxGdmaInitStruct, sp,
							 (IRQ_FUN)AudioRxSingleHandler, (u8 *)(*mem_block), (uint32_t)size);
		irq_connect_dynamic(33, 2, AudioRxSingleHandler, sp, 0);
		irq_enable(33);

		AUDIO_SP_RXGDMA_Init(cfg->index, GDMA_EXT, &sp->SpRxGdmaInitStructExt, sp,
							 (IRQ_FUN)AudioRxSingleHandlerExt, (u8 *)(*mem_block), (uint32_t)size);
		irq_connect_dynamic(34, 2, AudioRxSingleHandlerExt, sp, 0);
		irq_enable(34);
	}

	return 0;
}

static int i2s_ameba_write(const struct device *dev, void *mem_block,
						   size_t size)
{
	/*struct i2s_ameba_data *data = dev->data;*/
	const struct i2s_ameba_cfg *cfg = dev->config;

	struct SP_GDMA_STRUCT *sp = &audio_global;
	memset(sp, 0, sizeof(struct SP_GDMA_STRUCT));

	if (cfg->fifo_num == 0 || cfg->fifo_num == 1) {
		AUDIO_SP_TXGDMA_Init(cfg->index, GDMA_INT, &sp->SpTxGdmaInitStruct, sp,
							 (IRQ_FUN)AudioTxSingleHandler, (u8 *)mem_block, (uint32_t)size);
		irq_connect_dynamic(33, 2, AudioTxSingleHandler, sp, 0);
		irq_enable(33);
	} else {
		AUDIO_SP_TXGDMA_Init(cfg->index, GDMA_INT, &sp->SpTxGdmaInitStruct, sp,
							 (IRQ_FUN)AudioTxSingleHandler, (u8 *)mem_block, (uint32_t)size);
		irq_connect_dynamic(33, 2, AudioTxSingleHandler, sp, 0);
		irq_enable(33);

		AUDIO_SP_TXGDMA_Init(cfg->index, GDMA_EXT, &sp->SpTxGdmaInitStructExt, sp,
							 (IRQ_FUN)AudioTxSingleHandlerExt, (u8 *)mem_block, (uint32_t)size);
		irq_connect_dynamic(34, 2, AudioTxSingleHandlerExt, sp, 0);
		irq_enable(34);
	}

	return 0;
}

static const struct i2s_driver_api i2s_ameba_driver_api = {
	.configure = i2s_ameba_configure,
	.config_get = i2s_ameba_config_get,
	.read = i2s_ameba_read,
	.write = i2s_ameba_write,
	.trigger = i2s_ameba_trigger,
};

static int i2s_ameba_initialize(const struct device *dev)
{
	const struct i2s_ameba_cfg *cfg = dev->config;
	/*struct i2s_ameba_data *data = dev->data;*/
	int ret;

	float ppm = 234.35;
	float cal_ppm = 0;

	/* Enable I2S clock propagation */
	ret = i2s_ameba_enable_clock(dev);
	if (ret < 0) {
		LOG_ERR("%s: clock enabling failed: %d", __func__, ret);
		return -EIO;
	}

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("I2S pinctrl setup failed (%d)", ret);
		return ret;
	}

	switch (cfg->clock_mode) {
	case PLL_CLOCK_45P1584M:
		RCC_PeriphClockSource_SPORT(cfg->i2s, CKSL_I2S_CPUPLL);
		if (cfg->index == 0) {
			PLL_I2S0_CLK(ENABLE, 1U);
		} else {
			PLL_I2S1_CLK(ENABLE, 1U);
		}

		/*if (cfg->pll_tune == 0) {
		 *	cal_ppm = PLL_I2S_45P1584M_ClkTune(ppm, PLL_AUTO);
		 *} else if (cfg->pll_tune == 1) {
		 *	cal_ppm = PLL_I2S_45P1584M_ClkTune(ppm, PLL_AUTO);
		 *	cal_ppm = PLL_I2S_45P1584M_ClkTune(ppm, PLL_FASTER);
		 *} else if (cfg->pll_tune == 2) {
		 *	cal_ppm = PLL_I2S_45P1584M_ClkTune(ppm, PLL_AUTO);
		 *	cal_ppm = PLL_I2S_45P1584M_ClkTune(ppm, PLL_SLOWER);
		 *}
		 */
		break;

	case PLL_CLOCK_98P304M:
		RCC_PeriphClockSource_SPORT(cfg->i2s, CKSL_I2S_CPUPLL);
		if (cfg->index == 0) {
			PLL_I2S0_CLK(ENABLE, 0);
		} else {
			PLL_I2S1_CLK(ENABLE, 0);
		}

		/*if (cfg->pll_tune == 0) {
		 *	cal_ppm = PLL_I2S_98P304M_ClkTune(ppm, PLL_AUTO);
		 *} else if (cfg->pll_tune == 1) {
		 *	cal_ppm = PLL_I2S_98P304M_ClkTune(ppm, PLL_AUTO);
		 *	cal_ppm = PLL_I2S_98P304M_ClkTune(ppm, PLL_FASTER);
		 *} else if (cfg->pll_tune == 2) {
		 *	cal_ppm = PLL_I2S_98P304M_ClkTune(ppm, PLL_AUTO);
		 *	cal_ppm = PLL_I2S_98P304M_ClkTune(ppm, PLL_SLOWER);
		 *}
		 */
		break;

	case I2S_CLOCK_XTAL40M:
		RCC_PeriphClockSource_SPORT(cfg->i2s, CKSL_I2S_XTAL40M);
		break;

	default:
		LOG_ERR("invalid clcok");
		return -EINVAL;
	}

	return 0;
}

/* src_dev and dest_dev should be 'MEMORY' or 'PERIPHERAL'. */
#define I2S_DMA_CHANNEL_INIT(index, dir, dir_cap, src_dev, dest_dev)			\

#define I2S_AMEBA_INIT(n)			\
									\
	PINCTRL_DT_INST_DEFINE(n);		\
									\
	static const struct i2s_ameba_cfg i2s_ameba_config_##n = {		\
		.i2s = (AUDIO_SPORT_TypeDef *)DT_INST_REG_ADDR(n),			\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),   \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, idx),   \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
		.index = DT_INST_PROP(n, index),		\
		.mclk_multiple = DT_INST_PROP(n, mclk_multiple),		\
		.mclk_fixed_max = DT_INST_PROP(n, mclk_fixed_max),		\
		.tdmmode = DT_INST_PROP(n, tdmmode),		\
		.fifo_num = DT_INST_PROP(n, fifo_num),		\
		.MultiIO = DT_INST_PROP(n, multiio),		\
		.chn_len =  DT_INST_PROP(n, chn_len),		\
		.mono_stereo =  DT_INST_PROP(n, mono_stereo),		\
		.clock_mode = DT_INST_PROP(n, clock_mode),		\
		.pll_tune = DT_INST_PROP(n, pll_tune),		\
									\
	};								\
										\
	DEVICE_DT_INST_DEFINE(n,						\
				&i2s_ameba_initialize, NULL,			\
				/*&i2s_ameba_data_##n,	*/NULL,			\
				&i2s_ameba_config_##n, POST_KERNEL,		\
				CONFIG_I2S_INIT_PRIORITY, &i2s_ameba_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2S_AMEBA_INIT)
