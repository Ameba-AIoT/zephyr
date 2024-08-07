/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_i2s

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>
#include "ameba_audio_clock.h"
#include <string.h>

#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include "i2s_ameba.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2s_ameba);

#define NUM_DMA_BLOCKS_RX_PREP  2
#define MAX_TX_DMA_BLOCKS       1

static inline void i2s_purge_stream_buffers(struct stream *stream,
		struct k_mem_slab *mem_slab,
		bool in_drop, bool out_drop)
{
	void *buffer;

	if (in_drop) {
		while (k_msgq_get(&stream->in_queue, &buffer, K_NO_WAIT) == 0) {
			k_mem_slab_free(mem_slab, buffer);
		}
	}

	if (out_drop) {
		while (k_msgq_get(&stream->out_queue, &buffer, K_NO_WAIT) == 0) {
			k_mem_slab_free(mem_slab, buffer);
		}
	}
}

static void i2s_tx_stream_disable(const struct device *dev, bool drop)
{
	struct i2s_ameba_data *data = dev->data;
	struct stream *stream = &data->tx;
	const struct i2s_ameba_cfg *cfg = dev->config;

	PGDMA_InitTypeDef GDMA_InitStruct;
	GDMA_InitStruct = &data->sp.SpTxGdmaInitStruct;

	GDMA_ClearINT(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);
	GDMA_Abort(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);
	GDMA_ChnlFree(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);

	if (cfg->fifo_num == 2 || cfg->fifo_num == 3) {
		PGDMA_InitTypeDef GDMA_InitStructExt;
		GDMA_InitStructExt = &data->sp.SpTxGdmaInitStructExt;

		GDMA_ClearINT(GDMA_InitStructExt->GDMA_Index, GDMA_InitStructExt->GDMA_ChNum);
		GDMA_Abort(GDMA_InitStructExt->GDMA_Index, GDMA_InitStructExt->GDMA_ChNum);
		GDMA_ChnlFree(GDMA_InitStructExt->GDMA_Index, GDMA_InitStructExt->GDMA_ChNum);
	}

	AUDIO_SP_DmaCmd(cfg->index, DISABLE);
	AUDIO_SP_Deinit(cfg->index, SP_DIR_TX);

	/* purge buffers queued in the stream */
	if (drop) {
		i2s_purge_stream_buffers(stream, data->tx.cfg.mem_slab,
								 true, true);
	}
}

static void i2s_rx_stream_disable(const struct device *dev,
								  bool in_drop, bool out_drop)
{
	struct i2s_ameba_data *data = dev->data;
	struct stream *stream = &data->rx;
	const struct i2s_ameba_cfg *cfg = dev->config;

	PGDMA_InitTypeDef GDMA_InitStruct;
	GDMA_InitStruct = &data->sp.SpRxGdmaInitStruct;

	GDMA_ClearINT(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);
	GDMA_Abort(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);
	GDMA_ChnlFree(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);

	if (cfg->fifo_num == 2 || cfg->fifo_num == 3) {
		PGDMA_InitTypeDef GDMA_InitStructExt;
		GDMA_InitStructExt = &data->sp.SpRxGdmaInitStructExt;

		GDMA_ClearINT(GDMA_InitStructExt->GDMA_Index, GDMA_InitStructExt->GDMA_ChNum);
		GDMA_Abort(GDMA_InitStructExt->GDMA_Index, GDMA_InitStructExt->GDMA_ChNum);
		GDMA_ChnlFree(GDMA_InitStructExt->GDMA_Index, GDMA_InitStructExt->GDMA_ChNum);
	}

	AUDIO_SP_DmaCmd(cfg->index, DISABLE);
	AUDIO_SP_Deinit(cfg->index, SP_DIR_RX);

	/* purge buffers queued in the stream */
	if (in_drop || out_drop) {
		i2s_purge_stream_buffers(stream, data->rx.cfg.mem_slab,
								 in_drop, out_drop);
	}
}

static int i2s_ameba_tx_reload_multiple_dma_blocks(const struct device *dev,
		uint8_t *blocks_queued)
{
	struct i2s_ameba_data *data = dev->data;
	const struct i2s_ameba_cfg *cfg = dev->config;
	struct stream *stream = &data->tx;
	void *buffer = NULL;
	int ret = 0;
	unsigned int key;

	PGDMA_InitTypeDef GDMA_InitStruct;
	GDMA_InitStruct = &data->sp.SpTxGdmaInitStruct;

	if (cfg->fifo_num == 2 || cfg->fifo_num == 3) {
		PGDMA_InitTypeDef GDMA_InitStructExt;
		GDMA_InitStructExt = &data->sp.SpTxGdmaInitStructExt;
	}

	*blocks_queued = 0;

	key = irq_lock();

	/* queue additional blocks to DMA if in_queue and DMA has free blocks */
	while (stream->free_tx_dma_blocks) {
		/* get the next buffer from queue */
		ret = k_msgq_get(&stream->in_queue, &buffer, K_NO_WAIT);
		if (ret) {
			/* in_queue is empty, no more blocks to send to DMA */
			ret = 0;
			break;
		}

		/* reload the DMA */
		GDMA_SetSrcAddr(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum, (u32)buffer);

		(stream->free_tx_dma_blocks)--;

		ret = k_msgq_put(&stream->out_queue,
						 &buffer, K_NO_WAIT);
		if (ret != 0) {
			LOG_ERR("buffer %p -> out %p err %d",
					buffer, &stream->out_queue, ret);
			break;
		}

		(*blocks_queued)++;
	}

	irq_unlock(key);
	return ret;
}

uint32_t AudioTxSingleHandler(struct device *dev)
{
	struct i2s_ameba_data *data = dev->data;
	struct SP_GDMA_STRUCT *sp = (struct SP_GDMA_STRUCT *) &data->sp;
	PGDMA_InitTypeDef GDMA_InitStruct;
	GDMA_InitStruct = &sp->SpTxGdmaInitStruct;

	struct stream *stream = &data->tx;
	void *buffer = NULL;
	int ret;
	uint8_t blocks_queued;

	/* Clear Pending ISR */
	GDMA_ClearINT(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);

	ret = k_msgq_get(&stream->out_queue, &buffer, K_NO_WAIT);

	if (ret == 0) {
		/* transmission complete. free the buffer */
		k_mem_slab_free(stream->cfg.mem_slab, buffer);
		(stream->free_tx_dma_blocks)++;
	} else {
		LOG_ERR("no buf in out_queue");
	}

	if (stream->free_tx_dma_blocks > MAX_TX_DMA_BLOCKS) {
		stream->state = I2S_STATE_ERROR;
		LOG_ERR("free_tx_dma_blocks exceeded maximum, now %d",
				stream->free_tx_dma_blocks);
		i2s_tx_stream_disable(dev, false);
		return ret;
	}

	/* Received a STOP trigger, terminate TX immediately */
	if (stream->last_block) {
		stream->state = I2S_STATE_READY;
		LOG_DBG("TX STOPPED last_block set");
		i2s_tx_stream_disable(dev, false);
		return ret;
	}

	if (ret) {
		/* k_msgq_get() returned error, and was not last_block */
		stream->state = I2S_STATE_ERROR;
		i2s_tx_stream_disable(dev, false);
		return ret;
	}

	switch (stream->state) {
	case I2S_STATE_RUNNING:
	case I2S_STATE_STOPPING:
		ret = i2s_ameba_tx_reload_multiple_dma_blocks(dev, &blocks_queued);

		if (ret) {
			stream->state = I2S_STATE_ERROR;
			i2s_tx_stream_disable(dev, false);
			return ret;
		}
		/* dma_start */
		GDMA_Cmd(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum, ENABLE);

		if (blocks_queued ||
			(stream->free_tx_dma_blocks < MAX_TX_DMA_BLOCKS)) {
			return 0;
		} else {
			/* all DMA blocks are free but no blocks were queued */
			if (stream->state == I2S_STATE_STOPPING) {
				/* TX queue has drained */
				stream->state = I2S_STATE_READY;
				LOG_DBG("TX stream has stopped");
			} else {
				stream->state = I2S_STATE_ERROR;
				LOG_ERR("TX Failed to reload DMA");
			}
			i2s_tx_stream_disable(dev, false);
			return ret;
		}

	case I2S_STATE_ERROR:
	default:
		i2s_tx_stream_disable(dev, true);
		return ret;
	}

}

uint32_t AudioTxSingleHandlerExt(struct device *dev)
{
	struct i2s_ameba_data *data = dev->data;
	struct SP_GDMA_STRUCT *sp = (struct SP_GDMA_STRUCT *) &data->sp;
	PGDMA_InitTypeDef GDMA_InitStruct;
	GDMA_InitStruct = &sp->SpTxGdmaInitStructExt;

	struct stream *stream = &data->tx;
	void *buffer = NULL;
	int ret;
	uint8_t blocks_queued;

	/* Clear Pending ISR */
	GDMA_ClearINT(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);

	ret = k_msgq_get(&stream->out_queue, &buffer, K_NO_WAIT);

	if (ret == 0) {
		/* transmission complete. free the buffer */
		k_mem_slab_free(stream->cfg.mem_slab, buffer);
		(stream->free_tx_dma_blocks)++;
	} else {
		LOG_ERR("no buf in out_queue");
	}

	if (stream->free_tx_dma_blocks > MAX_TX_DMA_BLOCKS) {
		stream->state = I2S_STATE_ERROR;
		LOG_ERR("free_tx_dma_blocks exceeded maximum, now %d",
				stream->free_tx_dma_blocks);
		i2s_tx_stream_disable(dev, false);
		return ret;
	}

	/* Received a STOP trigger, terminate TX immediately */
	if (stream->last_block) {
		stream->state = I2S_STATE_READY;
		LOG_DBG("TX STOPPED last_block set");
		i2s_tx_stream_disable(dev, false);
		return ret;
	}

	if (ret) {
		/* k_msgq_get() returned error, and was not last_block */
		stream->state = I2S_STATE_ERROR;
		i2s_tx_stream_disable(dev, false);
		return ret;
	}

	switch (stream->state) {
	case I2S_STATE_RUNNING:
	case I2S_STATE_STOPPING:
		ret = i2s_ameba_tx_reload_multiple_dma_blocks(dev, &blocks_queued);

		if (ret) {
			stream->state = I2S_STATE_ERROR;
			i2s_tx_stream_disable(dev, false);
			return ret;
		}
		/* dma_start */
		GDMA_Cmd(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum, ENABLE);

		if (blocks_queued ||
			(stream->free_tx_dma_blocks < MAX_TX_DMA_BLOCKS)) {
			return 0;
		} else {
			/* all DMA blocks are free but no blocks were queued */
			if (stream->state == I2S_STATE_STOPPING) {
				/* TX queue has drained */
				stream->state = I2S_STATE_READY;
				LOG_DBG("TX stream has stopped");
			} else {
				stream->state = I2S_STATE_ERROR;
				LOG_ERR("TX Failed to reload DMA");
			}
			i2s_tx_stream_disable(dev, false);
			return ret;
		}

	case I2S_STATE_ERROR:
	default:
		i2s_tx_stream_disable(dev, true);
		return ret;
	}

}

uint32_t AudioRxSingleHandler(const struct device *dev)
{
	struct i2s_ameba_data *data = dev->data;
	struct SP_GDMA_STRUCT *sp = (struct SP_GDMA_STRUCT *) &data->sp;
	PGDMA_InitTypeDef GDMA_InitStruct;
	GDMA_InitStruct = &sp->SpRxGdmaInitStruct;
	struct stream *stream = &data->rx;
	void *buffer;
	int ret;

	/* Clear Pending ISR */
	GDMA_ClearINT(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);

	switch (stream->state) {
	case I2S_STATE_STOPPING:
	case I2S_STATE_RUNNING:
		/* retrieve buffer from input queue */
		ret = k_msgq_get(&stream->in_queue, &buffer, K_NO_WAIT);
		__ASSERT_NO_MSG(ret == 0);

		/* put buffer to output queue */
		ret = k_msgq_put(&stream->out_queue, &buffer, K_NO_WAIT);
		if (ret != 0) {
			LOG_ERR("buffer %p -> out_queue %p err %d",
					buffer,
					&stream->out_queue, ret);
			i2s_rx_stream_disable(dev, false, false);
			stream->state = I2S_STATE_ERROR;
			return ret;
		}
		if (stream->state == I2S_STATE_RUNNING) {
			/* allocate new buffer for next audio frame */
			ret = k_mem_slab_alloc(stream->cfg.mem_slab,
								   &buffer, K_NO_WAIT);
			if (ret != 0) {
				LOG_ERR("buffer alloc from slab %p err %d",
						stream->cfg.mem_slab, ret);
				i2s_rx_stream_disable(dev, false, false);
				stream->state = I2S_STATE_ERROR;
			} else {

				/* reload DMA */
				GDMA_SetDstAddr(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum, (u32)buffer);

				/* put buffer in input queue */
				ret = k_msgq_put(&stream->in_queue,
								 &buffer, K_NO_WAIT);
				if (ret != 0) {
					LOG_ERR("%p -> in_queue %p err %d",
							buffer, &stream->in_queue,
							ret);
				}

				/* dma_start */
				GDMA_Cmd(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum, ENABLE);
			}
		} else {
			i2s_rx_stream_disable(dev, true, false);
			/* Received a STOP/DRAIN trigger */
			stream->state = I2S_STATE_READY;
		}
		break;
	case I2S_STATE_ERROR:
		i2s_rx_stream_disable(dev, true, true);
		break;
	}

	return 0;
}

uint32_t AudioRxSingleHandlerExt(const struct device *dev)
{
	struct i2s_ameba_data *data = dev->data;
	struct SP_GDMA_STRUCT *sp = (struct SP_GDMA_STRUCT *) &data->sp;
	PGDMA_InitTypeDef GDMA_InitStruct;
	GDMA_InitStruct = &sp->SpRxGdmaInitStructExt;
	struct stream *stream = &data->rx;
	void *buffer;
	int ret;

	/* Clear Pending ISR */
	GDMA_ClearINT(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);

	switch (stream->state) {
	case I2S_STATE_STOPPING:
	case I2S_STATE_RUNNING:
		/* retrieve buffer from input queue */
		ret = k_msgq_get(&stream->in_queue, &buffer, K_NO_WAIT);
		__ASSERT_NO_MSG(ret == 0);

		/* put buffer to output queue */
		ret = k_msgq_put(&stream->out_queue, &buffer, K_NO_WAIT);
		if (ret != 0) {
			LOG_ERR("buffer %p -> out_queue %p err %d",
					buffer,
					&stream->out_queue, ret);
			i2s_rx_stream_disable(dev, false, false);
			stream->state = I2S_STATE_ERROR;
			return ret;
		}
		if (stream->state == I2S_STATE_RUNNING) {
			/* allocate new buffer for next audio frame */
			ret = k_mem_slab_alloc(stream->cfg.mem_slab,
								   &buffer, K_NO_WAIT);
			if (ret != 0) {
				LOG_ERR("buffer alloc from slab %p err %d",
						stream->cfg.mem_slab, ret);
				i2s_rx_stream_disable(dev, false, false);
				stream->state = I2S_STATE_ERROR;
			} else {

				/* reload DMA */
				GDMA_SetDstAddr(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum, (u32)buffer);

				/* put buffer in input queue */
				ret = k_msgq_put(&stream->in_queue,
								 &buffer, K_NO_WAIT);
				if (ret != 0) {
					LOG_ERR("%p -> in_queue %p err %d",
							buffer, &stream->in_queue,
							ret);
				}

				/* dma_start */
				GDMA_Cmd(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum, ENABLE);
			}
		} else {
			i2s_rx_stream_disable(dev, true, false);
			/* Received a STOP/DRAIN trigger */
			stream->state = I2S_STATE_READY;
		}
		break;
	case I2S_STATE_ERROR:
		i2s_rx_stream_disable(dev, true, true);
		break;
	}

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
	const struct i2s_ameba_cfg *const cfg = dev->config;
	uint8_t sp_dir;
	struct stream *stream;

	if ((data->tx.state != I2S_STATE_NOT_READY) &&
		(data->tx.state != I2S_STATE_READY) &&
		(data->rx.state != I2S_STATE_NOT_READY) &&
		(data->rx.state != I2S_STATE_READY)) {
		LOG_ERR("invalid state tx(%u) rx(%u)",
				data->tx.state,
				data->rx.state);
		if (dir == I2S_DIR_TX) {
			data->tx.state = I2S_STATE_NOT_READY;
		} else {
			data->rx.state = I2S_STATE_NOT_READY;
		}
		return -EINVAL;
	}

	if (dir == I2S_DIR_RX) {
		stream = &data->rx;
		sp_dir = SP_DIR_RX;
		AUDIO_SP_Deinit(cfg->index, SP_DIR_RX);
	} else if (dir == I2S_DIR_TX) {
		stream = &data->tx;
		sp_dir = SP_DIR_TX;
		AUDIO_SP_Deinit(cfg->index, SP_DIR_TX);
	} else {
		LOG_ERR("Unsupported I2S direction");
		data->tx.state = I2S_STATE_NOT_READY;
		data->rx.state = I2S_STATE_NOT_READY;
		return -EINVAL;
	}

	memcpy(&stream->cfg, i2s_cfg, sizeof(struct i2s_config));

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
	AUDIO_ClockParams Clock_Params;
	AUDIO_InitParams Init_Params;
	bool slave;

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

	if (cfg->clock_mode == I2S_CLOCK_XTAL40M) {
		xtal_enable = 1;
	} else {
		xtal_enable = 0;
	}

	/* get MCLK div */
	Audio_Clock_Choose(xtal_enable, &Init_Params, &Clock_Params);

	AUDIO_SP_StructInit(&SP_InitStruct);

	/*
	 * set I2S Data Format
	 * 16-bit data extended on 32-bit channel length excluded
	 */
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

	if ((i2s_cfg->options & I2S_OPT_PINGPONG) == I2S_OPT_PINGPONG) {
		LOG_ERR("Ping-pong mode not supported");
		if (dir == I2S_DIR_TX) {
			data->tx.state = I2S_STATE_NOT_READY;
		} else {
			data->rx.state = I2S_STATE_NOT_READY;
		}
		return -ENOTSUP;
	}

	/* set I2S Standard */
	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		SP_InitStruct.SP_SelDataFormat = i2s_cfg->format;
		break;

	case I2S_FMT_DATA_FORMAT_PCM_SHORT:                           /* PCM_A */
		SP_InitStruct.SP_SelDataFormat = i2s_cfg->format + 1U;
		break;

	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		LOG_ERR("Unsupported I2S data format");
		if (dir == I2S_DIR_TX) {
			data->tx.state = I2S_STATE_NOT_READY;
		} else {
			data->rx.state = I2S_STATE_NOT_READY;
		}
		return -EINVAL;

	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		SP_InitStruct.SP_SelDataFormat = i2s_cfg->format - 2U;
		break;

	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		LOG_ERR("Unsupported I2S data format");
		if (dir == I2S_DIR_TX) {
			data->tx.state = I2S_STATE_NOT_READY;
		} else {
			data->rx.state = I2S_STATE_NOT_READY;
		}
		return -EINVAL;

	default:
		LOG_ERR("Unsupported I2S data format");
		if (dir == I2S_DIR_TX) {
			data->tx.state = I2S_STATE_NOT_READY;
		} else {
			data->rx.state = I2S_STATE_NOT_READY;
		}
		return -EINVAL;
	}

	if (i2s_cfg->channels == 1U) {
		SP_InitStruct.SP_SelI2SMonoStereo = SP_CH_MONO;
	} else {
		SP_InitStruct.SP_SelI2SMonoStereo = SP_CH_STEREO;
	}

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
	}

	AUDIO_SP_Init(cfg->index, sp_dir, &SP_InitStruct);

	if (dir == I2S_DIR_TX) {
		data->tx.state = I2S_STATE_READY;
	} else {
		data->rx.state = I2S_STATE_READY;
	}

	if (i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE ||
		i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE) {
		slave = true;      /* slave */
	} else {
		slave = false;     /* master */
	}

	AUDIO_SP_SetMasterSlave(cfg->index, slave);  /* master:0 slave:1 */

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

static int i2s_tx_stream_start(const struct device *dev)
{
	int ret = 0;
	void *buffer;
	struct i2s_ameba_data *data = dev->data;
	struct stream *stream = &data->tx;
	const struct i2s_ameba_cfg *cfg = dev->config;

	struct SP_GDMA_STRUCT *sp = &data->sp;
	memset(sp, 0, sizeof(struct SP_GDMA_STRUCT));

	/* retrieve buffer from input queue */
	ret = k_msgq_get(&stream->in_queue, &buffer, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("No buffer in input queue to start");
		return -EIO;
	}

	/* Driver keeps track of how many DMA blocks can be loaded to the DMA */
	stream->free_tx_dma_blocks = MAX_TX_DMA_BLOCKS;

	(stream->free_tx_dma_blocks)--;

	/* put buffer in output queue */
	ret = k_msgq_put(&stream->out_queue, &buffer, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("failed to put buffer in output queue");
		return ret;
	}

	uint8_t blocks_queued;

	ret = i2s_ameba_tx_reload_multiple_dma_blocks(dev, &blocks_queued);
	if (ret) {
		LOG_ERR("i2s_ameba_tx_reload_multiple_dma_blocks() failed (%d)", ret);
		return ret;
	}

	/* DMA enable and register irq */
	if (cfg->fifo_num == 0 || cfg->fifo_num == 1) {
		AUDIO_SP_TXGDMA_Init(cfg->index, GDMA_INT, &sp->SpTxGdmaInitStruct, (void *)dev,
							 (IRQ_FUN)AudioTxSingleHandler, (u8 *)buffer, (uint32_t)stream->cfg.block_size);
		irq_connect_dynamic(GDMA0_CHANNEL0_IRQ + sp->SpTxGdmaInitStruct.GDMA_ChNum, 0,
							(void *)AudioTxSingleHandler, (void *)dev, 0);
		irq_enable(GDMA0_CHANNEL0_IRQ + sp->SpTxGdmaInitStruct.GDMA_ChNum);
	} else {
		AUDIO_SP_TXGDMA_Init(cfg->index, GDMA_INT, &sp->SpTxGdmaInitStruct, (void *)dev,
							 (IRQ_FUN)AudioTxSingleHandler, (u8 *)buffer, (uint32_t)stream->cfg.block_size);
		irq_connect_dynamic(GDMA0_CHANNEL0_IRQ + sp->SpTxGdmaInitStruct.GDMA_ChNum, 2,
							(void *)AudioTxSingleHandler, (void *)dev, 0);
		irq_enable(GDMA0_CHANNEL0_IRQ + sp->SpTxGdmaInitStruct.GDMA_ChNum);
		AUDIO_SP_TXGDMA_Init(cfg->index, GDMA_EXT, &sp->SpTxGdmaInitStructExt, (void *)dev,
							 (IRQ_FUN)AudioTxSingleHandlerExt, (u8 *)buffer, (uint32_t)stream->cfg.block_size);
		irq_connect_dynamic(GDMA0_CHANNEL0_IRQ + sp->SpTxGdmaInitStructExt.GDMA_ChNum, 2,
							(void *)AudioTxSingleHandlerExt, (void *)dev, 0);
		irq_enable(GDMA0_CHANNEL0_IRQ + sp->SpTxGdmaInitStructExt.GDMA_ChNum);
	}

	/* Enable Tx */
	AUDIO_SP_TXStart(cfg->index, ENABLE);

	return 0;
}

static int i2s_rx_stream_start(const struct device *dev)
{
	int ret = 0;
	void *buffer;
	struct i2s_ameba_data *data = dev->data;
	struct stream *stream = &data->rx;
	const struct i2s_ameba_cfg *cfg = dev->config;
	uint8_t num_of_bufs;

	PGDMA_InitTypeDef GDMA_InitStruct;
	GDMA_InitStruct = &data->sp.SpRxGdmaInitStruct;

	struct SP_GDMA_STRUCT *sp = &data->sp;
	memset(sp, 0, sizeof(struct SP_GDMA_STRUCT));

	num_of_bufs = k_mem_slab_num_free_get(stream->cfg.mem_slab);

	/*
	 * Need at least NUM_DMA_BLOCKS_RX_PREP buffers on the RX memory slab
	 * for reliable DMA reception.
	 */
	if (num_of_bufs < NUM_DMA_BLOCKS_RX_PREP) {
		return -EINVAL;
	}

	/* allocate 1st receive buffer from SLAB */
	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &buffer,
						   K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("buffer alloc from mem_slab failed (%d)", ret);
		return ret;
	}

	/* put buffer in input queue */
	ret = k_msgq_put(&stream->in_queue, &buffer, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("failed to put buffer in input queue, ret1 %d", ret);
		return ret;
	}

	/* prep DMA for each of remaining (NUM_DMA_BLOCKS_RX_PREP-1) buffers */
	for (int i = 0; i < NUM_DMA_BLOCKS_RX_PREP - 1; i++) {

		/* allocate receive buffer from SLAB */
		ret = k_mem_slab_alloc(stream->cfg.mem_slab, &buffer,
							   K_NO_WAIT);
		if (ret != 0) {
			LOG_ERR("buffer alloc from mem_slab failed (%d)", ret);
			return ret;
		}

		/* DMA reload */
		GDMA_SetDstAddr(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum, (u32)buffer);

		/* put buffer in input queue */
		ret = k_msgq_put(&stream->in_queue, &buffer, K_NO_WAIT);
		if (ret != 0) {
			LOG_ERR("failed to put buffer in input queue, ret2 %d", ret);
			return ret;
		}
	}

	/* DMA enable and register irq */
	if (cfg->fifo_num == 0 || cfg->fifo_num == 1) {
		AUDIO_SP_RXGDMA_Init(cfg->index, GDMA_INT, &sp->SpRxGdmaInitStruct, (void *)dev,
							 (IRQ_FUN)AudioRxSingleHandler, (u8 *)buffer, (uint32_t)stream->cfg.block_size);
		irq_connect_dynamic(GDMA0_CHANNEL0_IRQ + sp->SpRxGdmaInitStruct.GDMA_ChNum, 0,
							(void *)AudioRxSingleHandler, (void *)dev, 0);
		irq_enable(GDMA0_CHANNEL0_IRQ + sp->SpRxGdmaInitStruct.GDMA_ChNum);

	} else {
		AUDIO_SP_RXGDMA_Init(cfg->index, GDMA_INT, &sp->SpRxGdmaInitStruct, (void *)dev,
							 (IRQ_FUN)AudioRxSingleHandler, (u8 *)buffer, (uint32_t)stream->cfg.block_size);
		irq_connect_dynamic(GDMA0_CHANNEL0_IRQ + sp->SpRxGdmaInitStruct.GDMA_ChNum, 2,
							(void *)AudioRxSingleHandler, (void *)dev, 0);
		irq_enable(GDMA0_CHANNEL0_IRQ + sp->SpRxGdmaInitStruct.GDMA_ChNum);
		AUDIO_SP_RXGDMA_Init(cfg->index, GDMA_EXT, &sp->SpRxGdmaInitStructExt, (void *)dev,
							 (IRQ_FUN)AudioRxSingleHandlerExt, (u8 *)buffer, (uint32_t)stream->cfg.block_size);
		irq_connect_dynamic(GDMA0_CHANNEL0_IRQ + sp->SpRxGdmaInitStructExt.GDMA_ChNum, 2,
							(void *)AudioRxSingleHandlerExt, (void *)dev, 0);
		irq_enable(GDMA0_CHANNEL0_IRQ + sp->SpRxGdmaInitStructExt.GDMA_ChNum);
	}

	/* Enable Rx */
	AUDIO_SP_RXStart(cfg->index, ENABLE);
	return 0;
}


static int i2s_ameba_trigger(const struct device *dev, enum i2s_dir dir,
							 enum i2s_trigger_cmd cmd)
{
	struct i2s_ameba_data *data = dev->data;
	const struct i2s_ameba_cfg *cfg = dev->config;
	struct stream *stream;
	unsigned int key;
	int ret = 0;

	if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else {
		LOG_ERR("Unsupported I2S direction");
		return -ENOSYS;
	}

	key = irq_lock();
	switch (cmd) {
	case I2S_TRIGGER_START:
		if (stream->state != I2S_STATE_READY) {
			LOG_ERR("START trigger: invalid state %u",
					stream->state);
			ret = -EIO;
			break;
		}
		/* SPORT DMA handshake */
		AUDIO_SP_DmaCmd(cfg->index, ENABLE);

		if (dir == I2S_DIR_TX) {
			ret = i2s_tx_stream_start(dev);
		} else {
			ret = i2s_rx_stream_start(dev);
		}
		if (ret < 0) {
			LOG_DBG("START trigger failed %d", ret);
			ret = -EIO;
			break;
		}
		stream->state = I2S_STATE_RUNNING;
		stream->last_block = false;
		break;

	case I2S_TRIGGER_DROP:
		if (stream->state == I2S_STATE_NOT_READY) {
			LOG_ERR("DROP trigger: invalid state %d",
					stream->state);
			ret = -EIO;
			break;
		}

		stream->state = I2S_STATE_READY;
		if (dir == I2S_DIR_TX) {
			i2s_tx_stream_disable(dev, true);
		} else {
			i2s_rx_stream_disable(dev, true, true);
		}
		break;

	case I2S_TRIGGER_STOP:
		if (stream->state != I2S_STATE_RUNNING) {
			LOG_ERR("STOP trigger: invalid state %d", stream->state);
			ret = -EIO;
			break;
		}

		stream->state = I2S_STATE_STOPPING;
		stream->last_block = true;
		break;

	case I2S_TRIGGER_DRAIN:
		if (stream->state != I2S_STATE_RUNNING) {
			LOG_ERR("DRAIN/STOP trigger: invalid state %d",
					stream->state);
			ret = -EIO;
			break;
		}

		stream->state = I2S_STATE_STOPPING;
		break;

	case I2S_TRIGGER_PREPARE:
		if (stream->state != I2S_STATE_ERROR) {
			LOG_ERR("PREPARE trigger: invalid state %d",
					stream->state);
			ret = -EIO;
			break;
		}
		stream->state = I2S_STATE_READY;
		if (dir == I2S_DIR_TX) {
			i2s_tx_stream_disable(dev, true);
		} else {
			i2s_rx_stream_disable(dev, true, true);
		}
		break;

	default:
		LOG_ERR("Unsupported trigger cmd");
		ret = -EINVAL;
		break;
	}

	irq_unlock(key);
	return ret;
}

static int i2s_ameba_read(const struct device *dev, void **mem_block,
						  size_t *size)
{
	struct i2s_ameba_data *data = dev->data;
	struct stream *stream = &data->rx;

	void *buffer;
	int status, ret = 0;
	if (stream->state == I2S_STATE_NOT_READY) {
		LOG_ERR("invalid state %d", stream->state);
		return -EIO;
	}

	status = k_msgq_get(&stream->out_queue, &buffer,
						SYS_TIMEOUT_MS(stream->cfg.timeout));
	if (status != 0) {
		if (stream->state == I2S_STATE_ERROR) {
			ret = -EIO;
		} else {
			LOG_DBG("need retry");
			ret = -EAGAIN;
		}
		return ret;
	}

	*mem_block = buffer;
	*size = stream->cfg.block_size;
	return 0;
}

static int i2s_ameba_write(const struct device *dev, void *mem_block,
						   size_t size)
{
	struct i2s_ameba_data *data = dev->data;
	struct stream *stream = &data->tx;
	int ret = 0;

	if (stream->state != I2S_STATE_RUNNING &&
		stream->state != I2S_STATE_READY) {
		LOG_ERR("invalid state (%d)", stream->state);
		return -EIO;
	}

	ret = k_msgq_put(&stream->in_queue, &mem_block,
					 SYS_TIMEOUT_MS(stream->cfg.timeout));
	if (ret) {
		LOG_DBG("k_msgq_put returned code %d", ret);
		return ret;
	}

	return ret;
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
	struct i2s_ameba_data *data = dev->data;
	int ret;

	/*float ppm = 234.35;*/
	/*float cal_ppm = 0;*/

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
	/* Initialize the buffer queues */
	k_msgq_init(&data->tx.in_queue, (char *)data->tx_in_msgs,
				sizeof(void *), CONFIG_I2S_TX_BLOCK_COUNT);
	k_msgq_init(&data->rx.in_queue, (char *)data->rx_in_msgs,
				sizeof(void *), CONFIG_I2S_RX_BLOCK_COUNT);
	k_msgq_init(&data->tx.out_queue, (char *)data->tx_out_msgs,
				sizeof(void *), CONFIG_I2S_TX_BLOCK_COUNT);
	k_msgq_init(&data->rx.out_queue, (char *)data->rx_out_msgs,
				sizeof(void *), CONFIG_I2S_RX_BLOCK_COUNT);

	AUDIO_SP_Reset(cfg->index);

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
	static struct i2s_ameba_data i2s_ameba_data_##n = {        		\
		                                   				\
	}; 											\
	DEVICE_DT_INST_DEFINE(n,						\
				&i2s_ameba_initialize, NULL,			\
				&i2s_ameba_data_##n,			\
				&i2s_ameba_config_##n, POST_KERNEL,		\
				CONFIG_I2S_INIT_PRIORITY, &i2s_ameba_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2S_AMEBA_INIT)
