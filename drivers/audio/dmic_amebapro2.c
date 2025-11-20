/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek AmebaPro2 DMIC
 */

#define DT_DRV_COMPAT realtek_amebapro2_dmic

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#include "hal_audio.h"
#include "hal_sport.h"
#include "hal_cache.h"
#include "hal_pinmux.h"

LOG_MODULE_REGISTER(dmic_amebapro2, LOG_LEVEL_DBG);

/* DMIC pin definitions */
#define DMIC_CLK_PIN  PIN_D16
#define DMIC_DATA_PIN PIN_D18

/* Hardware DMA page size: 640 bytes = 20ms @ 16KHz, 40ms @ 8KHz */
#define AUDIO_DMA_PAGE_SIZE 640
#define AUDIO_DMA_PAGE_NUM  4
#define AUDIO_DROP_NUM      2 /* Drop first N frames to prevent invalid data */

/* EQ coefficient structure (5-band biquad IIR filter) */
typedef struct eq_cof_s {
	uint32_t eq_enable;
	uint32_t eq_h0;
	uint32_t eq_b1;
	uint32_t eq_b2;
	uint32_t eq_a1;
	uint32_t eq_a2;
} eq_cof_t;

/* Audio configuration parameters */
typedef struct audio_param_s {
	uint8_t adc_gain;
	uint8_t dac_gain;
	audio_mic_gain_t mic_gain;   /* Microphone analog gain: 0dB/20dB/30dB/40dB */
	audio_dmic_gain_t dmic_gain; /* DMIC boost gain: 0dB/12dB/24dB/36dB */

	bool hpf_enable;
	audio_hpf_fc_t hpf_fc; /* HPF cutoff frequency */

	bool adc_mute;
	bool dac_mute;

	eq_cof_t mic_l_eq[5];
	eq_cof_t mic_r_eq[5];
} audio_params_t;

/* Default configuration for DMIC */
static const audio_params_t default_audio_config = {
	.adc_gain = 0x66,
	.dac_gain = 0xAF,
	.mic_gain = AUDIO_MIC_0DB,
	.dmic_gain = AUDIO_DMIC_BOOST_12DB,
	.hpf_enable = true,
	.hpf_fc = AUDIO_HPF_FS_0,
	.adc_mute = false,
	.dac_mute = false,
	/* Left channel EQ: HPF 200Hz @ 8kHz sample rate */
	.mic_l_eq[0] = {1, 0x1ca2925, 0x1c000000, 0x2000000, 0x38ea551, 0x1e6600bf},
	/* Right channel EQ: HPF 200Hz @ 8kHz sample rate */
	.mic_r_eq[0] = {1, 0x1ca2925, 0x1c000000, 0x2000000, 0x38ea551, 0x1e6600bf},
};

struct dmic_amebapro2_config {
	void (*irq_config_func)(const struct device *dev);
};

/* Ring buffer for tracking DMA page pointers */
#define RX_PAGE_QUEUE_SIZE AUDIO_DMA_PAGE_NUM
struct rx_page_queue {
	uint8_t *pages[RX_PAGE_QUEUE_SIZE];
	uint32_t write_idx;
	uint32_t read_idx;
	uint32_t count;
};

struct dmic_amebapro2_data {
	enum dmic_state state;
	struct k_mem_slab *pcm_mem_slab;
	hal_audio_adapter_t audio_adapter; /* HAL audio adapter */
	hal_sport_adapter_t sport_adapter; /* HAL SPORT adapter */
	uint8_t *rx_buf;                   /* pages for DMA */
	struct k_sem rx_sem;               /* Semaphore for page ready notification */
	struct rx_page_queue rx_queue;     /* Queue of DMA pages ready to read */
	uint32_t drop_count;               /* Number of initial frames to drop */
	uint8_t num_channels;              /* 1 = mono, 2 = stereo */
};

static void dmic_rx_callback(uint32_t arg, uint8_t *pbuf)
{
	struct dmic_amebapro2_data *data = (struct dmic_amebapro2_data *)arg;
	hal_audio_adapter_t *audio_adapter = &data->audio_adapter;
	uint32_t ret;

	/* Drop first N frames to prevent pop sound and invalid data */
	if (data->drop_count > 0) {
		data->drop_count--;
		/* Zero the DMA buffer */
		memset(pbuf, 0, AUDIO_DMA_PAGE_SIZE);
		dcache_clean_by_addr((uint32_t *)pbuf, AUDIO_DMA_PAGE_SIZE);
		ret = hal_audio_sport_rx_page_recv(audio_adapter);
		if (ret != HAL_OK) {
			LOG_WRN("Failed to return dropped page (ret: 0x%x)", ret);
		}
		return;
	}

	/* Invalidate cache for DMA buffer before reading */
	dcache_invalidate_by_addr((uint32_t *)pbuf, AUDIO_DMA_PAGE_SIZE);

	/* Enqueue the DMA page pointer */
	if (data->rx_queue.count < RX_PAGE_QUEUE_SIZE) {
		data->rx_queue.pages[data->rx_queue.write_idx] = pbuf;
		data->rx_queue.write_idx = (data->rx_queue.write_idx + 1) % RX_PAGE_QUEUE_SIZE;
		data->rx_queue.count++;

		k_sem_give(&data->rx_sem);
	} else {
		/* Queue full - return page immediately to prevent DMA stall */
		ret = hal_audio_sport_rx_page_recv(audio_adapter);
		if (ret != HAL_OK) {
			LOG_WRN("Failed to return page when queue full (ret: 0x%x)", ret);
		}
	}
}

static void configure_dmic_pins(void)
{
	/* Configure PD_16 as DMIC_CLK and PD_18 as DMIC_DATA */
	hal_pinmux_register(DMIC_CLK_PIN, PID_DMIC);
	hal_pinmux_register(DMIC_DATA_PIN, PID_DMIC);
}

static void configure_audio_eq(struct dmic_amebapro2_data *data, bool stereo)
{
	hal_audio_adapter_t *adapter = &data->audio_adapter;
	hal_eq_params_t eq_params;

	/* Disable ADC CLK to prevent EQ setting error */
	hal_audio_adc_clk(adapter, FALSE, FALSE);

	/* Configure left channel EQ */
	for (uint32_t j = 0; j < 5; j++) {
		if (default_audio_config.mic_l_eq[j].eq_enable) {
			eq_params.h0 = default_audio_config.mic_l_eq[j].eq_h0;
			eq_params.b1 = default_audio_config.mic_l_eq[j].eq_b1;
			eq_params.b2 = default_audio_config.mic_l_eq[j].eq_b2;
			eq_params.a1 = default_audio_config.mic_l_eq[j].eq_a1;
			eq_params.a2 = default_audio_config.mic_l_eq[j].eq_a2;
			hal_audio_input_l_eq(adapter, (audio_eq_t)j, TRUE, &eq_params);
		} else {
			eq_params.h0 = 0x2000000;
			eq_params.b1 = 0;
			eq_params.b2 = 0;
			eq_params.a1 = 0;
			eq_params.a2 = 0;
			hal_audio_input_l_eq(adapter, (audio_eq_t)j, FALSE, &eq_params);
		}
	}

	/* Configure right channel EQ for stereo */
	if (stereo) {
		for (uint32_t j = 0; j < 5; j++) {
			if (default_audio_config.mic_r_eq[j].eq_enable) {
				eq_params.h0 = default_audio_config.mic_r_eq[j].eq_h0;
				eq_params.b1 = default_audio_config.mic_r_eq[j].eq_b1;
				eq_params.b2 = default_audio_config.mic_r_eq[j].eq_b2;
				eq_params.a1 = default_audio_config.mic_r_eq[j].eq_a1;
				eq_params.a2 = default_audio_config.mic_r_eq[j].eq_a2;
				hal_audio_input_r_eq(adapter, (audio_eq_t)j, TRUE, &eq_params);
			} else {
				eq_params.h0 = 0x2000000;
				eq_params.b1 = 0;
				eq_params.b2 = 0;
				eq_params.a1 = 0;
				eq_params.a2 = 0;
				hal_audio_input_r_eq(adapter, (audio_eq_t)j, FALSE, &eq_params);
			}
		}
	}

	hal_audio_adc_clk(adapter, TRUE, stereo ? TRUE : FALSE);
}

static int dmic_amebapro2_configure(const struct device *dev, struct dmic_cfg *cfg)
{
	struct dmic_amebapro2_data *const data = dev->data;
	hal_audio_adapter_t *audio_adapter = &data->audio_adapter;
	audio_sample_rate_t sample_rate;
	audio_word_len_t word_length;
	hal_audio_dma_params_t dma_params;
	size_t buffer_size;
	uint8_t *new_buf;
	bool stereo;

	if (data->state != DMIC_STATE_INITIALIZED && data->state != DMIC_STATE_CONFIGURED) {
		LOG_ERR("Cannot configure in state: %d", data->state);
		return -EPERM;
	}

	if (!cfg || !cfg->streams) {
		LOG_ERR("Invalid configuration");
		return -EINVAL;
	}

	if (cfg->channel.req_num_chan != 1 && cfg->channel.req_num_chan != 2) {
		LOG_ERR("Invalid channel count: %d (must be 1 or 2)", cfg->channel.req_num_chan);
		return -EINVAL;
	}

	if (!cfg->streams->mem_slab) {
		LOG_ERR("Memory slab is NULL");
		return -EINVAL;
	}

	LOG_INF("Configuring AmebaPro2 DMIC: %d channel(s), %d-bit, %d Hz",
		cfg->channel.req_num_chan, cfg->streams->pcm_width, cfg->streams->pcm_rate);

	/* Convert sample rate */
	switch (cfg->streams->pcm_rate) {
	case 8000:
		sample_rate = AUDIO_SR_8KHZ;
		break;
	case 16000:
		sample_rate = AUDIO_SR_16KHZ;
		break;
	case 32000:
		sample_rate = AUDIO_SR_32KHZ;
		break;
	case 44100:
		sample_rate = AUDIO_SR_44p1KHZ;
		break;
	case 48000:
		sample_rate = AUDIO_SR_48KHZ;
		break;
	default:
		LOG_ERR("Unsupported sample rate: %d", cfg->streams->pcm_rate);
		return -EINVAL;
	}

	/* Convert word length */
	switch (cfg->streams->pcm_width) {
	case 16:
		word_length = AUDIO_WL_16;
		break;
	case 24:
		word_length = AUDIO_WL_24;
		break;
	default:
		LOG_ERR("Unsupported word length: %d", cfg->streams->pcm_width);
		return -EINVAL;
	}

	stereo = (cfg->channel.req_num_chan == 2);
	data->num_channels = cfg->channel.req_num_chan;

	/* Allocate DMA buffer (32-byte aligned) */
	buffer_size = AUDIO_DMA_PAGE_NUM * AUDIO_DMA_PAGE_SIZE;
	new_buf = k_aligned_alloc(32, buffer_size);
	if (!new_buf) {
		LOG_ERR("Failed to allocate aligned RX buffer (%d bytes)", buffer_size);
		return -ENOMEM;
	}
	memset(new_buf, 0, buffer_size);

	/* Verify alignment */
	if (((uintptr_t)new_buf & 0x1F) != 0) {
		LOG_ERR("Buffer not 32-byte aligned: %p", new_buf);
		k_free(new_buf);
		return -ENOMEM;
	}

	/* Free old DMA buffer if reconfiguring */
	if (data->rx_buf) {
		k_free(data->rx_buf);
	}

	/* Update configuration */
	data->rx_buf = new_buf;
	data->pcm_mem_slab = cfg->streams->mem_slab;

	/* Initialize RX page queue */
	data->rx_queue.write_idx = 0;
	data->rx_queue.read_idx = 0;
	data->rx_queue.count = 0;

	/* Configure DMIC pins */
	configure_dmic_pins();

	/* Initialize audio HAL */
	hal_audio_init(audio_adapter, AUDIO_POWER_2p8V);
	hal_audio_sport_init(audio_adapter);
	hal_audio_output_power(audio_adapter, AUDIO_OUTPUT_DISABLE);
	if (stereo) {
		hal_audio_input_power(audio_adapter, AUDIO_STEREO_DMIC);
	} else {
		hal_audio_input_power(audio_adapter, AUDIO_LEFT_DMIC);
	}

	/* Re-enable IRQ */
	irq_enable(DT_INST_IRQN(0));

	/* Set audio DMA buffer */
	dma_params.page_num = AUDIO_DMA_PAGE_NUM;
	dma_params.page_size = AUDIO_DMA_PAGE_SIZE;
	dma_params.tx_buf = NULL;
	dma_params.rx_buf = data->rx_buf;
	hal_audio_sport_buf(audio_adapter, dma_params);

	/* Set digital volumes */
	hal_audio_adc_l_dvol(audio_adapter, default_audio_config.adc_gain);
	if (stereo) {
		hal_audio_adc_r_dvol(audio_adapter, default_audio_config.adc_gain);
	}
	hal_audio_dac_l_dvol(audio_adapter, default_audio_config.dac_gain);

	/* Register DMA callback */
	hal_audio_sport_rx_cb_handler(audio_adapter, (audio_sport_irq_cb_t)dmic_rx_callback,
				      (uint32_t *)data);

	/* Configure basic audio settings */
	hal_audio_sidetone_mixer(audio_adapter, ENABLE);
	hal_audio_format(audio_adapter, AUDIO_FORMAT_I2S);
	hal_audio_sck_inv(audio_adapter, DISABLE);
	hal_audio_loopback(audio_adapter, DISABLE);
	hal_audio_tx_ch(audio_adapter, AUDIO_L_R);
	hal_audio_digital_rst(audio_adapter, DISABLE);

	hal_audio_rate(audio_adapter, sample_rate, sample_rate);
	hal_audio_length(audio_adapter, word_length, word_length);

	hal_audio_dmic_input_clk(audio_adapter, AUDIO_DMIC_2p5M);
	hal_audio_dmic_latch(audio_adapter, AUDIO_DIMC_RISING, AUDIO_DIMC_RISING);

	/* Set SPORT parameters */
	hal_audio_sport_tx_params(audio_adapter, AUDIO_MONO, word_length, sample_rate);
	hal_audio_sport_rx_params(audio_adapter, stereo ? AUDIO_STEREO : AUDIO_MONO, word_length,
				  sample_rate);

	/* Set byte swap for 24-bit mode */
	if (word_length == AUDIO_WL_24) {
		hal_audio_sport_tx_byte_swap(audio_adapter, TRUE);
		hal_audio_sport_rx_byte_swap(audio_adapter, TRUE);
	} else {
		hal_audio_sport_tx_byte_swap(audio_adapter, FALSE);
		hal_audio_sport_rx_byte_swap(audio_adapter, FALSE);
	}

	/* Use (DMA page count -1) because occur RX interrupt in first */
	for (int i = 0; i < (AUDIO_DMA_PAGE_NUM - 1); i++) {
		hal_audio_sport_rx_page_recv(audio_adapter);
	}

	/* Set up HPF for microphone */
	hal_audio_adc_l_hpf(audio_adapter, default_audio_config.hpf_enable,
			    default_audio_config.hpf_fc);
	if (stereo) {
		hal_audio_adc_r_hpf(audio_adapter, default_audio_config.hpf_enable,
				    default_audio_config.hpf_fc);
	}

	/* Set up mic EQ (5-band biquad IIR filter) */
	configure_audio_eq(data, stereo);

	/* Configure analog gain and DMIC boost */
	hal_audio_mic_boost(audio_adapter, 1, default_audio_config.mic_gain);
	hal_audio_dmic_l_gain(audio_adapter, default_audio_config.dmic_gain);
	if (stereo) {
		hal_audio_dmic_r_gain(audio_adapter, default_audio_config.dmic_gain);
	}

	/* Set digital mute */
	hal_audio_adc_l_dmute(audio_adapter, default_audio_config.adc_mute);
	if (stereo) {
		hal_audio_adc_r_dmute(audio_adapter, default_audio_config.adc_mute);
	}
	hal_audio_dac_l_dmute(audio_adapter, default_audio_config.dac_mute);

	data->state = DMIC_STATE_CONFIGURED;
	return 0;
}

static int dmic_amebapro2_trigger(const struct device *dev, enum dmic_trigger cmd)
{
	struct dmic_amebapro2_data *const data = dev->data;
	hal_audio_adapter_t *audio_adapter = &data->audio_adapter;
	enum dmic_state next_state;

	switch (cmd) {
	case DMIC_TRIGGER_START:
		if (data->state != DMIC_STATE_CONFIGURED && data->state != DMIC_STATE_PAUSED) {
			LOG_WRN("Cannot start from state: %d", data->state);
			return -EPERM;
		}

		/* Reset counters */
		data->drop_count = AUDIO_DROP_NUM;

		/* Clear DMA error status before starting */
		audio_adapter->sport_adapter.dma_err_sta &= ~(0x05);

		hal_audio_sport_rx_dma_start(audio_adapter, ENABLE);

		next_state = DMIC_STATE_ACTIVE;
		break;

	case DMIC_TRIGGER_PAUSE:
	case DMIC_TRIGGER_STOP:
		if (data->state != DMIC_STATE_ACTIVE) {
			LOG_WRN("Cannot stop from state: %d", data->state);
			return -EPERM;
		}

		hal_audio_sport_rx_dma_start(audio_adapter, DISABLE);

		/* Reset semaphore and queue */
		k_sem_reset(&data->rx_sem);
		data->rx_queue.write_idx = 0;
		data->rx_queue.read_idx = 0;
		data->rx_queue.count = 0;

		if (cmd == DMIC_TRIGGER_PAUSE) {
			next_state = DMIC_STATE_PAUSED;
		} else {
			next_state = DMIC_STATE_CONFIGURED;
		}
		break;

	default:
		LOG_ERR("Unsupported trigger command: %d", cmd);
		return -EINVAL;
	}

	data->state = next_state;
	return 0;
}

static int dmic_amebapro2_read(const struct device *dev, uint8_t stream, void **buffer,
			       size_t *size, int32_t timeout)
{
	struct dmic_amebapro2_data *const data = dev->data;
	hal_audio_adapter_t *audio_adapter = &data->audio_adapter;
	void *mem_block;
	int ret;

	if (data->state != DMIC_STATE_ACTIVE) {
		LOG_ERR("Cannot read in state: %d", data->state);
		return -EPERM;
	}

	ret = k_mem_slab_alloc(data->pcm_mem_slab, &mem_block, K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("Failed to allocate memory block");
		return -ENOMEM;
	}

	/* Wait for callback to signal that a page is ready */
	ret = k_sem_take(&data->rx_sem, K_MSEC(timeout));
	if (ret < 0) {
		k_mem_slab_free(data->pcm_mem_slab, mem_block);
		return -EAGAIN;
	}

	/* Dequeue the DMA page pointer */
	if (data->rx_queue.count == 0) {
		k_mem_slab_free(data->pcm_mem_slab, mem_block);
		LOG_ERR("RX queue empty (should not happen)");
		return -EAGAIN;
	}

	uint8_t *dma_page = data->rx_queue.pages[data->rx_queue.read_idx];

	data->rx_queue.read_idx = (data->rx_queue.read_idx + 1) % RX_PAGE_QUEUE_SIZE;
	data->rx_queue.count--;

	/* Copy from DMA page to user buffer */
	memcpy(mem_block, dma_page, AUDIO_DMA_PAGE_SIZE);

	/* Return the DMA page to hardware after copying is complete */
	uint32_t page_ret = hal_audio_sport_rx_page_recv(audio_adapter);

	if (page_ret != HAL_OK) {
		LOG_WRN("Failed to return DMA page (ret: 0x%x)", page_ret);
	}

	*buffer = mem_block;
	*size = AUDIO_DMA_PAGE_SIZE;
	return 0;
}

static const struct _dmic_ops dmic_amebapro2_api = {
	.configure = dmic_amebapro2_configure,
	.trigger = dmic_amebapro2_trigger,
	.read = dmic_amebapro2_read,
};

static int dmic_amebapro2_init(const struct device *dev)
{
	const struct dmic_amebapro2_config *config = dev->config;
	struct dmic_amebapro2_data *const data = dev->data;

	/* Initialize semaphore for RX page notifications */
	k_sem_init(&data->rx_sem, 0, AUDIO_DMA_PAGE_NUM);

	/* Configure and enable SPORT interrupt */
	if (config->irq_config_func) {
		config->irq_config_func(dev);
	}

	data->state = DMIC_STATE_INITIALIZED;
	return 0;
}

#define DMIC_AMEBAPRO2_INIT(n)                                                                     \
	extern void hal_rtl_sport_irqhandler_ram(void);                                            \
	static void dmic_amebapro2_isr_##n(const void *arg)                                        \
	{                                                                                          \
		hal_rtl_sport_irqhandler_ram();                                                    \
	}                                                                                          \
	static void dmic_amebapro2_irq_config_##n(const struct device *dev)                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), dmic_amebapro2_isr_##n,     \
			    NULL, 0);                                                              \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
	static struct dmic_amebapro2_data dmic_amebapro2_data_##n = {                              \
		.state = DMIC_STATE_UNINIT,                                                        \
		.pcm_mem_slab = NULL,                                                              \
		.rx_buf = NULL,                                                                    \
		.rx_queue = {.write_idx = 0, .read_idx = 0, .count = 0},                           \
		.drop_count = 0,                                                                   \
	};                                                                                         \
	static const struct dmic_amebapro2_config dmic_amebapro2_config_##n = {                    \
		.irq_config_func = dmic_amebapro2_irq_config_##n,                                  \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, dmic_amebapro2_init, NULL, &dmic_amebapro2_data_##n,              \
			      &dmic_amebapro2_config_##n, POST_KERNEL,                             \
			      CONFIG_AUDIO_DMIC_INIT_PRIORITY, &dmic_amebapro2_api);

DMIC_AMEBAPRO2_INIT(0)
