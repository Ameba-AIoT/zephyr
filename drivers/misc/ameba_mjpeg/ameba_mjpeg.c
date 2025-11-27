/*
* Copyright (c) 2025 Realtek Semiconductor Corp.
*
* SPDX-License-Identifier: Apache-2.0
*/

#define DT_DRV_COMPAT realtek_ameba_mjpeg

#include <soc.h>
#include <ameba_soc.h>
#include "jpegdecapi.h"
#include "ppapi.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/cache.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/misc/ameba_mjpeg/ameba_mjpeg.h>

LOG_MODULE_REGISTER(ameba_mjpeg, CONFIG_AMEBA_MJPEG_LOG_LEVEL);

extern void hx170dec_isr(void *dev);
struct ameba_mjpeg_config {
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	void (*irq_config_func)(const struct device *dev);
};

struct ameba_mjpeg_data {
	struct k_mutex lock;
};

static uint32_t to_pp_pix_format(enum mjpeg_pix_fmt fmt) {
	uint32_t pp_fmt = PP_PIX_FMT_RGB16_CUSTOM;

	switch (fmt) {
		case PIX_RGB16_CUSTOM: 
			pp_fmt = PP_PIX_FMT_RGB16_CUSTOM;
			break;
		case PIX_RGB16_5_5_5:
			pp_fmt = PP_PIX_FMT_RGB16_5_5_5;
			break;
		case PIX_RGB16_5_6_5:
			pp_fmt = PP_PIX_FMT_RGB16_5_6_5;
			break;
		case PIX_BGR16_5_5_5:
			pp_fmt = PP_PIX_FMT_BGR16_5_5_5;
			break;
		case PIX_BGR16_5_6_5:
			pp_fmt = PP_PIX_FMT_BGR16_5_6_5;
			break;
		case PIX_RGB32_CUSTOM:
			pp_fmt = PP_PIX_FMT_RGB32_CUSTOM;
			break;
		case PIX_RGB32:
			pp_fmt = PP_PIX_FMT_RGB32;
			break;
		case PIX_BGR32:
			pp_fmt = PP_PIX_FMT_BGR32;
			break;
		default:
			break;
	}

	return pp_fmt;
}

static uint32_t jpeg_to_pp_pix_format(enum mjpeg_fmt_type fmt) {
	uint32_t pp_fmt = PP_PIX_FMT_YCBCR_4_2_0_SEMIPLANAR;

	switch (fmt) {
		case JPEG_YCbCr420: 
			pp_fmt = PP_PIX_FMT_YCBCR_4_2_0_SEMIPLANAR;
			break;
		case JPEG_YCbCr422:
			pp_fmt = PP_PIX_FMT_YCBCR_4_2_2_SEMIPLANAR;
			break;
		default:
			break;
	}

	return pp_fmt;
}

static void ameba_mjpeg_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	hx170dec_isr(NULL);
}

static int ameba_mjpeg_init(const struct device *dev)
{
	const struct ameba_mjpeg_config *cfg = dev->config;
	struct ameba_mjpeg_data *data = dev->data;
	int ret = 0;

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret < 0) {
		LOG_ERR("Could not initialize clock (%d)", ret);
		return ret;
	}

	hx170dec_init();

	k_mutex_init(&data->lock);
	cfg->irq_config_func(dev);
	return ret;
}

int ameba_mjpeg_decoder_get_image_info(const struct device *dev, 
									struct decoder_input_data *input_data,
									struct decoder_image_info *out_image_info)
{
	int ret = 0;
	struct ameba_mjpeg_data *data = dev->data;
	JpegDecImageInfo image_info;
	JpegDecInput jpeg_in;
	JpegDecInst jpeg_instance;

	k_mutex_lock(&data->lock, K_FOREVER);
	memset(&jpeg_in, 0, sizeof(jpeg_in));
	jpeg_in.streamBuffer.pVirtualAddress = (uint32_t *)input_data->stream_buffer;
	jpeg_in.streamBuffer.busAddress = (uint32_t)input_data->stream_buffer;
	jpeg_in.streamLength = input_data->buffer_size;

	ret = JpegDecInit(&jpeg_instance);
	if (ret != JPEGDEC_OK) {
		LOG_ERR("JpegDecInit initialize fail (%d)", ret);
		goto end;
	}

	ret = JpegDecGetImageInfo(jpeg_instance, &jpeg_in, &image_info);

	if (ret == JPEGDEC_OK) {
		out_image_info->width = image_info.outputWidth;
		out_image_info->height = image_info.outputHeight;
		out_image_info->format = image_info.outputFormat;
	} else {
		LOG_ERR("JpegDecGetImageInfo fail (%d)", ret);
	}

	if (jpeg_instance) {
		JpegDecRelease(jpeg_instance);
	}

end:
	k_mutex_unlock(&data->lock);
	return ret;
}

int ameba_mjpeg_decoder_decode(const struct device *dev, struct decoder_input_data *input_data,
							struct decoder_config *dec_config)
{
	int ret = 0;
	struct ameba_mjpeg_data *data = dev->data;

	JpegDecInst jpeg_instance;
	PPInst pp_instance;
	JpegDecInput jpeg_in;
	JpegDecOutput jpeg_out;
	PPConfig pp_conf;
	k_mutex_lock(&data->lock, K_FOREVER);
	memset(&jpeg_in, 0, sizeof(jpeg_in));
	memset(&jpeg_out, 0, sizeof(jpeg_out));
	memset(&pp_conf, 0, sizeof(pp_conf));

	jpeg_in.streamBuffer.pVirtualAddress = (uint32_t *)input_data->stream_buffer;
	jpeg_in.streamBuffer.busAddress = (uint32_t)input_data->stream_buffer;
	jpeg_in.streamLength = input_data->buffer_size;

	ret = JpegDecInit(&jpeg_instance);
	if (ret != JPEGDEC_OK) {
		LOG_ERR("JpegDecInit initialize fail (%d)", ret);
		k_mutex_unlock(&data->lock);
		return ret;
	}

	ret = PPInit(&pp_instance);
	if (ret == PP_OK) {
		ret = PPDecCombinedModeEnable(pp_instance, jpeg_instance, PP_PIPELINED_DEC_TYPE_JPEG);
		if (ret == PP_OK) {
			ret = PPGetConfig(pp_instance, &pp_conf);
			if (ret == PP_OK) {
				pp_conf.ppInImg.width = dec_config->width;
				pp_conf.ppInImg.height = dec_config->height;
				pp_conf.ppInImg.videoRange = 1;
				pp_conf.ppOutRgb.rgbTransform = PP_YCBCR2RGB_TRANSFORM_BT_709;
				pp_conf.ppInImg.pixFormat = jpeg_to_pp_pix_format(dec_config->pix_fmt);
				pp_conf.ppOutImg.width = dec_config->out_width;
				pp_conf.ppOutImg.height = dec_config->out_height;
				pp_conf.ppOutImg.pixFormat = to_pp_pix_format(dec_config->out_pix_fmt);
				pp_conf.ppOutImg.bufferBusAddr = (uint32_t)dec_config->decode_data;
				sys_cache_data_flush_and_invd_all();
				ret = PPSetConfig(pp_instance, &pp_conf);
				if (ret == PP_OK) {
					ret = JpegDecDecode(jpeg_instance, &jpeg_in, &jpeg_out);
					if (ret != JPEGDEC_FRAME_READY) {
						LOG_ERR("JpegDecDecode fail (%d)", ret);
					}
				} else {
					LOG_ERR("PPSetConfig fail (%d)", ret);
				}
			} else {
				LOG_ERR("PPGetConfig fail (%d)", ret);
			}

			ret = PPDecCombinedModeDisable(pp_instance, jpeg_instance);
		} else {
			LOG_ERR("PPDecCombinedModeEnable fail (%d)", ret);
		}

		PPRelease(pp_instance);
	} else {
		LOG_ERR("PPInit fail (%d)", ret);
	}

	JpegDecRelease(jpeg_instance);
	k_mutex_unlock(&data->lock);
	return ret;
}

static DEVICE_API(mjpeg, ameba_mjpeg_api_funcs) = {
	.decoder_get_image_info = ameba_mjpeg_decoder_get_image_info,
	.decoder_decode = ameba_mjpeg_decoder_decode,
};

static void ameba_mjpeg_irq_config(const struct device *dev)
{
	ARG_UNUSED(dev);
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), ameba_mjpeg_isr,
			DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));
}

static const struct ameba_mjpeg_config mjpeg_config = {
	.irq_config_func = ameba_mjpeg_irq_config,
	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.clock_subsys = (void *)DT_INST_CLOCKS_CELL(0, idx),
};

static struct ameba_mjpeg_data mjpeg_data;

DEVICE_DT_INST_DEFINE(0, &ameba_mjpeg_init, NULL, &mjpeg_data, &mjpeg_config, POST_KERNEL,
			CONFIG_AMEBA_MJPEG_INIT_PRIORITY, &ameba_mjpeg_api_funcs);