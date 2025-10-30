/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba VIDEO
 */

#define DT_DRV_COMPAT realtek_amebapro2_video_channel

#include <zephyr/kernel.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/logging/log.h>
#include <video_api.h>
LOG_MODULE_REGISTER(video_amebapro2, CONFIG_VIDEO_LOG_LEVEL);

struct video_amebapro2_data {
	const struct device *dev;
	struct video_format fmt;
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
	struct k_poll_signal *signal;
	video_params_t params;
};

struct video_channel_config {
	const struct device *parent_ctrl;
	uint8_t channel_index;
	uint16_t default_width;
	uint16_t default_height;
	uint8_t default_fps;
	const char *label;
	uint8_t stream_id;
};

#define CH_NUM 5
static int ch_forcei[CH_NUM];
#define MAX_FRAME_RATE 30
static const struct video_format_cap fmts[] = {
	{
		.pixelformat = VIDEO_PIX_FMT_H264,
		.width_min = 64,
		.width_max = 1920,
		.height_min = 64,
		.height_max = 1080,
		.width_step = 16,
		.height_step = 16,
	},
	{
		.pixelformat = VIDEO_PIX_FMT_H265,
		.width_min = 64,
		.width_max = 1920,
		.height_min = 64,
		.height_max = 1080,
		.width_step = 16,
		.height_step = 16,
	},
	{
		.pixelformat = VIDEO_PIX_FMT_JPEG,
		.width_min = 64,
		.width_max = 1920,
		.height_min = 64,
		.height_max = 1080,
		.width_step = 16,
		.height_step = 16,
	},
	{0} /* End format (Sentinel) */
};

bool is_h264_key_frame(uint8_t *ptr)
{
	if (!ptr) {
		return false;
	}

	return ptr[0] == 0 && ptr[1] == 0 && ptr[2] == 0 && ptr[3] == 1 && (ptr[4] & 0x1F) == 0x07;
}

bool is_hevc_key_frame(uint8_t *ptr)
{
	if (!ptr) {
		return false;
	}

	return ptr[0] == 0 && ptr[1] == 0 && ptr[2] == 0 && ptr[3] == 1 && ptr[4] == 0x40;
}

static bool handle_forced_key_frame_check(enc2out_t *enc2out)
{
	if (!ch_forcei[enc2out->ch]) {
		return true;
	}

	if (!(enc2out->codec & (CODEC_H264 | CODEC_HEVC))) {
		return true;
	}

	uint8_t *ptr = (uint8_t *)enc2out->enc_addr;

	if (ptr[0] != 0 || ptr[1] != 0) {
		LOG_ERR("\r\nH264/HEVC stream error while waiting for key frame\r\n");
		LOG_ERR("\r\n(%d/%d) %x %x %x %x\r\n", enc2out->enc_len, enc2out->finish, *ptr,
			*(ptr + 1), *(ptr + 2), *(ptr + 3));
		video_encbuf_release(enc2out->ch, enc2out->codec, enc2out->enc_len);
		return false;
	}

	bool is_key_frame_received = false;

	if (enc2out->codec & CODEC_H264) {
		if (is_h264_key_frame(ptr)) {
			is_key_frame_received = true;
		}
	}

	else if (enc2out->codec & CODEC_HEVC) {
		if (is_hevc_key_frame(ptr)) {
			is_key_frame_received = true;
		}
	}

	if (is_key_frame_received) {
		ch_forcei[enc2out->ch] = 0;
		return true;
	}

	video_encbuf_release(enc2out->ch, enc2out->codec, enc2out->enc_len);
	return false;
}

static void video_output_cb(void *param1, void *param2, uint32_t arg)
{
	enc2out_t *enc2out = (enc2out_t *)param1;

	struct device *dev = (struct device *)arg;
	struct video_amebapro2_data *ctx = dev->data;
	struct video_buffer *vbuf;

	if (enc2out->cmd_status == VOE_OK) {
		if (!handle_forced_key_frame_check(enc2out)) {
			return;
		}
	} else {
		switch (enc2out->cmd_status) {
		case VOE_ENC_BUF_OVERFLOW:
		case VOE_ENC_QUEUE_OVERFLOW:
			LOG_WRN("VOE CH%d ENC %s full (queue/used/out/rsvd) %d/%dKB%dKB%dKB\n",
				enc2out->ch,
				enc2out->cmd_status == VOE_ENC_BUF_OVERFLOW ? "buff" : "queue",
				enc2out->enc_time, enc2out->enc_used >> 10,
				ctx->params.out_buf_size >> 10, ctx->params.out_rsvd_size >> 10);
			video_encbuf_clean(enc2out->ch, CODEC_H264 | CODEC_HEVC);
			video_ctrl(enc2out->ch, VIDEO_FORCE_IFRAME, 1);
			break;
		case VOE_JPG_BUF_OVERFLOW:
		case VOE_JPG_QUEUE_OVERFLOW:
			LOG_WRN("VOE CH%d JPG %s full (queue/used/out/rsvd) %d/%dKB\n", enc2out->ch,
				enc2out->cmd_status == VOE_JPG_BUF_OVERFLOW ? "buff" : "queue",
				enc2out->jpg_time, enc2out->jpg_used >> 10);
			break;
		default:
			LOG_ERR("Error CH%d VOE cmd %x status %x\n", enc2out->ch, enc2out->cmd,
				enc2out->cmd_status);
			break;
		}
		return;
	}

	if ((enc2out->codec & (CODEC_H264 | CODEC_HEVC | CODEC_JPEG)) != 0) {
		vbuf = k_fifo_get(&ctx->fifo_in, K_NO_WAIT);
		if (vbuf == NULL) {
			video_encbuf_release(enc2out->ch, enc2out->codec, enc2out->enc_len);
		} else {
			if (enc2out->codec & CODEC_JPEG) {
				vbuf->size = enc2out->jpg_len;
				vbuf->buffer = (uint8_t *)enc2out->jpg_addr;
			} else {
				vbuf->size = enc2out->enc_len;
				vbuf->buffer = (uint8_t *)enc2out->enc_addr;
			}
			vbuf->timestamp = k_uptime_get_32();
			k_fifo_put(&ctx->fifo_out, vbuf);
		}
	} else if ((enc2out->codec & (CODEC_NV12 | CODEC_RGB | CODEC_NV16)) != 0) {
		video_ispbuf_release(enc2out->ch, (int)enc2out->isp_addr);
	}
}

static int video_amebapro2_set_fmt(const struct device *dev, enum video_endpoint_id ep,
				   struct video_format *fmt)
{
	struct video_amebapro2_data *data = dev->data;
	int i = 0;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(fmts); ++i) {
		if (fmt->pixelformat == fmts[i].pixelformat && fmt->width >= fmts[i].width_min &&
		    fmt->width <= fmts[i].width_max && fmt->height >= fmts[i].height_min &&
		    fmt->height <= fmts[i].height_max) {
			break;
		}
	}

	if (i == ARRAY_SIZE(fmts)) {
		LOG_ERR("Unsupported pixel format or resolution");
		return -ENOTSUP;
	}
	if (fmt->pixelformat == VIDEO_PIX_FMT_H264) {
		data->params.type = VIDEO_H264;
	} else if (fmt->pixelformat == VIDEO_PIX_FMT_H265) {
		data->params.type = VIDEO_HEVC;
	} else if (fmt->pixelformat == VIDEO_PIX_FMT_JPEG) {
		data->params.type = VIDEO_JPEG;
	}
	data->params.width = fmt->width;
	data->params.height = fmt->height;

	data->fmt = *fmt;

	return 0;
}

static int video_amebapro2_get_fmt(const struct device *dev, enum video_endpoint_id ep,
				   struct video_format *fmt)
{
	struct video_amebapro2_data *data = dev->data;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	*fmt = data->fmt;

	return 0;
}

static int video_amebapro2_set_stream(const struct device *dev, bool enable)
{
	struct video_amebapro2_data *data = dev->data;
	int ret = 0;

	if (enable) {
		ret = video_open(&data->params, video_output_cb, (void *)dev);
		if (data->params.type == VIDEO_JPEG) {
			video_ctrl(data->params.stream_id, VIDEO_JPEG_OUTPUT, 2);
		}
	} else {
		ret = video_close(data->params.stream_id);
	}
	return ret;
}

static int video_amebapro2_enqueue(const struct device *dev, enum video_endpoint_id ep,
				   struct video_buffer *vbuf)
{
	struct video_amebapro2_data *data = dev->data;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	if (vbuf->size != 0) {
		int codec_type = video_get_codec_type(data->params.type);

		if ((codec_type & (CODEC_H264 | CODEC_HEVC | CODEC_JPEG)) != 0) {
			video_encbuf_release(data->params.stream_id,
					     video_get_codec_type(data->params.type), vbuf->size);
		} else {
			video_ispbuf_release(data->params.stream_id, (int)vbuf->buffer);
		}
	}

	k_fifo_put(&data->fifo_in, vbuf);

	return 0;
}

static int video_amebapro2_dequeue(const struct device *dev, enum video_endpoint_id ep,
				   struct video_buffer **vbuf, k_timeout_t timeout)
{
	struct video_amebapro2_data *data = dev->data;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	*vbuf = k_fifo_get(&data->fifo_out, timeout);
	if (*vbuf == NULL) {
		return -EAGAIN;
	}

	return 0;
}

static int video_amebapro2_flush(const struct device *dev, enum video_endpoint_id ep, bool cancel)
{
	struct video_amebapro2_data *data = dev->data;
	struct video_buffer *vbuf;

	if (!cancel) {
		int timeout_ms = 5000; /*Setup 5ms timeout*/

		while (!k_fifo_is_empty(&data->fifo_in)) {
			if (timeout_ms <= 0) {
				LOG_ERR("Flush timed out waiting for fifo_in to become empty!");
				return -ETIMEDOUT;
			}
			k_sleep(K_MSEC(1));
			timeout_ms--;
		}
	} else {
		while ((vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT))) {
			k_fifo_put(&data->fifo_out, vbuf);
			if (IS_ENABLED(CONFIG_POLL) && data->signal) {
				k_poll_signal_raise(data->signal, VIDEO_BUF_ABORTED);
			}
		}
	}

	return 0;
}

static int video_amebapro2_get_caps(const struct device *dev, enum video_endpoint_id ep,
				    struct video_caps *caps)
{
	caps->format_caps = fmts;
	return 0;
}

static inline int video_amebapro2_set_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	struct video_amebapro2_data *data = dev->data;
	int ret = 0;

	switch (cid) {
	case VIDEO_CID_VENDOR_SET_PARAMS: {
		memcpy(&data->params, (void *)value, sizeof(video_params_t));
		break;
	}
	case VIDEO_CID_VENDOR_SET_RCPARAM: {
		ret = video_ctrl(data->params.stream_id, VIDEO_SET_RCPARAM, (int)value);
	} break;
	case VIDEO_CID_VENDOR_STREAMID: {
		data->params.stream_id = *(int *)value;
	} break;
	case VIDEO_CID_VENDOR_FORCE_IFRAME:
		ret = video_ctrl(data->params.stream_id, VIDEO_FORCE_IFRAME, *(int *)value);
		ch_forcei[data->params.stream_id] = 1;
		break;
	case VIDEO_CID_VENDOR_BPS:
		ret = video_ctrl(data->params.stream_id, VIDEO_BPS, *(int *)value);
		break;
	case VIDEO_CID_VENDOR_GOP:
		ret = video_ctrl(data->params.stream_id, VIDEO_GOP, *(int *)value);
		break;
	case VIDEO_CID_VENDOR_FPS:
		ret = video_ctrl(data->params.stream_id, VIDEO_FPS, *(int *)value);
		break;
	case VIDEO_CID_VENDOR_ISPFPS:
		ret = video_ctrl(data->params.stream_id, VIDEO_ISPFPS, *(int *)value);
		break;
	case VIDEO_CID_VENDOR_SNAPSHOT:
		ret = video_ctrl(data->params.stream_id, VIDEO_JPEG_OUTPUT, *(int *)value);
		break;
	case VIDEO_CID_VENDOR_PRE_INIT_PARM:
		video_pre_init_setup_parameters((video_pre_init_params_t *)value);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static inline int video_amebapro2_get_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	struct video_amebapro2_data *data = dev->data;

	switch (cid) {
	case VIDEO_CID_VENDOR_GET_PARAMS: {
		memcpy((void *)value, &data->params, sizeof(video_params_t));
	} break;
	case VIDEO_CID_VENDOR_GET_PRE_INIT_PARM:
		memcpy((void *)value, (video_pre_init_params_t *)video_get_pre_init_setup_params(),
		       sizeof(video_pre_init_params_t));
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int video_amebapro2_set_frmival(const struct device *dev, enum video_endpoint_id ep,
				       struct video_frmival *frmival)
{
	struct video_amebapro2_data *data = dev->data;

	if (frmival->denominator && frmival->numerator) {
		data->params.fps = MIN(DIV_ROUND_CLOSEST(frmival->denominator, frmival->numerator),
				       MAX_FRAME_RATE);
	} else {
		return -EINVAL;
	}

	frmival->numerator = 1;
	frmival->denominator = data->params.fps;
	return 0;
}

static int video_amebapro2_get_frmival(const struct device *dev, enum video_endpoint_id ep,
				       struct video_frmival *frmival)
{
	struct video_amebapro2_data *data = dev->data;

	frmival->numerator = 1;
	frmival->denominator = data->params.fps;

	return 0;
}

static int video_amebapro2_enum_frmival(const struct device *dev, enum video_endpoint_id ep,
					struct video_frmival_enum *fie)
{
	int i = 0;

	if (ep != VIDEO_EP_OUT || fie->index) {
		return -EINVAL;
	}

	while (fmts[i].pixelformat && (fmts[i].pixelformat != fie->format->pixelformat)) {
		i++;
	}

	if ((i == ARRAY_SIZE(fmts)) || (fie->format->width > fmts[i].width_max) ||
	    (fie->format->width < fmts[i].width_min) ||
	    (fie->format->height > fmts[i].height_max) ||
	    (fie->format->height < fmts[i].height_min)) {
		return -EINVAL;
	}

	fie->type = VIDEO_FRMIVAL_TYPE_STEPWISE;
	fie->stepwise.min.numerator = 1;
	fie->stepwise.min.denominator = MAX_FRAME_RATE;
	fie->stepwise.max.numerator = UINT32_MAX;
	fie->stepwise.max.denominator = 1;
	/* The frame interval step size is the minimum resolution of K_MSEC(), which is 1ms */
	fie->stepwise.step.numerator = 1;
	fie->stepwise.step.denominator = 1000;

	return 0;
}

#ifdef CONFIG_POLL
static int video_amebapro2_set_signal(const struct device *dev, enum video_endpoint_id ep,
				      struct k_poll_signal *signal)
{
	struct video_amebapro2_data *data = dev->data;

	if (data->signal && signal != NULL) {
		return -EALREADY;
	}

	data->signal = signal;

	return 0;
}
#endif

static DEVICE_API(video, ameba_video_driver_api) = {
	.set_format = video_amebapro2_set_fmt,
	.get_format = video_amebapro2_get_fmt,
	.set_stream = video_amebapro2_set_stream,
	.flush = video_amebapro2_flush,
	.enqueue = video_amebapro2_enqueue,
	.dequeue = video_amebapro2_dequeue,
	.get_caps = video_amebapro2_get_caps,
	.set_ctrl = video_amebapro2_set_ctrl,
	.get_ctrl = video_amebapro2_get_ctrl,
	.set_frmival = video_amebapro2_set_frmival,
	.get_frmival = video_amebapro2_get_frmival,
	.enum_frmival = video_amebapro2_enum_frmival,
#ifdef CONFIG_POLL
	.set_signal = video_amebapro2_set_signal,
#endif
};

static int video_amebapro2_channel_init(const struct device *dev)
{
	struct video_amebapro2_data *data = dev->data;
	const struct video_channel_config *config = dev->config;

	data->dev = dev;
	k_fifo_init(&data->fifo_in);
	k_fifo_init(&data->fifo_out);

	data->params.width = config->default_width;
	data->params.height = config->default_height;
	data->params.stream_id = config->stream_id;
	data->params.bps = 1 * 1024 * 1024;
	data->params.fps = config->default_fps;
	data->params.type = VIDEO_HEVC;
	data->fmt.height = config->default_height;
	data->fmt.width = config->default_width;
	if (config->stream_id == 0x00) {
		data->fmt.pixelformat = VIDEO_PIX_FMT_H265;
	} else {
		data->fmt.pixelformat = VIDEO_PIX_FMT_H264;
	}

	return 0;
}

#define VIDEO_CHANNEL_INIT(inst)                                                                   \
	static struct video_amebapro2_data video_data_##inst;                                      \
                                                                                                   \
	static const struct video_channel_config video_config_##inst = {                           \
		.parent_ctrl = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                \
		.channel_index = DT_INST_REG_ADDR(inst),                                           \
		.default_width = DT_INST_PROP_OR(inst, default_width, 1920),                       \
		.default_height = DT_INST_PROP_OR(inst, default_height, 1080),                     \
		.default_fps = DT_INST_PROP_OR(inst, default_fps, 30),                             \
		.label = DT_INST_PROP(inst, label),                                                \
		.stream_id = inst,                                                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, video_amebapro2_channel_init, NULL, &video_data_##inst,        \
			      &video_config_##inst, POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY,       \
			      &ameba_video_driver_api);

DT_INST_FOREACH_STATUS_OKAY(VIDEO_CHANNEL_INIT)
