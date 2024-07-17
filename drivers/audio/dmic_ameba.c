/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_codec

#include <zephyr/devicetree.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#define LOG_LEVEL CONFIG_AUDIO_DMIC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dmic_ameba);


struct dmic_ameba_config {
	const struct device *comm_master;
};

struct dmic_ameba_data {
	enum dmic_state		state;
	size_t			pcm_mem_size;
};

int dmic_ameba_configure(const struct device *dev, struct dmic_cfg *cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cfg);
	return 0;
}

int dmic_ameba_trigger(const struct device *dev, enum dmic_trigger cmd)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cmd);
	return 0;
}

int dmic_ameba_read(const struct device *dev, uint8_t stream, void **buffer,
					size_t *size, int32_t timeout)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(stream);
	ARG_UNUSED(buffer);
	ARG_UNUSED(size);
	ARG_UNUSED(timeout);
	return 0;
}

#define I2S(idx) 		DT_NODELABEL(i2s##idx)
#define I2S_NODE(idx) 	I2S(idx)
#define I2S_BINDING 	I2S_NODE(DT_INST_PROP(0, i2s_index))

static const struct _dmic_ops dmic_ameba_api = {
#if DT_NODE_HAS_STATUS(I2S_BINDING, okay)
	.configure		= dmic_ameba_configure,
	.trigger		= dmic_ameba_trigger,
	.read			= dmic_ameba_read,
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2s) */
};

static int dmic_ameba_init(const struct device *dev)
{
	const struct dmic_ameba_config *config = dev->config;
	struct dmic_ameba_data *const data = dev->data;

	if (!device_is_ready(config->comm_master)) {
		return -ENODEV;
	}

	data->state = DMIC_STATE_INITIALIZED;
	return 0;
}

static const struct dmic_ameba_config dmic_ameba_config = {
	.comm_master = DEVICE_DT_GET(I2S_BINDING),
};

static struct dmic_ameba_data dmic_ameba_data;

DEVICE_DT_INST_DEFINE(0, dmic_ameba_init, NULL, &dmic_ameba_data,
					  &dmic_ameba_config, POST_KERNEL,
					  CONFIG_AUDIO_DMIC_INIT_PRIORITY, &dmic_ameba_api);
