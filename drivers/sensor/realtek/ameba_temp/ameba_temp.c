/*
 * Copyright (c) 2022 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_temp

#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ameba_temp, CONFIG_SENSOR_LOG_LEVEL);

#define TEMP_TODO 0
#ifdef TEMP_TODO
typedef int temp_sensor_dac_offset_t;
typedef int *temp_sensor_config_t;
#endif

struct ameba_temp_data {
	struct k_mutex mutex;
	temp_sensor_config_t temp_sensor;
	float temp_out;
};

struct ameba_temp_config {
	temp_sensor_dac_offset_t range;
};

static int ameba_temp_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
#if TEMP_TODO

	struct ameba_temp_data *data = dev->data;
	int rc = 0;

	k_mutex_lock(&data->mutex, K_FOREVER);

	if (temp_sensor_read_celsius(&data->temp_out) != ESP_OK) {
		LOG_ERR("Temperature read error!");
		rc = -EFAULT;
		goto unlock;
	}

unlock:
	k_mutex_unlock(&data->mutex);

	return rc;
#endif
	return 0;
}

static int ameba_temp_channel_get(const struct device *dev, enum sensor_channel chan,
				  struct sensor_value *val)
{
#if TEMP_TODO
	struct ameba_temp_data *data = dev->data;

	if (chan != SENSOR_CHAN_DIE_TEMP) {
		return -ENOTSUP;
	}

	return sensor_value_from_double(val, data->temp_out);
#endif
	return 0;
}

static DEVICE_API(sensor, ameba_temp_driver_api) = {
	.sample_fetch = ameba_temp_sample_fetch,
	.channel_get = ameba_temp_channel_get,
};

static int ameba_temp_init(const struct device *dev)
{
#if TEMP_TODO
	struct ameba_temp_data *data = dev->data;
	const struct ameba_temp_config *conf = dev->config;

	k_mutex_init(&data->mutex);
	temp_sensor_get_config(&data->temp_sensor);
	data->temp_sensor.dac_offset = conf->range;
	temp_sensor_set_config(data->temp_sensor);
	temp_sensor_start();
	LOG_DBG("Temperature sensor started. Offset %d, clk_div %d", data->temp_sensor.dac_offset,
		data->temp_sensor.clk_div);
#endif
	return 0;
}

#define AMEBA_TEMP_DEFINE(inst)                                                                    \
	static struct ameba_temp_data ameba_temp_dev_data_##inst = {                               \
		.temp_sensor = NULL,                                                               \
	};                                                                                         \
                                                                                                   \
	static const struct ameba_temp_config ameba_temp_dev_config_##inst = {                     \
		.range = (temp_sensor_dac_offset_t)DT_INST_PROP(inst, range),                      \
	};                                                                                         \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, ameba_temp_init, NULL, &ameba_temp_dev_data_##inst,     \
				     &ameba_temp_dev_config_##inst, POST_KERNEL,                   \
				     CONFIG_SENSOR_INIT_PRIORITY, &ameba_temp_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AMEBA_TEMP_DEFINE)
