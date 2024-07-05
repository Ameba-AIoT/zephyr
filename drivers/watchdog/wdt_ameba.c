/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_watchdog

#include <string.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wdt_ameba, CONFIG_WDT_LOG_LEVEL);

struct wdt_ameba_data {
	uint32_t timeout;
	wdt_callback_t callback;
};

struct wdt_ameba_config {
	uint32_t wdt_inst;
};


static int wdt_ameba_disable(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static int wdt_ameba_feed(const struct device *dev, int channel_id)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);

	return 0;
}

static int wdt_ameba_set_config(const struct device *dev, uint8_t options)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(options);

	return 0;
}

static int wdt_ameba_install_timeout(const struct device *dev,
									 const struct wdt_timeout_cfg *cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cfg);

	return 0;
}

static int wdt_ameba_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static const struct wdt_driver_api wdt_api = {
	.setup = wdt_ameba_set_config,
	.disable = wdt_ameba_disable,
	.install_timeout = wdt_ameba_install_timeout,
	.feed = wdt_ameba_feed
};

#define AMEBA_WDT_INIT(idx)							  				\
	static struct wdt_ameba_data wdt##idx##_data;					\
	static const struct wdt_ameba_config wdt_ameba_config##idx = {	\
		.wdt_inst = idx,			\
	};									   	\
										   	\
	DEVICE_DT_INST_DEFINE(idx,				\
			      wdt_ameba_init,			\
			      NULL,						\
			      &wdt##idx##_data,					   \
			      &wdt_ameba_config##idx,				   \
			      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	   \
			      &wdt_api);

DT_INST_FOREACH_STATUS_OKAY(AMEBA_WDT_INIT)
