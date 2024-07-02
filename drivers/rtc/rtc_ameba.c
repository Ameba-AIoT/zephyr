/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_rtc

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/drivers/clock_control/ameba_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util.h>
#include <soc.h>

#include <zephyr/logging/log.h>

#include <stdbool.h>

LOG_MODULE_REGISTER(rtc_ameba, CONFIG_RTC_LOG_LEVEL);

/* RTC start time: 1st, Jan, 2000 */
#define RTC_YEAR_REF 2000
/* struct tm start time:   1st, Jan, 1900 */
#define TM_YEAR_REF 1900

/* Convert part per billion calibration value to a number of clock pulses added or removed each
 * 2^20 clock cycles so it is suitable for the CALR register fields
 *
 * nb_pulses = ppb * 2^20 / 10^9 = ppb * 2^11 / 5^9 = ppb * 2048 / 1953125
 */
#define PPB_TO_NB_PULSES(ppb) DIV_ROUND_CLOSEST((ppb) * 2048, 1953125)

/* Convert CALR register value (number of clock pulses added or removed each 2^20 clock cycles)
 * to part ber billion calibration value
 *
 * ppb = nb_pulses * 10^9 / 2^20 = nb_pulses * 5^9 / 2^11 = nb_pulses * 1953125 / 2048
 */
#define NB_PULSES_TO_PPB(pulses) DIV_ROUND_CLOSEST((pulses) * 1953125, 2048)

/* CALP field can only be 512 or 0 as in reality CALP is a single bit field representing 512 pulses
 * added every 2^20 clock cycles
 */
#define MAX_CALP (512)
#define MAX_CALM (511)

#define MAX_PPB NB_PULSES_TO_PPB(MAX_CALP)
#define MIN_PPB -NB_PULSES_TO_PPB(MAX_CALM)

/* Timeout in microseconds used to wait for flags */
#define RTC_TIMEOUT 1000000

struct rtc_ameba_config {
	uint32_t async_prescaler;
	uint32_t sync_prescaler;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
};

struct rtc_ameba_data {
	struct k_mutex lock;
};

static int rtc_ameba_init(const struct device *dev)
{
	const struct rtc_ameba_config *cfg = dev->config;
	struct rtc_ameba_data *data = dev->data;
	int err = 0;

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* Enable RTC bus clock */
	if (clock_control_on(cfg->clock_dev, cfg->clock_subsys)) {
		LOG_ERR("clock op failed\n");
		return -EIO;
	}

	ARG_UNUSED(data);
	ARG_UNUSED(err);
	return err;
}

static int rtc_ameba_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	int err = 0;
	ARG_UNUSED(dev);
	ARG_UNUSED(timeptr);

	return err;
}

static int rtc_ameba_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(timeptr);

	return 0;
}

struct rtc_driver_api rtc_ameba_driver_api = {
	.set_time = rtc_ameba_set_time,
	.get_time = rtc_ameba_get_time,
	/* RTC_ALARM not supported */
	/* RTC_UPDATE not supported */
#ifdef CONFIG_RTC_CALIBRATION
#endif /* CONFIG_RTC_CALIBRATION */
};

static const struct rtc_ameba_config rtc_config = {
#if DT_INST_CLOCKS_CELL_BY_IDX(0, 1, bus) == AMEBA_SRC_LSI
	/* prescaler values for LSI @ 32 KHz */
	.async_prescaler = 0x7F,
	.sync_prescaler = 0x00F9,
#else /* DT_INST_CLOCKS_CELL_BY_IDX(0, 1, bus) == AMEBA_SRC_LSE */
	/* prescaler values for LSE @ 32768 Hz */
	.async_prescaler = 0x7F,
	.sync_prescaler = 0x00FF,
#endif
	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(0, idx),
};

static struct rtc_ameba_data rtc_data;

DEVICE_DT_INST_DEFINE(0, &rtc_ameba_init, NULL, &rtc_data, &rtc_config, PRE_KERNEL_1,
					  CONFIG_RTC_INIT_PRIORITY, &rtc_ameba_driver_api);
