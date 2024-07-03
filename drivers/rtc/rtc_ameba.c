/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_rtc

#include "ameba_soc.h"
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/drivers/clock_control/ameba_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>

#include <stdbool.h>

LOG_MODULE_REGISTER(rtc_ameba, CONFIG_RTC_LOG_LEVEL);

/* RTC start time: 1st, Jan, 1900 */
#define RTC_YEAR_REF			RTC_BASE_YEAR

/* struct tm start time:   1st, Jan, 1900 */
#define TM_YEAR_REF 			1900

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

	/* todo: cfg? or DTS property? rtc.h force24*/
	/* uint32_t hour_fmt;	RTC_HourFormat_24, RTC_HourFormat_12 */
};

struct rtc_ameba_data {
	struct k_mutex lock;
};

static const uint8_t dim[12] = {
	31, 0, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

/**
  * @brief  Judge whether a year is a leap year or not.
  * @param  year: Actual year - 1900.
  * @return Result.
  * @retval 1: This year is a leap year.
  * @retval 0: This year is not a leap year.
  */
static inline bool is_leap_year(uint32_t year)
{
	uint32_t full_year = year + 1900;
	return (!(full_year % 4) && (full_year % 100)) || !(full_year % 400);
}

/**
  * @brief  Calculate total days in a specified month of a specified year.
  * @param  year: Actual year - 1900.
  * @param  month: Specified month, which can be 0~11.
  * @note 0 represents January.
  * @return Number of days in the month of the year.
  */
static uint8_t days_in_month(uint8_t month, uint8_t year)
{
	uint8_t ret = dim[month % 12];
	if (ret == 0) {
		ret = is_leap_year(year + month / 12) ? 29 : 28;
	}
	return ret;
}

/**
  * @brief  Calculate month and day of the month according to year and day of the year.
  * @param  year: Actual year - 1900.
  * @param  yday: Day of the year.
  * @param  mon: Pointer to the variable that stores month, which can be 0~11.
  * @note 0 represents January.
  * @param  mday: Pointer to the variable that stores day of month, which can be 1~31.
  * @retval none
  */
static void rtc_calculate_mday(int year, int yday, int *mon, int *mday)
{
	int t_mon = -1, t_yday = yday + 1;

	while (t_yday > 0) {
		t_mon ++;
		t_yday -= days_in_month(t_mon, year);
	}

	*mon = t_mon;
	*mday = t_yday + days_in_month(t_mon, year);
}

/**
  * @brief  Calculate the day of a week according to date.
  * @param  year: Actual year - 1900.
  * @param  mon: Month of the year, which can be 0~11.
  * @note 0 represents January.
  * @param  mday: Day of the month.
  * @param  wday: Pointer to the variable that stores day of a week, which can be 0~6.
  * @note 0 represents Sunday.
  * @retval none
  */
static void rtc_calculate_wday(int year, int mon, int mday, int *wday)
{
	int t_year = year + 1900, t_mon = mon + 1;

	if (t_mon == 1 || t_mon == 2) {
		t_year --;
		t_mon += 12;
	}

	int c = t_year / 100;
	int y = t_year % 100;
	int week = (c / 4) - 2 * c + (y + y / 4) + (26 * (t_mon + 1) / 10) + mday - 1;

	while (week < 0) {
		week += 7;
	}
	week %= 7;

	*wday = week;
}

static int rtc_ameba_configure(const struct device *dev)
{
	const struct rtc_ameba_config *cfg = dev->config;
	int err = 0;
	uint32_t initilized = 0;

	RTC_InitTypeDef rtc_initstruct;

	RTC_StructInit(&rtc_initstruct);
	rtc_initstruct.RTC_AsynchPrediv = cfg->async_prescaler;
	rtc_initstruct.RTC_SynchPrediv = cfg->sync_prescaler;
	rtc_initstruct.RTC_HourFormat = RTC_HourFormat_24;/* force this hour fmt for time_t reason */

	initilized = RTC_Init(&rtc_initstruct);

	if (!initilized) {
		LOG_ERR(" rtc_ameba_configure initilized fail . \r\n");
		err = -EIO;
	}

	return err;
}

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

	k_mutex_init(&data->lock);

	err = rtc_ameba_configure(dev);

	return err;
}

static int rtc_ameba_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	struct rtc_ameba_data *data = dev->data;
	int err = 0;
	RTC_TimeTypeDef rtc_timestruct;

	uint32_t real_year = timeptr->tm_year + TM_YEAR_REF;

	if (real_year < RTC_YEAR_REF) {
		/* RTC does not support years before 1900 */
		return -EINVAL;
	}

	if (timeptr->tm_wday == -1) {
		/* day of the week is expected */
		return -EINVAL;
	}

	// if (timeptr->tm_isdst != -1) {
	// 	DiagPrintf("@@@ Unknown rtc_ameba_set_time line%d \r\n", __LINE__);
	// 	return -EINVAL;
	// }

	// if (!(timeptr->tm_nsec)) {
	// 	DiagPrintf("@@@ Unknown rtc_ameba_set_time line%d \r\n", __LINE__);
	// 	return -EINVAL;
	// }

	err = k_mutex_lock(&data->lock, K_NO_WAIT);
	if (err != 0) {
		LOG_ERR(" rtc_ameba_set_time lock fail !!! \r\n");
		return err;
	}

	rtc_timestruct.RTC_H12_PMAM = RTC_H12_AM;/* cautious in zsdk */
	rtc_timestruct.RTC_Year = timeptr->tm_year + RTC_BASE_YEAR;
	rtc_timestruct.RTC_Days = timeptr->tm_yday;
	rtc_timestruct.RTC_Hours = timeptr->tm_hour;
	rtc_timestruct.RTC_Minutes = timeptr->tm_min;
	rtc_timestruct.RTC_Seconds = timeptr->tm_sec;

	RTC_SetTime(RTC_Format_BIN, &rtc_timestruct);

	k_mutex_unlock(&data->lock);

	return err;
}

static int rtc_ameba_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	/* const struct rtc_ameba_config *cfg = dev->config; */
	struct rtc_ameba_data *data = dev->data;
	int err = 0;
	uint32_t ydays_thr;
	RTC_TimeTypeDef rtc_timestruct;

	err = k_mutex_lock(&data->lock, K_NO_WAIT);
	if (err) {
		return err;
	}

	/* step1: get hour, min, sec from RTC */
	RTC_GetTime(RTC_Format_BIN, &rtc_timestruct);

	timeptr->tm_sec = rtc_timestruct.RTC_Seconds;
	timeptr->tm_min = rtc_timestruct.RTC_Minutes;
	timeptr->tm_hour = rtc_timestruct.RTC_Hours;

	timeptr->tm_yday = rtc_timestruct.RTC_Days;
	timeptr->tm_year = rtc_timestruct.RTC_Year - RTC_BASE_YEAR; /* struct tm start from 1900 */

	/* step2: convert to mon, mday */
	rtc_calculate_mday(timeptr->tm_year, timeptr->tm_yday, &timeptr->tm_mon, &timeptr->tm_mday);

	/* step3: convert to wday */
	rtc_calculate_wday(timeptr->tm_year, timeptr->tm_mon, timeptr->tm_mday, &timeptr->tm_wday);

	/* step4: check and update year or not */
	ydays_thr = (is_leap_year(rtc_timestruct.RTC_Year)) ? 366 : 365;

	if (rtc_timestruct.RTC_Days > (ydays_thr - 1)) {
		rtc_timestruct.RTC_Days -= ydays_thr;
		rtc_timestruct.RTC_Year++;

		/* fix for test_y2k */
		timeptr->tm_mon = 0;
		timeptr->tm_mday = 1;
		timeptr->tm_yday = rtc_timestruct.RTC_Days;
		timeptr->tm_year = rtc_timestruct.RTC_Year - RTC_BASE_YEAR; /* struct tm start from 1900 */

		RTC_SetTime(RTC_Format_BIN, &rtc_timestruct);
	}

	k_mutex_unlock(&data->lock);

	timeptr->tm_isdst = -1;
	timeptr->tm_nsec = 0;

	return 0;
}

struct rtc_driver_api rtc_ameba_driver_api = {
	.set_time = rtc_ameba_set_time,
	.get_time = rtc_ameba_get_time,

#if defined(CONFIG_RTC_ALARM) || defined(__DOXYGEN__)
	.alarm_get_supported_fields = rtc_ameba_alarm_get_supported_fields,
	.alarm_set_time = rtc_ameba_alarm_set_time,
	.alarm_get_time = rtc_ameba_alarm_get_time,
	.alarm_is_pending = rtc_ameba_alarm_is_pending,//aa means what??
	.alarm_set_callback = rtc_ameba_alarm_set_callback,
#endif /* CONFIG_RTC_ALARM */

#if defined(CONFIG_RTC_UPDATE) || defined(__DOXYGEN__)
	.update_set_callback = rtc_ameba_update_set_callback,//aa not support?
#endif /* CONFIG_RTC_UPDATE */

#if defined(CONFIG_RTC_CALIBRATION) || defined(__DOXYGEN__)
	.set_calibration = rtc_ameba_set_calibration,
	.get_calibration = rtc_ameba_get_calibration,
#endif /* CONFIG_RTC_CALIBRATION */

};

static const struct rtc_ameba_config rtc_config = {
	.async_prescaler = 0x7F,
	.sync_prescaler = 0x00FF,
	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(0, idx),

	// .hour_fmt = RTC_HourFormat_24,
};

static struct rtc_ameba_data rtc_data;

DEVICE_DT_INST_DEFINE(0,									\
					  &rtc_ameba_init,						\
					  NULL,									\
					  &rtc_data,								\
					  &rtc_config,							\
					  PRE_KERNEL_1,							\
					  CONFIG_RTC_INIT_PRIORITY,				\
					  &rtc_ameba_driver_api);
