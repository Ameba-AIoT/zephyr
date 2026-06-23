/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_TESTS_DRIVERS_CLOCK_CONTROL_API_AMEBA_DEVICE_SUBSYS_H_
#define ZEPHYR_TESTS_DRIVERS_CLOCK_CONTROL_API_AMEBA_DEVICE_SUBSYS_H_

#include "device_subsys.h"
#include <zephyr/drivers/clock_control/ameba_clock_control.h>

/* Use peripheral clocks that are OFF at boot and safe to toggle */
static const struct device_subsys_data subsys_data[] = {
	{.subsys = (clock_control_subsys_t)AMEBA_TRNG_CLK, .startup_us = 0},
	{.subsys = (clock_control_subsys_t)AMEBA_LEDC_CLK, .startup_us = 0},
};

static const struct device_data devices[] = {
	{
		.dev = DEVICE_DT_GET(AMEBA_CLOCK_CONTROL_NODE),
		.subsys_data = subsys_data,
		.subsys_cnt = ARRAY_SIZE(subsys_data),
	},
};

#endif /* ZEPHYR_TESTS_DRIVERS_CLOCK_CONTROL_API_AMEBA_DEVICE_SUBSYS_H_ */
