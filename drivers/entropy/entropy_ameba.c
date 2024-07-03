/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba TRNG
 */

#define DT_DRV_COMPAT                   realtek_ameba_trng

#include <string.h>
#include <ameba_soc.h>
#include <soc.h>
#include <zephyr/drivers/clock_control/ameba_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(ameba_trng, CONFIG_ENTROPY_LOG_LEVEL);

static int entropy_ameba_get_entropy(const struct device *dev, uint8_t *buf, uint16_t len)
{
	ARG_UNUSED(dev);

	if (NULL == buf || 0 == len) {
		LOG_ERR("Param error 0x%08x %d", (uint32_t)buf, len);
		return -EIO;
	}

	/* Depend on the soc, maybe need some time to get the result */
	TRNG_get_random_bytes(buf, len);

	return 0;
}

static int entropy_ameba_get_entropy_isr(const struct device *dev,
		uint8_t *buf,
		uint16_t len, uint32_t flags)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(flags);

	/* the TRNG may cost some time to generator the data, the speed is 10Mbps
	*  we assume that it is fast enough for ISR use
	*/
	int ret = entropy_ameba_get_entropy(dev, buf, len);

	if (ret == 0) {
		/* Data retrieved successfully. */
		return len;
	}

	return ret;
}


static int entropy_ameba_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	const struct device *clock = AMEBA_CLOCK_CONTROL_DEV;
	const clock_control_subsys_t clock_subsys = (clock_control_subsys_t)DT_CLOCKS_CELL_BY_IDX(
				DT_NODELABEL(trng), 0, idx);

	if (!device_is_ready(clock)) {
		LOG_ERR("Clock control device not ready");
		return -ENODEV;
	}

	/* enable clock */
	if (clock_control_on(clock, clock_subsys)) {
		LOG_ERR("Could not enable TRNG clock %d", (uint32_t)clock_subsys);
		return -EIO;
	}

	return 0;
}

static const struct entropy_driver_api entropy_ameba_api_funcs = {
	.get_entropy = entropy_ameba_get_entropy,
	.get_entropy_isr = entropy_ameba_get_entropy_isr,
};

DEVICE_DT_INST_DEFINE(0,
					  entropy_ameba_init,
					  NULL,
					  NULL,
					  NULL,
					  PRE_KERNEL_1,
					  CONFIG_ENTROPY_INIT_PRIORITY,
					  &entropy_ameba_api_funcs);
