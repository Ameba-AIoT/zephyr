/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba TRNG
 */
#define DT_DRV_COMPAT realtek_amebapro2_trng

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <hal_trng_sec.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/entropy.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ameba_trng, CONFIG_ENTROPY_LOG_LEVEL);

static int entropy_ameba_get_entropy(const struct device *dev, uint8_t *buf, uint16_t len)
{
	ARG_UNUSED(dev);

	uint32_t r = 0;

	if (NULL == buf || 0 == len) {
		LOG_ERR("Param error 0x%08x %d", (uint32_t)buf, len);
		return -EIO;
	}

	for (int i = 0; i < len / sizeof(uint32_t); i++) {
		r = hal_trng_sec_get_rand();
		memcpy(buf + i * sizeof(uint32_t), &r, sizeof(uint32_t));
	}

	if (len % sizeof(uint32_t)) {
		r = hal_trng_sec_get_rand();
		memcpy(buf + len / sizeof(uint32_t) * sizeof(uint32_t), &r, len % sizeof(uint32_t));
	}

	return 0;
}

static int entropy_ameba_get_entropy_isr(const struct device *dev, uint8_t *buf, uint16_t len,
					 uint32_t flags)
{
	ARG_UNUSED(flags);

	/* the TRNG may cost some time to generator the data,
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
	if (hal_trng_sec_init() != 0) {
		LOG_ERR("Could not enable TRNG clock");
		return -EIO;
	}

	return 0;
}

static const struct entropy_driver_api entropy_ameba_api_funcs = {
	.get_entropy = entropy_ameba_get_entropy,
	.get_entropy_isr = entropy_ameba_get_entropy_isr,
};

DEVICE_DT_INST_DEFINE(0, entropy_ameba_init, NULL, NULL, NULL, PRE_KERNEL_1,
		      CONFIG_ENTROPY_INIT_PRIORITY, &entropy_ameba_api_funcs);
