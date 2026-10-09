/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_otp

#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/drivers/otp.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(otp_ameba, CONFIG_OTP_LOG_LEVEL);

static K_MUTEX_DEFINE(otp_ameba_lock);

struct otp_ameba_config {
	size_t size;
};

static bool otp_ameba_range_valid(const struct device *dev, off_t offset, size_t len)
{
	const struct otp_ameba_config *config = dev->config;

	return (offset >= 0) && (len <= config->size) && ((size_t)offset <= config->size - len);
}

static int otp_ameba_read(const struct device *dev, off_t offset, void *buf, size_t len)
{
	uint8_t *dst = buf;
	int ret = 0;

	if (!otp_ameba_range_valid(dev, offset, len)) {
		LOG_ERR("Invalid range: offset 0x%lx, len %zu", (long)offset, len);
		return -EINVAL;
	}

	k_mutex_lock(&otp_ameba_lock, K_FOREVER);

	for (size_t i = 0; i < len; i++) {
		if (OTP_Read8(offset + i, &dst[i]) != RTK_SUCCESS) {
			LOG_ERR("Read failed at 0x%lx", (long)(offset + i));
			ret = -EIO;
			break;
		}
	}

	k_mutex_unlock(&otp_ameba_lock);

	return ret;
}

#ifdef CONFIG_OTP_PROGRAM
static int otp_ameba_program(const struct device *dev, off_t offset, const void *buf, size_t len)
{
	const uint8_t *src = buf;
	int ret = 0;

	if (!otp_ameba_range_valid(dev, offset, len)) {
		LOG_ERR("Invalid range: offset 0x%lx, len %zu", (long)offset, len);
		return -EINVAL;
	}

	k_mutex_lock(&otp_ameba_lock, K_FOREVER);

	for (size_t i = 0; i < len; i++) {
		if (OTP_Write8(offset + i, src[i]) != RTK_SUCCESS) {
			LOG_ERR("Program failed at 0x%lx", (long)(offset + i));
			ret = -EIO;
			break;
		}
	}

	k_mutex_unlock(&otp_ameba_lock);

	return ret;
}
#endif /* CONFIG_OTP_PROGRAM */

static DEVICE_API(otp, otp_ameba_api) = {
	.read = otp_ameba_read,
#ifdef CONFIG_OTP_PROGRAM
	.program = otp_ameba_program,
#endif
};

static const struct otp_ameba_config otp_ameba_config = {
	.size = DT_INST_PROP(0, size),
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &otp_ameba_config, POST_KERNEL, CONFIG_OTP_INIT_PRIORITY,
		      &otp_ameba_api);
