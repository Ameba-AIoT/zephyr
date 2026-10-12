/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_otp

#include <string.h>

#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/drivers/otp.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(otp_ameba, CONFIG_OTP_LOG_LEVEL);

#define OTP_AMEBA_PHYSICAL DT_INST_CHILD(0, physical)
#define OTP_AMEBA_LOGICAL  DT_INST_CHILD(0, logical)

static K_MUTEX_DEFINE(otp_ameba_lock);

struct otp_ameba_config {
	off_t start;
	size_t size;
	int (*read)(off_t offset, uint8_t *buf, size_t len);
#ifdef CONFIG_OTP_PROGRAM
	int (*program)(off_t offset, const uint8_t *buf, size_t len);
#endif
};

static bool otp_ameba_in_range(off_t offset, size_t len, size_t size)
{
	return (offset >= 0) && (len <= size) && ((size_t)offset <= size - len);
}

#if DT_NODE_HAS_STATUS_OKAY(OTP_AMEBA_PHYSICAL)
static int otp_ameba_physical_read(off_t offset, uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if (OTP_Read8(offset + i, &buf[i]) != RTK_SUCCESS) {
			LOG_ERR("Read failed at 0x%lx", (long)(offset + i));
			return -EIO;
		}
	}

	return 0;
}

#ifdef CONFIG_OTP_PROGRAM
static int otp_ameba_physical_program(off_t offset, const uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if (OTP_Write8(offset + i, buf[i]) != RTK_SUCCESS) {
			LOG_ERR("Program failed at 0x%lx", (long)(offset + i));
			return -EIO;
		}
	}

	return 0;
}
#endif /* CONFIG_OTP_PROGRAM */
#endif /* DT_NODE_HAS_STATUS_OKAY(OTP_AMEBA_PHYSICAL) */

#if DT_NODE_HAS_STATUS_OKAY(OTP_AMEBA_LOGICAL)
#if defined(CONFIG_SOC_SERIES_AMEBAD)
/*
 * On amebad OTP_LogicalRead() dumps the whole logical map into pbuf, then moves the
 * requested [offset, offset + len) bytes to the start of pbuf. So pbuf must hold
 * OTP_LMAP_LEN bytes, and the result is read back from pbuf[0].
 */
static uint8_t otp_ameba_lmap[OTP_LMAP_LEN];
#endif

static int otp_ameba_logical_read(off_t offset, uint8_t *buf, size_t len)
{
#if defined(CONFIG_SOC_SERIES_AMEBAD)
	if (OTP_LogicalRead(otp_ameba_lmap, offset, len) != RTK_SUCCESS) {
		LOG_ERR("Logical read failed at 0x%lx", (long)offset);
		return -EIO;
	}

	/* The HAL has already moved the requested bytes to the start of the buffer */
	memcpy(buf, otp_ameba_lmap, len);
#else
	if (OTP_LogicalRead(buf, offset, len) != RTK_SUCCESS) {
		LOG_ERR("Logical read failed at 0x%lx", (long)offset);
		return -EIO;
	}
#endif

	return 0;
}

#ifdef CONFIG_OTP_PROGRAM
static int otp_ameba_logical_program(off_t offset, const uint8_t *buf, size_t len)
{
	/* The HAL does not modify the data, it only takes a non-const pointer */
	if (OTP_LogicalWrite(offset, len, (uint8_t *)buf) != RTK_SUCCESS) {
		LOG_ERR("Logical program failed at 0x%lx", (long)offset);
		return -EIO;
	}

	return 0;
}
#endif /* CONFIG_OTP_PROGRAM */
#endif /* DT_NODE_HAS_STATUS_OKAY(OTP_AMEBA_LOGICAL) */

static int otp_ameba_read(const struct device *dev, off_t offset, void *buf, size_t len)
{
	const struct otp_ameba_config *config = dev->config;
	int ret;

	if (!otp_ameba_in_range(offset, len, config->size)) {
		LOG_ERR("Invalid range: offset 0x%lx, len %zu", (long)offset, len);
		return -EINVAL;
	}

	k_mutex_lock(&otp_ameba_lock, K_FOREVER);
	ret = config->read(config->start + offset, buf, len);
	k_mutex_unlock(&otp_ameba_lock);

	return ret;
}

#ifdef CONFIG_OTP_PROGRAM
static int otp_ameba_program(const struct device *dev, off_t offset, const void *buf, size_t len)
{
	const struct otp_ameba_config *config = dev->config;
	int ret;

	if (!otp_ameba_in_range(offset, len, config->size)) {
		LOG_ERR("Invalid range: offset 0x%lx, len %zu", (long)offset, len);
		return -EINVAL;
	}

	k_mutex_lock(&otp_ameba_lock, K_FOREVER);
	ret = config->program(config->start + offset, buf, len);
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

#ifdef CONFIG_OTP_PROGRAM
#define OTP_AMEBA_PROGRAM(fn) .program = fn,
#else
#define OTP_AMEBA_PROGRAM(fn)
#endif

#define OTP_AMEBA_DEFINE(node_id, read_fn, program_fn)                                             \
	static const struct otp_ameba_config _CONCAT(otp_ameba_config_, node_id) = {               \
		.start = DT_PROP_BY_IDX(node_id, address_range, 0),                                \
		.size = DT_PROP_BY_IDX(node_id, address_range, 1),                                 \
		.read = read_fn,                                                                   \
		OTP_AMEBA_PROGRAM(program_fn)};                                                    \
	DEVICE_DT_DEFINE(node_id, NULL, NULL, NULL, &_CONCAT(otp_ameba_config_, node_id),          \
			 POST_KERNEL, CONFIG_OTP_INIT_PRIORITY, &otp_ameba_api)

#if DT_NODE_HAS_STATUS_OKAY(OTP_AMEBA_PHYSICAL)
OTP_AMEBA_DEFINE(OTP_AMEBA_PHYSICAL, otp_ameba_physical_read, otp_ameba_physical_program);
#endif

#if DT_NODE_HAS_STATUS_OKAY(OTP_AMEBA_LOGICAL)
OTP_AMEBA_DEFINE(OTP_AMEBA_LOGICAL, otp_ameba_logical_read, otp_ameba_logical_program);
#endif
