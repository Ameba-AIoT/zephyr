/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_sdhc

#include <errno.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sdhc_ameba, CONFIG_SDHC_LOG_LEVEL);


struct sdhc_ameba_data {
	struct sdhc_host_props props;
};

/* SDHC configuration. */
struct sdhc_ameba_config {
	void (*irq_func)(void);
};

static int sdhc_ameba_init(const struct device *dev)
{
	LOG_INF("[%s]", __func__);

	return 0;
}

static int sdhc_ameba_card_busy(const struct device *dev)
{
	LOG_INF("[%s]", __func__);

	return 0;
}

static int sdhc_ameba_reset(const struct device *dev)
{
	LOG_INF("[%s]", __func__);

	return 0;
}

static int sdhc_ameba_request(const struct device *dev, struct sdhc_command *cmd,
			      struct sdhc_data *data)
{
	/* LOG_INF("[%s] opcode=%d", __func__, cmd->opcode); */

	return 0;
}

static int sdhc_ameba_get_card_present(const struct device *dev)
{
	LOG_INF("[%s]", __func__);

	/* 1: present; just for test purpose */
	return 1;
}

static int sdhc_ameba_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	LOG_INF("[%s]", __func__);

	return 0;
}

static int sdhc_ameba_set_io(const struct device *dev, struct sdhc_io *ios)
{
	LOG_INF("[%s]", __func__);

	return 0;
}

static DEVICE_API(sdhc, sdhc_ameba_driver_api) = {
	.reset = sdhc_ameba_reset,
	.request = sdhc_ameba_request,
	.set_io = sdhc_ameba_set_io,
	.get_card_present = sdhc_ameba_get_card_present,
	.card_busy = sdhc_ameba_card_busy,
	.get_host_props = sdhc_ameba_get_host_props,
	.enable_interrupt = NULL,
	.disable_interrupt = NULL,
	.execute_tuning = NULL,
};

#define SDHC_AMEBA_INIT(_num)                                                                    \
	static struct sdhc_ameba_data sdhc_ameba_data_##_num;                                      \
	static const struct sdhc_ameba_config sdhc_ameba_config_##_num = {                         \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(_num, sdhc_ameba_init, NULL, &sdhc_ameba_data_##_num,                \
			      &sdhc_ameba_config_##_num, POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY, \
				  &sdhc_ameba_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SDHC_AMEBA_INIT)
