/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba I2C interface
 */
#define DT_DRV_COMPAT realtek_ameba_i2c

#include <soc.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_ameba, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"

struct i2c_ameba_data {
	struct k_sem cmd_sem;
	struct k_sem transfer_sem;
	// volatile enum i2c_status_t status;
	uint32_t dev_config;
	int cmd_idx;
	int irq_line;
};

typedef void (*irq_connect_cb)(void);

struct i2c_ameba_config {
	int index;

	const struct device *clock_dev;
	const struct pinctrl_dev_config *pcfg;

	const clock_control_subsys_t clock_subsys;

	const struct {
		bool tx_lsb_first;
		bool rx_lsb_first;
	} mode;

	int irq_source;

	const uint32_t default_config;
	const uint32_t bitrate;
	const uint32_t scl_timeout;
};

static int i2c_ameba_recover(const struct device *dev)
{
	struct i2c_ameba_data *data = (struct i2c_ameba_data * const)(dev)->data;

	k_sem_take(&data->transfer_sem, K_FOREVER);

	//

	k_sem_give(&data->transfer_sem);

	return 0;
}

static int i2c_ameba_configure(const struct device *dev, uint32_t dev_config)
{
	return 0;
}

static int i2c_ameba_transfer(const struct device *dev, struct i2c_msg *msgs,
							  uint8_t num_msgs, uint16_t addr)
{
	return 0;
}

static const struct i2c_driver_api i2c_ameba_driver_api = {
	.configure = i2c_ameba_configure,
	.transfer = i2c_ameba_transfer,
	.recover_bus = i2c_ameba_recover
};

static int i2c_ameba_init(const struct device *dev)
{
	return 0;
}

#define AMEBA_I2C_INIT(n)						   \
												   \
	static struct i2c_ameba_data i2c_ameba_data_##n = {		   \
																   \
		.cmd_sem = Z_SEM_INITIALIZER(i2c_ameba_data_##n.cmd_sem, 0, 1),		   \
		.transfer_sem = Z_SEM_INITIALIZER(i2c_ameba_data_##n.transfer_sem, 1, 1),	   \
	};											   \
												   \
	static const struct i2c_ameba_config i2c_ameba_config_##n = {				   \
		.index = n,									   \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),				   \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, idx),	   \
		.default_config = I2C_MODE_CONTROLLER,						   \
	};					\
						\
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_ameba_init, NULL, &i2c_ameba_data_##n,		   \
			     &i2c_ameba_config_##n, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,	   \
			     &i2c_ameba_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AMEBA_I2C_INIT)
