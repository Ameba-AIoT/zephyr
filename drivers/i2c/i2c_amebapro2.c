/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba I2C interface
 */
#define DT_DRV_COMPAT realtek_amebapro2_i2c

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <hal_i2c.h>

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/clock_control.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_ameba, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"

hal_i2c_adapter_t i2c_adp;

struct i2c_ameba_data {
	uint32_t master_mode;
	uint32_t addr_mode;
	uint32_t slave_address;
	struct i2c_msg *current;
	volatile int flag_done;
};

struct i2c_ameba_config {
	int index;
	uint32_t bitrate;
#if defined(CONFIG_I2C_AMEBA_INTERRUPT)
	void (*irq_cfg_func)(void);
#endif
};

void i2c_master_txc_callback(const struct device *dev)
{
	struct i2c_ameba_data *data = dev->data;

	data->flag_done = 1;
}

void i2c_master_rxc_callback(const struct device *dev)
{
	struct i2c_ameba_data *data = dev->data;

	data->flag_done = 1;
}

void i2c_master_rd_req_callback(const struct device *dev)
{
	struct i2c_ameba_data *data = dev->data;

	data->flag_done = 1;
}

void i2c_master_err_callback(const struct device *dev)
{
	struct i2c_ameba_data *data = dev->data;

	LOG_ERR("master err callback %s\n\r", dev->name);
	data->flag_done = 1;
}

static int i2c_ameba_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_ameba_config *config = dev->config;
	hal_i2c_adapter_t *phal_i2c_adapter = (hal_i2c_adapter_t *)(&i2c_adp);
	uint32_t bitrate_cfg;
	int err = 0;

	bitrate_cfg = config->bitrate;

	hal_i2c_load_default(&i2c_adp, 1); /* PF1/PF2 */
	i2c_adp.op_mode = I2CModeInterrupt;
	hal_i2c_init(&i2c_adp, PIN_F1, PIN_F2);
	phal_i2c_adapter->init_dat.clock = bitrate_cfg / 1000;
	hal_i2c_set_clk(phal_i2c_adapter);

#if defined(CONFIG_I2C_AMEBA_INTERRUPT)
	phal_i2c_adapter->usr_cb.txc.cb = (void (*)(void *))i2c_master_txc_callback;
	phal_i2c_adapter->usr_cb.txc.dat = (uint32_t)dev;
	phal_i2c_adapter->usr_cb.rxc.cb = (void (*)(void *))i2c_master_rxc_callback;
	phal_i2c_adapter->usr_cb.rxc.dat = (uint32_t)dev;
	phal_i2c_adapter->usr_cb.rd_req.cb = (void (*)(void *))i2c_master_rd_req_callback;
	phal_i2c_adapter->usr_cb.rd_req.dat = (uint32_t)dev;
	phal_i2c_adapter->usr_cb.err.cb = (void (*)(void *))i2c_master_err_callback;
	phal_i2c_adapter->usr_cb.err.dat = (uint32_t)dev;
#endif

	return err;
}

static void i2c_wait_tx(hal_i2c_adapter_t *phal_i2c_adapter)
{
	uint32_t i2c_time_start;
	uint32_t i2c_timeout_bak;

	i2c_time_start = hal_read_cur_time();
	i2c_timeout_bak = phal_i2c_adapter->pltf_dat.tr_time_out;
	phal_i2c_adapter->pltf_dat.tr_time_out = HP_I2C_TIMEOUT_DISABLE;

	while ((phal_i2c_adapter->status == I2CStatusTxReady) ||
	       (phal_i2c_adapter->status == I2CStatusTxing)) {
		if (hal_i2c_timeout_chk(phal_i2c_adapter, i2c_time_start)) {
			if (phal_i2c_adapter->pltf_dat.tr_time_out != HP_I2C_TIMEOUT_DISABLE) {
				phal_i2c_adapter->status = I2CStatusTimeOut;
			}

			break;
		}
	}

	phal_i2c_adapter->pltf_dat.tr_time_out = i2c_timeout_bak;
}

static void i2c_wait_rx(hal_i2c_adapter_t *phal_i2c_adapter)
{
	uint32_t i2c_time_start;
	uint32_t i2c_timeout_bak;

	i2c_time_start = hal_read_cur_time();
	i2c_timeout_bak = phal_i2c_adapter->pltf_dat.tr_time_out;
	phal_i2c_adapter->pltf_dat.tr_time_out = HP_I2C_TIMEOUT_DISABLE;

	while ((phal_i2c_adapter->status == I2CStatusRxReady) ||
	       (phal_i2c_adapter->status == I2CStatusRxing)) {
		if (hal_i2c_timeout_chk(phal_i2c_adapter, i2c_time_start)) {
			if (phal_i2c_adapter->pltf_dat.tr_time_out != HP_I2C_TIMEOUT_DISABLE) {
				phal_i2c_adapter->status = I2CStatusTimeOut;
			}

			break;
		}
	}

	phal_i2c_adapter->pltf_dat.tr_time_out = i2c_timeout_bak;
}

static int i2c_ameba_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			      uint16_t addr)
{
	struct i2c_ameba_data *data = dev->data;
	int err = 0;

	data->slave_address = addr;

#if defined(CONFIG_I2C_AMEBA_INTERRUPT)
	for (uint8_t i = 0; i < num_msgs; ++i) {
		data->current = &msgs[i];
		if (data->master_mode == 1) {
			data->flag_done = 0;
			hal_i2c_adapter_t *phal_i2c_adapter = (hal_i2c_adapter_t *)(&i2c_adp);

			if ((data->current->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
				if (phal_i2c_adapter->tx_dat.addr != addr) {
					phal_i2c_adapter->tx_dat.addr = addr;
				}

				phal_i2c_adapter->tx_dat.buf = data->current->buf;
				phal_i2c_adapter->tx_dat.len = data->current->len;
				phal_i2c_adapter->tx_dat.mst_stop =
					(data->current->flags & I2C_MSG_STOP) >> 1;

				if (hal_i2c_send(phal_i2c_adapter) == HAL_OK) {
					i2c_wait_tx(phal_i2c_adapter);
				}

				while (data->flag_done == 0) {
				}
			} else {
				if (phal_i2c_adapter->rx_dat.addr != addr) {
					phal_i2c_adapter->rx_dat.addr = addr;
				}

				phal_i2c_adapter->rx_dat.buf = data->current->buf;
				phal_i2c_adapter->rx_dat.len = data->current->len;
				phal_i2c_adapter->rx_dat.mst_stop =
					(data->current->flags & I2C_MSG_STOP) >> 1;

				if (hal_i2c_receive(phal_i2c_adapter) == HAL_OK) {
					i2c_wait_rx(phal_i2c_adapter);
				}

				while (data->flag_done == 0) {
				}
			}
		} else {
			LOG_ERR("only support i2c master %s\n\r", dev->name);
			return -1;
		}
	}
#else
	LOG_ERR("only support interrupt mode  %s\n\r", dev->name);
	return -1;
#endif
	return err;
}

static const struct i2c_driver_api i2c_ameba_driver_api = {
	.configure = i2c_ameba_configure,
	.transfer = i2c_ameba_transfer,
};

static int i2c_ameba_init(const struct device *dev)
{
	struct i2c_ameba_data *data = dev->data;
	const struct i2c_ameba_config *config = dev->config;
	uint32_t bitrate_cfg;

	if (data->master_mode != 1) {
		LOG_ERR("only support i2c master %s\n\r", dev->name);
		return -1;
	}

	bitrate_cfg = config->bitrate;
	i2c_ameba_configure(dev, bitrate_cfg);

	return 0;
}

#define I2C_ADDR_7BIT 0
static struct i2c_ameba_data i2c_ameba_data = {
	.master_mode = 1, .addr_mode = I2C_ADDR_7BIT, .slave_address = 0x0};

static const struct i2c_ameba_config i2c_ameba_config = {.index = 1,
							 .bitrate = I2C_BITRATE_STANDARD};
I2C_DEVICE_DT_INST_DEFINE(0, i2c_ameba_init, NULL, &i2c_ameba_data, &i2c_ameba_config, POST_KERNEL,
			  CONFIG_I2C_INIT_PRIORITY, &i2c_ameba_driver_api);
