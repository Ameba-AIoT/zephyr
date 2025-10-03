/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

static const struct device *I2C_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

static void test_i2c(bool async)
{
	size_t len = 16;
	uint32_t slave_address = 0x55;

	__aligned(32) uint8_t test_buf_tx[len];
	__aligned(32) uint8_t test_buf_rx[len];
	struct i2c_msg msgs[1];

	msgs[0].buf = test_buf_tx;
	msgs[0].len = len;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	for (int i = 0; i < len; i++) {
		test_buf_tx[i] = i;
		test_buf_rx[i] = 0;
	}

	i2c_transfer(I2C_dev, msgs, 1, slave_address);

	msgs[0].buf = test_buf_rx;
	msgs[0].len = len;
	msgs[0].flags = I2C_MSG_READ | I2C_MSG_STOP;
	i2c_transfer(I2C_dev, msgs, 1, slave_address);

	for (int i = 0; i < len; i++) {
		printf("test_buf_rx [%d] = 0x%x\n\r", i, test_buf_rx[i]);
	}

	printf("I2C example finish\r\n");
}

int main(void)
{
	printf("I2C example! %s\n", CONFIG_BOARD_TARGET);

	test_i2c(false);

	return 0;
}
