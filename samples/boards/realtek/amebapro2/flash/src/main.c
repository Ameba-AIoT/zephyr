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

static const struct device *flash_dev = DEVICE_DT_GET(DT_NODELABEL(spic));

static void test_flash(bool async)
{
	off_t offset = 0xf64000;
	size_t len = 16;

	__aligned(32) char test_buf_tx[len];
	__aligned(32) char test_buf_rx[len];

	for (int i = 0; i < len; i++) {
		test_buf_tx[i] = i;
	}

	flash_read(flash_dev, offset, test_buf_rx, 16);

	for (int i = 0; i < len; i++) {
		printf("test_buf_rx [%d] = 0x%x\r\n", i, test_buf_rx[i]);
	}

	flash_erase(flash_dev, offset, 16);
	flash_read(flash_dev, offset, test_buf_rx, 16);

	for (int i = 0; i < len; i++) {
		printf("test_buf_rx [%d] = 0x%x\r\n", i, test_buf_rx[i]);
	}
	flash_write(flash_dev, offset, test_buf_tx, 16);
	flash_read(flash_dev, offset, test_buf_rx, 16);

	for (int i = 0; i < len; i++) {
		printf("test_buf_rx [%d] = 0x%x\r\n", i, test_buf_rx[i]);
	}

	printf("flash example finish\r\n");
}

int main(void)
{
	printf("Flash example! %s\n", CONFIG_BOARD_TARGET);

	test_flash(false);

	return 0;
}
