/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>
#include "hal.h"

struct test_data {
	struct k_work_delayable test_work;
	struct k_sem sem;
	int spim_alloc_idx;
	int spis_alloc_idx;
	struct spi_buf_set sets[4];
	struct spi_buf_set *mtx_set;
	struct spi_buf_set *mrx_set;
	struct spi_buf_set *stx_set;
	struct spi_buf_set *srx_set;
	struct spi_buf bufs[8];
	bool async;
};

static struct test_data tdata;

static const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi0));

#define SPI_MODE (SPI_WORD_SET(8) | SPI_LINES_SINGLE | SPI_TRANSFER_MSB)

#define SPIM_OP (SPI_OP_MODE_MASTER | SPI_MODE)

static const struct spi_config spi0_config = {
	.frequency = 1000000,
	.operation = SPIM_OP,

};

static void test_only_tx(bool async)
{
	size_t len = 16;

	__aligned(32) char test_buf_tx[len];
	__aligned(32) char test_buf_rx[len];

	for (int i = 0; i < len; i++) {
		test_buf_tx[i] = i;
	}

	/* MTX buffer */
	tdata.bufs[0].buf = &test_buf_tx[0];
	tdata.bufs[0].len = len;
	tdata.sets[0].buffers = &tdata.bufs[0];
	tdata.sets[0].count = tdata.bufs[0].len;
	tdata.bufs[1].buf = &test_buf_rx[0];
	tdata.bufs[1].len = 0;
	tdata.sets[1].buffers = &tdata.bufs[1];
	tdata.sets[1].count = tdata.bufs[1].len;
	tdata.mtx_set = &tdata.sets[0];
	tdata.mrx_set = &tdata.sets[1];

	printf("spi_tx!\n\r");

	spi_transceive(spi_dev, &spi0_config, tdata.mtx_set, tdata.mrx_set);
	spi_release(spi_dev, &spi0_config);
	hal_delay_us(5000000);

	for (int i = 0; i < len; i++) {
		test_buf_tx[i] = 0;
	}

	/* MRX buffer */
	tdata.bufs[0].buf = &test_buf_tx[0];
	tdata.bufs[0].len = 0;
	tdata.sets[0].buffers = &tdata.bufs[0];
	tdata.sets[0].count = tdata.bufs[0].len;
	tdata.bufs[1].buf = &test_buf_rx[0];
	tdata.bufs[1].len = len;
	tdata.sets[1].buffers = &tdata.bufs[1];
	tdata.sets[1].count = tdata.bufs[1].len;
	tdata.mtx_set = &tdata.sets[0];
	tdata.mrx_set = &tdata.sets[1];

	printf("spi_rx!\n\r");
	spi_transceive(spi_dev, &spi0_config, tdata.mtx_set, tdata.mrx_set);

	for (int i = 0; i < len; i++) {
		printf("test_buf_rx [i] = 0x%x\n\r", test_buf_rx[i]);
	}

	spi_release(spi_dev, &spi0_config);
	printf("spi example finish\n\r");
}

int main(void)
{
	printf("SPI example! %s\n", CONFIG_BOARD_TARGET);

	test_only_tx(false);

	return 0;
}
