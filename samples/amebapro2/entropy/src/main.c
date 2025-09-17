/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>

#include <zephyr/drivers/entropy.h>

#define BUF_LEN 16

static int get_entropy(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));
	uint8_t buffer[BUF_LEN];

	memset(buffer, 0, sizeof(buffer));

	if (entropy_get_entropy(dev, buffer, BUF_LEN - 1) != 0) {
		printf("Error: entropy_get_entropy\n");
		return -1;
	}

	if (buffer[BUF_LEN - 1] != 0) {
		printf("Error: buffer overflow\n");
		return -2;
	}

	for (int i = 0; i < BUF_LEN - 1; i++) {
		printf("[%d] 0x%02x\n", i, buffer[i]);
	}

	return 0;
}

int main(void)
{
	printf("get_entropy=%d\n", get_entropy());

	return 0;
}
