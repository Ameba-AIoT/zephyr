/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ameba_soc.h>
#include "bootutil/bootutil_public.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(nv_counter, CONFIG_LOG_DEFAULT_LEVEL);

#ifndef SEC_COUNTER_IMG0
#define SEC_COUNTER_IMG0 0x380 /* sysreg_sec 0x380 [31:0] for km4tz*/
#warning "Using default SEC_COUNTER_IMG0 = 0x380"
#endif

#ifndef SEC_COUNTER_IMG1
#define SEC_COUNTER_IMG1 0x380 /* sysreg_sec 0x380 [31:0] for km4ns */
#warning "Using default SEC_COUNTER_IMG1 = 0x380"
#endif

#define MAX_SEC_COUNTER_BITS 32u
#define WORD_BITS            32u
#define WORD_CNT             (MAX_SEC_COUNTER_BITS / WORD_BITS)

static bool get_addr_by_image(uint32_t image_id, uint32_t *addr)
{
	if (addr == NULL) {
		return false;
	}

	switch (image_id) {
	case 0u:
		*addr = SEC_COUNTER_IMG0;
		return true;
	case 1u:
		*addr = SEC_COUNTER_IMG1;
		return true;
	default:
		return false;
	}
}

/* Read WORD_CNT 32-bit words and store the results in buf */
static void otp_read_words(uint32_t addr, uint32_t *buf)
{
	for (uint8_t w = 0; w < WORD_CNT; w++) {
		OTP_Read32(addr + w * sizeof(uint32_t), &buf[w]);
	}
}

/* Count the number of consecutive 0s at LSB, stop when the first 1 is encountered */
static uint32_t count_trailing_zeros(const uint32_t *words)
{
	uint32_t counter = 0;

	for (uint8_t w = 0; w < WORD_CNT; w++) {
		uint32_t value = words[w];

		for (uint8_t bit = 0; bit < WORD_BITS; bit++) {
			if ((value >> bit) & 1u) {
				return counter;
			}
			counter++;
		}
	}
	return counter;
}

static void build_target_words(uint32_t img_security_cnt, uint32_t *target_words)
{
	uint32_t bit_pos = img_security_cnt;

	/* Initially set to all 1 */
	for (uint8_t w = 0; w < WORD_CNT; w++) {
		target_words[w] = 0xFFFFFFFFu;
	}

	for (uint8_t w = 0; w < WORD_CNT; w++) {
		if (bit_pos >= WORD_BITS) {
			/* This word is all 0 */
			target_words[w] = 0u;
			bit_pos -= WORD_BITS;
		} else {
			target_words[w] = ((~0u) << bit_pos);
			break;
		}
	}
}

static int32_t write_bytes(uint32_t addr, const uint32_t *target_value)
{
	for (uint8_t w = 0; w < WORD_CNT; w++) {
		for (uint8_t i = 0; i < sizeof(uint32_t); i++) {
			uint8_t target_byte = (target_value[w] >> (i * 8)) & 0xFFu;

			if (OTP_Write8(addr + w * sizeof(uint32_t) + i, target_byte) !=
			    RTK_SUCCESS) {
				LOG_ERR("OTP_Write8 failed at addr=0x%08x, byte=0x%02x",
					addr + w * sizeof(uint32_t) + i, target_byte);
				return -BOOT_EBADSTATUS;
			}
		}
	}
	return 0;
}

fih_ret boot_nv_security_counter_init(void)
{
	FIH_RET(FIH_SUCCESS);
}

fih_ret boot_nv_security_counter_get(uint32_t image_id, fih_int *security_counter)
{
	uint32_t addr;
	uint32_t current_value[WORD_CNT];
	uint32_t current_counter;

	if (!security_counter) {
		LOG_ERR("security_counter is NULL");
		FIH_RET(FIH_FAILURE);
	}
	if (!get_addr_by_image(image_id, &addr)) {
		LOG_ERR("Invalid image_id=%u", image_id);
		FIH_RET(FIH_FAILURE);
	}

	otp_read_words(addr, current_value);
	current_counter = count_trailing_zeros(current_value);
	LOG_INF("Get counter=%u, image_id=%u", current_counter, image_id);

	*security_counter = fih_int_encode(current_counter);
	FIH_RET(FIH_SUCCESS);
}

/**
 * @brief Update the security counter in OTP memory
 *
 * The counter is encoded as the number of consecutive zero bits starting from
 * the least significant bit (LSB).
 * Examples (hex value -> lowest bits -> counter):
 * - 0xFFFFFFFF -> ...1111b  - counter -> 0
 * - 0xFFFFFFFE -> ...1110b  - counter -> 1
 * - 0xFFFFFFFC -> ...1100b  - counter -> 2
 * - ...
 *
 * @param image_id Image identifier (0 or 1)
 * @param img_security_cnt New security counter value
 *
 * @return 0 on success
 * @return -BOOT_EBADARGS if arguments are invalid
 * @return -BOOT_EBADSTATUS if OTP operation fails
 */
int32_t boot_nv_security_counter_update(uint32_t image_id, uint32_t img_security_cnt)
{
	uint32_t addr;
	uint32_t current_counter;
	uint32_t current_value[WORD_CNT];
	uint32_t target_value[WORD_CNT];
	int32_t ret;

	if (img_security_cnt > MAX_SEC_COUNTER_BITS) {
		LOG_ERR("Invalid security count: %u (max: %u)", img_security_cnt,
			MAX_SEC_COUNTER_BITS);
		return -BOOT_EBADARGS;
	}

	if (!get_addr_by_image(image_id, &addr)) {
		LOG_ERR("Invalid image_id=%u", image_id);
		return -BOOT_EBADARGS;
	}
	otp_read_words(addr, current_value);
	current_counter = count_trailing_zeros(current_value);

	/* Rollback not allowed */
	if (img_security_cnt < current_counter) {
		LOG_ERR("Rollback attempt: current=%u, requested=%u", current_counter,
			img_security_cnt);
		return -BOOT_EBADARGS;
	}

	/* If the values are the same, no update is required */
	if (img_security_cnt == current_counter) {
		return 0;
	}

	/* Set the low img_security_cnt bits to 0, and the rest to 1 */
	build_target_words(img_security_cnt, target_value);
	for (uint8_t i = 0; i < WORD_CNT; i++) {
		if ((current_value[i] & target_value[i]) != target_value[i]) {
			LOG_ERR("OTP violation: 0->1 attempt (current =0x%08x, target =0x%08x)",
				current_value[i], target_value[i]);
			return -BOOT_EBADARGS;
		}
	}

	ret = write_bytes(addr, target_value);
	if (ret != 0) {
		return ret;
	}

	LOG_INF("Update success - image_id=%u, current_counter=%u -> %u", image_id, current_counter,
		img_security_cnt);
	return 0;
}
