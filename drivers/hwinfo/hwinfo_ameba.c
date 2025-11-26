/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/drivers/hwinfo.h>

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	uint8_t uuid[8];

	EFUSE_GetUUID((u32 *)uuid);

	if (length > sizeof(uuid)) {
		length = sizeof(uuid);
	}

	_memcpy(buffer, uuid, length);

	return length;
}

int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
	*supported = (RESET_POR | RESET_SOFTWARE | RESET_WATCHDOG | RESET_LOW_POWER_WAKE |
		      RESET_BROWNOUT);

	return 0;
}

int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
	uint32_t reason = BOOT_Reason();

	if (IS_SYS_RESET(reason)) {
		*cause = RESET_SOFTWARE;
	} else if (IS_WDG_RESET(reason)) {
		*cause = RESET_WATCHDOG;
	} else if ((reason & AON_BIT_RSTF_DSLP) != 0) {
		*cause = RESET_LOW_POWER_WAKE;
		return 0;
	} else if ((reason & AON_BIT_RSTF_BOR) != 0) {
		*cause = RESET_BROWNOUT;
	} else {
		*cause = RESET_POR;
	}
	return 0;
}
