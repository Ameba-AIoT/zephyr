/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_MISC_AMEBA_PPE_AMEBA_PPE_H_
#define ZEPHYR_DRIVERS_MISC_AMEBA_PPE_AMEBA_PPE_H_

/**
 * @brief PPE Interface
 * @defgroup ppe_interface PPE Interface
 * @since 2.1
 * @version 1.1.0
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/device.h>
#include <stddef.h>
#include <zephyr/kernel.h>

#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OPA_MAX 253

/**
 * @brief color_format_type enum
 *
 * Supported color format.
 */
enum ppe_color_format_type { //
	PPE_COLOR_FORMAT_RGB565 = 1,
	PPE_COLOR_FORMAT_RGB888 = 2,
	PPE_COLOR_FORMAT_XRGB8888 = 3,
	PPE_COLOR_FORMAT_ARGB8888 = 4,
};

/**
 * @struct ppe_header
 * @brief PPE header structure
 *
 * Used to configure layer format.
 */
struct ppe_header {
	uint32_t cf : 8;            /**< Color format*/
	uint32_t w: 16;
	uint32_t h: 16;
	uint32_t min_x: 16;
	uint32_t min_y: 16;
	uint32_t stride: 16;        /**< Number of bytes in a row*/
	uint32_t color: 32;         /**< Color*/
};

/**
 * @struct ppe_configuration
 * @brief PPE configuration 
 *
 * Used to describe ppe format configuration.
 */
struct ppe_configuration {
	void* src_buf;
	void* dest_buf;
	struct ppe_header *src_header;
	struct ppe_header *dest_header;
	float scale_x;              /**< can be 16/1, 16/2, 16/3, ..., 16/65535*/
	float scale_y;              /**< can be 16/1, 16/2, 16/3, ..., 16/65535*/
	uint32_t angle;             /**< can be 90/180/270*/
	uint32_t opa;               /**< can be 0-255*/
};

/**
 * @typedef ppe_configure_and_transfer_api
 * @brief Callback API for config ppe parameter and transfer
 * See configure_and_transfer() for argument description
 */
typedef int (*ppe_configure_and_transfer_api)(const struct device *dev,
				struct ppe_configuration *ppe_conf);

__subsystem struct ppe_driver_api {
	ppe_configure_and_transfer_api configure_transfer;
};

/**
 * @brief Config PPE and transfer buffer
 *
 * @param ppe_conf Pointer to a structure describing the ppe parameter
 *
 * @retval 0 on success else negative errno code.
 */
static inline int ppe_configure_and_transfer(struct ppe_configuration *ppe_conf)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(ppe));
	struct ppe_driver_api *api = (struct ppe_driver_api *)dev->api;

	return api->configure_transfer(dev, ppe_conf);
}


/**
 * @}
 */

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_DRIVERS_MISC_AMEBA_PPE_AMEBA_PPE_H_ */
