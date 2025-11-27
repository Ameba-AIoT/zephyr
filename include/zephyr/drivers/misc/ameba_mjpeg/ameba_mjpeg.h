/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_MISC_AMEBA_MJPEG_AMEBA_MJPEG_H_
#define ZEPHYR_DRIVERS_MISC_AMEBA_MJPEG_AMEBA_MJPEG_H_

/**
 * @brief MJPEG Interface
 * @defgroup mjpeg_interface MJPEG Interface
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

/**
 * @brief Jpeg format enum
 *
 * Jpeg format.
 */
enum mjpeg_fmt_type {
	JPEG_YCbCr400 = 0,
	JPEG_YCbCr420 = 1,
	JPEG_YCbCr422 = 2,
	JPEG_YCbCr440 = 3,
	JPEG_YCbCr411 = 4,
	JPEG_YCbCr444 = 5,
};

enum mjpeg_pix_fmt {
	PIX_RGB16_CUSTOM = 0,
	PIX_RGB16_5_5_5 = 1,
	PIX_RGB16_5_6_5 = 2,
	PIX_BGR16_5_5_5 = 3,
	PIX_BGR16_5_6_5 = 4,

	PIX_RGB32_CUSTOM = 5,
	PIX_RGB32 = 6,
	PIX_BGR32 = 7,
};

/**
 * @struct decoder_image_info
 * @brief Structure containing information about the jpeg image.
 */
struct decoder_image_info {
	uint32_t width;                 /* Number of pixels/line in the image  */
	uint32_t height;                /* Number of lines in in the image     */
	enum mjpeg_fmt_type format;     /* Jpeg format */
};

/**
 * @struct decoder_input_data
 * @brief Structure containing information about the required
 * resources for MJPEG Decoder.
 */
struct decoder_input_data {
	uint8_t *stream_buffer;         /* input stream buffer */
	uint32_t buffer_size;           /* input stream buffer size */
	uint32_t image_type;            /* Full image or Thumbnail to be decoded */
	uint32_t sliceMbSet;            /* slice mode: mcu rows to decode */
};

/**
 * @struct decoder_config
 * @brief Structure containing information about the required
 * resources for MJPEG Decoder.
 */
struct decoder_config {
	uint16_t width;                 /* image width */
	uint16_t height;                /* image height */
	uint16_t out_width;             /* out width */
	uint16_t out_height;            /* out height */
	enum mjpeg_pix_fmt pix_fmt;     /* image pix format */
	enum mjpeg_pix_fmt out_pix_fmt; /* out pix format */
	uint8_t *decode_data;           /* decode data */
};

/**
 * @typedef mjpeg_decoder_get_image_info
 * @brief API for get jpeg image information
 * See mjpeg_decoder_get_image_info() for argument description
 */
typedef int (*mjpeg_decoder_get_image_info_api)(const struct device *dev, 
									struct decoder_input_data *input_data,
									struct decoder_image_info *out_image_info);

/**
 * @typedef mjpeg_decoder_decode
 * @brief API for decode jpeg image
 * See mjpeg_decoder_decode() for argument description
 */
typedef int (*mjpeg_decoder_decode_api)(const struct device *dev, 
							struct decoder_input_data *input_data,
							struct decoder_config *dec_config);

__subsystem struct mjpeg_driver_api {
	mjpeg_decoder_get_image_info_api decoder_get_image_info;
	mjpeg_decoder_decode_api decoder_decode;
};

/**
 * @brief Get Image Information
 * @param[in] dev Pointer to the device structure for the MJPEG driver instance
 * @param[in] input_data Pointer to the decoder_input_data structure
 * @param[out] out_image_info Pointer to the decoder_image_info structure
 * @return 0 on success, negative values on error.
 */
static inline int mjpeg_decoder_get_image_info(struct decoder_input_data *input_data,
									struct decoder_image_info *out_image_info)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(mjpeg));

	if (dev) {
		const struct mjpeg_driver_api *api =
			(const struct mjpeg_driver_api *)dev->api;

		if (!api->decoder_get_image_info) {
			return -ENOTSUP;
		}

		return api->decoder_get_image_info(dev, input_data, out_image_info);
	}

	return -ENOTSUP;
}

/**
 * @brief Decode Jpeg Image
 * @param[in] dev Pointer to the device structure for the MJPEG driver instance
 * @param[in] input_data Pointer to the decoder_input_data structure
 * @param[out] out_data Pointer to the decoder_output_data structure
 * @return 0 on success, negative values on error.
 */
static inline int mjpeg_decoder_decode(struct decoder_input_data *input_data,
							struct decoder_config *dec_config)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(mjpeg));

	if (dev) {
		const struct mjpeg_driver_api *api =
			(const struct mjpeg_driver_api *)dev->api;

		if (!api->decoder_decode) {
			return -ENOTSUP;
		}

		return api->decoder_decode(dev, input_data, dec_config);
	}

	return -ENOTSUP;
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

#endif /* ZEPHYR_DRIVERS_MISC_AMEBA_MJPEG_AMEBA_MJPEG_H_ */
