/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_lcdc

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/drivers/display.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(display_ameba_lcdc, CONFIG_DISPLAY_LOG_LEVEL);

#if CONFIG_AMEBA_LCDC_ARGB8888
#define AMEBA_LCDC_INIT_PIXEL_SIZE   4u
#define AMEBA_LCDC_INIT_PIXEL_FORMAT LCDC_INPUT_FORMAT_ARGB8888
#define DISPLAY_INIT_PIXEL_FORMAT    PIXEL_FORMAT_ARGB_8888
#elif CONFIG_AMEBA_LCDC_RGB888
#define AMEBA_LCDC_INIT_PIXEL_SIZE   3u
#define AMEBA_LCDC_INIT_PIXEL_FORMAT LCDC_INPUT_FORMAT_RGB888
#define DISPLAY_INIT_PIXEL_FORMAT    PIXEL_FORMAT_RGB_888
#elif CONFIG_AMEBA_LCDC_RGB565
#define AMEBA_LCDC_INIT_PIXEL_SIZE   2u
#define AMEBA_LCDC_INIT_PIXEL_FORMAT LCDC_INPUT_FORMAT_RGB565
#define DISPLAY_INIT_PIXEL_FORMAT    PIXEL_FORMAT_RGB_565
#else
#error "Invalid LCDC pixel format chosen"
#endif

#define GET_LCDC_RGB_INTERFACE(idx)                                                                \
	((idx) == 0   ? LCDC_RGB_IF_6_BIT                                                          \
	 : (idx) == 1 ? LCDC_RGB_IF_8_BIT                                                          \
	 : (idx) == 2 ? LCDC_RGB_IF_16_BIT                                                         \
		      : LCDC_RGB_IF_24_BIT)

#define GET_LCDC_RGB_OUTPUT_FORMAT(panel_format)                                                   \
	((panel_format) == PANEL_PIXEL_FORMAT_RGB_888   ? LCDC_OUTPUT_FORMAT_RGB888                \
	 : (panel_format) == PANEL_PIXEL_FORMAT_RGB_565 ? LCDC_OUTPUT_FORMAT_RGB565                \
	 : (panel_format) == PANEL_PIXEL_FORMAT_BGR_565 ? LCDC_OUTPUT_FORMAT_BGR565                \
							: LCDC_OUTPUT_FORMAT_RGB888)

#define AMEBA_LCDC_FRAME_BUFFER_LEN(inst)                                                          \
	(AMEBA_LCDC_INIT_PIXEL_SIZE * DT_INST_PROP(inst, height) * DT_INST_PROP(inst, width))

/* Define the heap size. 512 bytes of padding are included for kernel heap structures */
/*
 * K_HEAP_DEFINE(display_heap, AMEBA_LCDC_FRAME_BUFFER_LEN(0) * CONFIG_AMEBA_LCDC_FB_NUM + 512);
 */
#define LCDC_FRAME_BUFFER_ADDR 0x60131000

/* #define CONFIG_AMEBA_LCDC_FB_NUM		2 */

struct lcdc_ameba_data {
	LCDC_RGBInitTypeDef rgb_init;

	enum display_pixel_format current_pixel_format;
	uint8_t current_pixel_size;

	uint8_t *frame_buffer;
	uint32_t frame_buffer_len;

	const uint8_t *pend_buf;
	const uint8_t *front_buf;

	struct k_sem sem;
};

struct lcdc_ameba_config {
	/* may separate iot driver to two parts or indenpent apis */
	Panel_InitDef lcdc_panel;
	Panel_RgbTimingDef lcdc_timing;

	/* pinctrl info */
	const struct pinctrl_dev_config *pincfg;

	/* register ISR and enable IRQn */
	void (*irq_config_func)(void);

	/* display and backlight gpios */
	struct gpio_dt_spec disp_on_gpio;
	struct gpio_dt_spec bl_ctrl_gpio;
};

static void lcdc_ameba_isr(const struct device *dev)
{
	/* const struct lcdc_ameba_config *cfg = dev->config; */
	struct lcdc_ameba_data *data = dev->data;
	uint32_t status;

	status = LCDC_GetINTStatus(LCDC);
	LCDC_ClearINT(LCDC, status);

	if (status & LCDC_BIT_LCD_LIN_INTS) {
		LOG_DBG("intr: line hit \r\n");

		if (data->front_buf != data->pend_buf) {
			data->front_buf = data->pend_buf;
			LCDC_DMAImgCfg(LCDC, (u32)(data->pend_buf));

			LCDC_ShadowReloadConfig(LCDC);
			k_sem_give(&data->sem);
		}
	}

	if (status & LCDC_BIT_DMA_UN_INTS) {
		LOG_WRN("intr: dma udf !!! \r\n");
	}
}

static int lcdc_ameba_write(const struct device *dev, const uint16_t x, const uint16_t y,
			    const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct lcdc_ameba_config *config = dev->config;
	struct lcdc_ameba_data *data = dev->data;
	uint8_t *dst = NULL;
	const uint8_t *pend_buf = NULL;
	const uint8_t *src = buf;
	uint16_t row;

	if ((x == 0) && (y == 0) && (desc->width == config->lcdc_panel.ImgWidth) &&
	    (desc->height == config->lcdc_panel.ImgHeight) && (desc->pitch == desc->width)) {
		/* Use buf as ltdc frame buffer directly if it length same as ltdc frame buffer. */
		pend_buf = buf;
	} else {
		if (CONFIG_AMEBA_LCDC_FB_NUM == 0) {
			LOG_ERR("Partial write requires internal frame buffer");
			return -ENOTSUP;
		}

		dst = data->frame_buffer;

		if (CONFIG_AMEBA_LCDC_FB_NUM == 2) {
			if (data->front_buf == data->frame_buffer) {
				dst = data->frame_buffer + data->frame_buffer_len;
			}

			memcpy(dst, data->front_buf, data->frame_buffer_len);
		}

		pend_buf = dst;

		/* dst = pointer to upper left pixel of the rectangle
		 *       to be updated in frame buffer.
		 */
		dst += (x * data->current_pixel_size);
		dst += (y * config->lcdc_panel.ImgWidth * data->current_pixel_size);

		for (row = 0; row < desc->height; row++) {
			(void)memcpy(dst, src, desc->width * data->current_pixel_size);
			sys_cache_data_flush_range(dst, desc->width * data->current_pixel_size);
			dst += (config->lcdc_panel.ImgWidth * data->current_pixel_size);
			src += (desc->pitch * data->current_pixel_size);
		}
	}

	if (data->front_buf == pend_buf) {
		return 0;
	}

	k_sem_reset(&data->sem);

	data->pend_buf = pend_buf;

	LCDC_ClearINT(LCDC, LCDC_BIT_LCD_LIN_INTS);
	LCDC_INTConfig(LCDC, LCDC_BIT_LCD_LIN_INTEN, ENABLE);
	k_sem_take(&data->sem, K_FOREVER);

	LCDC_INTConfig(LCDC, LCDC_BIT_LCD_LIN_INTEN, DISABLE);

	return 0;
}

static int lcdc_ameba_display_blanking_off(const struct device *dev)
{
	const struct lcdc_ameba_config *config = dev->config;

	return gpio_pin_set_dt(&config->bl_ctrl_gpio, 1);
}

static int lcdc_ameba_display_blanking_on(const struct device *dev)
{
	const struct lcdc_ameba_config *config = dev->config;

	return gpio_pin_set_dt(&config->bl_ctrl_gpio, 0);
}

static int lcdc_ameba_set_pixel_format(const struct device *dev,
				       const enum display_pixel_format pixel_format)
{
	int err = 0;
	struct lcdc_ameba_data *data = dev->data;

	switch (pixel_format) {
	case PIXEL_FORMAT_RGB_565:
		LCDC_ColorFomatInputConfig(LCDC, LCDC_INPUT_FORMAT_RGB565);
		data->current_pixel_format = PIXEL_FORMAT_RGB_565;
		data->current_pixel_size = 2u;
		break;
	case PIXEL_FORMAT_RGB_888:
		LCDC_ColorFomatInputConfig(LCDC, LCDC_INPUT_FORMAT_RGB888);
		data->current_pixel_format = PIXEL_FORMAT_RGB_888;
		data->current_pixel_size = 3u;
		break;
	case PIXEL_FORMAT_ARGB_8888:
		LCDC_ColorFomatInputConfig(LCDC, LCDC_INPUT_FORMAT_ARGB8888);
		data->current_pixel_format = PIXEL_FORMAT_ARGB_8888;
		data->current_pixel_size = 4u;
		break;
	default:
		err = -ENOTSUP;
		break;
	}

	return err;
}

static int lcdc_ameba_set_orientation(const struct device *dev,
				      const enum display_orientation orientation)
{
	if (orientation == DISPLAY_ORIENTATION_NORMAL) {
		return 0;
	}
	LOG_ERR("Changing display orientation not implemented");
	return -ENOTSUP;
}

static void lcdc_ameba_get_capabilities(const struct device *dev,
					struct display_capabilities *capabilities)
{
	const struct lcdc_ameba_config *config = dev->config;
	struct lcdc_ameba_data *data = dev->data;

	memset(capabilities, 0, sizeof(struct display_capabilities));
	capabilities->x_resolution = config->lcdc_panel.ImgWidth;
	capabilities->y_resolution = config->lcdc_panel.ImgHeight;
	capabilities->supported_pixel_formats =
		PIXEL_FORMAT_ARGB_8888 | PIXEL_FORMAT_RGB_888 | PIXEL_FORMAT_RGB_565;
	capabilities->screen_info = 0;
	capabilities->current_pixel_format = data->current_pixel_format;
	capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

static int lcdc_ameba_read(const struct device *dev, const uint16_t x, const uint16_t y,
			   const struct display_buffer_descriptor *desc, void *buf)
{
	const struct lcdc_ameba_config *config = dev->config;
	struct lcdc_ameba_data *data = dev->data;
	uint8_t *dst = buf;
	const uint8_t *src = data->front_buf;
	uint16_t row;

	/* src = pointer to upper left pixel of the rectangle to be read from frame buffer */
	src += (x * data->current_pixel_size);
	src += (y * config->lcdc_panel.ImgWidth * data->current_pixel_size);

	for (row = 0; row < desc->height; row++) {
		(void)memcpy(dst, src, desc->width * data->current_pixel_size);
		sys_cache_data_flush_range(dst, desc->width * data->current_pixel_size);
		src += (config->lcdc_panel.ImgWidth * data->current_pixel_size);
		dst += (desc->pitch * data->current_pixel_size);
	}

	return 0;
}

static void *lcdc_ameba_get_framebuffer(const struct device *dev)
{
	struct lcdc_ameba_data *data = dev->data;

	return ((void *)data->front_buf);
}

static int lcdc_ameba_init(const struct device *dev)
{
	const struct lcdc_ameba_config *cfg = dev->config;
	struct lcdc_ameba_data *data = dev->data;
	uint32_t line_num = cfg->lcdc_panel.ImgWidth / 2;
	int err;

	/* Configure and set display on/off GPIO */
	if (cfg->disp_on_gpio.port) {
		err = gpio_pin_configure_dt(&cfg->disp_on_gpio, GPIO_OUTPUT_ACTIVE);
		if (err < 0) {
			LOG_ERR("Configuration of display on/off control GPIO failed");
			return err;
		}
	}

	/* Configure and set display backlight control GPIO */
	if (cfg->bl_ctrl_gpio.port) {
		err = gpio_pin_configure_dt(&cfg->bl_ctrl_gpio, GPIO_OUTPUT_INACTIVE);
		if (err < 0) {
			LOG_ERR("Configuration of display backlight control GPIO failed");
			return err;
		}
	}

	err = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		LOG_ERR("Configuration pinctrl failed");
		return err;
	}

	data->current_pixel_format = DISPLAY_INIT_PIXEL_FORMAT;
	data->current_pixel_size = AMEBA_LCDC_INIT_PIXEL_SIZE;

	k_sem_init(&data->sem, 0, 1);

	/* register ISR and enable IRQn */
	cfg->irq_config_func();

	data->pend_buf = data->frame_buffer;
	data->front_buf = data->frame_buffer;

	/* enable function and clock */
	LCDC_RccEnable();

	data->rgb_init.Panel_Init = cfg->lcdc_panel;
	data->rgb_init.Panel_RgbTiming = cfg->lcdc_timing;
	LCDC_RGBInit(LCDC, &data->rgb_init);
	LCDC_DMABurstSizeConfig(LCDC, 2);
	LCDC_DMAImgCfg(LCDC, (uint32_t)data->frame_buffer);

	LCDC_LineINTPosConfig(LCDC, line_num);
	LCDC_INTConfig(LCDC, LCDC_BIT_DMA_UN_INTEN | LCDC_BIT_LCD_LIN_INTEN, ENABLE);

	/* Set default pixel format obtained from device tree */
	/* lcdc_ameba_set_pixel_format(dev, data->pixel_format); */

	LCDC_Cmd(LCDC, ENABLE);

	return 0;
}

static DEVICE_API(display, lcdc_ameba_api) = {
	.blanking_on = lcdc_ameba_display_blanking_on,
	.blanking_off = lcdc_ameba_display_blanking_off,
	.write = lcdc_ameba_write,
	.read = lcdc_ameba_read,
	.get_framebuffer = lcdc_ameba_get_framebuffer,
	.get_capabilities = lcdc_ameba_get_capabilities,
	.set_pixel_format = lcdc_ameba_set_pixel_format,
	.set_orientation = lcdc_ameba_set_orientation,
};

static void lcdc_ameba_irq_config(void)
{
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), lcdc_ameba_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));
}

PINCTRL_DT_INST_DEFINE(0);

#define DISPLAY_GPIO_CTRL                                                                          \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(0, display_gpios), \
	(GPIO_DT_SPEC_INST_GET(0, display_gpios)), ({ 0 }))

#define RGB_MODE_PANEL_PARAMETER                                                                   \
	{                                                                                          \
		.IfWidth = GET_LCDC_RGB_INTERFACE(DT_INST_ENUM_IDX(0, data_bus_width)),            \
		.ImgWidth = DT_INST_PROP(0, width),                                                \
		.ImgHeight = DT_INST_PROP(0, height),                                              \
		.InputFormat = DISPLAY_INIT_PIXEL_FORMAT,                                          \
		.OutputFormat = GET_LCDC_RGB_OUTPUT_FORMAT(DT_INST_PROP(0, pixel_format)),         \
		.RGBRefreshFreq = DT_INST_PROP(0, frame_rate),                                     \
	}

#define RGB_MODE_POLARITY_PARAMETER                                                                \
	{                                                                                          \
		.RgbEnPolar = ((DT_PROP(DT_INST_CHILD(0, display_timings), de_active))             \
				       ? LCDC_RGB_EN_PUL_HIGH_LEV_ACTIVE                           \
				       : LCDC_RGB_EN_PUL_LOW_LEV_ACTIVE),                          \
		.RgbHsPolar = ((DT_PROP(DT_INST_CHILD(0, display_timings), hsync_active))          \
				       ? LCDC_RGB_HS_PUL_HIGH_LEV_SYNC                             \
				       : LCDC_RGB_HS_PUL_LOW_LEV_SYNC),                            \
		.RgbVsPolar = ((DT_PROP(DT_INST_CHILD(0, display_timings), vsync_active))          \
				       ? LCDC_RGB_VS_PUL_HIGH_LEV_SYNC                             \
				       : LCDC_RGB_VS_PUL_LOW_LEV_SYNC),                            \
		.RgbDclkActvEdge = ((DT_PROP(DT_INST_CHILD(0, display_timings), pixelclk_active))  \
					    ? LCDC_RGB_DCLK_FALLING_EDGE_FETCH                     \
					    : LCDC_RGB_DCLK_RISING_EDGE_FETCH),                    \
	}

#define RGB_MODE_TIMING_PARAMETER                                                                  \
	{                                                                                          \
		.RgbVsw = DT_PROP(DT_INST_CHILD(0, display_timings), vsync_len),                   \
		.RgbVbp = DT_PROP(DT_INST_CHILD(0, display_timings), vback_porch),                 \
		.RgbVfp = DT_PROP(DT_INST_CHILD(0, display_timings), vfront_porch),                \
		.RgbHsw = DT_PROP(DT_INST_CHILD(0, display_timings), hsync_len),                   \
		.RgbHbp = DT_PROP(DT_INST_CHILD(0, display_timings), hback_porch),                 \
		.RgbHfp = DT_PROP(DT_INST_CHILD(0, display_timings), hfront_porch),                \
		.Flags = RGB_MODE_POLARITY_PARAMETER,                                              \
	}

static const struct lcdc_ameba_config lcdc_ameba_config = {
	/*.base = (LCDIF_Type *)DT_INST_REG_ADDR(id),*/
	.irq_config_func = lcdc_ameba_irq_config,
	.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
	.lcdc_panel = RGB_MODE_PANEL_PARAMETER,
	.lcdc_timing = RGB_MODE_TIMING_PARAMETER,
	.disp_on_gpio = DISPLAY_GPIO_CTRL,
	.bl_ctrl_gpio = GPIO_DT_SPEC_INST_GET(0, backlight_gpios),
};

static struct lcdc_ameba_data lcdc_ameba_data = {
	.frame_buffer = (u8 *)(LCDC_FRAME_BUFFER_ADDR),
	.frame_buffer_len = AMEBA_LCDC_FRAME_BUFFER_LEN(0),
	.front_buf = (u8 *)(LCDC_FRAME_BUFFER_ADDR),
	.pend_buf = (u8 *)(LCDC_FRAME_BUFFER_ADDR),
};

DEVICE_DT_INST_DEFINE(0, &lcdc_ameba_init, NULL, &lcdc_ameba_data, &lcdc_ameba_config, POST_KERNEL,
		      CONFIG_DISPLAY_INIT_PRIORITY, &lcdc_ameba_api);
