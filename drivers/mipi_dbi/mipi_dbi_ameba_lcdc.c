/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_mipi_dbi_ameba_lcdc

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>
#include <zephyr/dt-bindings/pinctrl/amebaG2-pinctrl.h>

#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mipi_dbi_ameba_lcdc, CONFIG_MIPI_DBI_LOG_LEVEL);

/* input format in 8080 protocal */
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
#elif CONFIG_AMEBA_LCDC_BGR565
#define AMEBA_LCDC_INIT_PIXEL_SIZE   2u
#define AMEBA_LCDC_INIT_PIXEL_FORMAT LCDC_INPUT_FORMAT_BGR565
#define DISPLAY_INIT_PIXEL_FORMAT    PIXEL_FORMAT_BGR_565
#else
#error "Invalid LCDC pixel format chosen"
#endif

#define GET_LCDC_DBI_OUTPUT_FORMAT(panel_format)                                                   \
	((panel_format) == PIXEL_FORMAT_RGB_888   ? LCDC_OUTPUT_FORMAT_RGB888                      \
	 : (panel_format) == PIXEL_FORMAT_RGB_565 ? LCDC_OUTPUT_FORMAT_RGB565                      \
	 : (panel_format) == PIXEL_FORMAT_BGR_565 ? LCDC_OUTPUT_FORMAT_BGR565                      \
						  : LCDC_OUTPUT_FORMAT_RGB888)

/* get property mipi-mode from lcd_ic node */
#define PANEL_PARENT_NODE DT_NODELABEL(zephyr_mipi_dbi_parallel)
#define GET_CHILD_PROP_MIPI_MODE(child)                                                            \
	DT_STRING_UPPER_TOKEN_OR(child, mipi_mode, MIPI_DBI_MODE_8080_BUS_16_BIT)
#define CHILD_PROP_MIPI_MODE DT_FOREACH_CHILD(PANEL_PARENT_NODE, GET_CHILD_PROP_MIPI_MODE)

/* get data bus io number from dts */
#define LCDC_PINCTRL_NODE          DT_NODELABEL(lcdc_mcu_default)
#define LCDC_PINCTRL_GROUP1_NODE   DT_CHILD(LCDC_PINCTRL_NODE, group1)
#define LCDC_PINCTRL_GROUP1_PINCNT DT_PROP_LEN(LCDC_PINCTRL_GROUP1_NODE, pinmux)

#ifdef CONFIG_AMEBA_LCDC_TE_ENABLE
#define LCDC_PINCTRL_GROUP3_NODE DT_CHILD(LCDC_PINCTRL_NODE, group3)
#define LCDC_PINCTRL_TE_PU       DT_PROP(LCDC_PINCTRL_GROUP3_NODE, bias_pull_up)
#define LCDC_PINCTRL_TE_PD       DT_PROP(LCDC_PINCTRL_GROUP3_NODE, bias_pull_down)
#endif

/*0000 8bit, 1100, 9bit, 0001 16bit, 1101 18bit, 0010 24bit */
enum ILI9806_MCU_Parallel {
	ILI9806_MCU_Parallel_8b_0000 = 0,
	ILI9806_MCU_Parallel_9b_1100,
	ILI9806_MCU_Parallel_16b_0001,
	ILI9806_MCU_Parallel_18b_1101,
	ILI9806_MCU_Parallel_24b_0010,
};

struct mipi_dbi_lcdc_ameba_data {
	struct k_sem lock;
	struct k_sem transfer_done;

	uint32_t te_delay_cycle;

	uint32_t color_out_format;
	bool clear_screen_finish;
	const struct mipi_dbi_config *active_cfg;
};

struct im_gpios {
	struct gpio_dt_spec im3;
	struct gpio_dt_spec im2;
	struct gpio_dt_spec im1;
	struct gpio_dt_spec im0;
};

struct mipi_dbi_lcdc_ameba_config {
	LCDC_TypeDef *base;

	/* pinctrl info */
	const struct pinctrl_dev_config *pincfg;

	/* register ISR and enable IRQn */
	void (*irq_config_func)(void);

	bool swap_bytes;

	uint32_t if_width;

	/* reset and backlight gpios */
	struct gpio_dt_spec reset_gpios;
	struct gpio_dt_spec bl_ctrl_gpios;

	struct im_gpios if_ctrl_gpios;
};

static void mipi_dbi_lcdc_ameba_isr(const struct device *dev)
{
	struct mipi_dbi_lcdc_ameba_data *data = dev->data;
	uint32_t status;
	status = LCDC_GetINTStatus(LCDC);
	LCDC_ClearINT(LCDC, status);

	LOG_DBG("ints:%x \r\n", status);

	if (status & LCDC_BIT_LCD_FRD_INTS) {
		LOG_DBG("intr: frame done \r\n");
		k_sem_give(&data->transfer_done);
	}

	if (status & LCDC_BIT_PANEL_TE_INTS) {
		LOG_DBG("intr: panel te input \r\n");
	}

	if (status & LCDC_BIT_DMA_UN_INTS) {
		LOG_WRN("intr: dma udf !!! \r\n");
	}
}

static void set_interface_bit(const struct device *dev, enum ILI9806_MCU_Parallel BitMode)
{
	const struct mipi_dbi_lcdc_ameba_config *cfg = dev->config;

	gpio_pin_configure_dt(&cfg->if_ctrl_gpios.im3, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&cfg->if_ctrl_gpios.im2, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&cfg->if_ctrl_gpios.im1, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&cfg->if_ctrl_gpios.im0, GPIO_OUTPUT_INACTIVE);

	switch (BitMode) {
	case ILI9806_MCU_Parallel_8b_0000:
		gpio_pin_set_dt(&cfg->if_ctrl_gpios.im3, 0);
		gpio_pin_set_dt(&cfg->if_ctrl_gpios.im2, 0);
		gpio_pin_set_dt(&cfg->if_ctrl_gpios.im1, 0);
		gpio_pin_set_dt(&cfg->if_ctrl_gpios.im0, 0);

		LOG_INF("ILI9806_MCU_Parallel_8b_0000 \r\n");
		break;

	case ILI9806_MCU_Parallel_16b_0001:
		gpio_pin_set_dt(&cfg->if_ctrl_gpios.im3, 0);
		gpio_pin_set_dt(&cfg->if_ctrl_gpios.im2, 0);
		gpio_pin_set_dt(&cfg->if_ctrl_gpios.im1, 0);
		gpio_pin_set_dt(&cfg->if_ctrl_gpios.im0, 1);

		LOG_INF("ILI9806_MCU_Parallel_16b_0001 \r\n");
		break;

	case ILI9806_MCU_Parallel_24b_0010:
		gpio_pin_set_dt(&cfg->if_ctrl_gpios.im3, 0);
		gpio_pin_set_dt(&cfg->if_ctrl_gpios.im2, 0);
		gpio_pin_set_dt(&cfg->if_ctrl_gpios.im1, 1);
		gpio_pin_set_dt(&cfg->if_ctrl_gpios.im0, 0);

		LOG_INF("ILI9806_MCU_Parallel_24b_0010 \r\n");
		break;

	default:
		LOG_WRN("Unknown ILI9806_MCU_Parallel_xbits !!! \r\n");
		break;
	}

	/* delay is required before send init commands */
	k_sleep(K_MSEC(50));
}

/* config lcdc work in io mode */
static int mipi_dbi_lcdc_dbi_configure(const struct device *dev,
				       const struct mipi_dbi_config *dbi_config)
{
	const struct mipi_dbi_lcdc_ameba_config *cfg = dev->config;
	struct mipi_dbi_lcdc_ameba_data *data = dev->data;
	uint8_t bus_type = dbi_config->mode & 0xFFU;
	LCDC_MCUInitTypeDef lcdc_dbi_config;

	/* for upgrade zephyr version usage */
	/* uint8_t color_coding = dbi_config->mode & 0xF0U; */

	/* No need to update if configuration is the same. */
	if (dbi_config == data->active_cfg) {
		return 0;
	}

	/* SPI mode and 6800 mode are not supported by the LCDC driver */
	if ((bus_type == MIPI_DBI_MODE_SPI_3WIRE) || (bus_type == MIPI_DBI_MODE_SPI_4WIRE) ||
	    (bus_type == MIPI_DBI_MODE_6800_BUS_16_BIT) ||
	    (bus_type == MIPI_DBI_MODE_6800_BUS_9_BIT) ||
	    (bus_type == MIPI_DBI_MODE_6800_BUS_8_BIT)) {
		LOG_ERR("%s SPI mode and 6800 mode bus type not supported.type[0x%x]", __func__,
			bus_type);
		return -EINVAL;
	}

	if (bus_type == MIPI_DBI_MODE_8080_BUS_9_BIT) {
		LOG_ERR("8080 9-bit bus type not supported.");
		return -EINVAL;
	}

	/* initialise the lcdc dbi init structure */
	LCDC_Cmd(LCDC, DISABLE);

	LCDC_MCUStructInit(&lcdc_dbi_config);
	lcdc_dbi_config.Panel_Init.InputFormat = AMEBA_LCDC_INIT_PIXEL_FORMAT;

	switch (bus_type) {
	case MIPI_DBI_MODE_8080_BUS_8_BIT:
		LOG_INF("%s bus_type=8bit color-out=RGB565 color-in%x \r\n(0x0-argb888, "
			"0x1-rgb888, 0x2-rgb565, 0xa-bgr565)",
			__func__, AMEBA_LCDC_INIT_PIXEL_FORMAT);
		lcdc_dbi_config.Panel_Init.IfWidth = LCDC_MCU_IF_8_BIT;
		lcdc_dbi_config.Panel_Init.OutputFormat = LCDC_OUTPUT_FORMAT_RGB565; /* BGR888 */
		data->color_out_format = PIXEL_FORMAT_RGB_565; /* PIXEL_FORMAT_RGB_888 */
		break;

	case MIPI_DBI_MODE_8080_BUS_16_BIT:
		LOG_INF("%s bus_type=16bit color-out=RGB565 color-in%x \r\n(0x0-argb888, "
			"0x1-rgb888, 0x2-rgb565, 0xa-bgr565)",
			__func__, AMEBA_LCDC_INIT_PIXEL_FORMAT);
		lcdc_dbi_config.Panel_Init.IfWidth = LCDC_MCU_IF_16_BIT;
		lcdc_dbi_config.Panel_Init.OutputFormat = LCDC_OUTPUT_FORMAT_RGB565; /* BGR565 */
		data->color_out_format = PIXEL_FORMAT_RGB_565;
		break;
#ifdef AMEBA_TODO
		/* note: it's extension of RealTek for
		 * zephyr/include/zephyr/dt-bindings/mipi_dbi/mipi_dbi.h
		 */

	case MIPI_DBI_MODE_8080_BUS_24_BIT:
		LOG_INF("%s bus_type=24bit color-out=RGB888 color-in%x \r\n(0x0-argb888, "
			"0x1-rgb888, 0x2-rgb565, 0xa-bgr565) \r\n",
			__func__, AMEBA_LCDC_INIT_PIXEL_FORMAT);
		lcdc_dbi_config.Panel_Init.IfWidth = LCDC_MCU_IF_24_BIT;
		lcdc_dbi_config.Panel_Init.OutputFormat = LCDC_OUTPUT_FORMAT_RGB888;
		data->color_out_format = PIXEL_FORMAT_RGB_888;
		break;
#endif
	default:
		LOG_ERR("%s unknown bus type %u", __func__, bus_type);
		return -EINVAL;
	}

	/* lcdc_dbi_config.Panel_Init.ImgWidth = WIDTH; */
	/* lcdc_dbi_config.Panel_Init.ImgHeight = HEIGHT; */

	/* KD043WVFBA085 */
	lcdc_dbi_config.Panel_McuTiming.McuRdPolar = LCDC_MCU_RD_PUL_RISING_EDGE_FETCH;
	lcdc_dbi_config.Panel_McuTiming.McuWrPolar = LCDC_MCU_WR_PUL_RISING_EDGE_FETCH;
	lcdc_dbi_config.Panel_McuTiming.McuRsPolar = LCDC_MCU_RS_PUL_LOW_LEV_CMD_ADDR;
	lcdc_dbi_config.Panel_McuTiming.McuTePolar = LCDC_MCU_TE_PUL_HIGH_LEV_ACTIVE;
	lcdc_dbi_config.Panel_McuTiming.McuSyncPolar = LCDC_MCU_VSYNC_PUL_LOW_LEV_ACTIVE;
	LCDC_MCUInit(LCDC, &lcdc_dbi_config);

	if (cfg->swap_bytes) {
		LCDC_MCUCtrlSwap(cfg->base, ENABLE);
		LOG_INF("enable swap!");
	}

	LCDC_Cmd(LCDC, ENABLE);
	data->active_cfg = dbi_config;
	return 0;
}

static int mipi_dbi_lcdc_ameba_write_display(const struct device *dev,
					     const struct mipi_dbi_config *dbi_config,
					     const uint8_t *framebuf,
					     struct display_buffer_descriptor *desc,
					     enum display_pixel_format pixfmt)
{
	struct mipi_dbi_lcdc_ameba_data *data = dev->data;
	const struct mipi_dbi_lcdc_ameba_config *cfg = dev->config;
	LCDC_TypeDef *reg = cfg->base;
	u32 lcdc_out_format;
	int ret = 0;

	k_sem_take(&data->lock, K_FOREVER);

	/* The DBI bus type and output color format. */
	ret = mipi_dbi_lcdc_dbi_configure(dev, dbi_config);
	if (ret) {
		LOG_ERR("%s %u", __func__, __LINE__);
		k_sem_give(&data->lock);
		return ret;
	}

	/* switch to auto/trigger DMA mode */
	LCDC_Cmd(LCDC, DISABLE);

	if (pixfmt != data->color_out_format) {
		LOG_INF("color-out: 0x%x-> 0x%x (bit0-rgb888, bit4-rgb565, bit5-bgr565) ",
			data->color_out_format, pixfmt);

		/* lcdc_out_format: 0x%x (0x0-rgb888, 0x1-rgb565, 0x8-bgr888,0x9 bgr565)" */
		lcdc_out_format = GET_LCDC_DBI_OUTPUT_FORMAT(pixfmt);
		LCDC_ColorFomatOutputConfig(LCDC, lcdc_out_format);

		data->color_out_format = pixfmt;
	}

	/* configure the plane size */
	LOG_DBG("width:%u, height:%u, pitch:%u, buflen:%u ", desc->width, desc->height, desc->pitch,
		desc->buf_size);

	if (desc->width != desc->pitch) {
		LOG_WRN("%s desc->width != desc->pitch !!!", __func__);
	}

#ifdef CONFIG_AMEBA_LCDC_IO_ENABLE
	uint8_t *pbuf8 = (uint8_t *)framebuf;
	uint16_t *pbuf16 = (uint16_t *)framebuf;
	uint32_t len = desc->buf_size;
	uint32_t if_width = cfg->if_width;

	LCDC_Cmd(LCDC, ENABLE);

	if (if_width == MIPI_DBI_MODE_8080_BUS_16_BIT) {
		if (data->clear_screen_finish == FALSE) {
			for (int32_t i = 0; i < len / 2; i++) {
				LCDC_MCUIOWriteData(cfg->base, (0xffff));
			}
			data->clear_screen_finish = TRUE;
		}
		for (int32_t i = 0; i < len / 2; i++) {
			LCDC_MCUIOWriteData(cfg->base, (pbuf16[i] & 0xFFFF));
		}
		LOG_INF("%s txdone-16bit", __func__);

	} else if (if_width == MIPI_DBI_MODE_8080_BUS_8_BIT) {
		if (data->clear_screen_finish == FALSE) {
			for (int32_t i = 0; i < len; i++) {
				LCDC_MCUIOWriteData(cfg->base, (0xFF));
			}
			data->clear_screen_finish = TRUE;
		}

		for (int32_t i = 0; i < len; i++) {
			LCDC_MCUIOWriteData(cfg->base, (pbuf8[i] & 0xFF));
		}
		LOG_INF("%s txdone-8bit", __func__);
	} else if (if_width == MIPI_DBI_MODE_8080_BUS_24_BIT) {
		if (data->clear_screen_finish == FALSE) {
			for (int32_t i = 0; i < len / 3; i++) {
				LCDC_MCUIOWriteData(cfg->base, 0xFFFFFF);
			}
			data->clear_screen_finish = TRUE;
		}

		for (int32_t i = 0; i < len / 3; i++) {
			uint8_t byte0 = pbuf8[3 * i];
			uint8_t byte1 = pbuf8[3 * i + 1];
			uint8_t byte2 = pbuf8[3 * i + 2];
			uint32_t data = (byte2 << 16) | (byte1 << 8) | byte0;

			LCDC_MCUIOWriteData(cfg->base, data & 0xFFFFFF);
		}
		LOG_INF("%s txdone-24bit", __func__);
	}
	k_sem_give(&data->lock);

#else /* MIPI_DBI_AMEBA_LCDC_IO_MODE */
	LCDC_PanelSizeConfig(reg, desc->width, desc->height);

#ifdef CONFIG_AMEBA_LCDC_VSYNC_ENABLE /* VSYNC mode */
	LOG_INF("%s Trigger mode", __func__);
	/* configure vsync mode */
	Lcdc_McuDmaCfgDef LCDC_MCUDmaCfgStruct;

	LCDC_MCUDmaCfgStruct.TeMode = DISABLE; /* DISABLE */
	LCDC_MCUDmaCfgStruct.TriggerDma = LCDC_TRIGGER_DMA_MODE;
	LCDC_MCUDmaMode(LCDC, &LCDC_MCUDmaCfgStruct);

	reg->LCDC_MCU_VSYNC_CFG &= ~LCDC_MASK_MCUVSPD;
	reg->LCDC_MCU_VSYNC_CFG |= LCDC_MCUVSPD(5);

	/* configure transfer info */
	LCDC_DMABurstSizeConfig(LCDC, LCDC_DMA_BURSTSIZE_4X64BYTES);
	LCDC_DMAImgCfg(LCDC, (u32)framebuf);
	DCache_Clean((u32)framebuf, desc->buf_size);

	LCDC_INTConfig(LCDC, LCDC_BIT_LCD_FRD_INTEN | LCDC_BIT_DMA_UN_INTEN, ENABLE);

	LCDC_Cmd(LCDC, ENABLE);
	if (LCDC_MCUGetRunStatus(LCDC) == LCDC_MCU_RUN_IO_MODE) {
		assert_param(0);
	}
	LOG_DBG("%s trigger one time", __func__);
	LCDC_MCUDMATrigger(LCDC);

	k_sem_take(&data->transfer_done, K_FOREVER);
	LCDC_INTConfig(LCDC, LCDC_BIT_LCD_FRD_INTEN, DISABLE);
	k_sem_give(&data->lock);

#elif defined(CONFIG_AMEBA_LCDC_TE_ENABLE)
	LOG_INF("%s TE cycle:%x", __func__, data->te_delay_cycle);
	/* configure te mode */
	Lcdc_McuDmaCfgDef LCDC_MCUDmaCfgStruct;

	memset(&LCDC_MCUDmaCfgStruct, 0, sizeof(Lcdc_McuDmaCfgDef));
	LCDC_MCUDmaCfgStruct.TeMode = 1;
	LCDC_MCUDmaCfgStruct.TeDelay = data->te_delay_cycle;

	LCDC_MCUDmaMode(LCDC, &LCDC_MCUDmaCfgStruct);

	/* configure transfer info */
	LCDC_DMABurstSizeConfig(LCDC, LCDC_DMA_BURSTSIZE_4X64BYTES);
	LCDC_DMAImgCfg(LCDC, (u32)framebuf);
	DCache_Clean((u32)framebuf, desc->buf_size);

	LCDC_INTConfig(LCDC,
		       LCDC_BIT_LCD_FRD_INTEN | LCDC_BIT_DMA_UN_INTEN | LCDC_BIT_PANEL_TE_INTEN,
		       ENABLE);

	LCDC_Cmd(LCDC, ENABLE);
	if (LCDC_MCUGetRunStatus(LCDC) == LCDC_MCU_RUN_IO_MODE) {
		assert_param(0);
	}
	k_sem_take(&data->transfer_done, K_FOREVER);
	LCDC_Cmd(LCDC, DISABLE);
	k_sem_give(&data->lock);
#endif
#endif
	return ret;
}

#if CONFIG_AMEBA_LCDC_TE_ENABLE
static int pinmux_find_pinname(uint32_t func_id)
{
	int i;

	for (i = 0; i < PIN_TOTAL_NUM; i++) {
		if (Pinmux_ConfigGet(i) == func_id) {
			LOG_INF("port%u_pin%u is configure as TE(%u) \r\n", PORT_NUM(i), PIN_NUM(i),
				func_id);
			break;
		}
	}

	if (i == PIN_TOTAL_NUM) {
		LOG_ERR("There is no assigned pin to TE signal !");
		return -ENOSYS;
	} else {
		return i;
	}
}

static int mipi_dbi_lcdc_ameba_configure_te(const struct device *dev, uint8_t edge,
					    k_timeout_t delay)
{
	struct mipi_dbi_lcdc_ameba_data *data = dev->data;
	const struct mipi_dbi_lcdc_ameba_config *cfg = dev->config;
	LCDC_TypeDef *reg = cfg->base;
	int ret = 0;
	int te_pin;
	uint32_t delay_us;

	delay_us = k_ticks_to_us_ceil32(delay.ticks);

	u32 wrdiv = LCDC_GET_WRPULW(reg->LCDC_MCU_TIMING_CFG);
	u32 t_wr_ns = 2 * 1000000000 / LCDC_SYS_CLK * (wrdiv + 1);
	/* calculate based on write period */
	u32 delay_cycle = delay_us * 1000 / t_wr_ns;
	/* u32 delay_cycle = 2 * delay_us * 1000 / t_wr_ns; */

	LOG_INF("%s ipclk:%u wrdiv:%u t_wr_ns:%u delay_us:%u delay_cycle:%u", __func__,
		LCDC_SYS_CLK, wrdiv, t_wr_ns, delay_us, delay_cycle);

	te_pin = pinmux_find_pinname(AMEBA_LCD_MCU_TE); /* 42 */
	if (te_pin < 0) {
		return ret;
	}

	LCDC_Cmd(reg, DISABLE);

	Lcdc_McuDmaCfgDef LCDC_MCUDmaCfgStruct;

	switch (edge) {
	case MIPI_DBI_TE_RISING_EDGE:
		reg->LCDC_MCU_CFG |= LCDC_BIT_TEPL;
		PAD_PullCtrl(te_pin, GPIO_PULL_DOWN);
		assert_param(LCDC_PINCTRL_TE_PD);
		break;
	case MIPI_DBI_TE_FALLING_EDGE:
		reg->LCDC_MCU_CFG &= ~LCDC_BIT_TEPL;
		PAD_PullCtrl(te_pin, GPIO_PULL_UP);
		assert_param(LCDC_PINCTRL_TE_PU);
		break;

	case MIPI_DBI_TE_NO_EDGE:
	default:
		LOG_WRN("%s TE disabled and set vsync trigger mode", __func__);
		LCDC_MCUDmaCfgStruct.TeMode = 0; /* DISABLE */
		LCDC_MCUDmaCfgStruct.TriggerDma = LCDC_TRIGGER_DMA_MODE;
		LCDC_MCUDmaMode(LCDC, &LCDC_MCUDmaCfgStruct);
		return -EINVAL;
	}

	LCDC_MCUDmaCfgStruct.TeMode = 1;
	if (delay_cycle < 5) {
		delay_cycle = 5;
	} else if (delay_cycle > 65535) {
		delay_cycle = 65535;
	}
	LCDC_MCUDmaCfgStruct.TeDelay = delay_cycle;
	LCDC_MCUDmaMode(LCDC, &LCDC_MCUDmaCfgStruct);
	data->te_delay_cycle = delay_cycle;
	LOG_INF("%s edge%x(1-rising, 2-falling) delay%uus(cycle:%x)", __func__, edge, delay_us,
		delay_cycle);

	return 0;
}
#endif

static int mipi_dbi_lcdc_ameba_write_cmd(const struct device *dev,
					 const struct mipi_dbi_config *dbi_config, uint8_t cmd,
					 const uint8_t *data_buf, size_t data_len)
{
	const struct mipi_dbi_lcdc_ameba_config *cfg = dev->config;
	LCDC_TypeDef *reg = cfg->base;
	struct mipi_dbi_lcdc_ameba_data *data = dev->data;
	int ret = 0;

	k_sem_take(&data->lock, K_FOREVER);

	/* The DBI bus type and output color format. */
	ret = mipi_dbi_lcdc_dbi_configure(dev, dbi_config);
	if (ret) {
		goto error_exit;
	}

	LOG_DBG("%s Enter IO mode", __func__);

	/* configure mcu io/dma mode to io mode */
	LCDC_MCUIOMode(reg);

	LCDC_MCUIOWriteCmd(cfg->base, (cmd & 0xFF));

	if (data_len != 0U) {
		if (data_buf != NULL) {
			for (int i = 0; i < data_len; i++) {
				LCDC_MCUIOWriteData(cfg->base, (data_buf[i] & 0xFF));
			}
		} else {
			LOG_ERR("%s data_len=%u data_buf is NULL", __func__, data_len);

			ret = -EINVAL;
			goto error_exit;
		}
	}

error_exit:
	k_sem_give(&data->lock);

	return ret;
}

static int mipi_dbi_lcdc_ameba_reset(const struct device *dev, k_timeout_t delay)
{
	int ret;
	const struct mipi_dbi_lcdc_ameba_config *cfg = dev->config;

	if (cfg->reset_gpios.port == NULL) {
		LOG_ERR("No reset pin is provided !");
		return -ENOSYS;
	}

	/* ACTIVE LOW */
	ret = gpio_pin_set_dt(&cfg->reset_gpios, 1);
	if (ret < 0) {
		return ret;
	}

	/* ili9xxx delay 1ms not enough for ili9806 */
	k_sleep(K_MSEC(10));

	ret = gpio_pin_set_dt(&cfg->reset_gpios, 0);
	if (ret < 0) {
		return ret;
	}

	/* ili9xxx delay 5ms not enough for ili9806 */
	k_sleep(K_MSEC(120));

	LOG_DBG("%s device reset complete", dev->name);

	return 0;
}

static int mipi_dbi_lcdc_ameba_init(const struct device *dev)
{
	const struct mipi_dbi_lcdc_ameba_config *cfg = dev->config;
	struct mipi_dbi_lcdc_ameba_data *data = dev->data;

	int ret = 0;

	/* DCache_Disable(); */

	/* Configure and set display on/off GPIO */
	if (cfg->bl_ctrl_gpios.port) {
		ret = gpio_pin_configure_dt(&cfg->bl_ctrl_gpios, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("configuration of display on/off control GPIO failed");
			return ret;
		}
	}

	if (cfg->reset_gpios.port) {
		ret = gpio_pin_configure_dt(&cfg->reset_gpios, GPIO_OUTPUT_HIGH);

		if (ret < 0) {
			LOG_ERR("configuration of reset control GPIO failed");
			return ret;
		}
	}

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("configuration pinctrl failed");
		return ret;
	}

	LOG_INF("%s %d cfg->if_width%x pinctrl_cnt:0x%x\r\n", __func__, __LINE__, cfg->if_width,
		LCDC_PINCTRL_GROUP1_PINCNT);

	if (cfg->if_width != 0) {
		if (cfg->if_ctrl_gpios.im3.port != NULL && cfg->if_ctrl_gpios.im2.port &&
		    cfg->if_ctrl_gpios.im1.port != NULL && cfg->if_ctrl_gpios.im0.port) {

			switch (cfg->if_width) {
			case MIPI_DBI_MODE_8080_BUS_8_BIT:
				assert_param(LCDC_PINCTRL_GROUP1_PINCNT == 8);
				set_interface_bit(dev, ILI9806_MCU_Parallel_8b_0000);
				break;
			case MIPI_DBI_MODE_8080_BUS_16_BIT:
				assert_param(LCDC_PINCTRL_GROUP1_PINCNT == 16);
				set_interface_bit(dev, ILI9806_MCU_Parallel_16b_0001);
				break;
#ifdef AMEBA_TODO
				/* note: it's extension of RealTek for
				 * zephyr/include/zephyr/dt-bindings/mipi_dbi/mipi_dbi.h
				 */

			case MIPI_DBI_MODE_8080_BUS_24_BIT:
				assert_param(LCDC_PINCTRL_GROUP1_PINCNT == 24);
				set_interface_bit(dev, ILI9806_MCU_Parallel_24b_0010);
				break;
#endif
			default:
				LOG_ERR("%s unknown if_width(0x%x)", __func__, cfg->if_width);
				return -EINVAL;
			}
		} else {
			LOG_WRN("im[3:0] is not assigned, make sure if_width is fixed");
		}
	} else {
		LOG_WRN("if_width is not configured");
		return -EINVAL;
	}

	/* enable function and clock */
	LCDC_RccEnable();

	/* register ISR and enable IRQn */
	cfg->irq_config_func();

	ret = k_sem_init(&data->lock, 1, 1);
	if (ret) {
		return ret;
	}

	ret = k_sem_init(&data->transfer_done, 0, 1);
	if (ret) {
		return ret;
	}

	LOG_INF("%s device init complete", dev->name);

	return 0;
}

static DEVICE_API(mipi_dbi, mipi_dbi_lcdc_ameba_driver_api) = {
	.command_write = mipi_dbi_lcdc_ameba_write_cmd,
	.write_display = mipi_dbi_lcdc_ameba_write_display,
#if CONFIG_AMEBA_LCDC_TE_ENABLE
	.configure_te = mipi_dbi_lcdc_ameba_configure_te,
#endif
	.reset = mipi_dbi_lcdc_ameba_reset,
};

static void mipi_dbi_lcdc_ameba_irq_config(void)
{
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), mipi_dbi_lcdc_ameba_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));
}

PINCTRL_DT_INST_DEFINE(0);

static const struct mipi_dbi_lcdc_ameba_config mipi_dbi_lcdc_config = {
	.base = (LCDC_TypeDef *)DT_REG_ADDR(DT_PARENT(DT_DRV_INST(0))),
	.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
	.irq_config_func = mipi_dbi_lcdc_ameba_irq_config,
	.if_width = (CHILD_PROP_MIPI_MODE), /*MIPI_DBI_MODE_8080_BUS_16_BIT*/
	.swap_bytes = DT_INST_PROP(0, swap_bytes),
	.reset_gpios = GPIO_DT_SPEC_INST_GET(0, reset_gpios),
	.bl_ctrl_gpios = GPIO_DT_SPEC_INST_GET_OR(0, backlight_gpios, {0}),
	.if_ctrl_gpios = COND_CODE_1(DT_INST_NODE_HAS_PROP(0, im_gpios),
		(
			{
				.im3 = GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(0), im_gpios, 0),
				.im2 = GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(0), im_gpios, 1),
				.im1 = GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(0), im_gpios, 2),
				.im0 = GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(0), im_gpios, 3),
			}
		),
		({0})),
};

static struct mipi_dbi_lcdc_ameba_data mipi_dbi_lcdc_data = {
	.clear_screen_finish = FALSE,
};

DEVICE_DT_INST_DEFINE(0, mipi_dbi_lcdc_ameba_init, NULL, &mipi_dbi_lcdc_data, &mipi_dbi_lcdc_config,
		      POST_KERNEL, CONFIG_MIPI_DBI_INIT_PRIORITY, &mipi_dbi_lcdc_ameba_driver_api);
