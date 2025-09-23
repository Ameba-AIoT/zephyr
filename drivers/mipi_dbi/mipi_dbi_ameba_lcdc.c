/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_mipi_dbi_ameba_lcdc

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

/* #include <zephyr/drivers/dma.h> */
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

	if (status & LCDC_BIT_LCD_FRD_INTS) {
		LOG_DBG("intr: frame done \r\n");
		k_sem_give(&data->transfer_done);
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

	switch (bus_type) {
	case MIPI_DBI_MODE_8080_BUS_8_BIT:
		lcdc_dbi_config.Panel_Init.IfWidth = LCDC_MCU_IF_8_BIT;
		if (cfg->if_ctrl_gpios.im3.port != NULL && cfg->if_ctrl_gpios.im2.port != NULL &&
		    cfg->if_ctrl_gpios.im1.port != NULL && cfg->if_ctrl_gpios.im0.port != NULL) {
			set_interface_bit(dev, ILI9806_MCU_Parallel_8b_0000);
		}
		break;

	case MIPI_DBI_MODE_8080_BUS_16_BIT:
		lcdc_dbi_config.Panel_Init.IfWidth = LCDC_MCU_IF_16_BIT;
		if (cfg->if_ctrl_gpios.im3.port != NULL && cfg->if_ctrl_gpios.im2.port != NULL &&
		    cfg->if_ctrl_gpios.im1.port != NULL && cfg->if_ctrl_gpios.im0.port != NULL) {
			set_interface_bit(dev, ILI9806_MCU_Parallel_16b_0001);
		}
		break;
#ifdef AMEBA_TODO
	case MIPI_DBI_MODE_8080_BUS_24_BIT:
		lcdc_dbi_config.Panel_Init.IfWidth = LCDC_MCU_IF_24_BIT;
		if (cfg->if_ctrl_gpios.im3.port != NULL && cfg->if_ctrl_gpios.im2.port != NULL &&
		    cfg->if_ctrl_gpios.im1.port != NULL && cfg->if_ctrl_gpios.im0.port != NULL) {
			set_interface_bit(dev, ILI9806_MCU_Parallel_24_0010);
		}
		break;
#endif
	default:
		LOG_ERR("%s unknown bus type %u", __func__, bus_type);
		return -EINVAL;
	}

	/* lcdc_dbi_config.Panel_Init.ImgWidth = WIDTH; */
	/* lcdc_dbi_config.Panel_Init.ImgHeight = HEIGHT; */
	lcdc_dbi_config.Panel_Init.InputFormat = AMEBA_LCDC_INIT_PIXEL_FORMAT;
	lcdc_dbi_config.Panel_Init.OutputFormat = LCDC_INPUT_FORMAT_RGB565;

	/* KD043WVFBA085 */
	lcdc_dbi_config.Panel_McuTiming.McuRdPolar = LCDC_MCU_RD_PUL_RISING_EDGE_FETCH;
	lcdc_dbi_config.Panel_McuTiming.McuWrPolar = LCDC_MCU_WR_PUL_RISING_EDGE_FETCH;
	lcdc_dbi_config.Panel_McuTiming.McuRsPolar = LCDC_MCU_RS_PUL_LOW_LEV_CMD_ADDR;
	lcdc_dbi_config.Panel_McuTiming.McuTePolar = LCDC_MCU_TE_PUL_HIGH_LEV_ACTIVE;
	lcdc_dbi_config.Panel_McuTiming.McuSyncPolar = LCDC_MCU_VSYNC_PUL_LOW_LEV_ACTIVE;
	LCDC_MCUInit(LCDC, &lcdc_dbi_config);

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
	u32 lcdc_out_format = GET_LCDC_DBI_OUTPUT_FORMAT(pixfmt);
	int ret = 0;

	k_sem_take(&data->lock, K_FOREVER);

	/* The DBI bus type and output color format. */
	ret = mipi_dbi_lcdc_dbi_configure(dev, dbi_config);
	if (ret) {
		goto error_exit;
	}

	/* switch to auto/trigger DMA mode */
	LCDC_Cmd(LCDC, DISABLE);

	LOG_INF("pixfmt:%x, lcdc_out_format:%x", pixfmt, lcdc_out_format);
	LCDC_ColorFomatOutputConfig(LCDC, lcdc_out_format);

	/* configure the plane size */
	LOG_INF("width:%u, height:%u, pitch:%u, buflen:%u ", desc->width, desc->height, desc->pitch,
		desc->buf_size);

	if (desc->width != desc->pitch) {
		LOG_WRN("%s desc->width != desc->pitch !!!", __func__);
	}

	reg->LCDC_PLANE_SIZE &= ~(LCDC_MASK_IMAGEHEIGHT | LCDC_MASK_IMAGEWIDTH);
	/* plane size is needed in dma mode */
	reg->LCDC_PLANE_SIZE |= (LCDC_IMAGEWIDTH(desc->width) | LCDC_IMAGEHEIGHT(desc->height));

#ifndef CONFIG_AMEBA_LCDC_TE_ENABLE /* TE mode */
	Lcdc_McuDmaCfgDef LCDC_MCUDmaCfgStruct;

	LCDC_MCUDmaCfgStruct.TeMode = 0; /* DISABLE */
	LCDC_MCUDmaCfgStruct.TriggerDma = LCDC_TRIGGER_DMA_MODE;
	LCDC_MCUDmaMode(LCDC, &LCDC_MCUDmaCfgStruct);

	reg->LCDC_MCU_VSYNC_CFG &= ~LCDC_MASK_MCUVSPD;
	reg->LCDC_MCU_VSYNC_CFG |= LCDC_MCUVSPD(5);
#endif

	LCDC_DMABurstSizeConfig(LCDC, LCDC_DMA_BURSTSIZE_4X64BYTES);
	LCDC_DMAImgCfg(LCDC, (u32)framebuf);
	DCache_Clean((u32)framebuf, desc->buf_size);

	/* LCDC_LineINTPosConfig(LCDC, LCDC_LINE_NUM_INTR_DEF); */
	/*LCDC_BIT_LCD_LIN_INTEN*/
	LCDC_INTConfig(LCDC, LCDC_BIT_LCD_FRD_INTEN | LCDC_BIT_DMA_UN_INTEN, ENABLE);

	LCDC_Cmd(LCDC, ENABLE);
	k_sem_give(&data->lock);

#ifndef CONFIG_AMEBA_LCDC_TE_ENABLE
	LOG_INF("%s trigger one time", __func__);
	LCDC_MCUDMATrigger(LCDC);
#endif

error_exit:
	k_sem_take(&data->transfer_done, K_FOREVER);

	return 0;
}

#if CONFIG_AMEBA_LCDC_TE_ENABLE
static int mipi_dbi_lcdc_ameba_configure_te(const struct device *dev, uint8_t edge,
					    k_timeout_t delay_us)
{
	mipi_dbi_lcdc_ameba_data *data = dev->data;
	mipi_dbi_lcdc_ameba_config *cfg = dev->config;
	LCDC_TypeDef *reg = cfg->base;
	int ret = 0;

	u32 wrdiv = LCDC_GET_WRPULW(reg->LCDC_MCU_TIMING_CFG);
	u32 t_wr_ns = (wrdiv + 1) * 2 * 1000000000 / LCDC_SYS_CLK;
	u32 delay_cycle = 2 * delay_us * 1000 / t_wr_ns;

	LOG_INF("ipclk:%u wrdiv:%u t_wr_ns:%u delay_cycle:%u", LCDC_SYS_CLK, wrdiv, t_wr_ns,
		delay_cycle);

	Lcdc_McuDmaCfgDef LCDC_MCUDmaCfgStruct;

	switch (edge) {
	case MIPI_DBI_TE_RISING_EDGE:
		reg->LCDC_MCU_CFG |= LCDC_BIT_TEPL;
		/* PAD_PullCtrl(pin, GPIO_PULL_DOWN); */
		break;
	case MIPI_DBI_TE_FALLING_EDGE:
		reg->LCDC_MCU_CFG &= ~LCDC_BIT_TEPL;
		/* PAD_PullCtrl(pin, GPIO_PULL_UP); */
		break;

	case MIPI_DBI_TE_NO_EDGE:
	default:
		LOG_INF("TE disabled %u and set vsync trigger mode", edge);
		LCDC_MCUDmaCfgStruct.TeMode = 0; /* DISABLE */
		LCDC_MCUDmaCfgStruct.TriggerDma = LCDC_TRIGGER_DMA_MODE;
		LCDC_MCUDmaMode(LCDC, &LCDC_MCUDmaCfgStruct);
		return -EINVAL;
	}

	LCDC_MCUDmaCfgStruct.TeMode = 1;
	delay_cycle = (delay_cycle < 5) ? 0 : (delay_cycle - 5);
	LCDC_MCUDmaCfgStruct.TeDelay = LCDC_TEDELAY(delay_cycle);
	LCDC_MCUDmaMode(LCDC, &LCDC_MCUDmaCfgStruct);
	return 0;
}
#endif

static int mipi_dbi_lcdc_ameba_write_cmd(const struct device *dev,
					 const struct mipi_dbi_config *dbi_config, uint8_t cmd,
					 const uint8_t *data_buf, size_t data_len)
{
	const struct mipi_dbi_lcdc_ameba_config *cfg = dev->config;
	struct mipi_dbi_lcdc_ameba_data *data = dev->data;
	int ret = 0;

	k_sem_take(&data->lock, K_FOREVER);

	/* The DBI bus type and output color format. */
	ret = mipi_dbi_lcdc_dbi_configure(dev, dbi_config);
	if (ret) {
		goto error_exit;
	}

	/*LCDC_Cmd(LCDC, ENABLE);*/

	LCDC_MCUIOWriteCmd(cfg->base, cmd);

	if (data_len != 0U) {
		if (data_buf != NULL) {
			for (int i = 0; i < data_len; i++) {
				LCDC_MCUIOWriteData(cfg->base, data_buf[i]);
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

	ret = gpio_pin_configure_dt(&cfg->reset_gpios, GPIO_OUTPUT_HIGH);
	if (ret) {
		return ret;
	}

	ret = gpio_pin_set_dt(&cfg->reset_gpios, 0);
	if (ret < 0) {
		return ret;
	}

	k_sleep(delay);

	ret = gpio_pin_set_dt(&cfg->reset_gpios, 1);
	if (ret < 0) {
		return ret;
	}

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
		ret = gpio_pin_configure_dt(&cfg->reset_gpios,
					    GPIO_OUTPUT_INACTIVE); /* GPIO_OUTPUT_ACTIVE */
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

	LOG_DBG("%s device init complete", dev->name);
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

};

DEVICE_DT_INST_DEFINE(0, mipi_dbi_lcdc_ameba_init, NULL, &mipi_dbi_lcdc_data, &mipi_dbi_lcdc_config,
		      POST_KERNEL, CONFIG_MIPI_DBI_INIT_PRIORITY, &mipi_dbi_lcdc_ameba_driver_api);
