/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * GH7002 1024x600 MIPI-DSI panel (Driver IC: gh7002-01, Screen: hj7001-02).
 *
 * Passive panel driver: issues the vendor power-on init sequence and attaches
 * to the MIPI-DSI host. Framebuffer scan-out is handled by the LCDC host.
 */

#define DT_DRV_COMPAT realtek_gh7002

#include <zephyr/kernel.h>
#include <zephyr/drivers/mipi_dsi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gh7002, CONFIG_DISPLAY_LOG_LEVEL);

struct gh7002_config {
	const struct device        *mipi_dsi;
	const struct gpio_dt_spec   reset;
	uint8_t                     channel;
	uint8_t                     data_lanes;
	uint32_t                    pixfmt;
	uint16_t                    width;
	uint16_t                    height;
	uint16_t                    hsync;
	uint16_t                    hbp;
	uint16_t                    hfp;
	uint16_t                    vsync;
	uint16_t                    vbp;
	uint16_t                    vfp;
};

/* Send DCS Short Write with 1-byte parameter (reg ← val). */
static int gh7002_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct gh7002_config *cfg = dev->config;
	int ret;

	ret = mipi_dsi_dcs_write(cfg->mipi_dsi, cfg->channel, reg, &val, 1);
	if (ret < 0) {
		LOG_ERR("DCS 0x%02x <- 0x%02x failed (%d)", reg, val, ret);
		return ret;
	}
	return 0;
}

/* Send DCS Short Write with no parameter (standalone command byte). */
static int gh7002_cmd(const struct device *dev, uint8_t cmd)
{
	const struct gh7002_config *cfg = dev->config;
	int ret;

	ret = mipi_dsi_dcs_write(cfg->mipi_dsi, cfg->channel, cmd, NULL, 0);
	if (ret < 0) {
		LOG_ERR("DCS cmd 0x%02x failed (%d)", cmd, ret);
	}
	return ret;
}

/* Power-on init table: all entries are DCS Short Write with 1-byte param. */
static const uint8_t gh7002_init_seq[][2] = {
	/* PAGE 1 */
	{0xee, 0x01}, {0xea, 0x07}, {0xeb, 0x12}, {0x0a, 0x55}, {0x0c, 0x70},
	{0x13, 0x14}, {0x15, 0x58}, {0x17, 0x32}, {0x1d, 0x33}, {0x21, 0x01},
	{0x28, 0x23}, {0x29, 0x23}, {0x2a, 0x03}, {0x2f, 0xf3},
	/* PAGE 2: positive gamma */
	{0xee, 0x02}, {0x39, 0xb0},
	{0x00, 0x00}, {0x01, 0x11}, {0x02, 0x18}, {0x03, 0x0d}, {0x04, 0x15},
	{0x05, 0x35}, {0x06, 0x0e}, {0x07, 0x10}, {0x08, 0x11}, {0x09, 0x0e},
	{0x0a, 0x12}, {0x0b, 0x55}, {0x0c, 0x12}, {0x0d, 0x15}, {0x0e, 0x3a},
	{0x0f, 0x3d}, {0x10, 0x3f},
	/* PAGE 2: negative gamma */
	{0x20, 0x00}, {0x21, 0x11}, {0x22, 0x18}, {0x23, 0x0d}, {0x24, 0x15},
	{0x25, 0x35}, {0x26, 0x0e}, {0x27, 0x10}, {0x28, 0x11}, {0x29, 0x0e},
	{0x2a, 0x12}, {0x2b, 0x55}, {0x2c, 0x12}, {0x2d, 0x15}, {0x2e, 0x3a},
	{0x2f, 0x3d}, {0x30, 0x3f},
	/* PAGE 3 */
	{0xee, 0x03}, {0x0f, 0xb9},
	/* PAGE 4: source/gate driver control */
	{0xee, 0x04},
	{0x00, 0x05}, {0x01, 0x01}, {0x02, 0x2c}, {0x03, 0x04}, {0x04, 0x00},
	{0x06, 0x06}, {0x07, 0x05}, {0x08, 0x15}, {0x09, 0x20}, {0x0a, 0x0a},
	{0x0b, 0x07}, {0x0f, 0x0a}, {0x19, 0xcc}, {0x1a, 0xcc}, {0x20, 0x40},
	{0x24, 0x08}, {0x25, 0x02}, {0x29, 0x00}, {0x30, 0x1d}, {0x31, 0x1d},
	{0x37, 0x22}, {0x40, 0x80}, {0x41, 0x55},
	/* PAGE 5: gate timing */
	{0xee, 0x05},
	{0x00, 0x01}, {0x01, 0x05}, {0x02, 0x45}, {0x03, 0x05}, {0x07, 0xbd},
	{0x08, 0xc1}, {0x09, 0x44}, {0x10, 0x03}, {0x11, 0x07}, {0x12, 0x45},
	{0x13, 0x05}, {0x19, 0xbb}, {0x1a, 0x74}, {0x30, 0x01}, {0x31, 0x01},
	{0x32, 0x00}, {0x33, 0x14}, {0x34, 0x14}, {0x35, 0x78}, {0x36, 0x01},
	{0x37, 0x01}, {0x38, 0x00}, {0x39, 0x14}, {0x3a, 0x14}, {0x40, 0xee},
	{0x41, 0x44}, {0x43, 0x13}, {0x44, 0x01}, {0x45, 0x81}, {0x46, 0x06},
	{0x47, 0x00},
	/* PAGE 6: GIP back */
	{0xee, 0x06},
	{0x00, 0x01}, {0x02, 0x45}, {0x06, 0xcd}, {0x08, 0x67}, {0x09, 0x45},
	{0x0a, 0x23}, {0x0b, 0x01},
	/* PAGE 7: GIP left pins 1-21 */
	{0xee, 0x07},
	{0x00, 0x01}, {0x01, 0x05}, {0x02, 0x0c}, {0x03, 0x0d}, {0x04, 0x3c},
	{0x05, 0x21}, {0x06, 0x20}, {0x07, 0x12}, {0x08, 0x10}, {0x09, 0x16},
	{0x0a, 0x14}, {0x0b, 0x3c}, {0x0c, 0x3c}, {0x0d, 0x3c}, {0x0e, 0x3c},
	{0x0f, 0x3c}, {0x10, 0x3c}, {0x11, 0x3c}, {0x12, 0x3c}, {0x13, 0x3c},
	{0x14, 0x3c}, {0x15, 0x3c},
	/* PAGE 7: GIP right pins 1-21 */
	{0x20, 0x00}, {0x21, 0x04}, {0x22, 0x0c}, {0x23, 0x0d}, {0x24, 0x3c},
	{0x25, 0x21}, {0x26, 0x20}, {0x27, 0x13}, {0x28, 0x11}, {0x29, 0x17},
	{0x2a, 0x15}, {0x2b, 0x3c}, {0x2c, 0x3c}, {0x2d, 0x3c}, {0x2e, 0x3c},
	{0x2f, 0x3c}, {0x30, 0x3c}, {0x31, 0x3c}, {0x32, 0x3c}, {0x33, 0x3c},
	{0x34, 0x3c}, {0x35, 0x3c},
	/* PAGE 8: power */
	{0xee, 0x08},
	{0x10, 0x00}, {0x12, 0xda}, {0x13, 0x1c}, {0x18, 0x10}, {0x20, 0x80},
	/* PAGE f: dual-gate enable */
	{0xee, 0x0f}, {0x00, 0x01}, {0x03, 0x95},
	/* PAGE 0: finalise */
	{0xee, 0x00}, {0xea, 0x00}, {0xeb, 0x00}, {0x36, 0x00},
};

static int gh7002_configure(const struct device *dev)
{
	int ret;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(gh7002_init_seq); i++) {
		ret = gh7002_write(dev, gh7002_init_seq[i][0],
				   gh7002_init_seq[i][1]);
		if (ret) {
			return ret;
		}
	}

	ret = gh7002_cmd(dev, MIPI_DCS_EXIT_SLEEP_MODE);
	if (ret) {
		return ret;
	}
	k_msleep(120);

	ret = gh7002_cmd(dev, MIPI_DCS_SET_DISPLAY_ON);
	if (ret) {
		return ret;
	}
	k_msleep(20);

	return 0;
}

static int gh7002_init(const struct device *dev)
{
	const struct gh7002_config *cfg = dev->config;
	struct mipi_dsi_device mdev = {0};
	int ret;

	/* Reset: hold HIGH 10ms → assert LOW 15ms → release HIGH, wait 120ms. */
	if (cfg->reset.port) {
		if (!gpio_is_ready_dt(&cfg->reset)) {
			LOG_ERR("reset GPIO not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			return ret;
		}
		k_msleep(10);
		gpio_pin_set_dt(&cfg->reset, 1);  /* assert (physical LOW) */
		k_msleep(15);
		gpio_pin_set_dt(&cfg->reset, 0);  /* release (physical HIGH) */
		k_msleep(120);
	}

	mdev.data_lanes = cfg->data_lanes;
	mdev.pixfmt     = cfg->pixfmt;
	mdev.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM;

	mdev.timings.hactive = cfg->width;
	mdev.timings.hsync   = cfg->hsync;
	mdev.timings.hbp     = cfg->hbp;
	mdev.timings.hfp     = cfg->hfp;
	mdev.timings.vactive = cfg->height;
	mdev.timings.vsync   = cfg->vsync;
	mdev.timings.vbp     = cfg->vbp;
	mdev.timings.vfp     = cfg->vfp;

	ret = mipi_dsi_attach(cfg->mipi_dsi, cfg->channel, &mdev);
	if (ret < 0) {
		LOG_ERR("MIPI-DSI attach failed (%d)", ret);
		return ret;
	}

	k_msleep(2);

	ret = gh7002_configure(dev);
	if (ret < 0) {
		LOG_ERR("GH7002 init sequence failed (%d)", ret);
		return ret;
	}

	LOG_INF("GH7002 %ux%u panel ready", cfg->width, cfg->height);
	return 0;
}

#define GH7002_TIMING(inst) DT_CHILD(DT_DRV_INST(inst), display_timings)

#define GH7002_DEFINE(inst)                                                          \
	static const struct gh7002_config gh7002_config_##inst = {                  \
		.mipi_dsi   = DEVICE_DT_GET(DT_INST_BUS(inst)),                     \
		.reset      = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),     \
		.channel    = DT_INST_REG_ADDR(inst),                                \
		.data_lanes = DT_INST_PROP_BY_IDX(inst, data_lanes, 0),             \
		.pixfmt     = DT_INST_PROP(inst, pixel_format),                      \
		.width      = DT_INST_PROP(inst, width),                             \
		.height     = DT_INST_PROP(inst, height),                            \
		.hsync      = DT_PROP(GH7002_TIMING(inst), hsync_len),              \
		.hbp        = DT_PROP(GH7002_TIMING(inst), hback_porch),            \
		.hfp        = DT_PROP(GH7002_TIMING(inst), hfront_porch),           \
		.vsync      = DT_PROP(GH7002_TIMING(inst), vsync_len),              \
		.vbp        = DT_PROP(GH7002_TIMING(inst), vback_porch),            \
		.vfp        = DT_PROP(GH7002_TIMING(inst), vfront_porch),           \
	};                                                                               \
	DEVICE_DT_INST_DEFINE(inst, &gh7002_init, NULL,                              \
			      NULL, &gh7002_config_##inst,                           \
			      POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY,         \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(GH7002_DEFINE)
