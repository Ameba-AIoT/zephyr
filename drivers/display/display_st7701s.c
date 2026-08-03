/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Sitronix ST7701S 480x800 MIPI-DSI panel driver.
 *
 * A passive DSI command target: reset, mipi_dsi_attach(), then stream the
 * power-on init table, leaving framebuffer/scan-out to the DSI host. Uses only
 * the generic mipi_dsi_* API. Init table is tuned for the AmebaSmart EVB glass.
 */

#define DT_DRV_COMPAT sitronix_st7701s

#include <zephyr/kernel.h>
#include <zephyr/drivers/mipi_dsi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(st7701s, CONFIG_DISPLAY_LOG_LEVEL);

/* Bank-select command byte */
#define ST7701S_CMD2BKX_SEL 0xFF

/* Bank selection values */
#define ST7701S_BK0_SEL  0x10
#define ST7701S_BK1_SEL  0x11
#define ST7701S_BKX_NONE 0x00

struct st7701s_config {
	const struct device *mipi_dsi;
	const struct gpio_dt_spec reset;
	uint8_t channel;
	uint8_t data_lanes;
	uint32_t pixfmt;
	uint16_t width;
	uint16_t height;
	/* Porch/sync timings from the zephyr,panel-timing child node. */
	uint16_t hsync;
	uint16_t hbp;
	uint16_t hfp;
	uint16_t vsync;
	uint16_t vbp;
	uint16_t vfp;
};

/* ------------------------------------------------------------------ helpers */

static int st7701s_dcs_write(const struct device *dev, uint8_t cmd,
			     const uint8_t *buf, size_t len)
{
	const struct st7701s_config *cfg = dev->config;
	int ret;

	ret = mipi_dsi_dcs_write(cfg->mipi_dsi, cfg->channel, cmd, buf, len);
	if (ret < 0) {
		LOG_ERR("DCS cmd 0x%02x write failed (%d)", cmd, ret);
	}
	return ret;
}

/* Send a byte buffer as a MIPI generic write (handles 0xFF bank-select). */
static int st7701s_gen_write(const struct device *dev, const uint8_t *buf, size_t len)
{
	const struct st7701s_config *cfg = dev->config;
	int ret;

	ret = mipi_dsi_generic_write(cfg->mipi_dsi, cfg->channel, buf, len);
	if (ret < 0) {
		LOG_ERR("generic write failed (%d)", ret);
	}
	return ret;
}

/* ---------------------------------------------------------------- bank helpers */

static int st7701s_bank_select(const struct device *dev, uint8_t bank)
{
	const uint8_t cmd[] = {ST7701S_CMD2BKX_SEL, 0x77, 0x01, 0x00, 0x00, bank};

	return st7701s_gen_write(dev, cmd, sizeof(cmd));
}

/* ---------------------------------------------------------------- init sequence */

static int st7701s_configure(const struct device *dev)
{
	int ret;

	/* ---- Bank 0: display control + gamma ---- */
	ret = st7701s_bank_select(dev, ST7701S_BK0_SEL);
	if (ret < 0) {
		return ret;
	}

	/* LNESET: display line setting */
	static const uint8_t c0[] = {0x63, 0x00};

	ret = st7701s_dcs_write(dev, 0xC0, c0, sizeof(c0));
	if (ret < 0) {
		return ret;
	}

	/* PORCTRL: porch control */
	static const uint8_t c1[] = {0x0C, 0x02};

	ret = st7701s_dcs_write(dev, 0xC1, c1, sizeof(c1));
	if (ret < 0) {
		return ret;
	}

	/* INVSEL: 2-dot inversion, frame rate */
	static const uint8_t c2[] = {0x31, 0x08};

	ret = st7701s_dcs_write(dev, 0xC2, c2, sizeof(c2));
	if (ret < 0) {
		return ret;
	}

	/* RGBCTRL */
	ret = st7701s_dcs_write(dev, 0xCC, (const uint8_t []){0x10}, 1);
	if (ret < 0) {
		return ret;
	}

	/* Positive voltage gamma (PVGAMCTRL) */
	static const uint8_t pvgam[] = {
		0x40, 0x02, 0x87, 0x0E, 0x15, 0x0A, 0x03, 0x0A,
		0x0A, 0x18, 0x08, 0x16, 0x13, 0x07, 0x09, 0x19,
	};

	ret = st7701s_dcs_write(dev, 0xB0, pvgam, sizeof(pvgam));
	if (ret < 0) {
		return ret;
	}

	/* Negative voltage gamma (NVGAMCTRL) */
	static const uint8_t nvgam[] = {
		0x40, 0x01, 0x86, 0x0D, 0x13, 0x09, 0x03, 0x0A,
		0x09, 0x1C, 0x09, 0x15, 0x13, 0x91, 0x16, 0x19,
	};

	ret = st7701s_dcs_write(dev, 0xB1, nvgam, sizeof(nvgam));
	if (ret < 0) {
		return ret;
	}

	/* ---- Bank 1: power control ---- */
	ret = st7701s_bank_select(dev, ST7701S_BK1_SEL);
	if (ret < 0) {
		return ret;
	}

	/* VRHS: Vop amplitude */
	ret = st7701s_dcs_write(dev, 0xB0, (const uint8_t []){0x4D}, 1);
	if (ret < 0) {
		return ret;
	}
	/* VCOM */
	ret = st7701s_dcs_write(dev, 0xB1, (const uint8_t []){0x64}, 1);
	if (ret < 0) {
		return ret;
	}
	/* VGHSS */
	ret = st7701s_dcs_write(dev, 0xB2, (const uint8_t []){0x07}, 1);
	if (ret < 0) {
		return ret;
	}
	/* TESTCMD */
	ret = st7701s_dcs_write(dev, 0xB3, (const uint8_t []){0x80}, 1);
	if (ret < 0) {
		return ret;
	}
	/* VGLS */
	ret = st7701s_dcs_write(dev, 0xB5, (const uint8_t []){0x47}, 1);
	if (ret < 0) {
		return ret;
	}
	/* PWCTLR1 */
	ret = st7701s_dcs_write(dev, 0xB7, (const uint8_t []){0x85}, 1);
	if (ret < 0) {
		return ret;
	}
	/* PWCTLR2 */
	ret = st7701s_dcs_write(dev, 0xB8, (const uint8_t []){0x21}, 1);
	if (ret < 0) {
		return ret;
	}
	ret = st7701s_dcs_write(dev, 0xB9, (const uint8_t []){0x10}, 1);
	if (ret < 0) {
		return ret;
	}
	/* SPD1, SPD2 */
	ret = st7701s_dcs_write(dev, 0xC1, (const uint8_t []){0x78}, 1);
	if (ret < 0) {
		return ret;
	}
	ret = st7701s_dcs_write(dev, 0xC2, (const uint8_t []){0x78}, 1);
	if (ret < 0) {
		return ret;
	}
	/* MIPISET1 */
	ret = st7701s_dcs_write(dev, 0xD0, (const uint8_t []){0x88}, 1);
	if (ret < 0) {
		return ret;
	}

	/* Datasheet: allow the Bank1 power-control rails (VRH/VCOM/VGH/VGL) to
	 * settle before programming the GIP timing registers.
	 */
	k_msleep(100);

	/* ---- GIP setting (Bank1 continuation) ---- */
	static const uint8_t e0[] = {0x00, 0x84, 0x02};

	ret = st7701s_dcs_write(dev, 0xE0, e0, sizeof(e0));
	if (ret < 0) {
		return ret;
	}

	static const uint8_t e1[] = {
		0x06, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
		0x00, 0x20, 0x20,
	};

	ret = st7701s_dcs_write(dev, 0xE1, e1, sizeof(e1));
	if (ret < 0) {
		return ret;
	}

	static const uint8_t e2[] = {
		0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00,
	};

	ret = st7701s_dcs_write(dev, 0xE2, e2, sizeof(e2));
	if (ret < 0) {
		return ret;
	}

	static const uint8_t e3[] = {0x00, 0x00, 0x33, 0x33};

	ret = st7701s_dcs_write(dev, 0xE3, e3, sizeof(e3));
	if (ret < 0) {
		return ret;
	}

	static const uint8_t e4[] = {0x44, 0x44};

	ret = st7701s_dcs_write(dev, 0xE4, e4, sizeof(e4));
	if (ret < 0) {
		return ret;
	}

	static const uint8_t e5[] = {
		0x09, 0x31, 0xBE, 0xA0, 0x0B, 0x31, 0xBE, 0xA0,
		0x05, 0x31, 0xBE, 0xA0, 0x07, 0x31, 0xBE, 0xA0,
	};

	ret = st7701s_dcs_write(dev, 0xE5, e5, sizeof(e5));
	if (ret < 0) {
		return ret;
	}

	static const uint8_t e6[] = {0x00, 0x00, 0x33, 0x33};

	ret = st7701s_dcs_write(dev, 0xE6, e6, sizeof(e6));
	if (ret < 0) {
		return ret;
	}

	static const uint8_t e7[] = {0x44, 0x44};

	ret = st7701s_dcs_write(dev, 0xE7, e7, sizeof(e7));
	if (ret < 0) {
		return ret;
	}

	static const uint8_t e8[] = {
		0x08, 0x31, 0xBE, 0xA0, 0x0A, 0x31, 0xBE, 0xA0,
		0x04, 0x31, 0xBE, 0xA0, 0x06, 0x31, 0xBE, 0xA0,
	};

	ret = st7701s_dcs_write(dev, 0xE8, e8, sizeof(e8));
	if (ret < 0) {
		return ret;
	}

	static const uint8_t ea[] = {
		0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00,
		0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00,
	};

	ret = st7701s_dcs_write(dev, 0xEA, ea, sizeof(ea));
	if (ret < 0) {
		return ret;
	}

	static const uint8_t eb[] = {0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};

	ret = st7701s_dcs_write(dev, 0xEB, eb, sizeof(eb));
	if (ret < 0) {
		return ret;
	}

	static const uint8_t ec[] = {0x02, 0x00};

	ret = st7701s_dcs_write(dev, 0xEC, ec, sizeof(ec));
	if (ret < 0) {
		return ret;
	}

	static const uint8_t ed[] = {
		0xF5, 0x47, 0x6F, 0x0B, 0x8F, 0x9F, 0xFF, 0xFF,
		0xFF, 0xFF, 0xF9, 0xF8, 0xB0, 0xF6, 0x74, 0x5F,
	};

	ret = st7701s_dcs_write(dev, 0xED, ed, sizeof(ed));
	if (ret < 0) {
		return ret;
	}

	static const uint8_t ef[] = {
		0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
		0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
	};

	ret = st7701s_dcs_write(dev, 0xEF, ef, sizeof(ef));
	if (ret < 0) {
		return ret;
	}

	/* ---- Deselect all banks ---- */
	ret = st7701s_bank_select(dev, ST7701S_BKX_NONE);
	if (ret < 0) {
		return ret;
	}

	/* Display ON */
	return st7701s_dcs_write(dev, MIPI_DCS_SET_DISPLAY_ON, NULL, 0);
}

/* ---------------------------------------------------------------- init */

/* Passive panel: attach + power-on init only, no display_driver_api (the LCDC
 * compositor owns the framebuffer and reported capabilities).
 */
static int st7701s_init(const struct device *dev)
{
	const struct st7701s_config *cfg = dev->config;
	struct mipi_dsi_device mdev = {0};
	int ret;

	/* Reset sequence: inactive → assert 10ms → release, wait 120ms. */
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
		gpio_pin_set_dt(&cfg->reset, 1);  /* assert reset (physical LOW) */
		k_msleep(10);
		gpio_pin_set_dt(&cfg->reset, 0);  /* release reset (physical HIGH) */
		k_msleep(120);
	}

	/* Attach to MIPI-DSI host — triggers clock + DPHY + DSI init */
	mdev.data_lanes = cfg->data_lanes;
	mdev.pixfmt = cfg->pixfmt;
	/* Use sync-pulse non-burst (VIDEO) mode with LP command transport. */
	mdev.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM;

	mdev.timings.hactive = cfg->width;
	mdev.timings.hsync = cfg->hsync;
	mdev.timings.hbp = cfg->hbp;
	mdev.timings.hfp = cfg->hfp;
	mdev.timings.vactive = cfg->height;
	mdev.timings.vsync = cfg->vsync;
	mdev.timings.vbp = cfg->vbp;
	mdev.timings.vfp = cfg->vfp;

	ret = mipi_dsi_attach(cfg->mipi_dsi, cfg->channel, &mdev);
	if (ret < 0) {
		LOG_ERR("MIPI-DSI attach failed (%d)", ret);
		return ret;
	}

	ret = st7701s_dcs_write(dev, MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0);
	if (ret < 0) {
		return ret;
	}
	k_msleep(120); /* let charge pumps settle */

	ret = st7701s_configure(dev);
	if (ret < 0) {
		LOG_ERR("ST7701S init sequence failed (%d)", ret);
		return ret;
	}

	LOG_INF("ST7701S %ux%u panel ready", cfg->width, cfg->height);

	return 0;
}

#define ST7701S_TIMING(inst) DT_CHILD(DT_DRV_INST(inst), display_timings)

#define ST7701S_DEFINE(inst)                                                        \
	static const struct st7701s_config st7701s_config_##inst = {           \
		.mipi_dsi = DEVICE_DT_GET(DT_INST_BUS(inst)),                             \
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                \
		.channel = DT_INST_REG_ADDR(inst),                                         \
		.data_lanes = DT_INST_PROP_BY_IDX(inst, data_lanes, 0),                   \
		.pixfmt = DT_INST_PROP(inst, pixel_format),                                \
		.width = DT_INST_PROP(inst, width),                                        \
		.height = DT_INST_PROP(inst, height),                                      \
		.hsync = DT_PROP(ST7701S_TIMING(inst), hsync_len),                         \
		.hbp = DT_PROP(ST7701S_TIMING(inst), hback_porch),                         \
		.hfp = DT_PROP(ST7701S_TIMING(inst), hfront_porch),                        \
		.vsync = DT_PROP(ST7701S_TIMING(inst), vsync_len),                         \
		.vbp = DT_PROP(ST7701S_TIMING(inst), vback_porch),                         \
		.vfp = DT_PROP(ST7701S_TIMING(inst), vfront_porch),                        \
	};                                                                                 \
	DEVICE_DT_INST_DEFINE(inst, &st7701s_init, NULL,                             \
			      NULL, &st7701s_config_##inst,                          \
			      POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY,               \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(ST7701S_DEFINE)
