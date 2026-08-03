/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_lcdc_dsi

/* Include <soc.h> before <ameba_soc.h> to avoid redefining the unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>

LOG_MODULE_REGISTER(display_ameba_lcdc_dsi, CONFIG_DISPLAY_LOG_LEVEL);

/* Framebuffer format, fixed at build time.  The DSI compositor's layer formats
 * are little-endian packed, so only the two Zephyr formats whose memory layout
 * matches are usable:
 *   - ARGB8888 : 32-bit word 0xAARRGGBB == Zephyr PIXEL_FORMAT_ARGB_8888.
 *   - RGB565X  : byte-swapped RGB565 (RrrrrGgg gggBbbbb) == Zephyr PIXEL_FORMAT_RGB_565X.
 * The 24-bit RGB888 layer stores bytes as B,G,R, which Zephyr's
 * PIXEL_FORMAT_RGB_888 (R,G,B) does not match and there is no BGR888 enum, so
 * RGB888 is excluded here (and hidden on the DSI path in Kconfig).
 */
#if defined(CONFIG_AMEBA_LCDC_ARGB8888)
#define AMEBA_LCDC_INIT_PIXEL_SIZE   4u
#define AMEBA_LCDC_INIT_LAYER_FORMAT LCDC_LAYER_IMG_FORMAT_ARGB8888
#define DISPLAY_INIT_PIXEL_FORMAT    PIXEL_FORMAT_ARGB_8888
#elif defined(CONFIG_AMEBA_LCDC_BGR565)
#define AMEBA_LCDC_INIT_PIXEL_SIZE   2u
#define AMEBA_LCDC_INIT_LAYER_FORMAT LCDC_LAYER_IMG_FORMAT_RGB565
#define DISPLAY_INIT_PIXEL_FORMAT    PIXEL_FORMAT_RGB_565X
#else
#error "AmebaSmart LCDC-DSI format must be ARGB8888 / BGR565"
#endif

/* DMA burst length: 4 × 64 bytes, matching the validated SDK reference. */
#define AMEBA_LCDC_DMA_BURST        LCDC_LAYER_BURSTSIZE_4X64BYTES

/* Number of scan-out buffers actually allocated.  CONFIG_AMEBA_LCDC_FB_NUM == 0
 * means "application owns the framebuffer" (full-frame zero-copy writes only)
 */
#define AMEBA_LCDC_NUM_FB \
	((CONFIG_AMEBA_LCDC_FB_NUM) < 1 ? 1 : (CONFIG_AMEBA_LCDC_FB_NUM))

struct lcdc_ameba_config {
	LCDC_TypeDef *base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	uint16_t width;
	uint16_t height;
	struct k_heap *fb_heap;
	uint32_t frame_buffer_len;
	void (*irq_config_func)(void);
};

struct lcdc_ameba_data {
	enum display_pixel_format current_pixel_format;
	uint8_t current_pixel_size;
	bool enabled;
	uint8_t *frame_buffer;
	/* front_buf: buffer being scanned out.  pend_buf: armed flip target.
	 * Invariant (FB_NUM >= 2): write() makes them differ only on a frame's
	 * final strip, so the FRD ISR flips once per frame, never on a partial one.
	 */
	const uint8_t *front_buf;
	const uint8_t *pend_buf;
	bool composing;            /* a multi-write frame is being assembled */
	struct k_sem flip_sem;     /* writer blocks here until the ISR confirms a flip */
	LCDC_InitTypeDef init_cfg; /* cached config: bring-up, flip base, blanking */
};

/* Frame-done ISR: fires after the last active pixel. */
static void lcdc_ameba_isr(const struct device *dev)
{
	const struct lcdc_ameba_config *cfg = dev->config;
	struct lcdc_ameba_data *data = dev->data;
	uint32_t status = LCDC_GetINTStatus(cfg->base);

	LCDC_ClearINT(cfg->base, status);

	/* Flip only when write() armed a new frame; during composition
	 * pend_buf == front_buf, so this is a no-op.
	 */
	if ((status & LCDC_BIT_LCD_FRD_INTS) && data->front_buf != data->pend_buf) {
		data->front_buf = data->pend_buf;
		k_sem_give(&data->flip_sem);
	}

	if (status & LCDC_BIT_DMA_UN_INTS) {
		LOG_WRN("LCDC DMA underrun");
	}
}

static int lcdc_ameba_write(const struct device *dev, const uint16_t x, const uint16_t y,
			    const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct lcdc_ameba_config *cfg = dev->config;
	struct lcdc_ameba_data *data = dev->data;

	if ((x + desc->width) > cfg->width || (y + desc->height) > cfg->height) {
		LOG_ERR("write window (%u,%u %ux%u) exceeds %ux%u",
			x, y, desc->width, desc->height, cfg->width, cfg->height);
		return -EINVAL;
	}

#if CONFIG_AMEBA_LCDC_FB_NUM == 0
	/* App owns the framebuffer and draws into it via get_framebuffer(), so the
	 * copy-based display_write() is unsupported.
	 */
	ARG_UNUSED(data);
	ARG_UNUSED(buf);
	return -ENOTSUP;

#elif CONFIG_AMEBA_LCDC_FB_NUM == 1
	/* Render in place into the sole buffer.  No TE in DSI video mode, so this
	 * tears on moving content; use only for static images.
	 */
	const uint8_t *src = buf;
	const uint32_t line_bytes = (uint32_t)cfg->width * data->current_pixel_size;
	uint8_t *dst = data->frame_buffer + (y * cfg->width + x) * data->current_pixel_size;

	for (uint16_t row = 0; row < desc->height; row++) {
		memcpy(dst, src, (size_t)desc->width * data->current_pixel_size);
		dst += line_bytes;
		src += (size_t)desc->pitch * data->current_pixel_size;
	}
	sys_cache_data_flush_range(data->frame_buffer + (size_t)y * line_bytes,
				   (size_t)desc->height * line_bytes);
	return 0;

#else /* CONFIG_AMEBA_LCDC_FB_NUM >= 2: compose into the back buffer, then flip. */
	const uint8_t *src = buf;
	const uint32_t pixel_size = data->current_pixel_size;
	const uint32_t line_bytes = (uint32_t)cfg->width * pixel_size;
	uint8_t *back = (data->front_buf == data->frame_buffer)
				? data->frame_buffer + cfg->frame_buffer_len
				: data->frame_buffer;
	const bool full_frame = (x == 0 && y == 0 &&
				 desc->width == cfg->width && desc->height == cfg->height);
	bool dirty_all = full_frame;
	uint8_t *dst;

	/* Seed the back buffer once per frame so a partial update keeps untouched
	 * pixels.  front_buf is stable here: pend_buf == front_buf during
	 * composition, so the ISR cannot flip it.
	 */
	if (!data->composing && !full_frame) {
		memcpy(back, data->front_buf, cfg->frame_buffer_len);
		dirty_all = true;
	}
	data->composing = true;

	dst = back + (y * cfg->width + x) * pixel_size;
	for (uint16_t row = 0; row < desc->height; row++) {
		memcpy(dst, src, (size_t)desc->width * pixel_size);
		dst += line_bytes;
		src += (size_t)desc->pitch * pixel_size;
	}

	if (dirty_all) {
		sys_cache_data_flush_range(back, cfg->frame_buffer_len);
	} else {
		sys_cache_data_flush_range(back + (size_t)y * line_bytes,
					   (size_t)desc->height * line_bytes);
	}

	/* More strips to come: leave pend_buf == front_buf so the ISR won't flip. */
	if (desc->frame_incomplete) {
		return 0;
	}

	/* Publish: point the layer base (a shadow register, safe to write mid-scan)
	 * at the back buffer, request a VBlank reload, then arm pend_buf last and
	 * wait for the FRD ISR to confirm the switch.
	 */
	data->composing = false;
	k_sem_reset(&data->flip_sem);
	data->init_cfg.layerx[LCDC_LAYER_LAYER1].LCDC_LayerImgBaseAddr = (uint32_t)back;
	cfg->base->LCDC_LAYER[LCDC_LAYER_LAYER1].LCDC_LAYERx_BASE_ADDR =
		LCDC_LAYERx_IMG_BASE_ADDR((uint32_t)back);
	LCDC_TrigerSHWReload(cfg->base);
	data->pend_buf = back;
	k_sem_take(&data->flip_sem, K_FOREVER);
	return 0;
#endif
}

static int lcdc_ameba_read(const struct device *dev, const uint16_t x, const uint16_t y,
			   const struct display_buffer_descriptor *desc, void *buf)
{
	const struct lcdc_ameba_config *cfg = dev->config;
	struct lcdc_ameba_data *data = dev->data;
	uint8_t *dst = buf;
	const uint8_t *src;
	uint16_t row;

	if ((x + desc->width) > cfg->width || (y + desc->height) > cfg->height) {
		return -EINVAL;
	}

	src = data->front_buf + (y * cfg->width + x) * data->current_pixel_size;

	for (row = 0; row < desc->height; row++) {
		memcpy(dst, src, (size_t)desc->width * data->current_pixel_size);
		src += cfg->width * data->current_pixel_size;
		dst += (size_t)desc->pitch * data->current_pixel_size;
	}

	return 0;
}

static void *lcdc_ameba_get_framebuffer(const struct device *dev)
{
	struct lcdc_ameba_data *data = dev->data;

	return (void *)data->front_buf;
}

/* Start scan-out once and leave it running; blanking only toggles the layer. */
static void lcdc_ameba_enable_stream(const struct device *dev)
{
	const struct lcdc_ameba_config *cfg = dev->config;
	struct lcdc_ameba_data *data = dev->data;

	LCDC_Init(cfg->base, &data->init_cfg);
	LCDC_DMAModeConfig(cfg->base, AMEBA_LCDC_DMA_BURST);
	LCDC_DMADebugConfig(cfg->base, LCDC_DMA_OUT_DISABLE, 0);

	LCDC_Cmd(cfg->base, ENABLE);
	LCDC_TrigerSHWReload(cfg->base);

#if CONFIG_AMEBA_LCDC_FB_NUM >= 2
	/* Double buffer: enable the frame-done IRQ so the ISR can confirm flips. */
	LCDC_INTConfig(cfg->base, LCDC_BIT_LCD_FRD_INTEN | LCDC_BIT_DMA_UN_INTEN, ENABLE);
#else
	LCDC_INTConfig(cfg->base, LCDC_BIT_DMA_UN_INTEN, ENABLE);
#endif

	data->enabled = true;
}

/* Toggle the image layer; the LCDC keeps scanning so the MIPI link stays in sync. */
static void lcdc_ameba_set_layer(const struct device *dev, bool on)
{
	const struct lcdc_ameba_config *cfg = dev->config;
	struct lcdc_ameba_data *data = dev->data;

	data->init_cfg.layerx[LCDC_LAYER_LAYER1].LCDC_LayerEn = on ? ENABLE : DISABLE;
	LCDC_LayerConfig(cfg->base, LCDC_LAYER_LAYER1,
			 &data->init_cfg.layerx[LCDC_LAYER_LAYER1]);
	LCDC_TrigerSHWReload(cfg->base);
}

static int lcdc_ameba_blanking_off(const struct device *dev)
{
	struct lcdc_ameba_data *data = dev->data;

	if (data->enabled) {
		return 0;
	}

	lcdc_ameba_set_layer(dev, true);
	data->enabled = true;
	return 0;
}

static int lcdc_ameba_blanking_on(const struct device *dev)
{
	struct lcdc_ameba_data *data = dev->data;

	if (!data->enabled) {
		return 0;
	}

	/* Blank by disabling the layer: the LCDC keeps scanning (MIPI stays in
	 * sync) and shows the black background.  No backlight/panel-power control
	 * here, so this is a black screen, not a true power-off.
	 */
	lcdc_ameba_set_layer(dev, false);
	data->enabled = false;
	return 0;
}

/* Map a Zephyr pixel format to the LCDC layer format + byte size.  Only the two
 * whose memory layout matches a DSI layer format are supported; the others are
 * byte-swapped vs the little-endian LCDC layers (RGB565 vs the 16-bit layer,
 * RGB888 vs the B,G,R 24-bit layer) and are rejected.
 */
static bool lcdc_ameba_map_format(enum display_pixel_format format, uint32_t *layer_fmt,
				  uint8_t *pixel_size)
{
	switch (format) {
	case PIXEL_FORMAT_ARGB_8888:
		*layer_fmt = LCDC_LAYER_IMG_FORMAT_ARGB8888;
		*pixel_size = 4u;
		return true;
	case PIXEL_FORMAT_RGB_565X:
		*layer_fmt = LCDC_LAYER_IMG_FORMAT_RGB565;
		*pixel_size = 2u;
		return true;
	default:
		return false;
	}
}

static int lcdc_ameba_set_pixel_format(const struct device *dev,
				       const enum display_pixel_format format)
{
	const struct lcdc_ameba_config *cfg = dev->config;
	struct lcdc_ameba_data *data = dev->data;
	uint32_t layer_fmt;
	uint8_t pixel_size;

	if (!lcdc_ameba_map_format(format, &layer_fmt, &pixel_size)) {
		return -ENOTSUP;
	}

	/* FB slots are sized for the build-time format; reject anything wider. */
	if (pixel_size > AMEBA_LCDC_INIT_PIXEL_SIZE) {
		return -ENOTSUP;
	}

	/* Don't switch mid-frame: the back buffer is composed at the old size, so
	 * reloading the layer format now would mismatch its contents.
	 */
	if (data->composing) {
		return -EBUSY;
	}

	if (format == data->current_pixel_format) {
		return 0;
	}

	data->current_pixel_format = format;
	data->current_pixel_size = pixel_size;

	data->init_cfg.layerx[LCDC_LAYER_LAYER1].LCDC_LayerImgFormat = layer_fmt;
	LCDC_LayerConfig(cfg->base, LCDC_LAYER_LAYER1,
			 &data->init_cfg.layerx[LCDC_LAYER_LAYER1]);
	LCDC_TrigerSHWReload(cfg->base);
	return 0;
}

static void lcdc_ameba_get_capabilities(const struct device *dev,
					struct display_capabilities *caps)
{
	const struct lcdc_ameba_config *cfg = dev->config;
	struct lcdc_ameba_data *data = dev->data;

	memset(caps, 0, sizeof(struct display_capabilities));
	caps->x_resolution = cfg->width;
	caps->y_resolution = cfg->height;
	/* Advertise the mappable formats that fit the build-time-sized FB slot. */
	caps->supported_pixel_formats = PIXEL_FORMAT_RGB_565X
#if AMEBA_LCDC_INIT_PIXEL_SIZE >= 4
					| PIXEL_FORMAT_ARGB_8888
#endif
		;
	caps->current_pixel_format = data->current_pixel_format;
	caps->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

static DEVICE_API(display, lcdc_ameba_api) = {
	.blanking_on = lcdc_ameba_blanking_on,
	.blanking_off = lcdc_ameba_blanking_off,
	.write = lcdc_ameba_write,
	.read = lcdc_ameba_read,
	.get_framebuffer = lcdc_ameba_get_framebuffer,
	.get_capabilities = lcdc_ameba_get_capabilities,
	.set_pixel_format = lcdc_ameba_set_pixel_format,
};

static int lcdc_ameba_init(const struct device *dev)
{
	const struct lcdc_ameba_config *cfg = dev->config;
	struct lcdc_ameba_data *data = dev->data;
	LCDC_InitTypeDef *init = &data->init_cfg;
	int ret;

	data->current_pixel_format = DISPLAY_INIT_PIXEL_FORMAT;
	data->current_pixel_size = AMEBA_LCDC_INIT_PIXEL_SIZE;
	data->enabled = false;
	data->composing = false;

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("failed to enable LCDC clock: %d", ret);
		return ret;
	}

	/* 64-byte alignment matches the CA32 cache line. */
	data->frame_buffer = k_heap_aligned_alloc(cfg->fb_heap, 64,
						  cfg->frame_buffer_len * AMEBA_LCDC_NUM_FB,
						  K_NO_WAIT);
	if (data->frame_buffer == NULL) {
		LOG_ERR("failed to allocate %u-byte FB pool (%u bufs)",
			cfg->frame_buffer_len * AMEBA_LCDC_NUM_FB, AMEBA_LCDC_NUM_FB);
		return -ENOMEM;
	}

	LCDC_StructInit(init);
	init->LCDC_ImageWidth = cfg->width;
	init->LCDC_ImageHeight = cfg->height;

	/* Explicit black background: shown where no layer covers and while the
	 * layer is disabled (blanking).  LCDC_Init applies it via LCDC_SetBkgColor.
	 */
	init->LCDC_BgColorRed = 0;
	init->LCDC_BgColorGreen = 0;
	init->LCDC_BgColorBlue = 0;

	/* The LCDC has 3 blendable layers; the Zephyr display API is single-
	 * framebuffer, so drive one full-screen opaque layer and disable the rest.
	 */
	init->layerx[LCDC_LAYER_LAYER1].LCDC_LayerEn = ENABLE;
	/* Colour-keying off: with the default key 0 every black pixel would
	 * become transparent, breaking the opaque single-layer intent.
	 */
	init->layerx[LCDC_LAYER_LAYER1].LCDC_LayerColorKeyingEn = DISABLE;
	init->layerx[LCDC_LAYER_LAYER1].LCDC_LayerImgFormat = AMEBA_LCDC_INIT_LAYER_FORMAT;
	init->layerx[LCDC_LAYER_LAYER1].LCDC_LayerImgBaseAddr = (uint32_t)data->frame_buffer;
	init->layerx[LCDC_LAYER_LAYER1].LCDC_LayerHorizontalStart = 1;
	init->layerx[LCDC_LAYER_LAYER1].LCDC_LayerHorizontalStop = cfg->width;
	init->layerx[LCDC_LAYER_LAYER1].LCDC_LayerVerticalStart = 1;
	init->layerx[LCDC_LAYER_LAYER1].LCDC_LayerVerticalStop = cfg->height;
	init->layerx[LCDC_LAYER_LAYER1].LCDC_LayerConstAlpha = 0xFF;

	init->layerx[LCDC_LAYER_LAYER2].LCDC_LayerEn = DISABLE;
	init->layerx[LCDC_LAYER_LAYER3].LCDC_LayerEn = DISABLE;

	data->front_buf = data->frame_buffer;
	data->pend_buf = data->frame_buffer;
	k_sem_init(&data->flip_sem, 0, 1);

	cfg->irq_config_func();

	memset(data->frame_buffer, 0, cfg->frame_buffer_len * AMEBA_LCDC_NUM_FB);
	sys_cache_data_flush_range(data->frame_buffer,
				   cfg->frame_buffer_len * AMEBA_LCDC_NUM_FB);

	LOG_DBG("AmebaSmart LCDC %ux%u fmt=0x%x %uBpp fb=%p (%u buf)",
		cfg->width, cfg->height, DISPLAY_INIT_PIXEL_FORMAT,
		AMEBA_LCDC_INIT_PIXEL_SIZE, (void *)data->frame_buffer, AMEBA_LCDC_NUM_FB);

	lcdc_ameba_enable_stream(dev);

	return 0;
}

#define LCDC_AMEBA_FB_LEN(n) \
	(DT_INST_PROP(n, width) * DT_INST_PROP(n, height) * AMEBA_LCDC_INIT_PIXEL_SIZE)

/* Per-instance display heap: AMEBA_LCDC_NUM_FB full-screen buffers plus slack. */
#define LCDC_AMEBA_INIT(n)                                                                         \
	K_HEAP_DEFINE(lcdc_ameba_heap_##n,                                                         \
		      LCDC_AMEBA_FB_LEN(n) * AMEBA_LCDC_NUM_FB + 512);                            \
	static void lcdc_ameba_irq_config_##n(void)                                                \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),                            \
			    lcdc_ameba_isr, DEVICE_DT_INST_GET(n), 0);                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
	static const struct lcdc_ameba_config lcdc_ameba_config_##n = {                            \
		.base = (LCDC_TypeDef *)DT_INST_REG_ADDR(n),                                       \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, idx),               \
		.width = DT_INST_PROP(n, width),                                                   \
		.height = DT_INST_PROP(n, height),                                                 \
		.fb_heap = &lcdc_ameba_heap_##n,                                                   \
		.frame_buffer_len = LCDC_AMEBA_FB_LEN(n),                                          \
		.irq_config_func = lcdc_ameba_irq_config_##n,                                      \
	};                                                                                         \
	static struct lcdc_ameba_data lcdc_ameba_data_##n;                                         \
	DEVICE_DT_INST_DEFINE(n, &lcdc_ameba_init, NULL, &lcdc_ameba_data_##n,                     \
			      &lcdc_ameba_config_##n, POST_KERNEL,                                 \
			      CONFIG_DISPLAY_INIT_PRIORITY, &lcdc_ameba_api);

DT_INST_FOREACH_STATUS_OKAY(LCDC_AMEBA_INIT)
